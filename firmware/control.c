#include "control.h"
#include "bsp_encoder.h"
#include "bsp_gray.h"
#include "bsp_imu.h"
#include "bsp_motor.h"
#include "mathx.h"
#include "pid.h"

static ctrl_state_t s_st;
static ctrl_gains_t s_g = {
    GAIN_SP_KP_DEF, GAIN_SP_KI_DEF,
    GAIN_HD_KP_DEF, GAIN_HD_KI_DEF, GAIN_HD_KD_DEF,
    GAIN_LN_KP_DEF, GAIN_LN_KI_DEF, GAIN_LN_KD_DEF,
    GAIN_TURN_KP_DEF
};

/* 外环状态 */
static float s_hdg_ref;        /* 直线/圆弧航向参考（场地坐标） */
static float s_turn_target;
static float s_hdg_i, s_hdg_prev;
static float s_ln_i, s_ln_prev;
static float s_v_cmd;
static float s_curvature;
static float s_yaw_off;        /* 场地航向 = 陀螺仪相对航向 + offset */
static float s_yaw_f;          /* 航向低通（陀螺仪 100Hz，控制 200Hz） */
static float s_hdg_d;          /* 航向误差微分低通 */
static float s_line_f;         /* 灰度位置低通 */
static uint16_t s_line_lost_ms;

/* 轮速环 */
static float s_duty_l, s_duty_r;
static float s_sp_i_l, s_sp_i_r;
static float s_v_l_f, s_v_r_f;

static int16_t s_raw_l, s_raw_r;
static uint8_t s_raw_mode;

void control_init(void)
{
    s_st.mode = CM_IDLE;
    s_hdg_ref = 0;
    s_turn_target = 0;
    s_hdg_i = s_hdg_prev = 0;
    s_ln_i = s_ln_prev = 0;
    s_v_cmd = 0;
    s_duty_l = s_duty_r = 0;
    s_sp_i_l = s_sp_i_r = 0;
    s_v_l_f = s_v_r_f = 0;
    s_raw_mode = 0;
    s_line_lost_ms = 0;
    s_yaw_f = 0;
    s_hdg_d = 0;
    s_line_f = 0;
    s_st.line_mm = 0;
    s_st.line_on = 0;
    s_st.line_cnt = 0;
    motor_stop();
}

const ctrl_state_t *control_state(void) { return &s_st; }
ctrl_gains_t *control_gains(void) { return &s_g; }

void control_set_yaw_offset(float off)
{
    s_yaw_off = off;
    s_yaw_f = wrap180(imu_yaw() + s_yaw_off);
}

/* 当前场地航向（度），任何时刻都可由中断上下文调用 */
static float ctrl_yaw(void)
{
    return wrap180(imu_yaw() + s_yaw_off);
}

void control_idle(void)
{
    s_raw_mode = 0;
    s_st.mode = CM_IDLE;
    s_duty_l = s_duty_r = 0;
    s_sp_i_l = s_sp_i_r = 0;
    motor_stop();
}

void control_brake(void)
{
    s_raw_mode = 0;
    s_st.mode = CM_BRAKE;
    s_duty_l = s_duty_r = 0;
    motor_brake();
}

void control_raw(int16_t dutyL, int16_t dutyR)
{
    s_raw_mode = 1;
    s_raw_l = dutyL;
    s_raw_r = dutyR;
    s_st.mode = CM_IDLE;
    motor_set(dutyL, dutyR);
}

uint8_t control_is_raw(void)
{
    return s_raw_mode;
}

void control_straight(float heading_deg, float v_cm_s)
{
    s_raw_mode = 0;
    s_st.yaw = ctrl_yaw();
    s_yaw_f = s_st.yaw;
    if (s_st.mode != CM_STRAIGHT) {
        s_hdg_d = 0;
        s_hdg_i = 0;
        s_hdg_prev = ang_diff(heading_deg, s_st.yaw);
    }
    s_hdg_ref = heading_deg;
    s_v_cmd = v_cm_s;
    s_st.mode = CM_STRAIGHT;
}

void control_line(float heading_deg, float v_cm_s, float curvature)
{
    s_raw_mode = 0;
    s_st.yaw = ctrl_yaw();
    s_yaw_f = s_st.yaw;
    if (s_st.mode != CM_LINE) {
        s_hdg_i = 0;
        s_ln_i = 0;
        s_hdg_d = 0;
        s_hdg_prev = ang_diff(heading_deg, s_st.yaw);
        s_ln_prev = s_st.line_mm;
        s_line_f = s_st.line_mm;
    }
    s_hdg_ref = heading_deg;
    s_v_cmd = v_cm_s;
    s_curvature = curvature;
    s_st.mode = CM_LINE;
}

void control_turn(float target_heading_deg)
{
    s_raw_mode = 0;
    if (s_st.mode != CM_TURN) {
        s_turn_target = target_heading_deg;
    }
    s_st.mode = CM_TURN;
}

/* ------------------------------------------------------------------ */
/* 轮速 PI + 前馈                                                      */
/* ------------------------------------------------------------------ */
static float speed_loop(float vref, float vmeas, float *integ, float duty_prev)
{
    float err = vref - vmeas;

    if (vref <= 0.01f && vmeas < 1.0f) {
        *integ = 0.0f;
        return 0.0f;
    }

    float ff = vref * DUTY_PER_CM_S;

    *integ += err * CTRL_DT;
    *integ = clampf(*integ, -400.0f, 400.0f);

    float duty = ff + s_g.sp_kp * err + s_g.sp_ki * (*integ);

    if (duty > DUTY_MAX) {
        *integ -= err * CTRL_DT;
        duty = DUTY_MAX;
    } else if (duty < 0.0f) {
        if (vref <= 0.01f) *integ = 0.0f;
        else *integ -= err * CTRL_DT;
        duty = 0.0f;
    }

    if (vref > 1.0f && duty < DUTY_MIN_MOVE) duty = DUTY_MIN_MOVE;

    /* 限制占空比变化率，保护电机/齿轮 */
    const float max_step = 90.0f;
    if (duty > duty_prev + max_step) duty = duty_prev + max_step;
    if (duty < duty_prev - max_step) duty = duty_prev - max_step;

    return duty;
}

void control_tick(float dt)
{
    (void)dt;

    /* ---- 采样 ---- */
    encoder_read_speed(CTRL_DT, &s_st.vL, &s_st.vR);
    s_v_l_f = lpf(s_v_l_f, s_st.vL, 0.45f);
    s_v_r_f = lpf(s_v_r_f, s_st.vR, 0.45f);
    s_yaw_f = lpf(s_yaw_f, ctrl_yaw(), 0.5f);
    s_st.yaw = s_yaw_f;

    const gray_t *g = gray_read();
    s_st.line_mm = g->pos_mm;
    s_st.line_on = g->on_line;
    s_st.line_cnt = g->count;

    if (g->on_line) s_line_lost_ms = 0;
    else if (s_line_lost_ms < 60000u) s_line_lost_ms += 5;

    if (s_raw_mode) {
        motor_set(s_raw_l, s_raw_r);
        return;
    }

    float vrefL = 0.0f, vrefR = 0.0f;

    switch (s_st.mode) {
    case CM_STRAIGHT: {
        float err = ang_diff(s_hdg_ref, s_st.yaw);
        float d_raw = (err - s_hdg_prev) / CTRL_DT;
        s_hdg_prev = err;
        s_hdg_d = lpf(s_hdg_d, d_raw, 0.35f);
        s_hdg_i += err * CTRL_DT;
        s_hdg_i = clampf(s_hdg_i, -60.0f, 60.0f);

        float omega = s_g.hd_kp * err + s_g.hd_ki * s_hdg_i +
                      s_g.hd_kd * s_hdg_d;
        omega = clampf(omega, -2.6f, 2.6f);

        float diff = omega * CAR_W_CM * 0.5f;
        vrefL = s_v_cmd - diff;
        vrefR = s_v_cmd + diff;
        break;
    }
    case CM_LINE: {
        float e = s_st.line_mm;
        if (s_line_lost_ms > 200u) e = 0.0f;   /* 长时间丢线：只保航向 */

        /* 数字灰度是离散跳变，先低通再做微分，避免 D 项被量化噪声打爆 */
        s_line_f = lpf(s_line_f, e, 0.45f);
        float ef = s_line_f;
        float d_raw = (ef - s_ln_prev) / CTRL_DT;
        s_ln_prev = ef;
        float d = clampf(d_raw, -3000.0f, 3000.0f);

        s_ln_i += ef * CTRL_DT;
        s_ln_i = clampf(s_ln_i, -600.0f, 600.0f);

        float hdg_err = ang_diff(s_hdg_ref, s_st.yaw);

        /* 曲率前馈：沿已知圆弧所需的角速度 v/R，PD 只修偏差 */
        float omega_ff = s_v_cmd * s_curvature;

        float omega = omega_ff
                      - (s_g.ln_kp * ef + s_g.ln_ki * s_ln_i +
                         s_g.ln_kd * d)
                      + s_g.hd_kp * 2.0f * hdg_err;
        omega = clampf(omega, -3.2f, 3.2f);

        float diff = omega * CAR_W_CM * 0.5f;
        float v = s_v_cmd;

        float ae = fabsf(ef);
        if (ae > 25.0f) v *= clampf(1.0f - (ae - 25.0f) / 60.0f, 0.45f, 1.0f);

        vrefL = v - diff;
        vrefR = v + diff;
        break;
    }
    case CM_TURN: {
        float err = ang_diff(s_turn_target, s_st.yaw);
        float ae = fabsf(err);

        float vout = clampf(s_g.turn_kp * ae, 3.0f, V_TURN_MAX);
        if (ae < 4.0f) vout = clampf(s_g.turn_kp * ae, 3.0f, 7.0f);

        if (err >= 0.0f) { vrefL = 0.0f; vrefR = vout; }   /* 左转：右轮外 */
        else             { vrefL = vout; vrefR = 0.0f; }   /* 右转：左轮外 */

        s_v_cmd = vout;
        break;
    }
    case CM_BRAKE:
    default:
        motor_brake();
        s_duty_l = s_duty_r = 0;
        s_sp_i_l = s_sp_i_r = 0;
        return;
    }

    /* 只允许前进 */
    vrefL = clampf(vrefL, 0.0f, V_MAX_CM_S);
    vrefR = clampf(vrefR, 0.0f, V_MAX_CM_S);

    s_st.vrefL = vrefL;
    s_st.vrefR = vrefR;

    float dl = speed_loop(vrefL, s_v_l_f, &s_sp_i_l, s_duty_l);
    float dr = speed_loop(vrefR, s_v_r_f, &s_sp_i_r, s_duty_r);

    s_duty_l = dl;
    s_duty_r = dr;
    s_st.dutyL = (int16_t)(dl + 0.5f);
    s_st.dutyR = (int16_t)(dr + 0.5f);

    motor_set(s_st.dutyL, s_st.dutyR);
}
