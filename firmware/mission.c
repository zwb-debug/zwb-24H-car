#include "mission.h"
#include "bsp_encoder.h"
#include "bsp_imu.h"
#include "bsp_motor.h"
#include "bsp_ui.h"
#include "control.h"
#include "mathx.h"
#include "timebase.h"

#define EV_A 1
#define EV_B 2
#define EV_C 3
#define EV_D 4

#define TURN_ENTRY_V 16.0f

/* ---------------- 第 1 题：A -> B ---------------- */
static const segment_t SEG_T1[] = {
    {SEG_LINE, LEN_AB_CM, H_AB_DEG, 0, V_TASK1, EV_B, 1},
};

/* ---------------- 第 2 题：A-B-C-D-A ---------------- */
static const segment_t SEG_T2[] = {
    {SEG_LINE, LEN_AB_CM,  H_AB_DEG, 0, V_TASK2, EV_B, 0},
    {SEG_ARC,  LEN_ARC_CM, H_AB_DEG, -1, V_TASK2 * V_ARC_SCALE, EV_C, 0},
    {SEG_LINE, LEN_AB_CM,  H_CD_DEG, 0, V_TASK2, EV_D, 0},
    {SEG_ARC,  LEN_ARC_CM, H_CD_DEG, -1, V_TASK2 * V_ARC_SCALE, EV_A, 1},
};

/* ---------------- 第 3 题：A-C-B-D-A ---------------- */
static const segment_t SEG_T3[] = {
    {SEG_LINE, LEN_DIAG_CM, H_AC_DEG, 0, V_TASK3, EV_C, 0},
    {SEG_TURN, 0,           H_AB_DEG, 0, 0,      0,    0},
    {SEG_ARC,  LEN_ARC_CM,  H_AB_DEG, +1, V_TASK3 * V_ARC_SCALE, EV_B, 0},
    {SEG_TURN, 0,           H_BD_DEG, 0, 0,      0,    0},
    {SEG_LINE, LEN_DIAG_CM, H_BD_DEG, 0, V_TASK3, EV_D, 0},
    {SEG_TURN, 0,           H_CD_DEG, 0, 0,      0,    0},
    {SEG_ARC,  LEN_ARC_CM,  H_CD_DEG, -1, V_TASK3 * V_ARC_SCALE, EV_A, 1},
};

/* ---------------- 第 4 题：第 3 题路线 x4 ---------------- */
static const segment_t SEG_T4[] = {
    {SEG_LINE, LEN_DIAG_CM, H_AC_DEG, 0, V_TASK4, EV_C, 0},
    {SEG_TURN, 0,           H_AB_DEG, 0, 0,      0,    0},
    {SEG_ARC,  LEN_ARC_CM,  H_AB_DEG, +1, V_TASK4 * V_ARC_SCALE, EV_B, 0},
    {SEG_TURN, 0,           H_BD_DEG, 0, 0,      0,    0},
    {SEG_LINE, LEN_DIAG_CM, H_BD_DEG, 0, V_TASK4, EV_D, 0},
    {SEG_TURN, 0,           H_CD_DEG, 0, 0,      0,    0},
    {SEG_ARC,  LEN_ARC_CM,  H_CD_DEG, -1, V_TASK4 * V_ARC_SCALE, EV_A, 0},
};

#define N_SEG(a) (sizeof(a) / sizeof((a)[0]))

static mission_info_t s_mi;
static const segment_t *s_seg;
static uint8_t s_seg_n;
static uint8_t s_idx;
static uint8_t s_interlap;          /* 第 4 题圈间转向中 */
static float   s_interlap_target;
static float   s_yaw_offset;
static float   s_v_now;
static uint32_t s_t0;
static uint8_t s_start_req;
static task_id_t s_start_task;
static uint8_t s_abort_req;
static uint16_t s_imu_bad_ms;
static uint32_t s_end_ms;
static uint8_t s_hold_done;

static inline float yaw_field(void)
{
    return wrap180(s_yaw_offset + imu_yaw());
}

static float seg_v_end(uint8_t idx)
{
    /* 第 4 题最后一段圆弧：最后一圈要停在 A，其它圈要减速进圈间转向 */
    if (s_mi.task == TASK_4 && (uint8_t)(idx + 1) == s_seg_n) {
        return (s_mi.lap < s_mi.lap_total) ? TURN_ENTRY_V : 0.0f;
    }
    if (s_seg[idx].stop_at_end) return 0.0f;
    if (idx + 1 < s_seg_n) {
        if (s_seg[idx + 1].type == SEG_TURN) return TURN_ENTRY_V;
        if (s_seg[idx + 1].type == SEG_LINE) return s_seg[idx].speed;
    }
    return s_seg[idx].speed;
}

static void seg_begin(uint8_t idx)
{
    const segment_t *sg = &s_seg[idx];
    const ctrl_state_t *cs = control_state();
    s_mi.s_seg = 0.0f;
    s_v_now = 0.5f * (cs->vL + cs->vR);
    if (s_v_now > sg->speed) s_v_now = sg->speed;

    switch (sg->type) {
    case SEG_LINE:
        control_straight(sg->heading_deg, s_v_now);
        break;
    case SEG_ARC:
        control_line(sg->heading_deg, s_v_now, (float)sg->dir / ARC_R_CM);
        break;
    case SEG_TURN:
        control_turn(sg->heading_deg);
        break;
    default:
        control_brake();
        break;
    }
}

static void mission_finish(uint8_t ok)
{
    control_brake();
    s_mi.state = ok ? ST_DONE : ST_FAULT;
    s_mi.v_ref = 0;
    s_mi.v_now = 0;
    s_end_ms = millis();
    s_hold_done = 0;
    if (ok) {
        ui_beep(420);
        ui_led(1, 1);
    } else {
        ui_beep_fail();
        ui_led(0, 1);
    }
}

static void mission_start(task_id_t t)
{
    s_mi.task = t;
    s_mi.lap = 1;
    s_mi.lap_total = (t == TASK_4) ? 4 : 1;
    s_mi.fault_code = 0;
    s_mi.s_total = 0;
    s_mi.seg_idx = 0;

    switch (t) {
    case TASK_1:
        s_seg = SEG_T1; s_seg_n = N_SEG(SEG_T1); s_mi.limit_ms = T_LIMIT1_MS;
        s_yaw_offset = H_AB_DEG;
        break;
    case TASK_2:
        s_seg = SEG_T2; s_seg_n = N_SEG(SEG_T2); s_mi.limit_ms = T_LIMIT2_MS;
        s_yaw_offset = H_AB_DEG;
        break;
    case TASK_3:
        s_seg = SEG_T3; s_seg_n = N_SEG(SEG_T3); s_mi.limit_ms = T_LIMIT3_MS;
        s_yaw_offset = H_AC_DEG;
        break;
    case TASK_4:
        s_seg = SEG_T4; s_seg_n = N_SEG(SEG_T4); s_mi.limit_ms = T_LIMIT4_MS;
        s_yaw_offset = H_AC_DEG;
        break;
    default:
        return;
    }

    imu_zero();
    encoder_reset();
    control_init();
    control_set_yaw_offset(s_yaw_offset);

    s_idx = 0;
    s_mi.seg_num = s_seg_n;
    s_mi.s_seg = 0;
    s_interlap = 0;
    s_imu_bad_ms = 0;
    s_v_now = 0;
    s_t0 = millis();
    s_mi.state = ST_RUN;
    s_mi.elapsed_ms = 0;

    ui_led(1, 0);
    ui_beep_node();            /* 发车提示 */
    seg_begin(0);
}

/* 段结束：处理节点提示、切换下一段 */
static void seg_complete(void)
{
    const segment_t *sg = &s_seg[s_idx];

    if (sg->event) {
        ui_beep_node();
        ui_led_flash(2, 120);
    }

    s_idx++;

    if (s_idx < s_seg_n) {
        s_mi.seg_idx = s_idx;
        seg_begin(s_idx);
        return;
    }

    /* 本题路线跑完 */
    if (s_mi.task == TASK_4 && s_mi.lap < s_mi.lap_total) {
        s_mi.lap++;
        s_interlap = 1;
        s_interlap_target = H_AC_DEG;
        control_turn(s_interlap_target);
        return;
    }

    mission_finish(1);
}

static void run_tick(void)
{
    const segment_t *sg = &s_seg[s_idx];
    const ctrl_state_t *cs = control_state();
    float yaw = yaw_field();
    float v = 0.5f * (cs->vL + cs->vR);

    s_mi.elapsed_ms = millis() - s_t0;
    s_mi.s_seg += v * CTRL_DT;
    s_mi.s_total += v * CTRL_DT;

    /* 陀螺仪掉线保护 */
    if (!imu_ok()) {
        s_imu_bad_ms += 5;
        if (s_imu_bad_ms > 600) {
            s_mi.fault_code = 2;
            mission_finish(0);
            return;
        }
    } else {
        s_imu_bad_ms = 0;
    }

    if (s_mi.elapsed_ms > s_mi.limit_ms) {
        s_mi.fault_code = 1;
        mission_finish(0);
        return;
    }

    /* 第 4 题圈间转向 */
    if (s_interlap) {
        float err = ang_diff(s_interlap_target, yaw);
        if (fabsf(err) < 1.2f) {
            s_interlap = 0;
            s_idx = 0;
            s_mi.seg_idx = 0;
            seg_begin(0);
        } else {
            control_turn(s_interlap_target);
        }
        return;
    }

    switch (sg->type) {
    case SEG_LINE: {
        float remain = sg->len_cm - s_mi.s_seg;
        float v_end = seg_v_end(s_idx);

        float v_allow = sqrtf(v_end * v_end + 2.0f * DEC_LIMIT_CM_S2 *
                              (remain > 0.0f ? remain : 0.0f));
        float vt = fminf(sg->speed, v_allow);

        float dv = ACC_LIMIT_CM_S2 * CTRL_DT;
        if (vt > s_v_now + dv) vt = s_v_now + dv;
        if (vt < s_v_now - dv * 2.0f) vt = s_v_now - dv * 2.0f;
        s_v_now = vt;

        s_mi.v_ref = vt;

        /* 航向偏差过大：说明起跑姿态或转角严重错误 */
        if (fabsf(ang_diff(sg->heading_deg, yaw)) > 45.0f) {
            s_mi.fault_code = 3;
            mission_finish(0);
            return;
        }

        control_straight(sg->heading_deg, vt);

        if (remain <= 0.8f || (sg->stop_at_end && remain < 3.0f && v < 3.0f)) {
            seg_complete();
        }
        break;
    }
    case SEG_ARC: {
        float remain = sg->len_cm - s_mi.s_seg;
        float tangent = wrap180(sg->heading_deg +
            (float)sg->dir * (s_mi.s_seg / ARC_R_CM) * 57.29578f);

        float v_end = seg_v_end(s_idx);
        float v_allow = sqrtf(v_end * v_end + 2.0f * DEC_LIMIT_CM_S2 *
                              (remain > 0.0f ? remain : 0.0f));
        float vt = fminf(sg->speed, v_allow);

        float dv = ACC_LIMIT_CM_S2 * CTRL_DT;
        if (vt > s_v_now + dv) vt = s_v_now + dv;
        if (vt < s_v_now - dv * 2.0f) vt = s_v_now - dv * 2.0f;
        s_v_now = vt;

        s_mi.v_ref = vt;

        /* 丢线也继续按航向+里程走完圆弧，不轻易判故障 */
        control_line(tangent, vt, (float)sg->dir / ARC_R_CM);

        if (remain <= 0.8f || (sg->stop_at_end && remain < 2.5f && v < 3.0f)) {
            seg_complete();
        }
        break;
    }
    case SEG_TURN: {
        float err = ang_diff(sg->heading_deg, yaw);
        if (fabsf(err) < 1.2f) {
            seg_complete();
        } else {
            control_turn(sg->heading_deg);
        }
        break;
    }
    case SEG_STOP:
    default:
        mission_finish(1);
        break;
    }
}

void mission_init(void)
{
    s_mi.state = ST_IDLE;
    s_mi.task = TASK_NONE;
    s_mi.lap = 1;
    s_mi.lap_total = 1;
    s_mi.limit_ms = 0;
    s_mi.fault_code = 0;
    s_start_req = 0;
    s_abort_req = 0;
    s_interlap = 0;
    control_init();
}

void mission_request_start(task_id_t t)
{
    if (s_mi.state == ST_RUN) return;
    s_start_task = t;
    s_start_req = 1;
}

void mission_request_abort(void)
{
    s_abort_req = 1;
}

void mission_tick(void)
{
    if (s_abort_req) {
        s_abort_req = 0;
        s_start_req = 0;
        s_interlap = 0;
        if (s_mi.state == ST_RUN) {
            mission_finish(0);
            s_mi.fault_code = 9;
        } else {
            control_idle();
            s_mi.state = ST_IDLE;
        }
        return;
    }

    if (s_start_req) {
        s_start_req = 0;
        mission_start(s_start_task);
        return;
    }

    switch (s_mi.state) {
    case ST_RUN:
        run_tick();
        break;
    case ST_DONE:
    case ST_FAULT:
        if (!s_hold_done && (millis() - s_end_ms) > 900u) {
            s_hold_done = 1;
            motor_stop();
        }
        if (!s_hold_done) control_brake();
        else              control_idle();
        break;
    default:
        if (!control_is_raw()) control_idle();
        break;
    }
}

const mission_info_t *mission_info(void)
{
    return &s_mi;
}
