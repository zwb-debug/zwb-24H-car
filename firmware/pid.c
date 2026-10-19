#include "pid.h"
#include "mathx.h"

void pid_init(pid_t *p, float kp, float ki, float kd,
              float out_min, float out_max,
              float i_min, float i_max)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->out_min = out_min; p->out_max = out_max;
    p->i_min = i_min; p->i_max = i_max;
    p->d_alpha = 0.35f;
    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    p->integ = 0.0f;
    p->prev_x = 0.0f;
    p->d_state = 0.0f;
    p->started = 0;
}

void pid_set(pid_t *p, float kp, float ki, float kd)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integ = 0.0f;
    p->started = 0;
}

float pid_update(pid_t *p, float target, float meas, float dt)
{
    float err = target - meas;
    float d_raw;

    if (!p->started) {
        p->prev_x = meas;
        p->started = 1;
    }

    /* 微分先行：对测量值求导，避免目标阶跃导致冲击 */
    d_raw = -(meas - p->prev_x) / dt;
    p->prev_x = meas;
    p->d_state += p->d_alpha * (d_raw - p->d_state);

    p->integ += err * dt;
    p->integ = clampf(p->integ, p->i_min, p->i_max);

    float out = p->kp * err + p->ki * p->integ + p->kd * p->d_state;

    /* 条件积分抗饱和 */
    if (out > p->out_max) {
        if (err > 0.0f) p->integ -= err * dt;
        out = p->out_max;
    } else if (out < p->out_min) {
        if (err < 0.0f) p->integ -= err * dt;
        out = p->out_min;
    }
    return out;
}
