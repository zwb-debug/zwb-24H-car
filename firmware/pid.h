/**
 * @file  pid.h
 * @brief 带积分限幅/输出限幅的通用 PID（位置式，微分先行）
 */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float integ;
    float prev_x;
    float d_state;
    float out_min, out_max;
    float i_min, i_max;
    float d_alpha;     /* 微分低通系数 0..1，1=不滤波 */
    int   started;
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd,
               float out_min, float out_max,
               float i_min, float i_max);
void  pid_reset(pid_t *p);
void  pid_set(pid_t *p, float kp, float ki, float kd);
float pid_update(pid_t *p, float target, float meas, float dt);

#endif /* PID_H */
