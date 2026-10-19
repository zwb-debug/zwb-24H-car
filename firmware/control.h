/**
 * @file  control.h
 * @brief 底层运动控制：轮速闭环 + 航向/循迹外环
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "board.h"

typedef enum {
    CM_IDLE = 0,
    CM_STRAIGHT,     /* 陀螺仪航向保持 + 定速        */
    CM_LINE,         /* 灰度循迹 + 航向阻尼（圆弧）   */
    CM_TURN,         /* 原地差速转向（内轮停转，不倒车）*/
    CM_BRAKE
} ctrl_mode_t;

typedef struct {
    float yaw;        /* 场地航向，度                 */
    float vL, vR;     /* 实测轮速 cm/s                */
    float vrefL, vrefR;
    float line_mm;
    uint8_t line_on;
    uint8_t line_cnt;
    int16_t dutyL, dutyR;
    ctrl_mode_t mode;
} ctrl_state_t;

void control_init(void);
const ctrl_state_t *control_state(void);
ctrl_gains_t *control_gains(void);
/** 设置“场地航向 = 陀螺仪相对航向 + offset”，发车前调用 */
void control_set_yaw_offset(float off);

void control_straight(float heading_deg, float v_cm_s);
/** curvature: 期望曲率 1/cm，左转为正（圆弧 = dir/R，直线 = 0） */
void control_line(float heading_deg, float v_cm_s, float curvature);
void control_turn(float target_heading_deg);
void control_brake(void);
void control_idle(void);

/** 200Hz 调用：采样 + 外环 + 轮速闭环 + 输出 PWM */
void control_tick(float dt);

/** 轮速开环测试（duty -1600..1600），仅调试用 */
void control_raw(int16_t dutyL, int16_t dutyR);
uint8_t control_is_raw(void);

#endif
