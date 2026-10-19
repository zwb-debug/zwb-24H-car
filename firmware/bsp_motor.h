/**
 * @file  bsp_motor.h
 * @brief TB6612 双电机驱动（TIMG0 两路 20kHz PWM + 方向 GPIO）
 */
#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

void motor_init(void);
/** duty 范围 -PWM_PERIOD..+PWM_PERIOD；负值仅用于调试，正式比赛禁止倒车 */
void motor_set(int16_t left, int16_t right);
void motor_stop(void);     /* 滑行 */
void motor_brake(void);    /* 短路刹车 */
void motor_enable(uint8_t en);

#endif
