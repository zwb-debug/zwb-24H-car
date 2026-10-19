/**
 * @file  bsp_encoder.h
 * @brief 双路 AB 相编码器（GPIO 双边沿中断，四倍频软件解码）
 */
#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

void    encoder_init(void);
void    encoder_reset(void);
int32_t encoder_count_l(void);
int32_t encoder_count_r(void);
/** 读取自上次调用以来的增量并换算出 cm/s（dt 单位 s） */
void    encoder_read_speed(float dt, float *vl, float *vr);

#endif
