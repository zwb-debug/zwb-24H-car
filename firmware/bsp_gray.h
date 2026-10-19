/**
 * @file  bsp_gray.h
 * @brief 8 路数字灰度循迹传感器
 */
#ifndef BSP_GRAY_H
#define BSP_GRAY_H

#include <stdint.h>

typedef struct {
    uint8_t raw;        /* bit0..bit7 对应 CH0..CH7（已按有效电平归一） */
    uint8_t count;      /* 压线通道数                                  */
    uint8_t on_line;    /* 1=至少一路压线                              */
    float   pos_mm;     /* 线相对车体中心的位置，右正左负              */
} gray_t;

void        gray_init(void);
const gray_t *gray_read(void);
uint8_t     gray_raw(void);

#endif
