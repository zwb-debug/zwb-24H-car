/**
 * @file  timebase.h
 * @brief 1ms 系统时基（SysTick 中断里维护）
 */
#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

extern volatile uint32_t g_ms;      /* 上电后毫秒数，1ms 递增 */
extern volatile uint32_t g_ctrl_tick;/* 每 5ms +1（200Hz 控制节拍） */

void     timebase_init(void);
uint32_t millis(void);
void     delay_ms(uint32_t ms);

#endif
