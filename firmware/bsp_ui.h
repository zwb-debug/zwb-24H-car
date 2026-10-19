/**
 * @file  bsp_ui.h
 * @brief 按键、蜂鸣器、LED（全部非阻塞）
 */
#ifndef BSP_UI_H
#define BSP_UI_H

#include <stdint.h>

#define KEY1 0x01
#define KEY2 0x02
#define KEY3 0x04
#define KEY4 0x08

void    ui_init(void);
void    ui_tick_5ms(void);          /* 5ms 调一次：扫描按键、更新声光 */
uint8_t ui_key_pressed(void);       /* 返回并清除“新按下”事件位 */
uint8_t ui_key_down(void);

void    ui_beep(uint16_t ms);       /* 触发一次鸣叫（非阻塞） */
void    ui_beep_ok(void);           /* 短促“滴” */
void    ui_beep_node(void);         /* 经过节点：两短声 */
void    ui_beep_fail(void);         /* 长鸣 */
void    ui_led(uint8_t led1, uint8_t led2);
void    ui_led_flash(uint8_t which, uint16_t ms);

#endif
