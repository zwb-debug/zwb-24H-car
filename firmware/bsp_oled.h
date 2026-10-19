/**
 * @file  bsp_oled.h
 * @brief SSD1306 128x64 I2C OLED，带 1KB 显存
 */
#ifndef BSP_OLED_H
#define BSP_OLED_H

#include <stdint.h>

#define OLED_PAGES 8
#define OLED_COLS  128

void oled_init(void);
void oled_clear(void);
void oled_flush(void);                 /* 把显存刷到屏幕 */

void oled_text(uint8_t page, uint8_t col, const char *s);
void oled_textf(uint8_t page, uint8_t col, const char *fmt, ...);

void oled_bar(uint8_t page, uint8_t value);   /* 0..100 的进度条 */
void oled_pixel(uint8_t x, uint8_t y, uint8_t on);

/** 把浮点数格式化成 "%.1f" 风格，避免引入 printf 浮点支持 */
void oled_ftoa(char *out, float v, int decimals);

#endif
