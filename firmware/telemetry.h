/**
 * @file  telemetry.h
 * @brief 调试串口（UART0/CH340）：50Hz CSV 遥测 + 在线调参命令
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

void telemetry_init(void);
void telemetry_poll(void);       /* 主循环 20ms 调一次 */
void telemetry_send(const char *s);

#endif
