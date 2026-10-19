/**
 * @file  bsp_imu.h
 * @brief 维特 JY61P / WT61 串口陀螺仪（0x55 0x53 角度帧，100~200Hz）
 */
#ifndef BSP_IMU_H
#define BSP_IMU_H

#include <stdint.h>

void   imu_init(void);
void   imu_zero(void);          /* 把当前航向记为 0° */
float  imu_yaw(void);           /* 相对零点的航向，度，左正右负 */
float  imu_yaw_raw(void);       /* 未去零的原始航向 */
uint8_t imu_ok(void);           /* 1=最近 200ms 收到有效帧 */
uint32_t imu_last_ms(void);
uint32_t imu_frames(void);

#endif
