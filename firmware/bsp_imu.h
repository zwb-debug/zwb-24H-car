/**
 * @file  bsp_imu.h
 * @brief MPU6050 I2C 陀螺仪，Z 轴角速度积分得到航向
 */
#ifndef BSP_IMU_H
#define BSP_IMU_H

#include <stdint.h>

void   imu_init(void);
void   imu_tick(float dt);       /* 200Hz 调用；读取、校零并积分航向 */
void   imu_zero(void);          /* 把当前航向记为 0° */
float  imu_yaw(void);           /* 相对零点的航向，度，左正右负 */
float  imu_yaw_raw(void);       /* 未去零的原始航向 */
uint8_t imu_ok(void);           /* 1=最近 200ms 收到有效帧 */
uint32_t imu_last_ms(void);
uint32_t imu_frames(void);

#endif
