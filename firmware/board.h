/**
 * @file    board.h
 * @brief   2024 电赛 H 题（自动行驶小车）全局配置
 *
 * 目标平台：立创·地猛星 MSPM0G3507（LQFP-64，32MHz 内部时钟）
 *
 * 本文件集中放置“需要按实车标定”的参数。所有长度单位 cm，角度单位度，
 * 速度单位 cm/s，时间单位 s（除非特别说明）。
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ===================== 场地几何（依 2024 H 题题图） ===================== */
#define FIELD_W_CM          220.0f   /* 场地宽 220cm                       */
#define FIELD_H_CM          120.0f   /* 场地高 120cm                       */
#define ARC_R_CM            40.0f    /* 半圆弧半径 40cm                    */
#define LINE_W_CM           1.8f     /* 黑线线宽 1.8cm                     */
/* 顶点坐标（左下角为原点，x 向右，y 向上）：
 *   A(60,100)  B(160,100)
 *   D(60, 20)  C(160, 20)
 * 左半圆 A-D 向左凸，右半圆 B-C 向右凸，场地内没有其它标记。 */
#define PT_AX               60.0f
#define PT_AY               100.0f
#define PT_BX               160.0f
#define PT_BY               100.0f
#define PT_CX               160.0f
#define PT_CY               20.0f
#define PT_DX               60.0f
#define PT_DY               20.0f

#define LEN_AB_CM           100.0f   /* A->B / C->D 直线段               */
#define LEN_DIAG_CM         128.062f /* A->C = sqrt(100^2+80^2)          */
#define LEN_ARC_CM          (3.14159265f * ARC_R_CM) /* 半圆 125.66cm      */

#define H_AB_DEG            0.0f
#define H_CD_DEG            180.0f
#define H_AC_DEG            (-38.6598f)  /* atan2(-80, 100)             */
#define H_BD_DEG            (218.6598f)  /* atan2(-80,-100) = -141.34   */

/* ===================== 小车机械参数（必须实测标定） ===================== */
#define CAR_W_CM            12.0f    /* 左右轮中心距（轮距）               */
#define WHEEL_D_CM          6.5f     /* 轮径（N20 65mm 轮）                */
#define GEAR_RATIO          30.0f    /* 减速比 1:30                        */
#define ENC_PPR             13.0f    /* 电机轴霍尔编码器每转脉冲数          */
#define ENC_MULT            4.0f     /* 四倍频                             */
/* 每 cm 行驶对应的编码器计数 = PPR*4*减速比 / (pi*D) */
#define ENC_CPR             (ENC_PPR * ENC_MULT * GEAR_RATIO)
#define COUNTS_PER_CM       (ENC_CPR / (3.14159265f * WHEEL_D_CM))

/* 左右编码器/电机安装方向：若推车前进时计数为负，把对应符号改成 -1 */
#define ENC_L_SIGN          (+1)
#define ENC_R_SIGN          (-1)
#define MOTOR_L_INVERT      (0)
#define MOTOR_R_INVERT      (0)

/* 灰度传感器：8 路，间距 15mm，探头在车头，位于两轮轴前方 SENSOR_AHEAD_CM */
#define GRAY_NUM            8
#define GRAY_SPACING_MM     15.0f
#define GRAY_ACTIVE_HIGH    1       /* 压黑线输出高电平=1；相反则改 0     */
#define SENSOR_AHEAD_CM     8.0f    /* 探头到两轮轴中点的纵向距离          */

/* ===================== 运行参数 ===================== */
#define CTRL_HZ             200.0f
#define CTRL_DT             (1.0f / CTRL_HZ)

#define V_MAX_CM_S          85.0f
#define ACC_LIMIT_CM_S2     160.0f   /* 加速度限幅                         */
#define DEC_LIMIT_CM_S2     180.0f   /* 减速度限幅                         */

/* 各题巡航速度（按实车抓地力调整） */
#define V_TASK1             32.0f
#define V_TASK2             36.0f
#define V_TASK3             36.0f
#define V_TASK4             58.0f
#define V_ARC_SCALE         0.86f    /* 圆弧段降速系数                      */
#define V_TURN_MAX          26.0f
#define V_TURN_MIN          9.0f

/* 时间保护（题目规定时间的一倍；超时自动停车，防止跑飞） */
#define T_LIMIT1_MS         30000u
#define T_LIMIT2_MS         60000u
#define T_LIMIT3_MS         80000u
#define T_LIMIT4_MS         120000u

/* ===================== 传感器/执行器标定 ===================== */
#define IMU_YAW_SIGN        (+1.0f)  /* 车左转 yaw 增大取 +1，否则 -1      */
#define IMU_BAUD            115200u
#define IMU_TIMEOUT_MS      200u     /* 超过该时间没收到角度帧 → 报错       */

#define PWM_PERIOD          1600.0f  /* TIMG0: 32MHz/1600 = 20kHz          */
#define DUTY_MAX            1600.0f
#define DUTY_MIN_MOVE       95.0f    /* 克服静摩擦的最小占空比              */
#define DUTY_PER_CM_S       14.0f    /* 前馈：每 cm/s 对应占空比            */

/* ===================== 控制参数默认值（可串口在线改） ===================== */
typedef struct {
    float sp_kp, sp_ki;          /* 轮速 PI（占空比 / (cm/s)）           */
    float hd_kp, hd_ki, hd_kd;   /* 直线航向 PID（rad/s per deg）        */
    float ln_kp, ln_ki, ln_kd;   /* 弧线循迹 PID（rad/s per mm）         */
    float turn_kp;               /* 转向 P（cm/s per deg）               */
} ctrl_gains_t;

#define GAIN_SP_KP_DEF      3.2f
#define GAIN_SP_KI_DEF      18.0f
#define GAIN_HD_KP_DEF      0.020f
#define GAIN_HD_KI_DEF      0.0015f
#define GAIN_HD_KD_DEF      0.0025f
#define GAIN_LN_KP_DEF      0.010f
#define GAIN_LN_KI_DEF      0.00002f
#define GAIN_LN_KD_DEF      0.00060f
#define GAIN_TURN_KP_DEF    1.10f

/* ===================== 任务/状态枚举 ===================== */
typedef enum {
    TASK_NONE = 0,
    TASK_1 = 1,
    TASK_2 = 2,
    TASK_3 = 3,
    TASK_4 = 4
} task_id_t;

typedef enum {
    ST_IDLE = 0,
    ST_ARMED,        /* 已选任务，等待发车            */
    ST_RUN,          /* 正在跑                        */
    ST_DONE,         /* 正常完成                      */
    ST_FAULT,        /* 故障停车（丢线/超时/陀螺仪）   */
    ST_CALIB
} run_state_t;

typedef enum {
    SEG_LINE = 0,
    SEG_TURN,
    SEG_ARC,
    SEG_STOP
} seg_type_t;

typedef struct {
    uint8_t  type;        /* seg_type_t                              */
    float    len_cm;      /* LINE/ARC 的长度；STOP 忽略              */
    float    heading_deg; /* LINE: 目标绝对航向；TURN: 目标绝对航向   */
    int8_t   dir;         /* ARC: +1 逆时针(左转) / -1 顺时针(右转)   */
    float    speed;       /* 该段巡航速度 cm/s                       */
    uint8_t  event;       /* 到达该段终点时的声光提示编号，0=不提示   */
    uint8_t  stop_at_end; /* 1=段末精确停车（仅最后一段）             */
} segment_t;

#endif /* BOARD_H */
