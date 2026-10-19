#include "bsp_imu.h"
#include "board.h"
#include "mathx.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"

static uint8_t  s_buf[11];
static uint8_t  s_idx = 0;

static volatile float    s_yaw_raw = 0.0f;   /* 已乘符号，未去零 */
static volatile float    s_offset = 0.0f;
static volatile uint32_t s_last_ms = 0;
static volatile uint32_t s_frames = 0;
static volatile uint32_t s_bad = 0;

void imu_init(void)
{
    s_idx = 0;
    s_yaw_raw = 0.0f;
    s_offset = 0.0f;
    s_last_ms = 0;
    s_frames = 0;
    s_bad = 0;
    NVIC_ClearPendingIRQ(GYRO_UART_INST_INT_IRQN);
    NVIC_SetPriority(GYRO_UART_INST_INT_IRQN, 1);
    NVIC_EnableIRQ(GYRO_UART_INST_INT_IRQN);
}

void imu_zero(void)
{
    s_offset = s_yaw_raw;
}

float imu_yaw(void)
{
    return wrap180(s_yaw_raw - s_offset);
}

float imu_yaw_raw(void)
{
    return s_yaw_raw;
}

uint8_t imu_ok(void)
{
    return (s_frames > 0) && ((millis() - s_last_ms) < IMU_TIMEOUT_MS);
}

uint32_t imu_last_ms(void) { return s_last_ms; }
uint32_t imu_frames(void)  { return s_frames; }

/*
 * JY61P 角度帧（11 字节）：
 *  0x55 0x53 RollL RollH PitchL PitchH YawL YawH VL VH SUM
 *  SUM = 前 10 字节之和的低 8 位
 */
void GYRO_UART_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(GYRO_UART_INST)) {
    case DL_UART_IIDX_RX: {
        uint8_t b = DL_UART_Main_receiveData(GYRO_UART_INST);

        if (s_idx == 0) {                /* 等帧头 */
            if (b == 0x55) {
                s_buf[0] = 0x55;
                s_idx = 1;
            }
            break;
        }

        s_buf[s_idx++] = b;
        if (s_idx >= 11) {
            uint8_t sum = 0;
            for (int i = 0; i < 10; i++) sum += s_buf[i];

            if (sum == s_buf[10] && s_buf[1] == 0x53) {
                int16_t yaw = (int16_t)((uint16_t)s_buf[7] << 8 | s_buf[6]);
                s_yaw_raw = IMU_YAW_SIGN * ((float)yaw * 180.0f / 32768.0f);
                s_last_ms = millis();
                s_frames++;
            } else {
                s_bad++;
                /* 校验失败时若末字节是 0x55，可能是新帧头 */
                if (s_buf[10] == 0x55) {
                    s_buf[0] = 0x55;
                    s_idx = 1;
                    break;
                }
            }
            s_idx = 0;
        }
        break;
    }
    default:
        break;
    }
}
