#include "bsp_imu.h"
#include "board.h"
#include "mathx.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"

#define MPU6050_REG_SMPLRT_DIV   0x19u
#define MPU6050_REG_CONFIG       0x1Au
#define MPU6050_REG_GYRO_CONFIG  0x1Bu
#define MPU6050_REG_GYRO_ZOUT_H  0x47u
#define MPU6050_REG_PWR_MGMT_1   0x6Bu
#define MPU6050_REG_WHO_AM_I     0x75u

#define MPU6050_WHO_AM_I_VALUE   0x68u
#define MPU6050_GYRO_LSB_DPS     65.5f   /* FS_SEL=1: +/-500 dps */
#define I2C_GUARD                100000u

static volatile float    s_yaw_raw = 0.0f;
static volatile float    s_offset = 0.0f;
static volatile uint32_t s_last_ms = 0;
static volatile uint32_t s_frames = 0;
static volatile uint32_t s_bad = 0;

static float    s_bias_sum = 0.0f;
static float    s_bias_dps = 0.0f;
static uint16_t s_cal_count = 0;
static uint8_t  s_present = 0;
static uint8_t  s_calibrated = 0;

static uint8_t i2c_wait_done(void)
{
    uint32_t guard = I2C_GUARD;
    delay_cycles(240); /* MSPM0 I2C_ERR_13 */
    while ((DL_I2C_getControllerStatus(MPU_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY) && --guard) {
    }
    return (guard != 0u) &&
           !(DL_I2C_getControllerStatus(MPU_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_ERROR);
}

static uint8_t i2c_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    uint32_t guard = I2C_GUARD;

    while (!(DL_I2C_getControllerStatus(MPU_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE) && --guard) {
    }
    if (guard == 0u) return 0;

    DL_I2C_flushControllerTXFIFO(MPU_I2C_INST);
    DL_I2C_fillControllerTXFIFO(MPU_I2C_INST, data, 2);
    DL_I2C_startControllerTransfer(MPU_I2C_INST, MPU6050_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    return i2c_wait_done();
}

static uint8_t i2c_read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint32_t guard = I2C_GUARD;

    while (!(DL_I2C_getControllerStatus(MPU_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE) && --guard) {
    }
    if (guard == 0u) return 0;

    DL_I2C_flushControllerTXFIFO(MPU_I2C_INST);
    DL_I2C_fillControllerTXFIFO(MPU_I2C_INST, &reg, 1);
    DL_I2C_startControllerTransfer(MPU_I2C_INST, MPU6050_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    if (!i2c_wait_done()) return 0;

    DL_I2C_flushControllerRXFIFO(MPU_I2C_INST);
    DL_I2C_startControllerTransfer(MPU_I2C_INST, MPU6050_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    if (!i2c_wait_done()) return 0;

    for (uint8_t i = 0; i < len; i++) {
        guard = I2C_GUARD;
        while (DL_I2C_isControllerRXFIFOEmpty(MPU_I2C_INST) && --guard) {
        }
        if (guard == 0u) return 0;
        data[i] = DL_I2C_receiveControllerData(MPU_I2C_INST);
    }
    return 1;
}

void imu_init(void)
{
    uint8_t who = 0;

    s_yaw_raw = 0.0f;
    s_offset = 0.0f;
    s_last_ms = 0;
    s_frames = 0;
    s_bad = 0;
    s_bias_sum = 0.0f;
    s_bias_dps = 0.0f;
    s_cal_count = 0;
    s_present = 0;
    s_calibrated = 0;

    if (!i2c_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01u)) return;
    delay_cycles(320000); /* 唤醒后等待约 10ms */
    if (!i2c_write_reg(MPU6050_REG_SMPLRT_DIV, 0x04u)) return; /* 200Hz */
    if (!i2c_write_reg(MPU6050_REG_CONFIG, 0x03u)) return;      /* 44Hz DLPF */
    if (!i2c_write_reg(MPU6050_REG_GYRO_CONFIG, 0x08u)) return; /* +/-500dps */
    if (!i2c_read_regs(MPU6050_REG_WHO_AM_I, &who, 1)) return;

    s_present = ((who & 0x7Eu) == MPU6050_WHO_AM_I_VALUE);
}

void imu_tick(float dt)
{
    uint8_t data[2];
    if (!s_present || !i2c_read_regs(MPU6050_REG_GYRO_ZOUT_H, data, 2)) {
        s_bad++;
        return;
    }

    int16_t raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    float rate_dps = (float)raw / MPU6050_GYRO_LSB_DPS;
    s_frames++;
    s_last_ms = millis();

    if (!s_calibrated) {
        s_bias_sum += rate_dps;
        if (++s_cal_count >= MPU6050_CAL_SAMPLES) {
            s_bias_dps = s_bias_sum / (float)s_cal_count;
            s_calibrated = 1;
            s_yaw_raw = 0.0f;
            s_offset = 0.0f;
        }
        return;
    }

    rate_dps -= s_bias_dps;
    s_yaw_raw = wrap180(s_yaw_raw + IMU_YAW_SIGN * rate_dps * dt);
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
    return s_present && s_calibrated &&
           ((millis() - s_last_ms) < IMU_TIMEOUT_MS);
}

uint32_t imu_last_ms(void) { return s_last_ms; }
uint32_t imu_frames(void)  { return s_frames; }
