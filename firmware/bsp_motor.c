#include "bsp_motor.h"
#include "board.h"
#include "mathx.h"
#include "ti_msp_dl_config.h"

static uint8_t s_enabled = 0;

static void set_dir(uint8_t left, uint8_t fwd)
{//先判断是左轮还是右轮
    if (left) {
        if (MOTOR_L_INVERT) fwd = !fwd;
        if (fwd) {
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN2_PIN);
        } else {
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN1_PIN);
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN2_PIN);
        }//配置正转反转
    } else {
        if (MOTOR_R_INVERT) fwd = !fwd;
        if (fwd) {
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN2_PIN);
        } else {
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN1_PIN);
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN2_PIN);
        }
    }
}
//配置速度
static void set_pwm(uint8_t left, uint16_t duty)
{
    if (duty > (uint16_t)PWM_PERIOD) duty = (uint16_t)PWM_PERIOD;//判断最大PWM量
    /* 边沿对齐下计数：CC = 周期 -> 0% ，CC = 0 -> 100% */
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        (uint32_t)(PWM_PERIOD - (float)duty),
        left ? DL_TIMER_CC_0_INDEX : DL_TIMER_CC_1_INDEX);//比较值到达后电平反转 从低变高
}

void motor_init(void)
{
    DL_TimerG_startCounter(MOTOR_PWM_INST);//开启PWM方波
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_STBY_PIN);   /* STBY = 1 */
    motor_enable(1);
    motor_stop();
}

void motor_enable(uint8_t en)
{
    s_enabled = en;
    if (en) DL_GPIO_setPins(MOTOR_PORT, MOTOR_STBY_PIN);//开启STBY，开启待机引脚
    else    DL_GPIO_clearPins(MOTOR_PORT, MOTOR_STBY_PIN);
}

void motor_set(int16_t left, int16_t right)
{    //明确你的配速和方向
    if (!s_enabled) return;

    int16_t l = left, r = right;

    if (l >= 0) { set_dir(1, 1); set_pwm(1, (uint16_t)l); }
    else        { set_dir(1, 0); set_pwm(1, (uint16_t)(-l)); }

    if (r >= 0) { set_dir(0, 1); set_pwm(0, (uint16_t)r); }
    else        { set_dir(0, 0); set_pwm(0, (uint16_t)(-r)); }
}

void motor_stop(void)
{
    /* 滑行：IN1=IN2=0，PWM=0 */
    DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN2_PIN);
    DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN1_PIN);
    DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN2_PIN);
    set_pwm(1, 0);
    set_pwm(0, 0);
}

void motor_brake(void)
{
    /* TB6612 短路刹车：IN1=IN2=1，PWM 满占空 */
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN1_PIN);
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN2_PIN);
    set_pwm(1, (uint16_t)PWM_PERIOD);
    set_pwm(0, (uint16_t)PWM_PERIOD);
}
