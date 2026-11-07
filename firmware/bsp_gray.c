#include "bsp_gray.h"
#include "board.h"
#include "mathx.h"
#include "ti_msp_dl_config.h"

static gray_t s_g;

void gray_init(void)
{
    s_g.raw = 0;
    s_g.count = 0;
    s_g.on_line = 0;
    s_g.pos_mm = 0.0f;
}

const gray_t *gray_read(void)
{
    uint8_t raw = 0;
    if (DL_GPIO_readPins(GRAY_CH0_PORT, GRAY_CH0_PIN)) raw |= 0x01;
    if (DL_GPIO_readPins(GRAY_CH1_PORT, GRAY_CH1_PIN)) raw |= 0x02;
    if (DL_GPIO_readPins(GRAY_CH2_PORT, GRAY_CH2_PIN)) raw |= 0x04;
    if (DL_GPIO_readPins(GRAY_CH3_PORT, GRAY_CH3_PIN)) raw |= 0x08;
    if (DL_GPIO_readPins(GRAY_CH4_PORT, GRAY_CH4_PIN)) raw |= 0x10;
    if (DL_GPIO_readPins(GRAY_CH5_PORT, GRAY_CH5_PIN)) raw |= 0x20;
    if (DL_GPIO_readPins(GRAY_CH6_PORT, GRAY_CH6_PIN)) raw |= 0x40;
    if (DL_GPIO_readPins(GRAY_CH7_PORT, GRAY_CH7_PIN)) raw |= 0x80;

#if !GRAY_ACTIVE_HIGH
    raw = (uint8_t)~raw;//判断8路传感器如果是低电平在赛道上还是高电平在赛道上
#endif

    s_g.raw = raw;

    uint8_t cnt = 0;
    float   sum = 0.0f;
    for (int i = 0; i < GRAY_NUM; i++) {
        if (raw & (1u << i)) {
            cnt++;
            sum += (float)i;
        }
    }
    s_g.count = cnt;
    s_g.on_line = (cnt > 0) ? 1 : 0;//判断是否正常在线上，如果在线上则cnt=0
    if (cnt > 0) {
        float mean = sum / (float)cnt;
        s_g.pos_mm = (mean - 3.5f) * GRAY_SPACING_MM;//3.5f为线性平均值，用它来确定离标准线差距多少
    }
    /* 丢线时保持上一次位置，由控制层判断超时 */
    return &s_g;
}

uint8_t gray_raw(void)
{
    return s_g.raw;
}
