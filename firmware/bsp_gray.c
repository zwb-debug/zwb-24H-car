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
    uint32_t v = DL_GPIO_readPins(GRAY_PORT,
        GRAY_CH0_PIN | GRAY_CH1_PIN | GRAY_CH2_PIN | GRAY_CH3_PIN |
        GRAY_CH4_PIN | GRAY_CH5_PIN | GRAY_CH6_PIN | GRAY_CH7_PIN);

    uint8_t raw = 0;
    if (v & GRAY_CH0_PIN) raw |= 0x01;
    if (v & GRAY_CH1_PIN) raw |= 0x02;
    if (v & GRAY_CH2_PIN) raw |= 0x04;
    if (v & GRAY_CH3_PIN) raw |= 0x08;
    if (v & GRAY_CH4_PIN) raw |= 0x10;
    if (v & GRAY_CH5_PIN) raw |= 0x20;
    if (v & GRAY_CH6_PIN) raw |= 0x40;
    if (v & GRAY_CH7_PIN) raw |= 0x80;

#if !GRAY_ACTIVE_HIGH
    raw = (uint8_t)~raw;
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
    s_g.on_line = (cnt > 0) ? 1 : 0;
    if (cnt > 0) {
        float mean = sum / (float)cnt;
        s_g.pos_mm = (mean - 3.5f) * GRAY_SPACING_MM;
    }
    /* 丢线时保持上一次位置，由控制层判断超时 */
    return &s_g;
}

uint8_t gray_raw(void)
{
    return s_g.raw;
}
