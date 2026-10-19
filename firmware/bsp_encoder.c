#include "bsp_encoder.h"
#include "board.h"
#include "ti_msp_dl_config.h"

/* 正转计数方向表：idx = (prev<<2)|now, state = A<<1|B */
static const int8_t qtab[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static volatile int32_t s_cnt_l = 0;
static volatile int32_t s_cnt_r = 0;
static volatile uint8_t s_state_l = 0;
static volatile uint8_t s_state_r = 0;

static int32_t s_prev_l = 0;
static int32_t s_prev_r = 0;

#define ENC_L_MASK (ENC_L_A_PIN | ENC_L_B_PIN)
#define ENC_R_MASK (ENC_R_A_PIN | ENC_R_B_PIN)

void encoder_init(void)
{
    uint32_t v = DL_GPIO_readPins(ENC_PORT, ENC_L_MASK | ENC_R_MASK);

    s_state_l = (uint8_t)(((v & ENC_L_A_PIN) ? 2 : 0) |
                          ((v & ENC_L_B_PIN) ? 1 : 0));
    s_state_r = (uint8_t)(((v & ENC_R_A_PIN) ? 2 : 0) |
                          ((v & ENC_R_B_PIN) ? 1 : 0));

    s_cnt_l = s_cnt_r = 0;
    s_prev_l = s_prev_r = 0;

    NVIC_ClearPendingIRQ(ENC_INT_IRQN);
    NVIC_SetPriority(ENC_INT_IRQN, 0);   /* 编码器最高优先级 */
    NVIC_EnableIRQ(ENC_INT_IRQN);
}

void encoder_reset(void)
{
    s_cnt_l = s_cnt_r = 0;
    s_prev_l = s_prev_r = 0;
}

int32_t encoder_count_l(void) { return s_cnt_l; }
int32_t encoder_count_r(void) { return s_cnt_r; }

void encoder_read_speed(float dt, float *vl, float *vr)
{
    int32_t cl = s_cnt_l;
    int32_t cr = s_cnt_r;

    int32_t dl = cl - s_prev_l;
    int32_t dr = cr - s_prev_r;
    s_prev_l = cl;
    s_prev_r = cr;

    *vl = ((float)dl * (float)ENC_L_SIGN) / COUNTS_PER_CM / dt;
    *vr = ((float)dr * (float)ENC_R_SIGN) / COUNTS_PER_CM / dt;
}

/* GPIOA 中断：四个编码器引脚共用 */
void GPIOA_IRQHandler(void)
{
    uint32_t stat = DL_GPIO_getEnabledInterruptStatus(
        ENC_PORT, ENC_L_MASK | ENC_R_MASK);
    if (stat == 0) return;

    uint32_t v = DL_GPIO_readPins(ENC_PORT, ENC_L_MASK | ENC_R_MASK);

    uint8_t sl = (uint8_t)(((v & ENC_L_A_PIN) ? 2 : 0) |
                           ((v & ENC_L_B_PIN) ? 1 : 0));
    uint8_t sr = (uint8_t)(((v & ENC_R_A_PIN) ? 2 : 0) |
                           ((v & ENC_R_B_PIN) ? 1 : 0));

    s_cnt_l += qtab[(s_state_l << 2) | sl];
    s_state_l = sl;
    s_cnt_r += qtab[(s_state_r << 2) | sr];
    s_state_r = sr;

    DL_GPIO_clearInterruptStatus(ENC_PORT, stat);
}
