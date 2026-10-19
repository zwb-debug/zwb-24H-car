#include "timebase.h"
#include "ti_msp_dl_config.h"

volatile uint32_t g_ms = 0;
volatile uint32_t g_ctrl_tick = 0;

void timebase_init(void)
{
    /* 32MHz 内核，1ms = 32000 个 SysTick 时钟 */
    SysTick_Config(CPUCLK_FREQ / 1000u);
    NVIC_SetPriority(SysTick_IRQn, 3);
}

uint32_t millis(void)
{
    return g_ms;
}

void delay_ms(uint32_t ms)
{
    uint32_t t0 = g_ms;
    while ((g_ms - t0) < ms) {
        __WFI();
    }
}
