#include "bsp_ui.h"
#include "board.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"

static uint8_t  s_key_last = 0;     /* 上一次稳定电平（1=按下） */
static uint8_t  s_key_cnt[4];
static volatile uint8_t s_key_press = 0;

static uint16_t s_beep_left = 0;
static uint16_t s_beep_gap = 0;
static uint8_t  s_beep_seq = 0;     /* 剩余鸣叫次数 */

static uint16_t s_led1_flash = 0;
static uint16_t s_led2_flash = 0;

void ui_init(void)
{
    DL_GPIO_clearPins(UI_LED1_PORT, UI_LED1_PIN);
    DL_GPIO_clearPins(UI_LED2_PORT, UI_LED2_PIN);
    DL_GPIO_clearPins(UI_BEEP_PORT, UI_BEEP_PIN);
    s_key_last = 0;
    s_key_press = 0;
    for (int i = 0; i < 4; i++) s_key_cnt[i] = 0;
    s_beep_left = s_beep_gap = 0;
    s_beep_seq = 0;
    s_led1_flash = s_led2_flash = 0;
}

static uint8_t read_keys(void)
{
    uint32_t v = DL_GPIO_readPins(KEY_PORT,
        KEY_K1_PIN | KEY_K2_PIN | KEY_K3_PIN | KEY_K4_PIN);
    uint8_t k = 0;
    if (!(v & KEY_K1_PIN)) k |= KEY1;   /* 上拉输入，按下为低 */
    if (!(v & KEY_K2_PIN)) k |= KEY2;
    if (!(v & KEY_K3_PIN)) k |= KEY3;
    if (!(v & KEY_K4_PIN)) k |= KEY4;
    return k;
}

void ui_tick_5ms(void)
{
    /* ---- 按键消抖（4 次 5ms 一致才确认） ---- */
    uint8_t now = read_keys();
    uint8_t changed = 0;

    for (int i = 0; i < 4; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        uint8_t pressed = (now & bit) ? 1 : 0;
        uint8_t stable  = (s_key_last & bit) ? 1 : 0;

        if (pressed != stable) {
            if (++s_key_cnt[i] >= 4) {
                s_key_cnt[i] = 0;
                if (pressed) changed |= bit;
                if (pressed) s_key_last |= bit;
                else         s_key_last &= (uint8_t)~bit;
            }
        } else {
            s_key_cnt[i] = 0;
        }
    }
    s_key_press |= changed;

    /* ---- 蜂鸣器 ---- */
    if (s_beep_left > 0) {
        s_beep_left--;
        DL_GPIO_setPins(UI_BEEP_PORT, UI_BEEP_PIN);
        if (s_beep_left == 0) {
            DL_GPIO_clearPins(UI_BEEP_PORT, UI_BEEP_PIN);
            if (s_beep_seq > 0) {
                s_beep_seq--;
                if (s_beep_seq > 0) {
                    s_beep_gap = 22;      /* 两声之间 110ms */
                }
            }
        }
    } else if (s_beep_gap > 0) {
        s_beep_gap--;
        if (s_beep_gap == 0 && s_beep_seq > 0) {
            s_beep_left = 8;              /* 第二声 40ms */
        }
    }

    /* ---- LED 闪烁（节点提示） ---- */
    if (s_led1_flash > 0) {
        if (--s_led1_flash == 0) DL_GPIO_clearPins(UI_LED1_PORT, UI_LED1_PIN);
    }
    if (s_led2_flash > 0) {
        if (--s_led2_flash == 0) DL_GPIO_clearPins(UI_LED2_PORT, UI_LED2_PIN);
    }
}

uint8_t ui_key_pressed(void)
{
    __disable_irq();
    uint8_t k = s_key_press;
    s_key_press = 0;
    __enable_irq();
    return k;
}

uint8_t ui_key_down(void)
{
    return s_key_last;
}

void ui_beep(uint16_t ms)
{
    s_beep_seq = 0;
    s_beep_gap = 0;
    s_beep_left = ms / 5;
}

void ui_beep_ok(void)    { ui_beep(35); }

void ui_beep_node(void)
{
    s_beep_seq = 2;
    s_beep_left = 7;      /* 35ms */
    s_beep_gap = 0;
}

void ui_beep_fail(void)  { ui_beep(600); }

void ui_led(uint8_t led1, uint8_t led2)
{
    if (led1) DL_GPIO_setPins(UI_LED1_PORT, UI_LED1_PIN);
    else      DL_GPIO_clearPins(UI_LED1_PORT, UI_LED1_PIN);

    if (led2) DL_GPIO_setPins(UI_LED2_PORT, UI_LED2_PIN);
    else      DL_GPIO_clearPins(UI_LED2_PORT, UI_LED2_PIN);
}

void ui_led_flash(uint8_t which, uint16_t ms)
{
    uint16_t ticks = (uint16_t)(ms / 5u);
    if (ticks == 0) ticks = 1;
    if (which == 1) {
        s_led1_flash = ticks;
        DL_GPIO_setPins(UI_LED1_PORT, UI_LED1_PIN);
    } else {
        s_led2_flash = ticks;
        DL_GPIO_setPins(UI_LED2_PORT, UI_LED2_PIN);
    }
}
