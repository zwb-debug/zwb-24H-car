#include "bsp_ui.h"
#include "board.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"

static uint8_t  s_key_last = 0;     /* 上一次稳定电平（1=按下） */
static uint8_t  s_key_cnt[4];
static volatile uint8_t s_key_press = 0;

static uint16_t s_beep_ms = 0;      /* 剩余鸣叫时间（ms） */
static uint16_t s_beep_gap_ms = 0;  /* 两声之间的静音时间（ms） */
static uint8_t  s_beep_seq = 0;     /* 剩余鸣叫次数 */
static uint8_t  s_beep_phase = 0;   /* 方波相位，1ms 翻转一次 */

static uint16_t s_led1_flash = 0;
static uint16_t s_led2_flash = 0;

void ui_init(void)
{
    DL_GPIO_clearPins(UI_LED1_PORT, UI_LED1_PIN);
    DL_GPIO_clearPins(UI_LED2_PORT, UI_LED2_PIN);
    DL_GPIO_disableOutput(UI_BEEP_PORT, UI_BEEP_PIN);   /* 空闲高阻，避免常响 */
    s_key_last = 0;
    s_key_press = 0;
    for (int i = 0; i < 4; i++) s_key_cnt[i] = 0;
    s_beep_ms = s_beep_gap_ms = 0;
    s_beep_seq = 0;
    s_beep_phase = 0;
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

/* 蜂鸣器服务（1ms 调用）。
 * 空闲时 PB24 为高阻：低电平触发模块靠上拉、NPN 基极无电流，都不响；
 * 鸣叫时输出 500Hz 方波：高电平触发（NPN）与低电平触发模块都能发声。
 * 这样不用区分蜂鸣器极性，也不会出现上电常响。 */
void ui_tick_1ms(void)
{
    if (s_beep_ms > 0) {
        DL_GPIO_enableOutput(UI_BEEP_PORT, UI_BEEP_PIN);
        if (s_beep_phase) {
            DL_GPIO_setPins(UI_BEEP_PORT, UI_BEEP_PIN);
        } else {
            DL_GPIO_clearPins(UI_BEEP_PORT, UI_BEEP_PIN);
        }
        s_beep_phase ^= 1u;

        if (--s_beep_ms == 0) {
            DL_GPIO_disableOutput(UI_BEEP_PORT, UI_BEEP_PIN);
            if (s_beep_seq > 0) {
                s_beep_seq--;
                if (s_beep_seq > 0) {
                    s_beep_gap_ms = 110;      /* 两声之间 110ms */
                }
            }
        }
    } else if (s_beep_gap_ms > 0) {
        if (--s_beep_gap_ms == 0 && s_beep_seq > 0) {
            s_beep_ms = 35;                   /* 第二声 35ms */
        }
    }
}

void ui_beep(uint16_t ms)
{
    if (ms == 0) return;
    s_beep_seq = 0;
    s_beep_gap_ms = 0;
    s_beep_ms = ms;
    s_beep_phase = 0;
}

void ui_beep_ok(void)    { ui_beep(35); }

void ui_beep_node(void)
{
    s_beep_seq = 2;
    s_beep_ms = 35;
    s_beep_gap_ms = 0;
    s_beep_phase = 0;
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
