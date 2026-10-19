/**
 * @file  main.c
 * @brief 2024 电赛 H 题 —— 自动行驶小车（立创·地猛星 MSPM0G3507）
 *
 * 任务分配：
 *   K1 选择题目 1->2->3->4
 *   K2 发车（开始跑当前题目）
 *   K3 急停
 *   K4 切换显示页
 *
 * 控制节拍：SysTick 1ms；每 5ms 执行一次 UI/任务/控制（200Hz）。
 * 主循环：OLED 刷新 + 串口遥测/调参。
 */
#include "ti_msp_dl_config.h"

#include "board.h"
#include "bsp_encoder.h"
#include "bsp_gray.h"
#include "bsp_imu.h"
#include "bsp_motor.h"
#include "bsp_oled.h"
#include "bsp_ui.h"
#include "control.h"
#include "mathx.h"
#include "mission.h"
#include "telemetry.h"
#include "timebase.h"

static task_id_t s_sel = TASK_1;
static uint8_t   s_page = 0;

/* ------------------------------------------------------------------ */
/* 1ms 时基 + 200Hz 控制                                                */
/* ------------------------------------------------------------------ */
void SysTick_Handler(void)
{
    static uint8_t div = 0;

    g_ms++;
    if (++div >= 5) {
        div = 0;
        g_ctrl_tick++;
        ui_tick_5ms();
        mission_tick();
        control_tick(CTRL_DT);
    }
}

/* ------------------------------------------------------------------ */
/* 显示                                                                */
/* ------------------------------------------------------------------ */
static void show_f(uint8_t page, uint8_t col, const char *label,
                   float v, int dec, const char *unit)
{
    char num[16];
    oled_ftoa(num, v, dec);
    oled_textf(page, col, "%s%s%s", label, num, unit);
}

static void screen_main(void)
{
    const mission_info_t *m = mission_info();
    const ctrl_state_t *st = control_state();

    oled_clear();
    oled_textf(0, 0, "2024H CAR  T:%u", (unsigned)s_sel);

    const char *ss = "IDLE";
    switch (m->state) {
    case ST_RUN:   ss = "RUN "; break;
    case ST_DONE:  ss = "DONE"; break;
    case ST_FAULT: ss = "FAIL"; break;
    default: break;
    }
    oled_textf(1, 0, "ST:%s LAP:%u/%u", ss, (unsigned)m->lap,
               (unsigned)m->lap_total);

    show_f(2, 0, "S:", m->s_total, 1, "cm");
    show_f(3, 0, "YAW:", st->yaw, 1, "");
    oled_textf(4, 0, "E:%dmm #%u %s", (int)st->line_mm,
               (unsigned)st->line_cnt, st->line_on ? "LINE" : "----");

    oled_textf(5, 0, "V:%d,%d ref %d", (int)st->vL, (int)st->vR,
               (int)m->v_ref);
    oled_textf(6, 0, "D:%d,%d", (int)st->dutyL, (int)st->dutyR);

    show_f(7, 0, "T:", (float)m->elapsed_ms / 1000.0f, 1, "s");
}

static void screen_gains(void)
{
    ctrl_gains_t *g = control_gains();
    char a[12], b[12];

    oled_clear();
    oled_text(0, 0, "-- TUNING --");

    oled_ftoa(a, g->hd_kp, 3); oled_ftoa(b, g->hd_ki, 4);
    oled_textf(1, 0, "HD %s %s", a, b);
    oled_ftoa(a, g->hd_kd, 4);
    oled_textf(2, 0, "   %s", a);

    oled_ftoa(a, g->ln_kp, 4); oled_ftoa(b, g->ln_ki, 5);
    oled_textf(3, 0, "LN %s %s", a, b);
    oled_ftoa(a, g->ln_kd, 5);
    oled_textf(4, 0, "   %s", a);

    oled_ftoa(a, g->sp_kp, 2); oled_ftoa(b, g->sp_ki, 1);
    oled_textf(5, 0, "SP %s %s", a, b);
    oled_ftoa(a, g->turn_kp, 2);
    oled_textf(6, 0, "TURN %s", a);

    oled_textf(7, 0, "IMU %lu frm", (unsigned long)imu_frames());
}

static void screen_gray(void)
{
    const gray_t *g = gray_read();

    oled_clear();
    oled_text(0, 0, "-- SENSOR BAR --");

    for (int i = 0; i < 8; i++) {
        uint8_t on = (g->raw >> i) & 1;
        for (int y = 0; y < 7; y++) {
            for (int x = 0; x < 11; x++) {
                oled_pixel((uint8_t)(4 + i * 15 + x), (uint8_t)(20 + y), on);
            }
        }
        oled_textf(4, (uint8_t)(5 + i * 2), "%d", i);
    }

    oled_textf(5, 0, "POS %d mm  CNT %u", (int)g->pos_mm,
               (unsigned)g->count);
    oled_textf(6, 0, "RAW 0x%02X", g->raw);
    oled_textf(7, 0, "YAW %d  IMU %s", (int)imu_yaw(),
               imu_ok() ? "OK" : "LOST");
}

/* ------------------------------------------------------------------ */
/* 按键                                                                */
/* ------------------------------------------------------------------ */
static void handle_keys(void)
{
    uint8_t k = ui_key_pressed();
    if (!k) return;

    if (k & KEY1) {
        s_sel = (task_id_t)((s_sel % 4) + 1);
        ui_beep_ok();
    }
    if (k & KEY2) {
        if (!imu_ok()) {
            ui_beep_fail();
        } else {
            mission_request_start(s_sel);
        }
    }
    if (k & KEY3) {
        mission_request_abort();
        ui_beep_ok();
    }
    if (k & KEY4) {
        s_page = (uint8_t)((s_page + 1) % 3);
        ui_beep_ok();
    }
}

/* ------------------------------------------------------------------ */
int main(void)
{
    SYSCFG_DL_init();

    motor_init();
    encoder_init();
    gray_init();
    imu_init();
    ui_init();
    telemetry_init();
    oled_init();
    mission_init();
    control_init();

    timebase_init();   /* 最后再开 SysTick，避免初始化期间进中断 */

    /* 开机画面 */
    oled_clear();
    oled_text(0, 12, "2024 H  AUTO CAR");
    oled_text(2, 12, "MSPM0G3507");
    oled_text(3, 12, "LCKFB DMX BOARD");
    oled_text(5, 6, "K1:TASK K2:GO");
    oled_text(6, 6, "K3:STOP K4:PAGE");
    oled_flush();
    delay_ms(900);

    uint32_t t_ui = 0;
    uint32_t t_telem = 0;

    while (1) {
        uint32_t now = millis();

        handle_keys();

        if (now - t_ui >= 100) {
            t_ui = now;
            if (s_page == 0) screen_main();
            else if (s_page == 1) screen_gains();
            else screen_gray();
            oled_flush();
        }

        if (now - t_telem >= 10) {
            t_telem = now;
            telemetry_poll();
        }
    }
}
