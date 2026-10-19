#include "telemetry.h"
#include "board.h"
#include "bsp_imu.h"
#include "bsp_oled.h"
#include "control.h"
#include "mathx.h"
#include "mission.h"
#include "timebase.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char s_cmd[64];
static uint8_t s_cmd_len;
static volatile uint8_t s_rx_buf[128];
static volatile uint8_t s_rx_head, s_rx_tail;

void telemetry_init(void)
{
    s_cmd_len = 0;
    s_rx_head = s_rx_tail = 0;
    NVIC_ClearPendingIRQ(DEBUG_UART_INST_INT_IRQN);
    NVIC_SetPriority(DEBUG_UART_INST_INT_IRQN, 2);
    NVIC_EnableIRQ(DEBUG_UART_INST_INT_IRQN);
}

void telemetry_send(const char *s)
{
    while (*s) {
        while (DL_UART_isBusy(DEBUG_UART_INST)) { }
        DL_UART_Main_transmitData(DEBUG_UART_INST, (uint8_t)*s++);
    }
}

void UART0_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(DEBUG_UART_INST)) {
    case DL_UART_IIDX_RX: {
        uint8_t b = DL_UART_Main_receiveData(DEBUG_UART_INST);
        uint8_t next = (uint8_t)((s_rx_head + 1) & 0x7F);
        if (next != s_rx_tail) {
            s_rx_buf[s_rx_head] = b;
            s_rx_head = next;
        }
        break;
    }
    default:
        break;
    }
}

static uint8_t cmd_pop(char *c)
{
    if (s_rx_tail == s_rx_head) return 0;
    *c = (char)s_rx_buf[s_rx_tail];
    s_rx_tail = (uint8_t)((s_rx_tail + 1) & 0x7F);
    return 1;
}

static int app_str(char *b, int n, const char *s)
{
    while (*s) b[n++] = *s++;
    return n;
}

static int app_uint(char *b, int n, unsigned long v)
{
    char t[12];
    int k = 0;
    if (v == 0) t[k++] = '0';
    while (v) { t[k++] = (char)('0' + v % 10); v /= 10; }
    while (k > 0) b[n++] = t[--k];
    return n;
}

static int app_int(char *b, int n, long v)
{
    if (v < 0) { b[n++] = '-'; v = -v; }
    return app_uint(b, n, (unsigned long)v);
}

static void print_gains(void)
{
    ctrl_gains_t *g = control_gains();
    static char a[10], b[10], c[10], d[10], e[10], f[10];
    static char buf[110];
    int n = 0;

    oled_ftoa(a, g->hd_kp, 3); oled_ftoa(b, g->hd_ki, 4); oled_ftoa(c, g->hd_kd, 4);
    oled_ftoa(d, g->ln_kp, 4); oled_ftoa(e, g->ln_ki, 5); oled_ftoa(f, g->ln_kd, 5);

    n = app_str(buf, n, "# G hd ");
    n = app_str(buf, n, a); n = app_str(buf, n, " "); n = app_str(buf, n, b);
    n = app_str(buf, n, " "); n = app_str(buf, n, c);

    n = app_str(buf, n, "  ln ");
    n = app_str(buf, n, d); n = app_str(buf, n, " "); n = app_str(buf, n, e);
    n = app_str(buf, n, " "); n = app_str(buf, n, f);

    oled_ftoa(a, g->sp_kp, 2); oled_ftoa(b, g->sp_ki, 2);
    oled_ftoa(c, g->turn_kp, 2);
    n = app_str(buf, n, "  sp "); n = app_str(buf, n, a);
    n = app_str(buf, n, " ");    n = app_str(buf, n, b);
    n = app_str(buf, n, "  turn "); n = app_str(buf, n, c);
    n = app_str(buf, n, "\r\n");
    buf[n] = 0;
    telemetry_send(buf);
}

static void handle_cmd(char *line)
{
    ctrl_gains_t *g = control_gains();

    switch (line[0]) {
    case '1': case '2': case '3': case '4':
        mission_request_start((task_id_t)(line[0] - '0'));
        telemetry_send("# start\r\n");
        break;
    case 'x':
        mission_request_abort();
        telemetry_send("# abort\r\n");
        break;
    case 'z':
        imu_zero();
        telemetry_send("# yaw zero\r\n");
        break;
    case 'p':
        print_gains();
        break;
    case 'h': {
        char *s = line + 1;
        g->hd_kp = strtof(s, &s); if (*s == ',') s++;
        g->hd_ki = strtof(s, &s); if (*s == ',') s++;
        g->hd_kd = strtof(s, &s);
        telemetry_send("# hd set\r\n");
        break;
    }
    case 'l': {
        char *s = line + 1;
        g->ln_kp = strtof(s, &s); if (*s == ',') s++;
        g->ln_ki = strtof(s, &s); if (*s == ',') s++;
        g->ln_kd = strtof(s, &s);
        telemetry_send("# ln set\r\n");
        break;
    }
    case 'v': {
        char *s = line + 1;
        g->sp_kp = strtof(s, &s); if (*s == ',') s++;
        g->sp_ki = strtof(s, &s);
        telemetry_send("# sp set\r\n");
        break;
    }
    case 't': {
        g->turn_kp = strtof(line + 1, NULL);
        telemetry_send("# turn set\r\n");
        break;
    }
    case 'm': {
        char *s = line + 1;
        long l = strtol(s, &s, 10); if (*s == ',') s++;
        long r = strtol(s, &s, 10);
        control_raw((int16_t)l, (int16_t)r);
        telemetry_send("# raw\r\n");
        break;
    }
    case 'n':
        control_idle();
        telemetry_send("# idle\r\n");
        break;
    case '?':
        telemetry_send("# cmds: 1-4 start, x stop, z yaw0, p print, "
                       "h<kp,ki,kd> l<...> v<kp,ki> t<kp> m<l,r> n\r\n");
        break;
    default:
        break;
    }
}

void telemetry_poll(void)
{
    static uint32_t last_tx = 0;
    char c;

    /* 收命令：回车/换行结束 */
    while (cmd_pop(&c)) {
        if (c == '\n' || c == '\r') {
            if (s_cmd_len > 0) {
                s_cmd[s_cmd_len] = 0;
                handle_cmd(s_cmd);
                s_cmd_len = 0;
            }
        } else if (s_cmd_len < sizeof(s_cmd) - 1) {
            s_cmd[s_cmd_len++] = c;
        }
    }

    /* 50Hz 遥测 */
    uint32_t now = millis();
    if (now - last_tx < 20) return;
    last_tx = now;

    const mission_info_t *m = mission_info();
    const ctrl_state_t *st = control_state();

    static char buf[128];
    int n = 0;
    n = app_str(buf, n, "$D,");
    n = app_uint(buf, n, (unsigned long)m->elapsed_ms); n = app_str(buf, n, ",");
    n = app_uint(buf, n, (unsigned)m->task);            n = app_str(buf, n, ",");
    n = app_uint(buf, n, (unsigned)m->state);           n = app_str(buf, n, ",");
    n = app_uint(buf, n, (unsigned)m->seg_idx);         n = app_str(buf, n, ",");
    n = app_uint(buf, n, (unsigned)m->lap);             n = app_str(buf, n, ",");
    n = app_uint(buf, n, (unsigned)(m->s_total * 10.0f)); n = app_str(buf, n, ",");
    n = app_int(buf, n, (long)(imu_yaw() * 10.0f));     n = app_str(buf, n, ",");
    n = app_int(buf, n, (long)(m->v_ref * 10.0f));      n = app_str(buf, n, ",");
    n = app_int(buf, n, (long)(st->vL * 10.0f));        n = app_str(buf, n, ",");
    n = app_int(buf, n, (long)(st->vR * 10.0f));        n = app_str(buf, n, ",");
    n = app_int(buf, n, (long)st->line_mm);             n = app_str(buf, n, ",");
    n = app_int(buf, n, (long)st->dutyL);               n = app_str(buf, n, ",");
    n = app_int(buf, n, (long)st->dutyR);               n = app_str(buf, n, ",");
    n = app_uint(buf, n, (unsigned)(m->s_seg * 10.0f)); n = app_str(buf, n, ",");
    n = app_uint(buf, n, (unsigned)imu_frames());
    n = app_str(buf, n, "\r\n");
    buf[n] = 0;
    telemetry_send(buf);
}
