#include "bsp_oled.h"
#include "font6x8.h"
#include "mathx.h"
#include "ti_msp_dl_config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define OLED_ADDR   0x3C
#define I2C_CHUNK   7

static uint8_t s_fb[OLED_PAGES][OLED_COLS];
static uint8_t s_shadow[OLED_PAGES][OLED_COLS];
static uint8_t s_dirty[OLED_PAGES];

static uint8_t s_present = 0;

/* ------------------------------------------------------------------ */
/* 底层 I2C                                                            */
/* ------------------------------------------------------------------ */
static uint32_t i2c_wait_idle(void)
{
    uint32_t guard = 200000;
    while (!(DL_I2C_getControllerStatus(OLED_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--guard == 0) return 0;
    }
    return 1;
}

static void i2c_send(uint8_t ctrl, const uint8_t *data, uint16_t len)
{
    uint16_t off = 0;
    uint8_t  buf[I2C_CHUNK + 1];

    while (off < len) {
        uint16_t n = len - off;
        if (n > I2C_CHUNK) n = I2C_CHUNK;

        buf[0] = ctrl;
        for (uint16_t i = 0; i < n; i++) buf[1 + i] = data[off + i];

        if (!i2c_wait_idle()) return;

        DL_I2C_fillControllerTXFIFO(OLED_I2C_INST, buf, (uint32_t)(n + 1));
        DL_I2C_startControllerTransfer(OLED_I2C_INST, OLED_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_TX, (uint32_t)(n + 1));

        /* errata I2C_ERR_13：启动后延时几个 I2C 功能时钟 */
        delay_cycles(240);

        uint32_t guard = 200000;
        while ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
                DL_I2C_CONTROLLER_STATUS_BUSY) && --guard) {
        }
        if (DL_I2C_getControllerStatus(OLED_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) {
            return;
        }
        off += n;
    }
}

static void oled_cmd(uint8_t c)
{
    i2c_send(0x00, &c, 1);
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
void oled_init(void)
{
    static const uint8_t init_seq[] = {
        0xAE,        /* display off */
        0xD5, 0x80,  /* clock div */
        0xA8, 0x3F,  /* multiplex 64 */
        0xD3, 0x00,  /* display offset */
        0x40,        /* start line 0 */
        0x8D, 0x14,  /* charge pump on */
        0x20, 0x00,  /* horizontal addressing */
        0xA1,        /* segment remap */
        0xC8,        /* COM scan dec */
        0xDA, 0x12,  /* COM pins */
        0x81, 0xCF,  /* contrast */
        0xD9, 0xF1,  /* precharge */
        0xDB, 0x40,  /* VCOMH */
        0xA4,        /* resume RAM */
        0xA6,        /* normal */
        0xAF         /* display on */
    };

    memset(s_fb, 0, sizeof(s_fb));
    memset(s_shadow, 0xFF, sizeof(s_shadow));  /* 强制首帧全刷 */
    memset(s_dirty, 1, sizeof(s_dirty));
    s_present = 1;

    for (uint8_t i = 0; i < sizeof(init_seq); i++) {
        oled_cmd(init_seq[i]);
    }
}

void oled_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
    memset(s_dirty, 1, sizeof(s_dirty));
}

void oled_pixel(uint8_t x, uint8_t y, uint8_t on)
{
    if (x >= OLED_COLS || y >= 64) return;
    uint8_t page = (uint8_t)(y >> 3);
    uint8_t bit  = (uint8_t)(1u << (y & 7));
    if (on) s_fb[page][x] |= bit;
    else    s_fb[page][x] &= (uint8_t)~bit;
    s_dirty[page] = 1;
}

void oled_flush(void)
{
    if (!s_present) return;

    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        if (!s_dirty[page]) continue;
        if (memcmp(s_fb[page], s_shadow[page], OLED_COLS) == 0) {
            s_dirty[page] = 0;
            continue;
        }
        oled_cmd((uint8_t)(0xB0 | page));
        oled_cmd(0x00);
        oled_cmd(0x10);
        i2c_send(0x40, s_fb[page], OLED_COLS);
        memcpy(s_shadow[page], s_fb[page], OLED_COLS);
        s_dirty[page] = 0;
    }
}

/* ------------------------------------------------------------------ */
/* 文本                                                                */
/* ------------------------------------------------------------------ */
void oled_text(uint8_t page, uint8_t col, const char *s)
{
    if (page >= OLED_PAGES) return;
    while (*s && col < OLED_COLS - 5) {
        uint8_t c = (uint8_t)*s++;
        if (c < 32 || c > 126) c = '?';
        const uint8_t *g = font6x8[c - 32];
        for (uint8_t x = 0; x < 6; x++) {
            s_fb[page][col + x] = g[x];
        }
        col = (uint8_t)(col + 6);
    }
    s_dirty[page] = 1;
}

/* ---------------- 极简格式化（不引入 printf，省 Flash 与栈） ---------------- */
static int fmt_uint(char *out, unsigned long v, int base, int width, char pad)
{
    char tmp[12];
    int n = 0, len = 0;
    if (v == 0) tmp[n++] = '0';
    while (v) {
        int d = (int)(v % (unsigned long)base);
        tmp[n++] = (char)(d < 10 ? '0' + d : 'A' + d - 10);
        v /= (unsigned long)base;
    }
    while (n < width) tmp[n++] = pad;
    while (n > 0) out[len++] = tmp[--n];
    return len;
}

void oled_textf(uint8_t page, uint8_t col, const char *fmt, ...)
{
    char buf[40];
    int n = 0;
    va_list ap;
    va_start(ap, fmt);

    while (*fmt && n < (int)sizeof(buf) - 12) {
        if (*fmt != '%') { buf[n++] = *fmt++; continue; }
        fmt++;
        if (*fmt == '%') { buf[n++] = '%'; fmt++; continue; }

        char pad = ' ';
        int width = 0;
        if (*fmt == '0') { pad = '0'; fmt++; }
        while (*fmt >= '0' && *fmt <= '9') { width = width * 10 + (*fmt++ - '0'); }

        int is_long = 0;
        if (*fmt == 'l') { is_long = 1; fmt++; }

        switch (*fmt++) {
        case 'd': {
            long v = is_long ? va_arg(ap, long) : (long)va_arg(ap, int);
            if (v < 0) { buf[n++] = '-'; v = -v; }
            n += fmt_uint(&buf[n], (unsigned long)v, 10, 0, ' ');
            break;
        }
        case 'u':
            n += fmt_uint(&buf[n],
                is_long ? va_arg(ap, unsigned long) : va_arg(ap, unsigned int),
                10, 0, ' ');
            break;
        case 'x':
        case 'X':
            n += fmt_uint(&buf[n],
                is_long ? va_arg(ap, unsigned long) : va_arg(ap, unsigned int),
                16, width, pad);
            break;
        case 'c':
            buf[n++] = (char)va_arg(ap, int);
            break;
        case 's': {
            const char *s = va_arg(ap, const char *);
            if (!s) s = "(null)";
            while (*s && n < (int)sizeof(buf) - 1) buf[n++] = *s++;
            break;
        }
        default:
            break;
        }
    }
    va_end(ap);
    buf[n] = 0;
    oled_text(page, col, buf);
}

void oled_bar(uint8_t page, uint8_t value)
{
    if (page >= OLED_PAGES) return;
    if (value > 100) value = 100;
    uint8_t w = (uint8_t)((value * 100u) / 100u);  /* 100px 宽 */
    for (uint8_t x = 0; x < 100; x++) {
        s_fb[page][x] = (x < w) ? 0x7E : 0x18;
    }
    s_dirty[page] = 1;
}

void oled_ftoa(char *out, float v, int decimals)
{
    int neg = 0;
    if (v < 0.0f) { neg = 1; v = -v; }
    if (v > 9999.0f) v = 9999.0f;

    int32_t scale = 1;
    for (int i = 0; i < decimals; i++) scale *= 10;

    int32_t iv = (int32_t)(v * (float)scale + 0.5f);
    int32_t ip = iv / scale;
    int32_t fp = iv % scale;

    char tmp[10];
    int n = 0;

    if (neg) *out++ = '-';
    if (ip == 0) tmp[n++] = '0';
    while (ip > 0) { tmp[n++] = (char)('0' + ip % 10); ip /= 10; }
    while (n > 0) *out++ = tmp[--n];

    if (decimals == 0) { *out = 0; return; }

    *out++ = '.';
    for (int i = decimals - 1; i >= 0; i--) {
        tmp[i] = (char)('0' + fp % 10);
        fp /= 10;
    }
    for (int i = 0; i < decimals; i++) *out++ = tmp[i];
    *out = 0;
}
