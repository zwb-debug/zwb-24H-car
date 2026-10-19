#!/usr/bin/env python3
"""生成硬件接线框图 docs/wiring.png（Pillow，信号为主、电源另注）。"""
import os
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W, H = 1760, 980

C_BG = (255, 255, 255)
C_BOX = (246, 249, 253)
C_EDGE = (72, 92, 122)
C_T = (24, 34, 54)
C_SIG = (32, 104, 214)
C_PWR = (198, 44, 44)
C_GND = (40, 140, 70)
C_SUB = (86, 96, 112)


def font(sz, bold=False):
    for p in (r"C:\Windows\Fonts\msyhbd.ttc" if bold else r"C:\Windows\Fonts\msyh.ttc",
              r"C:\Windows\Fonts\simhei.ttf"):
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, sz)
            except Exception:
                pass
    return ImageFont.load_default()


F_T = font(30, True)
F_B = font(24, True)
F_S = font(19)
F_L = font(17)


def box(d, x, y, w, h, title, sub, fill=C_BOX):
    d.rounded_rectangle([x, y, x + w, y + h], radius=12, fill=fill,
                        outline=C_EDGE, width=3)
    d.text((x + 14, y + 10), title, font=F_B, fill=C_T)
    yy = y + 44
    for line in sub:
        d.text((x + 14, yy), line, font=F_S, fill=C_SUB)
        yy += 25


def arrow(d, x1, y1, x2, y2, color=C_SIG, width=4, label=None, ly=-6):
    d.line([(x1, y1), (x2, y2)], fill=color, width=width)
    import math
    ang = math.atan2(y2 - y1, x2 - x1)
    L = 14
    for da in (math.radians(150), math.radians(-150)):
        d.line([(x2, y2), (x2 + L * math.cos(ang + da), y2 + L * math.sin(ang + da))],
               fill=color, width=width)
    if label:
        d.text(((x1 + x2) / 2, min(y1, y2) + ly), label, font=F_L,
               fill=color, anchor="mb")


def main():
    img = Image.new("RGB", (W, H), C_BG)
    d = ImageDraw.Draw(img)

    d.text((36, 22), "2024 H 题小车 —— 信号接线框图", font=F_T, fill=C_T)
    d.text((36, 66), "蓝=信号；电源/地见下方电源说明。所有 GND 单点汇合到电池负极。",
           font=F_L, fill=(90, 90, 90))

    # MCU
    mx, my, mw, mh = 500, 110, 470, 620
    box(d, mx, my, mw, mh, "地猛星 MSPM0G3507",
        ["32MHz / 128KB Flash / 32KB RAM",
         "TIMG0 20kHz PWM：PA12/PA13",
         "编码器：PA16/17、PA24/25",
         "灰度：PB8–PB15",
         "陀螺仪：UART1 PA9",
         "OLED：I2C1 PB2/PB3",
         "CH340：UART0 PA10/PA11",
         "按键：PA22/23/26/27",
         "声光：PB7 / PA14 / PA15", "", "200Hz 控制：轮速 PI + 航向 PID + 循迹 PD"])

    # 左侧
    box(d, 30, 140, 330, 100, "8 路数字灰度", ["PB8–PB15；5V 供电", "注意：5V 输出需电平转换"])
    box(d, 30, 300, 330, 100, "JY61P 陀螺仪", ["UART1 115200；TX→PA9", "箭头朝车头，静置 3s"])
    box(d, 30, 460, 330, 100, "4 个按键", ["PA22/23/26/27", "按下接 GND，内部上拉"])
    box(d, 30, 620, 330, 100, "调试串口 CH340E", ["UART0 115200", "USB-C 直连电脑"])

    # 右侧
    box(d, 1120, 140, 380, 100, "TB6612FNG 驱动", ["PWMA/B→PA12/13；AIN/BIN→PB0/1/4/5", "STBY→PB6；VM=电池，VCC=5V"])
    box(d, 1120, 300, 380, 100, "左/右 N20 电机", ["编码器 A/B → PA16/17、PA24/25", "13 线 × 4 倍频 × 30 减速比"])
    box(d, 1120, 460, 380, 100, "0.96\" OLED", ["I2C1 400kHz", "SCL=PB2  SDA=PB3"])
    box(d, 1120, 620, 380, 100, "声光提示", ["蜂鸣器 PB7（S8050 驱动）", "LED PA14（板载）/ PA15"])

    # 信号箭头
    for y, lab in ((190, "PB8–15"), (350, "PA9 (RX)"), (510, "GPIO"), (670, "PA10/11")):
        arrow(d, 360, y, mx, y, label=lab)
    for y, lab in ((190, "PWM/方向"), (350, "编码器"), (510, "I2C"), (670, "GPIO")):
        arrow(d, mx + mw, y, 1120, y, label=lab)

    # 电源说明
    py = 790
    d.rounded_rectangle([30, py, W - 30, H - 30], radius=12, fill=(255, 250, 248),
                        outline=C_PWR, width=3)
    d.text((50, py + 12), "电源树", font=F_B, fill=C_PWR)
    box(d, 150, py + 55, 260, 90, "2S LiPo 7.4V", ["850–1500mAh 25C"], fill=(255, 245, 245))
    box(d, 520, py + 55, 260, 90, "5V/3A 降压", ["mini560 / MP1584"], fill=(255, 245, 245))
    arrow(d, 410, py + 100, 520, py + 100, color=C_PWR, width=5)
    d.text((830, py + 56),
           "电池 → 保险丝 → 开关 → TB6612 VM（并 1000µF+0.1µF）",
           font=F_S, fill=C_PWR)
    d.text((830, py + 90),
           "同路电池 → 5V/3A → TB6612 VCC、灰度、陀螺仪、OLED、地猛星 5V",
           font=F_S, fill=C_PWR)
    d.text((830, py + 124),
           "注意：灰度若为 5V 输出，必须电平转换后再进 PB8–PB15",
           font=F_S, fill=(180, 60, 20))

    out = os.path.join(ROOT, "docs", "wiring.png")
    img.save(out)
    print("wrote", out)


if __name__ == "__main__":
    main()
