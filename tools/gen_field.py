#!/usr/bin/env python3
"""生成 2024 电赛 H 题场地/路径示意图（docs/field_layout.png）。

按题图几何：场地 220x120cm，两个半径 40cm 的半圆弧，
A(60,100) B(160,100) C(160,20) D(60,20)，线宽 1.8cm。
"""
import math
import os
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS = os.path.join(ROOT, "docs")

S = 10          # 4x 超采样下的像素/cm
SS = 4
W, H = 220, 120
MARGIN_L, MARGIN_T, MARGIN_R, MARGIN_B = 26, 22, 26, 30

PX = (W + MARGIN_L + MARGIN_R) * S * SS
PY = (H + MARGIN_T + MARGIN_B) * S * SS


def font(size_cm, bold=False):
    px = int(size_cm * S * SS)
    for p in (r"C:\Windows\Fonts\msyhbd.ttc" if bold else r"C:\Windows\Fonts\msyh.ttc",
              r"C:\Windows\Fonts\simhei.ttf"):
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, px)
            except Exception:
                pass
    return ImageFont.load_default()


def cm2px(x, y):
    """场地坐标(cm, y 向上) -> 图像像素"""
    return ((MARGIN_L + x) * S * SS, (MARGIN_T + (H - y)) * S * SS)


def arc_points(cx, cy, r, a0, a1, n=180):
    pts = []
    for i in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * i / n)
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def draw_poly(draw, pts, **kw):
    draw.line([cm2px(x, y) for x, y in pts], **kw)


def arrow(draw, p, q, color, width, size=1.4):
    """在 q 处画箭头"""
    x1, y1 = cm2px(*p)
    x2, y2 = cm2px(*q)
    draw.line([(x1, y1), (x2, y2)], fill=color, width=width)
    ang = math.atan2(y2 - y1, x2 - x1)
    l = size * S * SS
    for da in (math.radians(152), math.radians(-152)):
        draw.line([(x2, y2), (x2 + l * math.cos(ang + da),
                             y2 + l * math.sin(ang + da))],
                  fill=color, width=width)


def dashed(draw, pts, color, width, dash=1.2, gap=1.0):
    """沿折线画虚线"""
    for i in range(len(pts) - 1):
        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]
        L = math.hypot(x2 - x1, y2 - y1)
        if L < 1e-6:
            continue
        n = max(1, int(L / (dash + gap)))
        for k in range(n):
            t0 = k * (dash + gap) / L
            t1 = min(1.0, t0 + dash / L)
            if t0 >= 1:
                break
            a = (x1 + (x2 - x1) * t0, y1 + (y2 - y1) * t0)
            b = (x1 + (x2 - x1) * t1, y1 + (y2 - y1) * t1)
            draw.line([cm2px(*a), cm2px(*b)], fill=color, width=width)


def main():
    os.makedirs(DOCS, exist_ok=True)
    img = Image.new("RGB", (PX, PY), "white")
    d = ImageDraw.Draw(img)

    f_title = font(3.2, True)
    f_lab = font(2.6, True)
    f_dim = font(2.2)
    f_note = font(2.0)

    # 场地
    x0, y0 = cm2px(0, H)
    x1, y1 = cm2px(W, 0)
    d.rectangle([x0, y0, x1, y1], fill=(250, 250, 250), outline=(150, 150, 150),
                width=int(0.3 * S * SS))

    line_w = int(1.8 * S * SS)

    # 左半圆 A-D（向左凸），右半圆 B-C（向右凸）
    left = arc_points(60, 60, 40, 90, 270)
    right = arc_points(160, 60, 40, -90, 90)
    draw_poly(d, left, fill=(20, 20, 20), width=line_w, joint="curve")
    draw_poly(d, right, fill=(20, 20, 20), width=line_w, joint="curve")

    # 顶点
    pts = {"A": (60, 100), "B": (160, 100), "C": (160, 20), "D": (60, 20)}
    for name, (x, y) in pts.items():
        px, py = cm2px(x, y)
        r = 0.55 * S * SS
        d.ellipse([px - r, py - r, px + r, py + r], fill=(200, 30, 30))
        off = (0.9 * S * SS, -2.6 * S * SS)
        d.text((px + off[0], py + off[1]), name, font=f_lab, fill=(180, 20, 20))

    # ---------- 第 2 题路径（蓝色实线，顺时针） ----------
    c2 = (30, 90, 200)
    # 黑色弧线保持干净，行车方向用弧线外侧的蓝色箭头表示

    # ---------- 第 3 题路径（橙红虚线，8 字） ----------
    c3 = (230, 110, 20)
    diag1 = [(60, 100), (160, 20)]
    diag2 = [(160, 100), (60, 20)]
    dashed(d, diag1, c3, int(0.32 * S * SS))
    dashed(d, diag2, c3, int(0.32 * S * SS))

    # 方向箭头
    aw = int(0.34 * S * SS)
    arrow(d, (60, 100), (100, 68), c3, aw)      # A->C
    arrow(d, (160, 100), (120, 68), c3, aw)     # B->D
    arrow(d, (80, 106), (115, 106), c2, aw)     # A->B（顶边外侧）
    arrow(d, (207, 75), (207, 45), c2, aw)      # B->C（右侧外侧）
    arrow(d, (140, 14), (105, 14), c2, aw)      # C->D（底边外侧）
    arrow(d, (13, 45), (13, 75), c2, aw)        # D->A（左侧外侧）

    # ---------- 尺寸标注 ----------
    dimc = (90, 90, 90)
    tick = 0.8 * S * SS
    # 宽度 220
    ya = (MARGIN_T + H + 14) * S * SS
    d.line([cm2px(0, 0)[0], ya, cm2px(W, 0)[0], ya], fill=dimc,
           width=int(0.12 * S * SS))
    for x in (0, W):
        px = cm2px(x, 0)[0]
        d.line([px, ya - tick, px, ya + tick], fill=dimc, width=int(0.12 * S * SS))
    d.text(((MARGIN_L + W / 2) * S * SS - 3.0 * S * SS, ya - 3.4 * S * SS),
           "220cm", font=f_dim, fill=dimc)

    # 高度 120
    xa = (MARGIN_L - 14) * S * SS
    d.line([xa, cm2px(0, 0)[1], xa, cm2px(0, H)[1]], fill=dimc,
           width=int(0.12 * S * SS))
    d.text((xa - 1.0 * S * SS, (MARGIN_T + H / 2) * S * SS), "120cm",
           font=f_dim, fill=dimc, anchor="rm")

    # AB=100
    d.line([cm2px(60, 110)[0], cm2px(60, 110)[1], cm2px(160, 110)[0],
            cm2px(160, 110)[1]], fill=dimc, width=int(0.12 * S * SS))
    for x in (60, 160):
        px, py = cm2px(x, 110)
        d.line([px, py - tick, px, py + tick], fill=dimc, width=int(0.12 * S * SS))
    d.text(cm2px(110, 113), "AB = 100cm", font=f_dim, fill=dimc, anchor="mb")

    # R=40 标注（左弧）
    cx, cy = cm2px(60, 60)
    px, py = cm2px(20, 60)
    d.line([cx, cy, px, py], fill=(120, 120, 200), width=int(0.12 * S * SS))
    d.text(((cx + px) / 2, (cy + py) / 2 - 1.2 * S * SS), "R40", font=f_dim,
           fill=(80, 80, 180), anchor="mb")

    # 说明
    tx = cm2px(0, 118)[1] - 3.0 * S * SS
    d.text((MARGIN_L * S * SS, 2 * S * SS), "2024 电赛 H 题 场地与两条行驶路径",
           font=f_title, fill=(30, 30, 30))
    d.text((MARGIN_L * S * SS, 6.5 * S * SS),
           "蓝箭头：第2题行车方向 A→B→C→D→A（整圈）    橙虚线：第3/4题对角线 A→C、B→D（8字，4圈）",
           font=f_note, fill=(70, 70, 70))
    d.text((MARGIN_L * S * SS, 10.0 * S * SS),
           "场地内只有两条黑色半圆弧（线宽1.8cm），AB/CD 段无引导线，靠陀螺仪+编码器直行",
           font=f_note, fill=(70, 70, 70))

    img = img.resize((PX // SS, PY // SS), Image.LANCZOS)
    out = os.path.join(DOCS, "field_layout.png")
    img.save(out)
    print("wrote", out, img.size)


if __name__ == "__main__":
    main()
