#!/usr/bin/env python3
"""生成 6x8 点阵字库（ASCII 32..126），输出 firmware/font6x8.c。

用 Windows 自带等宽字体 Consolas 渲染 -> 阈值二值化 -> 6 列 x 8 行
（列优先，bit0 在上，符合 SSD1306 page 格式）。
同时输出 docs/font_preview.png 供人工检查。
"""
import os
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FONT_CANDIDATES = [
    r"C:\Windows\Fonts\consola.ttf",
    r"C:\Windows\Fonts\lucon.ttf",
    r"/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
]

CHAR_W, CHAR_H = 6, 8
SIZE = 9
THRESH = 110


def load_font():
    for p in FONT_CANDIDATES:
        if os.path.exists(p):
            return ImageFont.truetype(p, SIZE)
    return ImageFont.load_default()


def glyph(font, ch):
    img = Image.new("L", (CHAR_W, CHAR_H), 0)
    d = ImageDraw.Draw(img)
    # Consolas 在 size=9 时基线合适，上移 1px 让大写字母落在 0..5 行
    d.text((0, -1), ch, font=font, fill=255)
    px = img.load()
    cols = []
    for x in range(CHAR_W):
        b = 0
        for y in range(CHAR_H):
            if px[x, y] >= THRESH:
                b |= (1 << y)
        cols.append(b)
    return cols


def main():
    font = load_font()
    chars = list(range(32, 127))
    table = [glyph(font, chr(c)) for c in chars]

    lines = []
    lines.append("/**")
    lines.append(" * @file  font6x8.c")
    lines.append(" * @brief 6x8 ASCII 点阵字库（自动生成，见 tools/gen_font.py）")
    lines.append(" */")
    lines.append('#include "font6x8.h"')
    lines.append("")
    lines.append("const uint8_t font6x8[95][6] = {")
    for i, cols in enumerate(table):
        ch = chars[i]
        disp = chr(ch) if 32 <= ch < 127 else "?"
        if disp in ("\\", "'"):
            disp = "\\" + disp
        lines.append("    {%3d,%3d,%3d,%3d,%3d,%3d}, /* 0x%02X '%s' */" %
                     (cols[0], cols[1], cols[2], cols[3], cols[4], cols[5], ch, disp))
    lines.append("};")
    lines.append("")

    out_c = os.path.join(ROOT, "firmware", "font6x8.c")
    with open(out_c, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("wrote", out_c)

    # 预览：把一行文字放大 4 倍
    text = "T:2 S:RUN 0123456789"
    scale = 4
    img = Image.new("RGB", (len(text) * CHAR_W * scale, CHAR_H * scale), (0, 0, 0))
    d = ImageDraw.Draw(img)
    for i, ch in enumerate(text):
        c = ord(ch)
        if c < 32 or c > 126:
            continue
        cols = table[c - 32]
        for x in range(CHAR_W):
            for y in range(CHAR_H):
                if cols[x] >> y & 1:
                    x0 = (i * CHAR_W + x) * scale
                    y0 = y * scale
                    d.rectangle([x0, y0, x0 + scale - 1, y0 + scale - 1],
                                fill=(120, 220, 255))
    out_png = os.path.join(ROOT, "docs", "font_preview.png")
    img.save(out_png)
    print("wrote", out_png)

    # 简单校验：所有字符至少有一个像素（空格除外）
    empty = [chr(c) for c in chars if c != 32 and all(v == 0 for v in table[c - 32])]
    print("empty glyphs:", empty)


if __name__ == "__main__":
    main()
