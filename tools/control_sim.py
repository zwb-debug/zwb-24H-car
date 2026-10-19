#!/usr/bin/env python3
"""控制律快速仿真（验证符号与默认增益，不依赖硬件）。

两部分：
 1. 圆弧循迹：两轮差速 + 一阶速度响应 + 前伸 8cm 的灰度探头，
    控制律与 firmware/control.c 的 CM_LINE 一致（曲率前馈 + PD + 航向阻尼）；
 2. 直线航向：与 CM_STRAIGHT 一致（PID）。
运行： python tools/control_sim.py
"""
import math

R = 40.0
W = 12.0
SENSOR = 8.0
DT = 0.005

# ---- 与 board.h 默认值一致 ----
ARC_V = 31.0
KP_L, KD_L, KH = 0.010, 0.0006, 0.04
KP_H, KI_H, KD_H = 0.020, 0.0015, 0.0025
TAU = 0.05


def wrap(a):
    while a > 180:
        a -= 360
    while a <= -180:
        a += 360
    return a


def arc(offset_cm, heading_err_deg=0.0, T=6.0):
    """从 CCW 圆弧底部出发，返回 (t, 横向偏差cm, 航向误差deg) 历史。"""
    x, y = 0.0, -(R + offset_cm)
    th = heading_err_deg
    vl = vr = ARC_V
    e_prev = offset_cm * 10.0
    hist = []
    t = 0.0
    while t < T:
        sx = x + SENSOR * math.cos(math.radians(th))
        sy = y + SENSOR * math.sin(math.radians(th))
        d = math.hypot(sx, sy) - R          # 传感器在圆外为正
        e = -d * 10.0                        # 线在车右为正
        de = (e - e_prev) / DT
        e_prev = e

        tangent = math.degrees(math.atan2(y, x)) + 90.0
        hdg_err = wrap(tangent - th)

        omega = ARC_V / R - (KP_L * e + KD_L * de) + KH * hdg_err
        vL = max(0.0, ARC_V - omega * W / 2)
        vR = max(0.0, ARC_V + omega * W / 2)

        vl += (vL - vl) * DT / TAU
        vr += (vR - vr) * DT / TAU
        v = (vl + vr) / 2
        w = (vr - vl) / W

        x += v * math.cos(math.radians(th)) * DT
        y += v * math.sin(math.radians(th)) * DT
        th = wrap(th + math.degrees(w) * DT)
        hist.append((t, d, wrap(th - tangent)))
        t += DT
    return hist


def straight(th0, dist=100.0):
    """从原点、初始航向 th0 出发，直行 dist cm，返回 (y, th)。"""
    x = y = 0.0
    th = th0
    vl = vr = 32.0
    integ = 0.0
    prev = wrap(0.0 - th)
    dstate = 0.0
    while x < dist:
        err = wrap(0.0 - th)
        d = (err - prev) / DT
        prev = err
        dstate += 0.35 * (d - dstate)
        integ = max(-60.0, min(60.0, integ + err * DT))
        om = max(-2.6, min(2.6, KP_H * err + KI_H * integ + KD_H * dstate))
        vL = max(0.0, 32.0 - om * W / 2)
        vR = max(0.0, 32.0 + om * W / 2)
        vl += (vL - vl) * DT / TAU
        vr += (vR - vr) * DT / TAU
        v = (vl + vr) / 2
        x += v * math.cos(math.radians(th)) * DT
        y += v * math.sin(math.radians(th)) * DT
        th = wrap(th + math.degrees((vr - vl) / W) * DT)
    return y, th


if __name__ == "__main__":
    print("=== 圆弧循迹（R=40cm，v=31cm/s）===")
    print("初始横向偏差  初始航向偏差   1s 后最大|偏差|   稳态偏差   稳态航向误差")
    for off in (5.0, 3.0, 0.0, -3.0):
        for he in (0.0, 5.0):
            h = arc(off, he)
            late = [p for p in h if p[0] > 1.0]
            md = max(abs(p[1]) for p in late)
            fin = h[-1]
            print(f"  {off:+5.1f}cm      {he:+5.1f}°        {md:6.2f}cm"
                  f"        {fin[1]:+6.2f}cm     {fin[2]:+6.2f}°")

    print("\n=== 直线航向（v=32cm/s，走 100cm）===")
    print("初始航向偏差   终点横向偏差   终点航向")
    for th0 in (1, 2, 3, 5, -3):
        y, th = straight(th0)
        print(f"  {th0:+5.0f}°        {y:+6.2f}cm      {th:+6.2f}°")
