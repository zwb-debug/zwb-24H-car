#!/usr/bin/env python3
"""几何自检：把 mission.c 里的分段表用同一套几何复算一遍。

检查内容：
 1. 每段结束时小车位置是否落在题图顶点 A/B/C/D 上（误差 < 1cm）；
 2. 进入圆弧/离开圆弧时的航向是否与切线一致；
 3. 按给定巡航速度/加减速估算四题用时，核对是否满足 15/30/40s 限制。
运行： py tools/path_check.py
"""
import math

# ---------------- 场地 ----------------
A = (60.0, 100.0)
B = (160.0, 100.0)
C = (160.0, 20.0)
D = (60.0, 20.0)
R = 40.0
LEN_AB = 100.0
LEN_DIAG = math.hypot(100.0, 80.0)
LEN_ARC = math.pi * R
H_AB = 0.0
H_CD = 180.0
H_AC = math.degrees(math.atan2(-80.0, 100.0))
H_BD = math.degrees(math.atan2(-80.0, -100.0))

V = {1: 32.0, 2: 36.0, 3: 36.0, 4: 58.0}
ARC_SCALE = 0.86
TURN_ENTRY = 16.0
ACC = 160.0
DEC = 180.0

LINE, TURN, ARC = "LINE", "TURN", "ARC"


def task1():
    return [(LINE, LEN_AB, H_AB, 0)]


def task2():
    return [
        (LINE, LEN_AB, H_AB, 0),
        (ARC, LEN_ARC, H_AB, -1),
        (LINE, LEN_AB, H_CD, 0),
        (ARC, LEN_ARC, H_CD, -1),
    ]


def task3(stop=True):
    return [
        (LINE, LEN_DIAG, H_AC, 0),
        (TURN, 0, H_AB, 0),
        (ARC, LEN_ARC, H_AB, +1),
        (TURN, 0, H_BD, 0),
        (LINE, LEN_DIAG, H_BD, 0),
        (TURN, 0, H_CD, 0),
        (ARC, LEN_ARC, H_CD, -1),
    ]


def wrap(a):
    while a > 180:
        a -= 360
    while a <= -180:
        a += 360
    return a


def run(segs, start, h0, laps=1, interlap=None):
    x, y = start
    h = h0
    total = 0.0
    t = 0.0
    log = []
    for lap in range(laps):
        for typ, ln, target, d in segs:
            if typ == LINE:
                # 加减速时间（简化梯形/三角形）
                v = V_SEG
                v_end = v
                if ln > 0:
                    t += ln / v + v / (2 * ACC) + v_end / (2 * DEC)
                s = ln
                x += s * math.cos(math.radians(h))
                y += s * math.sin(math.radians(h))
                total += s
                log.append((lap + 1, "LINE", ln, h, x, y))
            elif typ == ARC:
                v = V_SEG * ARC_SCALE
                t += ln / v
                # 数值积分
                n = 2000
                ds = ln / n
                for _ in range(n):
                    h = wrap(h + d * math.degrees(ds / R))
                    x += ds * math.cos(math.radians(h))
                    y += ds * math.sin(math.radians(h))
                total += ln
                log.append((lap + 1, "ARC", ln, h, x, y))
            elif typ == TURN:
                dh = wrap(target - h)
                t += abs(dh) / 90.0 * 0.45 + 0.05
                h = wrap(target)
                log.append((lap + 1, "TURN", abs(dh), h, x, y))
        if interlap is not None and lap < laps - 1:
            dh = wrap(interlap - h)
            t += abs(dh) / 90.0 * 0.45 + 0.05
            h = wrap(interlap)
    return (x, y, h, total, t, log)


def nearest(p):
    best = None
    for name, q in (("A", A), ("B", B), ("C", C), ("D", D)):
        d = math.hypot(p[0] - q[0], p[1] - q[1])
        if best is None or d < best[1]:
            best = (name, d)
    return best


def check(name, segs, start, h0, expect, laps=1, limit=None):
    global V_SEG
    x, y, h, total, t, log = run(segs, start, h0, laps,
                                  interlap=(H_AC if laps > 1 else None))
    nm, dist = nearest((x, y))
    ok = (nm == expect and dist < 1.0)
    print(f"{name}: 终点 ({x:7.2f},{y:7.2f}) 航向 {h:8.2f}° "
          f"总里程 {total:7.2f}cm  估算 {t:5.1f}s  "
          f"最近顶点 {nm} 偏差 {dist:4.2f}cm  {'OK' if ok else 'FAIL'}")
    if limit:
        print(f"        时间限制 {limit}s -> {'满足' if t < limit else '超时!'}")
    if not ok:
        for row in log:
            print("        ", row)
    return ok


if __name__ == "__main__":
    print("场地: A(60,100) B(160,100) C(160,20) D(60,20) R=40cm")
    print(f"关键长度: AB=100  AD弧={LEN_ARC:.2f}  对角线={LEN_DIAG:.2f}")
    print(f"关键航向: AC={H_AC:.2f}°  BD={H_BD:.2f}°\n")

    V_SEG = V[1]
    ok1 = check("第1题", task1(), A, 0.0, "B", limit=15)

    V_SEG = V[2]
    ok2 = check("第2题", task2(), A, 0.0, "A", limit=30)

    V_SEG = V[3]
    ok3 = check("第3题", task3(), A, H_AC, "A", limit=40)

    V_SEG = V[4]
    ok4 = check("第4题(4圈)", task3(), A, H_AC, "A", laps=4, limit=120)

    print("\n结论:", "全部通过" if (ok1 and ok2 and ok3 and ok4) else "存在错误")
