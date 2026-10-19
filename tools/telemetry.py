#!/usr/bin/env python3
"""串口遥测 / 记录 / 画图工具（2024 H 题小车）。

用法：
    python tools/telemetry.py --port COM5
    python tools/telemetry.py --port COM5 --task 3 --log run3.csv
    python tools/telemetry.py --port COM5 --cmd "h0.02,0.0015,0.0025"
    python tools/telemetry.py --port COM5 --log run.csv --plot

协议（固件 telemetry.c）：
    $D,t_ms,task,state,seg,lap,s10,yaw10,vref10,vL10,vR10,line_mm,dutyL,dutyR,segs10,frames

依赖：pyserial；--plot 另需 matplotlib。
"""
import argparse
import csv
import os
import sys
import time

try:
    import serial
except ImportError:
    print("需要 pyserial：pip install pyserial")
    sys.exit(1)

FIELDS = ["t_ms", "task", "state", "seg", "lap", "s", "yaw", "vref",
          "vL", "vR", "line_mm", "dutyL", "dutyR", "seg_s", "frames"]

STATE = {0: "IDLE", 1: "ARMED", 2: "RUN", 3: "DONE", 4: "FAULT", 5: "CALIB"}


def parse(line):
    if not line.startswith("$D,"):
        return None
    parts = line.strip().split(",")[1:]
    if len(parts) != len(FIELDS):
        return None
    try:
        vals = [int(p) for p in parts]
    except ValueError:
        return None
    d = dict(zip(FIELDS, vals))
    # 定点还原
    for k in ("s", "yaw", "vref", "vL", "vR", "seg_s"):
        d[k] = d[k] / 10.0
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log", default=None)
    ap.add_argument("--task", type=int, default=None, help="连接后自动发车 1-4")
    ap.add_argument("--cmd", action="append", default=[], help="连接后发送的命令，可多次")
    ap.add_argument("--duration", type=float, default=0, help="运行秒数，0=直到 Ctrl+C")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    if args.task:
        ser.write(f"{args.task}\n".encode())
        print(f"# 已发送启动命令：{args.task}")
    for c in args.cmd:
        ser.write((c + "\n").encode())
        print(f"# 已发送：{c}")

    rows = []
    t0 = time.time()
    f = None
    writer = None
    if args.log:
        f = open(args.log, "w", newline="", encoding="utf-8-sig")
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()

    print("# t(s)  state  seg lap   s(cm)  yaw(deg)  vref  vL   vR   line(mm)  duty")
    try:
        while True:
            if args.duration and time.time() - t0 > args.duration:
                break
            raw = ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            d = parse(line)
            if not d:
                continue
            rows.append(d)
            if writer:
                writer.writerow(d)

            if not args.quiet and len(rows) % 5 == 0:
                print(f"  {d['t_ms']/1000:6.2f}  {STATE.get(d['state'],'?'):5s}"
                      f"  {d['seg']:3d} {d['lap']:2d}  {d['s']:7.2f}"
                      f"  {d['yaw']:8.2f}  {d['vref']:4.0f}"
                      f"  {d['vL']:4.0f} {d['vR']:4.0f}"
                      f"  {d['line_mm']:7d}  {d['dutyL']:4d}/{d['dutyR']:4d}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if f:
            f.close()
            print(f"# 已保存 {len(rows)} 行到 {args.log}")

    if args.plot and rows:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("需要 matplotlib 才能画图：pip install matplotlib")
            return
        t = [(r["t_ms"] - rows[0]["t_ms"]) / 1000.0 for r in rows]
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
        ax[0].plot(t, [r["yaw"] for r in rows], label="yaw")
        ax[0].set_ylabel("yaw (deg)")
        ax[0].grid(True)
        ax[0].legend()
        ax[1].plot(t, [r["line_mm"] for r in rows], label="line pos")
        ax[1].set_ylabel("line (mm)")
        ax[1].grid(True)
        ax[1].legend()
        ax[2].plot(t, [r["vref"] for r in rows], label="vref")
        ax[2].plot(t, [r["vL"] for r in rows], label="vL")
        ax[2].plot(t, [r["vR"] for r in rows], label="vR")
        ax[2].set_ylabel("speed (cm/s)")
        ax[2].set_xlabel("t (s)")
        ax[2].grid(True)
        ax[2].legend()
        out = os.path.splitext(args.log or "telemetry")[0] + ".png"
        plt.tight_layout()
        plt.savefig(out, dpi=120)
        print("# 图已保存", out)
        plt.show()


if __name__ == "__main__":
    main()
