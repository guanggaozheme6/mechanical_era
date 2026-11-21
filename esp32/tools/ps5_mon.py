#!/usr/bin/env python3
"""ps5_mon.py — PS5 DualSense 终端实时面板（读 HID report 0x01）。"""
import sys
import time

import hid

VID = 0x054C
PID = 0x0CE6
ESC = "\033"
HOME = f"{ESC}[H"
GREEN = f"{ESC}[32m"
DIM = f"{ESC}[2m"
RESET = f"{ESC}[0m"

BTN0 = ["Sq", "Cr", "Ci", "Tr", "L1", "R1", "L2", "R2"]
BTN1 = ["Cre", "Opt", "L3", "R3", "PS", "TP", "Mute"]


def bar(v, width=21):
    p = int(v * (width - 1) / 255.0 + 0.5)
    p = max(0, min(width - 1, p))
    mid = width // 2
    out = []
    for i in range(width):
        if i == p:
            out.append(GREEN + "@" + RESET)
        elif i == mid:
            out.append(DIM + "|" + RESET)
        else:
            out.append(".")
    return "".join(out)


def keys(b, names):
    out = []
    for i, n in enumerate(names):
        pressed = ((b >> i) & 1) == 0  # 低有效
        out.append(GREEN + n + RESET if pressed else DIM + n + RESET)
    return " ".join(out)


def find():
    for d in hid.enumerate(VID, PID):
        if d.get("usage_page") == 1 and d.get("usage") == 5:
            return d
    return None


def main():
    d = find()
    if d is None:
        print("没有找到 DualSense (054C:0CE6) Game Pad 接口")
        return 1
    dev = hid.device()
    dev.open_path(d["path"])
    print(f"{ESC}[2J", end="")
    t0 = time.time()
    n = 0
    rate = 0.0
    try:
        while True:
            data = dev.read(96, 200)
            if not data:
                continue
            b = bytes(data)
            if len(b) < 10:
                continue
            n += 1
            now = time.time()
            if now - t0 >= 1.0:
                rate = n / (now - t0)
                n = 0
                t0 = now
            lx, ly, rx, ry = b[1], b[2], b[3], b[4]
            l2, r2 = b[5], b[6]
            b0, b1 = b[7], b[8]

            sb = []
            sb.append(HOME)
            sb.append("+" + "-" * 52 + "+\n")
            sb.append(f"| DualSense  {rate:5.1f} pkt/s   report 0x{b[0]:02X} len {len(b):2d}    |\n")
            sb.append("+" + "-" * 52 + "+\n")
            sb.append(f"| LX {bar(lx)} {lx:3d}        |\n")
            sb.append(f"| LY {bar(ly)} {ly:3d}        |\n")
            sb.append(f"| RX {bar(rx)} {rx:3d}        |\n")
            sb.append(f"| RY {bar(ry)} {ry:3d}        |\n")
            sb.append("+" + "-" * 52 + "+\n")
            sb.append(f"| L2 [{l2:3d}] R2 [{r2:3d}]                                |\n")
            sb.append("| " + keys(b0, BTN0).ljust(48) + " |\n")
            sb.append("| " + keys(b1, BTN1).ljust(48) + " |\n")
            sb.append("+" + "-" * 52 + "+\n")
            sb.append(f"  raw: {b[:16].hex(' ')}\n")
            sb.append("  Ctrl+C to quit\n")
            sys.stdout.write("".join(sb))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        dev.close()


if __name__ == "__main__":
    raise SystemExit(main())
