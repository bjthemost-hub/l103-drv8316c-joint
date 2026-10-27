#!/usr/bin/env python3
"""L103 FOC 到位抖动探针 (只走串口, 不改固件).

用法:
  E:\\Py311\\python.exe scripts\\jitter_probe.py --label base --poskp 20 --spdkp 0.05 --spdki 4.0 --move 0.05

流程: 清故障 -> 使能(必要时先对齐) -> 等到 al=1 -> 可选小幅 P 移动 -> 等 1.5s
      -> 25ms 采样 ? 共 N 秒 -> 打印 pos/vel/iq 峰峰值 -> M0/D.

安全: 任一帧 flt!=0 / |vel|>SAFE_VEL / |pos-pos_cmd|>SAFE_ERR 立即停机并抛异常。
"""

import argparse
import csv
import os
import re
import sys
import time

import serial

PORT = "COM15"
BAUD = 115200

ST_RE = re.compile(
    r"ST mode=(\w+) en=(\d) al=(\d) flt=(\d+)/(\d+) "
    r"pos=([-+0-9.]+) vel=([-+0-9.]+) iq=([-+0-9.]+) id=([-+0-9.]+) th=([-+0-9.]+)"
)
REF_RE = re.compile(
    r"ref vel=([-+0-9.]+) iq=([-+0-9.]+) vd=([-+0-9.]+) vq=([-+0-9.]+) "
    r"idfw=([-+0-9.]+) imax=([-+0-9.]+) vbus=([-+0-9.]+)"
)

SAFE_VEL = 5.0
SAFE_ERR = 0.20
ECHO_RE = re.compile(r"OK (?:pos_cmd|spd kp|poskp|cur kp)=([-+0-9.]+)")


class Bench:
    def __init__(self, port=PORT):
        self.s = serial.Serial(port, BAUD, timeout=0.02)
        self.s.reset_input_buffer()

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass

    def send(self, line, wait=0.08):
        self.s.reset_input_buffer()
        self.s.write((line + "\n").encode("ascii"))
        time.sleep(wait)
        return self.s.read_all().decode("ascii", "replace")

    def status(self):
        self.s.reset_input_buffer()
        self.s.write(b"?\n")
        buf = ""
        deadline = time.time() + 0.12
        while time.time() < deadline:
            line = self.s.readline().decode("ascii", "replace")
            if line:
                buf += line
            m = ST_RE.search(buf)
            if m and REF_RE.search(buf):
                break
        m = ST_RE.search(buf)
        if not m:
            return None
        r = REF_RE.search(buf)
        return {
            "mode": m.group(1),
            "en": int(m.group(2)),
            "al": int(m.group(3)),
            "flt": int(m.group(4)),
            "fcode": int(m.group(5)),
            "pos": float(m.group(6)),
            "vel": float(m.group(7)),
            "iq": float(m.group(8)),
            "id": float(m.group(9)),
            "th": float(m.group(10)),
            "ref_vel": float(r.group(1)) if r else float("nan"),
            "ref_iq": float(r.group(2)) if r else float("nan"),
            "vbus": float(r.group(7)) if r else float("nan"),
        }

    def diag(self):
        return self.send("Q", 0.12)


def wait_aligned(b, timeout=5.0):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        st = b.status()
        if st:
            last = st
            if st["flt"]:
                raise RuntimeError(f"fault while aligning: flt={st['flt']}/{st['fcode']}")
            if st["al"] == 1 and st["mode"] in ("POSITION", "MOT_POSITION"):
                return st
        time.sleep(0.03)
    raise RuntimeError(f"align timeout, last={last}")


def fmt(v, n=5):
    return f"{v:+.{n}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="probe")
    ap.add_argument("--poskp", type=float, default=None)
    ap.add_argument("--spdkp", type=float, default=None)
    ap.add_argument("--spdki", type=float, default=None)
    ap.add_argument("--curkp", type=float, default=None)
    ap.add_argument("--curki", type=float, default=None)
    ap.add_argument("--move", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--outdir", default=os.path.join(os.path.dirname(os.path.dirname(__file__)), "work"))
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    csv_path = os.path.join(args.outdir, f"jitter_{args.label}.csv")
    b = Bench()
    rows = []
    try:
        b.send("D", 0.10)
        b.send("M0", 0.10)
        b.send("X", 0.10)
        if args.poskp is not None:
            print("KP:", b.send(f"KP{args.poskp:g}").strip())
        if args.spdkp is not None or args.spdki is not None:
            print("S :", b.send(f"S{args.spdkp:g},{args.spdki:g}").strip())
        if args.curkp is not None or args.curki is not None:
            print("C :", b.send(f"C{args.curkp:g},{args.curki:g}").strip())

        print("align/enable:", b.send("E", 0.10).strip())
        st = wait_aligned(b)
        print("aligned:", st)

        base = st["pos"]
        target = base
        if abs(args.move) > 1e-9:
            target = base + args.move
            for attempt in range(3):
                resp = b.send(f"P{target:.6f}", 0.10)
                m = ECHO_RE.search(resp)
                if m and abs(float(m.group(1)) - target) < 0.002:
                    print("P:", resp.strip())
                    break
                print(f"P retry {attempt + 1}: {resp.strip()!r}")
            else:
                raise RuntimeError(f"P command not echoed correctly (target={target:.6f})")
            time.sleep(1.5)
            st = b.status()
            print("settled:", st)

        t0 = time.time()
        next_t = t0
        while time.time() - t0 < args.seconds:
            now = time.time()
            if now < next_t:
                time.sleep(min(next_t - now, 0.003))
                continue
            st = b.status()
            next_t += 0.025
            if not st:
                continue
            t = time.time() - t0
            st["dt"] = t
            st["err"] = target - st["pos"]
            rows.append(st)
            if st["flt"]:
                raise RuntimeError(f"FAULT at t={t:.3f}: flt={st['flt']}/{st['fcode']}")
            if abs(st["vel"]) > SAFE_VEL:
                raise RuntimeError(f"VEL LIMIT at t={t:.3f}: vel={st['vel']:.3f}")
            if abs(st["err"]) > SAFE_ERR:
                raise RuntimeError(f"POS LIMIT at t={t:.3f}: err={st['err']:.4f}")

        print("Q after:", b.diag().replace("\r", "").strip())
        if rows:
            key = ["pos", "vel", "iq", "id", "ref_vel", "ref_iq", "err"]
            print(f"N={len(rows)}  target={target:+.6f}")
            for k in key:
                vals = [r[k] for r in rows]
                mn, mx = min(vals), max(vals)
                print(f"  {k:7s} min={mn:+.6f} max={mx:+.6f} pp={mx-mn:.6f}")
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=["dt"] + key + ["mode", "en", "al", "flt", "fcode", "vbus"])
                w.writeheader()
                for r in rows:
                    w.writerow({k: r.get(k, "") for k in w.fieldnames})
            print("CSV:", csv_path)
        return 0
    finally:
        try:
            b.send("M0", 0.08)
            b.send("D", 0.08)
        except Exception:
            pass
        b.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print("ABORT:", e, file=sys.stderr)
        sys.exit(2)
