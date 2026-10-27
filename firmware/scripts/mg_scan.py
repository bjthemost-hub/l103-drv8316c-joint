#!/usr/bin/env python3
"""慢速开环扫一圈, 记录 MT6701 的 Mg[3:0] 磁场状态和原始角.

用途: 判断 2~3 mm 气隙下, 磁场是否在全周都正常; 以及偏心/弱场是否随角度变化.
安全: 2 V / 1 Hz 电频率 (机械约 5.45 rpm), 发现不转/过流/故障立即 M0+D.
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
MG_RE = re.compile(r"MG raw=(\d+) mg=0x([0-9A-Fa-f]+) field=(\w+) push=(\d) overspeed=(\d)")
ST_RE = re.compile(r"ST mode=(\w+) en=(\d) al=(\d) flt=(\d+)/(\d+) pos=([-+0-9.]+) vel=([-+0-9.]+) iq=([-+0-9.]+)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--volts", type=float, default=2.0)
    ap.add_argument("--hz", type=float, default=1.0)
    ap.add_argument("--seconds", type=float, default=24.0)
    ap.add_argument("--period", type=float, default=0.10)
    ap.add_argument("--outdir", default=os.path.join(os.path.dirname(os.path.dirname(__file__)), "work"))
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    path = os.path.join(args.outdir, f"mg_scan_{args.volts:g}V_{args.hz:g}Hz.csv")
    s = serial.Serial(PORT, BAUD, timeout=0.02)
    rows = []

    def cmd(line, wait=0.06):
        s.reset_input_buffer()
        s.write((line + "\n").encode("ascii"))
        time.sleep(wait)
        return s.read_all().decode("ascii", "replace")

    try:
        print(cmd("D", 0.10).strip())
        print(cmd("M0", 0.10).strip())
        print(cmd("X", 0.10).strip())
        print(cmd(f"R{args.volts:g},{args.hz:g}", 0.15).strip())

        t0 = time.time()
        last_raw = None
        still = 0
        while time.time() - t0 < args.seconds:
            t = time.time() - t0
            s.reset_input_buffer()
            s.write(b"MG\n?\n")
            time.sleep(0.055)
            text = s.read_all().decode("ascii", "replace")
            mg = MG_RE.search(text)
            st = ST_RE.search(text)
            if not mg or not st:
                time.sleep(max(0.0, args.period - 0.055))
                continue
            raw = int(mg.group(1))
            mgv = int(mg.group(2), 16)
            imax = None
            m = re.search(r"imax=([-+0-9.]+)", text)
            if m:
                imax = float(m.group(1))
            row = {
                "t": round(t, 4),
                "raw": raw,
                "mg": mgv,
                "field": mg.group(3),
                "push": int(mg.group(4)),
                "overspeed": int(mg.group(5)),
                "mode": st.group(1),
                "flt": int(st.group(4)),
                "fcode": int(st.group(5)),
                "pos": float(st.group(6)),
                "vel": float(st.group(7)),
                "iq": float(st.group(8)),
                "imax": imax,
            }
            rows.append(row)

            if row["flt"]:
                raise RuntimeError(f"FAULT flt={row['flt']}/{row['fcode']} at t={t:.2f}")
            if imax is not None and imax > 2.0:
                raise RuntimeError(f"OVERCURRENT imax={imax:.2f} A at t={t:.2f}")
            if last_raw is not None and raw == last_raw:
                still += 1
                if still >= 10:  # 约 1 s 没动
                    raise RuntimeError(f"NO MOTION raw={raw} for ~1s at t={t:.2f}")
            else:
                still = 0
            last_raw = raw
            time.sleep(max(0.0, args.period - 0.055))

        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["t"])
            w.writeheader()
            w.writerows(rows)

        print(f"N={len(rows)} duration={args.seconds}s -> {path}")
        if rows:
            fields = {}
            for r in rows:
                fields[r["field"]] = fields.get(r["field"], 0) + 1
            print("field counts:", fields)
            print("raw min/max:", min(r["raw"] for r in rows), max(r["raw"] for r in rows))
            print("mg values:", sorted(set(r["mg"] for r in rows)))
        return 0
    finally:
        try:
            print(cmd("M0", 0.08).strip())
            print(cmd("D", 0.08).strip())
        except Exception:
            pass
        s.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print("ABORT:", e, file=sys.stderr)
        sys.exit(2)
