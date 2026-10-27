#!/usr/bin/env python3
"""长期监听串口, 每行前加相对时间戳, 同时写进日志文件。

用法:
    E:\\Py311\\python.exe scripts/serial_log.py --port COM15 --baud 115200 --out work/session.log

行首的 [  12.34] 是"打开串口后过了多少秒", 用来对时序(上电/banner/命令/动作)。
Ctrl+C 结束。日志每行都 flush, 中途被杀也不丢。
"""
import argparse
import os
import sys
import time

import serial


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM15")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="")
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    out = args.out or os.path.join("work", "serial_%s.log" % time.strftime("%H%M%S"))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as exc:                       # 口被占用 / 不存在
        print("OPEN FAILED %s: %s" % (args.port, exc), flush=True)
        return 2

    t0 = time.monotonic()
    buf = b""
    fh = open(out, "w", encoding="utf-8", errors="replace", newline="\n")
    fh.write("# serial_log %s port=%s baud=%d %s\n"
             % (time.strftime("%Y-%m-%d %H:%M:%S"), args.port, args.baud, args.label))
    fh.flush()
    print("LISTENING %s @%d -> %s" % (args.port, args.baud, out), flush=True)

    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.rstrip(b"\r").decode("utf-8", "replace")
                stamp = "[%7.2f] " % (time.monotonic() - t0)
                fh.write(stamp + text + "\n")
                fh.flush()
                print(stamp + text, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        fh.close()
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
