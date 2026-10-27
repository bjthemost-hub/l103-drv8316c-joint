#!/usr/bin/env python3
"""边监听边定时查询: 把固件吐的每一行都记下来, 同时每 <poll> ms 发一条 `?`,
用来给"nFAULT 什么时候掉下来""al/en 什么时候被清"建立时间线。

用法:
    E:\\Py311\\python.exe scripts/serial_probe.py --port COM15 --poll 500 \
        --out work/probe.log --seconds 180

只用串口调试口(115200), 不碰 CAN, 不改变板上状态; 发的只有 `?` 这种只读命令。
"""
import argparse
import os
import sys
import threading
import time

import serial


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM15")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="")
    ap.add_argument("--poll", type=int, default=500, help="查询间隔 ms, 0 = 不查询")
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--cmd", default="?")
    args = ap.parse_args()

    out = args.out or os.path.join("work", "probe_%s.log" % time.strftime("%H%M%S"))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    t0 = time.monotonic()
    fh = open(out, "w", encoding="utf-8", errors="replace", newline="\n")
    fh.write("# serial_probe %s port=%s poll=%dms\n"
             % (time.strftime("%Y-%m-%d %H:%M:%S"), args.port, args.poll))
    fh.flush()
    print("PROBING %s @%d poll=%dms -> %s" % (args.port, args.baud, args.poll, out), flush=True)

    stop = threading.Event()

    def poller() -> None:
        while not stop.is_set():
            if args.poll > 0:
                ser.write((args.cmd + "\n").encode())
            if stop.wait(args.poll / 1000.0):
                return

    th = threading.Thread(target=poller, daemon=True)
    th.start()

    buf = b""
    deadline = t0 + args.seconds
    try:
        while time.monotonic() < deadline:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.rstrip(b"\r").decode("utf-8", "replace")
                stamp = "[%8.2f] " % (time.monotonic() - t0)
                fh.write(stamp + text + "\n")
                fh.flush()
                print(stamp + text, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        fh.close()
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
