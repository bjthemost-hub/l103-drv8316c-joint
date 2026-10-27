#!/usr/bin/env python3
"""复位 MCU 并抓 COM15 的上电 banner/前几秒输出."""

import argparse
import subprocess
import time

import serial

OPENOCD = r"C:\MounRiver\MounRiver_Studio3\resources\app\resources\win32\components\WCH\OpenOCD\OpenOCD\bin\openocd.exe"
SCRIPTS = r"C:\MounRiver\MounRiver_Studio3\resources\app\resources\win32\components\WCH\OpenOCD\OpenOCD\bin"
CFG = r"debug\openocd.cfg"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM15")
    ap.add_argument("--seconds", type=float, default=4.0)
    args = ap.parse_args()

    s = serial.Serial(args.port, 115200, timeout=0.1)
    s.reset_input_buffer()
    subprocess.run(
        [OPENOCD, "-f", CFG, "-s", SCRIPTS, "-c", "init", "-c", "reset run", "-c", "exit"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        check=False,
    )
    t0 = time.time()
    data = bytearray()
    while time.time() - t0 < args.seconds:
        data.extend(s.read(4096))
    s.close()
    print(data.decode("ascii", "replace"))


if __name__ == "__main__":
    main()
