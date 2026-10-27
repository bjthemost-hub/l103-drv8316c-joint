# -*- coding: utf-8 -*-
"""CAN 功能号 15/23 参数表读写自测 (L103)。

用法: python can_param_test.py
做四件事: INFO -> READ 0..15 -> WRITE 一项(回写原值) -> SAVE -> 再 READ 核对。
SAVE 前要求板子已失能 (固件会拒绝使能中的 SAVE)。
"""
import ctypes, struct, sys, time
sys.path.insert(0, 'scripts')
import can_9433 as C

NODE = 1
FN_PARAM = 15
ID_TX = (NODE << 5) | FN_PARAM
MAGIC = 0x5A
SIG = 0xA5


def frame(op, idx=0, val=0.0, magic=True, count=1):
    b = bytearray(8)
    b[0] = op
    b[1] = idx
    if op == 0x01:                      # READ: b2 = 个数
        b[2] = count
    else:
        b[2:6] = struct.pack('<f', val)
    if magic:
        b[6] = MAGIC
    return list(b)


def collect(d, dev, sec=0.35, want=None):
    rd = C.CanRecvData()
    out = []
    t0 = time.time()
    while time.time() - t0 < sec:
        r = d.CanRecvFrame(dev, ctypes.byref(rd))
        if r == 1:
            for k in range(rd.nNum):
                e = rd.Data[k]
                if e.nType != 0:
                    continue
                f = e.u.Frame
                if f.FrameId != ((NODE << 5) | 23):
                    continue
                data = bytes(f.Buf[:f.nSize])
                if len(data) < 8 or data[7] != SIG:
                    continue
                op = data[0] & 0x7F
                idx = data[1]
                val = struct.unpack('<f', data[2:6])[0]
                out.append((op, idx, val, data[6]))
                if want and len(out) >= want:
                    return out
        else:
            time.sleep(0.004)
    return out


def main():
    d = C.load()
    dev = C.open_dev(d, 1000000, 0)
    try:
        print('== INFO ==')
        C.send(d, dev, ID_TX, frame(0x00))
        r = collect(d, dev, 0.4, 1)
        print(r)
        n = int(r[0][2] + 0.5) if r else 0
        print('参数个数 =', n)

        print('== READ 0..%d ==' % (n - 1))
        C.send(d, dev, ID_TX, frame(0x01, 0, 0, True, n))
        r = collect(d, dev, 0.8, n)
        for (op, idx, val, fl) in sorted(r, key=lambda x: x[1]):
            print('  [%2d] %12.5f  flags=0x%02X' % (idx, val, fl))
        if len(r) < n:
            print('!! 只收到 %d/%d 帧' % (len(r), n))

        print('== WRITE 回写原值 (索引 5 pos_kp) ==')
        if r:
            keep = [x for x in r if x[1] == 5]
            if keep:
                old = keep[0][2]
                C.send(d, dev, ID_TX, frame(0x02, 5, old))
                print('  write 5 =', old, '->', collect(d, dev, 0.4, 1))

        print('== SAVE ==')
        C.send(d, dev, ID_TX, frame(0x03))
        print('  ', collect(d, dev, 0.8, 1))

        print('== READ 再一次 (核对还在) ==')
        C.send(d, dev, ID_TX, frame(0x01, 0, 0, True, n))
        r2 = collect(d, dev, 0.8, n)
        print('  收到', len(r2), '帧')
    finally:
        try:
            d.CanCloseDev(dev)
        except Exception as e:
            print('close err', e)
        print('CLOSED')


main()
