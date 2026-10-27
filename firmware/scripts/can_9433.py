import ctypes, os, sys, time, argparse

DLL = os.environ.get("USBCAN_DLL", r"C:\path\to\USBCANx64.dll")

STATE_NAMES = {0x01:"TXMB0", 0x02:"TXMB1", 0x04:"TXMB2", 0x08:"RXMB0_FULL",
               0x10:"RX0_OVERFLOW", 0x20:"RXMB1_FULL", 0x40:"RX1_OVERFLOW",
               0x80:"ERROR_SUMMARY"}
ERR_NAMES = {0:"NO_ERROR", 1:"BUS_OFF", 2:"TX_PASSIVE", 3:"RX_PASSIVE",
             4:"TX_WARNING", 5:"RX_WARNING"}

class CanCfg(ctypes.Structure):
    _fields_ = [("nWorkMode", ctypes.c_uint32), ("bOneShot", ctypes.c_int32), ("nBaudRate", ctypes.c_uint32)]
class _Frame(ctypes.Structure):
    _fields_ = [("nSize", ctypes.c_ubyte), ("FrameId", ctypes.c_uint32), ("Buf", ctypes.c_ubyte*8),
                ("bExtend", ctypes.c_int32), ("bRemote", ctypes.c_int32)]
class _U(ctypes.Union):
    _fields_ = [("nStateFlag", ctypes.c_ubyte), ("Frame", _Frame)]
class _D(ctypes.Structure):
    _fields_ = [("nType", ctypes.c_ubyte), ("u", _U)]
class CanRecvData(ctypes.Structure):
    _fields_ = [("nNum", ctypes.c_uint32), ("Data", _D*4)]

def load():
    d = ctypes.WinDLL(DLL)
    d.DLLInit.restype = None
    d.CanGetDevName.restype = ctypes.c_char_p
    d.CanGetDevName.argtypes = [ctypes.c_uint32]
    d.CanOpenDev.restype = ctypes.c_void_p
    d.CanOpenDev.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.POINTER(CanCfg)]
    d.CanSendFrame.restype = ctypes.c_int
    d.CanSendFrame.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_ubyte, ctypes.c_uint32, ctypes.c_int, ctypes.c_int]
    d.CanRecvFrame.restype = ctypes.c_int
    d.CanRecvFrame.argtypes = [ctypes.c_uint32, ctypes.POINTER(CanRecvData)]
    d.CanRecovery.restype = ctypes.c_int
    d.CanRecovery.argtypes = [ctypes.c_uint32, ctypes.c_ubyte, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]
    d.CanReset.restype = ctypes.c_int
    d.CanReset.argtypes = [ctypes.c_uint32]
    d.CanCloseDev.argtypes = [ctypes.c_uint32]
    return d

def open_dev(d, baud, mode, oneshot=0, do_reset=0, do_recover=0):
    d.DLLInit()
    idx = None
    for i in range(16):
        n = d.CanGetDevName(i)
        if not n:
            break
        if idx is None:
            idx = i
    if idx is None:
        print("NO DEVICE FOUND"); sys.exit(1)
    cfg = CanCfg(mode, oneshot, baud)
    h = d.CanOpenDev(idx, 1000, ctypes.byref(cfg))
    if not h or h == 0xFFFFFFFFFFFFFFFF:
        print("OPEN FAILED (USBCanDemo might own MI_00)"); sys.exit(2)
    if do_reset:
        print("CanReset ->", d.CanReset(idx))
    if do_recover:
        code = ctypes.c_uint32(0)
        ok = d.CanRecovery(idx, 0x80, 1, ctypes.byref(code))
        print("CanRecovery(0x80,restart) -> ok=%d code=%d (%s)" % (ok, code.value, ERR_NAMES.get(code.value, "?")))
    return idx

def describe_state(flag):
    parts = [v for k, v in STATE_NAMES.items() if flag & k]
    return "0x%02X %s" % (flag, "+".join(parts) if parts else "?")

def send(d, idx, can_id, data, extend=0, remote=0):
    buf = (ctypes.c_ubyte*8)()
    for i, b in enumerate(data): buf[i] = b
    t0 = time.time()
    r = d.CanSendFrame(idx, buf, len(data), can_id, extend, remote)
    dt = (time.time()-t0)*1000
    verdict = {1: "OK(queued)", 0: "TIMEOUT", -1: "FAIL"}.get(r, "r=%d" % r)
    print("TX id=0x%03X dlc=%d data=%s -> %s (%.1f ms)" % (
        can_id, len(data), " ".join("%02X" % b for b in data), verdict, dt))
    return r

def drain(d, idx, sec, id_filter=None, quiet=False):
    rd = CanRecvData(); t0 = time.time(); got = 0; states = []
    while time.time() - t0 < sec:
        r = d.CanRecvFrame(idx, ctypes.byref(rd))
        if r == 1:
            for k in range(rd.nNum):
                e = rd.Data[k]
                if e.nType == 0:
                    f = e.u.Frame
                    if id_filter is not None and f.FrameId != id_filter:
                        continue
                    got += 1
                    if not quiet:
                        print("%8.3f RX id=0x%03X dlc=%d data=%s%s" % (
                            time.time()-t0, f.FrameId, f.nSize,
                            " ".join("%02X" % b for b in f.Buf[:f.nSize]),
                            " EXT" if f.bExtend else ""))
                else:
                    states.append((time.time()-t0, e.u.nStateFlag))
                    if not quiet:
                        print("%8.3f STATUS %s" % (time.time()-t0, describe_state(e.u.nStateFlag)))
        elif r < 0:
            print("recv err", r); break
        else:
            time.sleep(0.005)
    return got, states

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["listen", "send", "probe", "loopback", "busstate"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--sec", type=float, default=2.0)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--mode", type=int, default=0)
    ap.add_argument("--oneshot", type=int, default=0)
    ap.add_argument("--reset", type=int, default=0)
    ap.add_argument("--recover", type=int, default=0)
    ap.add_argument("--uart", default="COM15")
    ap.add_argument("--ubaud", type=int, default=115200)
    ap.add_argument("--filter", default=None)
    a = ap.parse_args()
    filt = int(a.filter, 0) if a.filter else None
    d = load(); idx = open_dev(d, a.baud, a.mode, a.oneshot, a.reset, a.recover)
    try:
        if a.cmd == "listen":
            if a.args: a.sec = float(a.args[0])
            print("listening %.1fs @ %d bit/s mode=%d" % (a.sec, a.baud, a.mode))
            print("frames:", drain(d, idx, a.sec, filt)[0])
        elif a.cmd == "send":
            can_id = int(a.args[0], 0)
            data = [int(x, 16) for x in a.args[1:]] or [0]
            send(d, idx, can_id, data)
            if a.sec > 0:
                print("frames:", drain(d, idx, a.sec, filt)[0])
        elif a.cmd == "loopback":
            print("== loopback (mode=1) @ %d bit/s ==" % a.baud)
            pat = [0x11, 0x22, 0x33, 0x44]
            send(d, idx, 0x123, pat)
            got, states = drain(d, idx, 1.5, 0x123)
            print("frames: %d  states: %s" % (got, [describe_state(s) for _, s in states] or "none"))
            print("VERDICT:", "LOOPBACK OK" if got else "LOOPBACK FAILED")
        elif a.cmd == "busstate":
            print("== bus state probe (normal mode) ==")
            send(d, idx, 0x7FF, [0x01])
            got, states = drain(d, idx, 2.5, None)
            for _, s in states:
                if s & 0x80:
                    code = ctypes.c_uint32(0)
                    ok = d.CanRecovery(idx, s, 1, ctypes.byref(code))
                    print("CanRecovery -> ok=%d code=%d (%s)" % (ok, code.value, ERR_NAMES.get(code.value, "?")))
            print("frames: %d" % got)
        else:
            import serial
            s = serial.Serial(a.uart, a.ubaud, timeout=0.02)
            time.sleep(0.15); s.reset_input_buffer()
            print("UART> %s" % a.args[0])
            s.write((a.args[0] + "\r").encode("ascii")); s.flush()
            rd = CanRecvData(); t0 = time.time(); ncan = 0; ubuf = b""
            while time.time() - t0 < a.sec:
                r = d.CanRecvFrame(idx, ctypes.byref(rd))
                if r == 1:
                    for k in range(rd.nNum):
                        e = rd.Data[k]
                        if e.nType == 0:
                            f = e.u.Frame
                            if filt is not None and f.FrameId != filt: continue
                            print("%8.3f RX id=0x%03X dlc=%d data=%s" % (
                                time.time()-t0, f.FrameId, f.nSize,
                                " ".join("%02X" % b for b in f.Buf[:f.nSize])))
                            ncan += 1
                        else:
                            print("%8.3f STATUS %s" % (time.time()-t0, describe_state(e.u.nStateFlag)))
                else:
                    time.sleep(0.005)
                chunk = s.read(256)
                if chunk:
                    ubuf += chunk
                    while b"\n" in ubuf:
                        ln, ubuf = ubuf.split(b"\n", 1)
                        print("%8.3f UART< %s" % (time.time()-t0, ln.decode("utf-8","replace").strip()))
            s.close()
            print("CAN frames: %d" % ncan)
    finally:
        d.CanCloseDev(idx)
        print("closed")

if __name__ == "__main__":
    main()
