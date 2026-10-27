import serial, time, sys, re

PORT, BAUD = "COM15", 115200
sec = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0

s = serial.Serial(PORT, BAUD, timeout=0.02)
time.sleep(0.15); s.reset_input_buffer()
buf = b""
t0 = time.time()
next_q = 0.0
st = {}
rows = []
peak = 0.0
prev_en = None
trip_t = None

def num(k, default=0.0):
    try: return float(st.get(k, default))
    except: return default

while time.time() - t0 < sec:
    now = time.time()
    if now >= next_q:
        s.write(b"?\r"); next_q = now + 0.05
    d = s.read(4096)
    if not d:
        continue
    buf += d
    while b"\n" in buf:
        ln, buf = buf.split(b"\n", 1)
        txt = ln.decode("utf-8", "replace").strip()
        if txt.startswith("ST "):
            for k, v in re.findall(r"(\w+)=([-\w./]+)", txt):
                st[k] = v
        elif txt.startswith("ref "):
            for k, v in re.findall(r"(\w+)=([-\w./]+)", txt):
                st[k] = v
        else:
            continue
        if "imax" not in st or "mode" not in st:
            continue
        imax = num("imax")
        if imax > peak: peak = imax
        en = st.get("en", "?")
        if prev_en == "1" and en == "0" and trip_t is None:
            trip_t = time.time() - t0
            print(">>> EN DROPPED at t=%.2fs  flt=%s  iq=%s  imax=%.3f" %
                  (trip_t, st.get("flt"), st.get("iq"), imax))
        prev_en = en
        rows.append((time.time() - t0, en, st.get("flt"), num("iq"), imax,
                     num("vel"), num("pos")))
        if len(rows) % 5 == 0:
            print("t=%5.2f en=%s flt=%-5s iq=%6.3f imax=%5.3f vel=%8.3f pos=%9.4f" %
                  rows[-1])

s.close()
print("---- summary ----")
print("samples=%d  peak_imax=%.3f A" % (len(rows), peak))
if trip_t is not None:
    win = [r for r in rows if trip_t - 0.6 <= r[0] <= trip_t + 0.05]
    print("last 0.6s before trip:")
    for r in win:
        print("   t=%5.2f en=%s flt=%-5s iq=%6.3f imax=%5.3f" % (r[0], r[1], r[2], r[3], r[4]))
else:
    print("no enable->disable edge in window")
print("final: en=%s flt=%s iq=%s imax=%s" % (st.get("en"), st.get("flt"), st.get("iq"), st.get("imax")))