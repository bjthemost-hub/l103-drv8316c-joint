import io, re, sys, statistics as st
path = sys.argv[1]
rows = []
for ln in io.open(path, encoding="utf-8", errors="replace"):
    m = re.match(r"T(\d+)\t(.*)", ln.rstrip("\n"))
    if not m: continue
    t = int(m.group(1)); body = m.group(2)
    for part in body.split(" | "):
        if part.startswith("ST "):
            mode = re.search(r"mode=(\w+)", part).group(1)
            d = dict(re.findall(r"(\w+)=(-?[\d.]+)", part)); d["mode"] = mode
            rows.append(("ST", t, d))
        elif part.strip().startswith("ref vel="):
            rows.append(("REF", t, dict(re.findall(r"(\w+)=(-?[\d.]+)", part))))
pts = []
for i in range(len(rows) - 1):
    if rows[i][0] == "ST" and rows[i+1][0] == "REF":
        a, b = rows[i], rows[i+1]
        if a[2].get("mode") != "VELOCITY": continue
        pts.append(dict(t=a[1], pos=float(a[2]["pos"]), vel=float(a[2]["vel"]),
                        iq=float(a[2]["iq"]), id=float(a[2]["id"]),
                        ref=float(b[2]["vel"]), imax=float(b[2].get("imax", "nan"))))
if not pts:
    print("no VELOCITY samples"); sys.exit()
t0 = pts[0]["t"]
steady = [p for p in pts if p["t"] - t0 >= 1200]
print("samples=%d span=%.2fs ref=%.3f" % (len(pts), (pts[-1]["t"]-pts[0]["t"])/1000.0, pts[-1]["ref"]))
dt = (steady[-1]["t"] - steady[0]["t"]) / 1000.0
print("pos %.2f -> %.2f, true avg %.3f rad/s over %.2fs" % (steady[0]["pos"], steady[-1]["pos"], (steady[-1]["pos"]-steady[0]["pos"])/dt, dt))
for k in ("vel", "iq", "id", "imax"):
    v = [p[k] for p in steady]
    print("%-4s mean %7.3f  min %7.3f  max %7.3f" % (k, st.mean(v), min(v), max(v)))
stalled = sum(1 for p in steady if abs(p["pos"] - steady[0]["pos"]) < 1e-9)
