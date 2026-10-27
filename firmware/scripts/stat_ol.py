import io, re, sys, statistics as st
rows=[]
for ln in io.open(sys.argv[1], encoding="utf-8", errors="replace"):
    m=re.match(r"T(\d+)\t(.*)", ln.rstrip("\n"))
    if not m: continue
    for part in m.group(2).split(" | "):
        if part.startswith("ST "):
            d=dict(re.findall(r"(\w+)=(-?[\d.]+)", part)); d["t"]=int(m.group(1)); rows.append(d)
rows=[r for r in rows if r.get("mode")=="OPENLOOP" and "pos" in r]
rows=[r for r in rows if r["t"]-rows[0]["t"]>1500]
dt=(rows[-1]["t"]-rows[0]["t"])/1000.0
true=(float(rows[-1]["pos"])-float(rows[0]["pos"]))/dt
v=[float(r["vel"]) for r in rows]
print("n=%d  true=%.3f rad/s   winvel mean=%.3f  median=%.3f  std=%.3f  min=%.3f max=%.3f"
      % (len(rows), true, st.mean(v), st.median(v), st.pstdev(v), min(v), max(v)))
print("ratio mean/true = %.3f" % (st.mean(v)/true))
