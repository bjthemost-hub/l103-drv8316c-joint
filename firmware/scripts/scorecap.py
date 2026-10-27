import io, sys, re, math, statistics as st
CPR=16384.0; DT=0.002; R=2*math.pi/CPR
def load(p):
    seq=[];raw=[]
    for ln in io.open(p,encoding="utf-8",errors="replace"):
        m=re.match(r"^([0-9A-Fa-f]+),([0-9A-Fa-f]+)\s*$",ln.strip())
        if m: seq.append(int(m.group(1),16)); raw.append(int(m.group(2),16))
    acc=[0]
    for i in range(1,len(raw)):
        d=(raw[i]-raw[i-1])&0x3FFF
        if d>0x2000: d-=0x4000
        acc.append(acc[-1]+d)
    return acc
def score(p):
    acc=load(p)
    tail=acc[-2500:] if len(acc)>2500 else acc   # 后 5 s
    inc=[tail[i]-tail[i-1] for i in range(1,len(tail))]
    span=len(tail)*DT
    mean=(tail[-1]-tail[0])*R/span
    zero=100.0*sum(1 for x in inc if x==0)/len(inc)
    rev=sum(1 for x in inc if x<0)
    def w(ms):
        n=int(ms/1000.0/DT); v=[]
        for i in range(0,len(tail)-n,n//2): v.append((tail[i+n]-tail[i])*R/(n*DT))
        return v
    v100=w(100); v50=w(50)
    return dict(rpm=mean*60/(2*math.pi), zero=zero, rev=rev,
                p100=100.0*(max(v100)-min(v100))/abs(mean) if mean else 0,
                sd100=st.pstdev(v100), sd50=st.pstdev(v50),
                vmin50=min(v50), vmax50=max(v50))
for p in sys.argv[1:]:
    s=score(p)
    print("%-28s avg %6.2f rpm | 静止采样 %5.1f%% | 回退 %3d | 100ms峰峰 %7.1f%% | sd50 %5.2f sd100 %5.2f | 50ms %.1f..%.1f rad/s" % (
        p.split("\\")[-1], s["rpm"], s["zero"], s["rev"], s["p100"], s["sd50"], s["sd100"], s["vmin50"], s["vmax50"]))
