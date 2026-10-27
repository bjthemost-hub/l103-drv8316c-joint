import io,sys,re,math,statistics as st
CPR=16384.0; DT=0.002; R=2*math.pi/CPR
def load(p):
    raw=[]
    for ln in io.open(p,encoding="utf-8",errors="replace"):
        m=re.match(r"^([0-9A-Fa-f]+),([0-9A-Fa-f]+)\s*$",ln.strip())
        if m: raw.append(int(m.group(2),16))
    acc=[0]
    for i in range(1,len(raw)):
        d=(raw[i]-raw[i-1])&0x3FFF
        if d>0x2000: d-=0x4000
        acc.append(acc[-1]+d)
    return acc
def winv(acc,ms):
    n=int(ms/1000.0/DT); out=[]
    for i in range(0,len(acc)-n,max(1,n//2)):
        out.append((acc[i+n]-acc[i])*R/(n*DT))
    return out
def domfreq(v,ms):
    n=len(v)
    if n<16: return 0,0
    mu=st.mean(v); x=[a-mu for a in v]
    best=(0,0)
    for k in range(1,min(n//2,60)):
        c=sum(x[i]*math.cos(2*math.pi*k*i/n) for i in range(n))
        s=sum(x[i]*math.sin(2*math.pi*k*i/n) for i in range(n))
        amp=math.hypot(c,s)*2/n
        if amp>best[1]: best=(k/(n*DT),amp)
    return best
for p in sys.argv[1:]:
    acc=load(p)
    inc=[acc[i]-acc[i-1] for i in range(1,len(acc))]
    span=len(acc)*DT; mean=(acc[-1]-acc[0])*R/span
    v50=winv(acc,50); v100=winv(acc,100); v25=winv(acc,25)
    f,a=domfreq(v50,50)
    print("=== %s" % p.split("\\")[-1])
    print("  平均 %7.3f rad/s (%.1f rpm), 时长 %.2fs" % (mean, mean*60/(2*math.pi), span))
    for nm,v in (("25ms",v25),("50ms",v50),("100ms",v100)):
        pp=max(v)-min(v); sd=st.pstdev(v)
        print("  %5s: sd %.4f rad/s (%.1f%%), 峰峰 %.4f rad/s (%.1f%%), 范围 %.3f..%.3f" % (
            nm, sd, 100*sd/abs(mean), pp, 100*pp/abs(mean), min(v), max(v)))
    print("  主导脉动频率 %.1f Hz (幅度 %.3f rad/s), 每转脉动 %.1f 次" % (f, a, f/(mean/(2*math.pi)) if mean else 0))
