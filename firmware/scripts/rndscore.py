import io,sys,re,math,glob,statistics as st
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
    n=int(ms/1000.0/DT)
    if len(acc)<=n: return []
    return [(acc[i+n]-acc[i])*R/(n*DT) for i in range(0,len(acc)-n,max(1,n//2))]
print("%-24s %7s %8s %7s %6s %9s %8s %8s" % ("cmd","实测rpm","时长","零位移","回退","100ms sd","100ms峰峰","50ms sd"))
for p in sorted(glob.glob(sys.argv[1])):
    acc=load(p)
    if len(acc)<50: continue
    span=len(acc)*DT; mean=(acc[-1]-acc[0])*R/span
    inc=[acc[i]-acc[i-1] for i in range(1,len(acc))]
    z=100.0*sum(1 for x in inc if x==0)/len(inc); rv=sum(1 for x in inc if x<0)
    v100=winv(acc,100); v50=winv(acc,50)
    sd100=st.pstdev(v100) if len(v100)>2 else float('nan')
    pp100=(max(v100)-min(v100)) if v100 else float('nan')
    sd50=st.pstdev(v50) if len(v50)>2 else float('nan')
    m=re.search(r"_v([\d.]+)_",p)
    cmd=float(m.group(1)) if m else 0
    print("%-24s %7.1f %7.2fs %6.1f%% %6d %7.1f%% %7.1f%% %8.1f%%" % (
        p.split("\\")[-1].replace(".log",""), mean*60/(2*math.pi), span, z, rv,
        100*sd100/abs(mean), 100*pp100/abs(mean), 100*sd50/abs(mean)))
