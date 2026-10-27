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
p=sys.argv[1]; n0=int(sys.argv[2]); n1=int(sys.argv[3])
acc=load(p)[n0:n1]
inc=[acc[i]-acc[i-1] for i in range(1,len(acc))]
span=len(acc)*DT
print("样本 %d, %.2fs, 总位移 %d cnt, 平均 %.2f rpm" % (len(acc),span,acc[-1]-acc[0],(acc[-1]-acc[0])*R/span*60/(2*math.pi)))
print("2ms 计数差: min %d max %d mean %.2f, ==0 占 %.1f%%, <0 占 %.1f%%" % (min(inc),max(inc),st.mean(inc),100.0*sum(1 for x in inc if x==0)/len(inc),100.0*sum(1 for x in inc if x<0)/len(inc)))
from collections import Counter
c=Counter(inc); print("分布:", ", ".join("%d:%d"%(k,c[k]) for k in sorted(c)))
