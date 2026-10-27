import io, sys, re, math
CPR=16384.0; DT=0.002; R=2*math.pi/CPR
p=sys.argv[1]
seqs=[];raws=[]
for ln in io.open(p,encoding="utf-8",errors="replace"):
    m=re.match(r"^([0-9A-Fa-f]+),([0-9A-Fa-f]+)\s*$",ln.strip())
    if m: seqs.append(int(m.group(1),16)); raws.append(int(m.group(2),16))
acc=[0]
for i in range(1,len(raws)):
    d=(raws[i]-raws[i-1])&0x3FFF
    if d>0x2000: d-=0x4000
    acc.append(acc[-1]+d)
# 每 100 ms 打印一段: 位移(cnt) 与速度
print("t(s)  dcount/100ms  rpm")
n=50
for i in range(0,len(acc)-n,n):
    d=acc[i+n]-acc[i]
    print("%5.2f  %6d      %7.2f" % (i*DT, d, d*R/(n*DT)*60/(2*math.pi)))
