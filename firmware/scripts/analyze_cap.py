import io, sys, re, statistics as st, math

CPR = 16384.0
DT  = 0.002                      # ENC_CAP_DIV=20 @10kHz -> 2 ms
RAD_PER_CNT = 2*math.pi/CPR

path = sys.argv[1]
seqs, raws = [], []
for ln in io.open(path, encoding="utf-8", errors="replace"):
    m = re.match(r"^([0-9A-Fa-f]+),([0-9A-Fa-f]+)\s*$", ln.strip())
    if not m: continue
    seqs.append(int(m.group(1),16)); raws.append(int(m.group(2),16))

# seq 是 12 位回绕
ds = []
for i in range(1,len(seqs)):
    d = (seqs[i]-seqs[i-1]) & 0xFFF
    ds.append(d)
gaps = sum(1 for d in ds if d != 1)
span = len(seqs)*DT
print("样本 %d, 名义时长 %.2fs (%.0f Hz), seq 跳变(丢样) %d" % (len(seqs), span, len(seqs)/span, gaps))

# 展开绝对计数
acc=[0]
for i in range(1,len(raws)):
    d = (raws[i]-raws[i-1]) & 0x3FFF
    if d > 0x2000: d -= 0x4000
    acc.append(acc[-1]+d)
tot = acc[-1]
print("总位移 %d cnt = %.3f 圈 = %.2f rad, 平均 %.2f rpm" % (tot, tot/CPR, tot*RAD_PER_CNT, tot*RAD_PER_CNT/span*60/(2*math.pi)))

d=np=None
inc = [acc[i]-acc[i-1] for i in range(1,len(acc))]
print("每 2ms 计数差: min %d max %d mean %.2f  |  ==0 占 %.1f%%  |  <0 占 %.1f%% (%d 次回退)" % (
    min(inc), max(inc), st.mean(inc), 100.0*sum(1 for x in inc if x==0)/len(inc),
    100.0*sum(1 for x in inc if x<0)/len(inc), sum(1 for x in inc if x<0)))
from collections import Counter
c = Counter(inc)
print("分布:", ", ".join("%d:%d" % (k,c[k]) for k in sorted(c)[:12]))

def win(n_ms):
    n = int(round(n_ms/1000.0/DT))
    out=[]
    for i in range(0, len(acc)-n, max(1,n//2)):
        out.append((acc[i+n]-acc[i])*RAD_PER_CNT/(n*DT))
    return out

for w in (10, 25, 50, 100):
    v = win(w)
    if not v: continue
    mu = st.mean(v)
    print("%3dms 窗口速度: 均值 %.3f rad/s (%.1f rpm)  min %.3f  max %.3f  峰峰 %.2f%%  std %.3f" % (
        w, mu, mu*60/(2*math.pi), min(v), max(v), 100.0*(max(v)-min(v))/abs(mu) if mu else 0, st.pstdev(v)))

# 停顿: 连续 dcount==0 超过 5 个点(10ms)
run=0; worst=0
for x in inc:
    if x==0: run+=1; worst=max(worst,run)
    else: run=0
print("最长连续零位移: %.0f ms" % (worst*DT*1000))
