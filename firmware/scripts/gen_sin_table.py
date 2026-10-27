import math, io, os
N = 256
lines = []
lines.append("/* 自动生成, 不要手改!  生成器: scripts\\gen_sin_table.py")
lines.append(" * 256 点正弦表, 覆盖 [0, 2*pi), 线性插值。")
lines.append(" * 线性插值最大误差 = (2*pi/256)^2/8 = %.3e rad (算术值)" % ((2*math.pi/N)**2/8))
lines.append(" * 覆盖到 N+1 项, 末项重复首项, 省掉查表时的回绕判断。 */")
lines.append("#ifndef __FOC_MATH_TAB_H")
lines.append("#define __FOC_MATH_TAB_H")
lines.append("")
lines.append("#define SIN_TAB_N   %du" % N)
lines.append("static const float sin_tab[SIN_TAB_N + 1u] = {")
for i in range(N + 1):
    v = math.sin(2.0 * math.pi * (i % N) / N)
    lines.append("    %+.9ef," % v)
lines.append("};")
lines.append("")
lines.append("/* 1.0f / (2*pi) —— 角度 -> 表索引 的乘法倒数 */")
lines.append("#define SIN_TAB_INV_TWO_PI   %.9ef" % (1.0/(2.0*math.pi)))
lines.append("")
lines.append("#endif")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "User", "foc_math_tab.h")
open(OUT,"w",encoding="utf-8",newline="\n").write("\n".join(lines)+"\n")
print("ok", len(lines))
