# -*- coding: utf-8 -*-
"""生成 1024 点 int16 (Q15) 正弦表 -> User/foc_tab_q15.h

索引 = 电角度 uint16 >> 6, 即 65536 一圈 / 1024 点。
最近点取样误差 = 2*pi/1024/2 = 3.07e-3 rad = 0.176 电角度 —— 对电流环可忽略。
表长 1024 * 2 B = 2 KiB (const, 进 Flash)。
"""
import math, os, io

N = 1024
AMP = 32767

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "..", "User", "foc_tab_q15.h")
    buf = io.StringIO()
    buf.write("/* 自动生成, 不要手改! 生成器: scripts\\gen_foc_tab.py\n")
    buf.write(" * 1024 点 int16 Q15 正弦表, 索引 = (uint16 电角度) >> 6。\n")
    buf.write(" * 最近点取样最大误差 3.07e-3 rad (0.176 电角度), 对电流环可忽略。\n")
    buf.write(" * 表长 1024*2B = 2 KiB, const -> Flash。*/\n")
    buf.write("#ifndef __FOC_TAB_Q15_H\n#define __FOC_TAB_Q15_H\n\n")
    buf.write("#include <stdint.h>\n\n")
    buf.write("#define FOC_TAB_N     1024u\n")
    buf.write("#define FOC_TAB_SHIFT 6u\n\n")
    buf.write("static const int16_t foc_sin_q15[FOC_TAB_N] = {\n")
    for i in range(N):
        v = int(round(math.sin(2.0 * math.pi * i / N) * AMP))
        if v > 32767: v = 32767
        if v < -32768: v = -32768
        buf.write("%6d," % v)
        if (i + 1) % 12 == 0:
            buf.write("\n")
        else:
            buf.write(" ")
    buf.write("\n};\n\n#endif\n")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(buf.getvalue())
    print("wrote", os.path.abspath(out))

if __name__ == "__main__":
    main()
