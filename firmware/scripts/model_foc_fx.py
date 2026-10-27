# -*- coding: utf-8 -*-
"""Bit-exact model of foc_fx.h scaling. ASCII output on purpose."""
import math

def i32(x):
    x = int(x) & 0xFFFFFFFF
    return x - 0x100000000 if x >= 0x80000000 else x

N = 1024
AMP = 32767
TAB = [max(-32768, min(32767, int(round(math.sin(2*math.pi*i/N)*AMP)))) for i in range(N)]

def fx_sin(a): return TAB[(a & 0xFFFF) >> 6]
def fx_cos(a): return TAB[(((a & 0xFFFF) + 16384) & 0xFFFF) >> 6]

INV_SQRT3_Q15, SQRT3_2_Q15 = 18919, 28378

def park(al, be, s, c): return i32((al*c + be*s) >> 15), i32((be*c - al*s) >> 15)
def invpark(d, q, s, c): return i32((d*c - q*s) >> 15), i32((d*s + q*c) >> 15)

# 1. sincos
worst = (0.0, 0)
for k in range(0, 65536, 7):
    th = 2*math.pi*k/65536
    e = max(abs(fx_sin(k)/32768.0 - math.sin(th)), abs(fx_cos(k)/32768.0 - math.cos(th)))
    if e > worst[0]: worst = (e, k)
print("1) sincos max err = %.6f at k=%d (half-step bound %.6f)" % (worst[0], worst[1], math.pi/N))

# 2. park roundtrip
w = 0
for k in range(0, 65536, 997):
    s, c = fx_sin(k), fx_cos(k)
    for al, be in ((1000,0),(0,1000),(700,-700),(-16384,0),(19116,0)):
        d, q = park(al, be, s, c)
        a2, b2 = invpark(d, q, s, c)
        w = max(w, abs(a2-al), abs(b2-be))
print("2) Park roundtrip max err = %d LSB12 = %.3f mA" % (w, w*0.244))

# 3. SVPWM: rebuild line voltages from duty, Clarke back, compare to command
VBUS = 24.0
vbus_q12 = int(VBUS*4096)
arr = 96_000_000//2//20000 - 1
half = (arr+1)//2
kccr = (half << 16)//vbus_q12
print("3) ARR=%d half=%d kccr=%d" % (arr, half, kccr))

def svm(al, be):
    va = al
    vb = i32((-al)//2 + ((SQRT3_2_Q15*be) >> 15))
    vc = i32((-al)//2 - ((SQRT3_2_Q15*be) >> 15))
    vmax, vmin = max(va,vb,vc), min(va,vb,vc)
    voff = -i32((vmax+vmin)//2)
    out = []
    for v in (va,vb,vc):
        cc = half + i32(((v+voff)*kccr) >> 16)
        out.append(max(0, min(arr, cc)))
    return out

worst = 0.0
for k in range(0, 65536, 331):
    s, c = fx_sin(k), fx_cos(k)
    for vd, vq in ((0,12288),(3000,0),(-4000,5000),(9000,-9000)):
        al, be = invpark(vd, vq, s, c)
        cc = svm(al, be)
        # duty -> phase voltage (relative to dc midpoint), then Clarke
        pa = (cc[0]-half)/half*VBUS/2
        pb = (cc[1]-half)/half*VBUS/2
        pc = (cc[2]-half)/half*VBUS/2
        A = pa
        B = (pb-pc)/math.sqrt(3)
        worst = max(worst, abs(A-al/4096.0), abs(B-be/4096.0))
print("    SVPWM -> line volt -> Clarke, max err = %.4f V" % worst)

# 4. saturation scaling
vlim = int(0.95*VBUS/math.sqrt(3)*4096)
d8, q8 = vlim>>4, vlim>>4
m2 = d8*d8+q8*q8
m = math.isqrt(m2)
print("4) vlim=%.2f V  m2=%d  isqrt=%d  check=%.2f V" % (vlim/4096, m2, m, (vlim*m//m)/4096))

# 5. current scaling
a_lsb = 3.3/4095/0.3
print("5) ADC->A = %.6f A/count ; *4096 = %.2f -> use 11 (err %.2f%%)" % (a_lsb, a_lsb*4096, (11-a_lsb*4096)/(a_lsb*4096)*100))
print("   full scale +-%.3f A ; OC 4 A = %d LSB12" % (2048*a_lsb, 4*4096))
