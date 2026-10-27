#include "foc_math.h"
#include "foc_math_tab.h"

/* -----------------------------------------------------------------------------
 * 正弦: 256 点表 + 线性插值。输入 t = 角度/2pi (圈), 只接受 [0,1)
 * 线性插值误差 ~7.5e-5 rad 级 (见 foc_math_tab.h 抬头), 对电流环完全够。
 * -------------------------------------------------------------------------- */
static inline float sin_turn(float t)
{
    float    x = t * (float)SIN_TAB_N;
    uint32_t i = (uint32_t)x;
    float    f;

    if (i >= SIN_TAB_N) { i = SIN_TAB_N - 1u; x = (float)i + 0.999f; }
    f = x - (float)i;
    return sin_tab[i] + (sin_tab[i + 1u] - sin_tab[i]) * f;
}

/* 归一到 [0,2pi)。调用方保证 |a| < 1e6 (本工程里是增量累加后再调用, 天然满足) */
float fm_wrap_2pi(float a)
{
    float n;

    if (a >= 0.0f && a < FM_TWO_PI) return a;
    if (a <  0.0f && a > -FM_TWO_PI) return a + FM_TWO_PI;

    if (a <= -1.0e6f || a >= 1.0e6f) return 0.0f;      /* 兜底, 不产生 UB */

    n = (float)(int)(a * SIN_TAB_INV_TWO_PI);           /* 向 0 截断 */
    a -= n * FM_TWO_PI;
    if (a < 0.0f)        a += FM_TWO_PI;
    if (a >= FM_TWO_PI)  a -= FM_TWO_PI;
    return a;
}

float fm_wrap_pi(float a)
{
    a = fm_wrap_2pi(a);
    if (a > FM_PI) a -= FM_TWO_PI;
    return a;
}

void fm_sincos(float theta, float *ps, float *pc)
{
    float t = theta * SIN_TAB_INV_TWO_PI;      /* 圈 */
    float tc;

    t -= (float)(int)t;
    if (t < 0.0f) t += 1.0f;

    if (ps) *ps = sin_turn(t);

    tc = t + 0.25f;                            /* cos = sin(x + pi/2) */
    if (tc >= 1.0f) tc -= 1.0f;
    if (pc) *pc = sin_turn(tc);
}

float fm_sin(float theta) { float s, c; fm_sincos(theta, &s, &c); return s; }
float fm_cos(float theta) { float s, c; fm_sincos(theta, &s, &c); return c; }

float fm_abs(float a) { return (a < 0.0f) ? -a : a; }

/* 牛顿法开方。初值用位运算魔数 (float 位模式 >>1 + 偏置), 两次迭代到 ~1e-6。
 * 不用 libm 的 sqrtf: newlib 里它是个 double 壳, 白吃约 2.5KB Flash。 */
float fm_sqrt(float x)
{
    union { float f; uint32_t u; } v;
    float r;
    int   k;

    if (x <= 0.0f) return 0.0f;

    v.f = x;
    v.u = (v.u >> 1) + 0x1fbd1df5u;            /* 经典初值 */
    r   = v.f;

    for (k = 0; k < 2; k++) r = 0.5f * (r + x / r);
    return r;
}

/* ---- 坐标变换 (幅值不变) ---- */
void fm_clarke(float ia, float ib, float ic, float *alpha, float *beta)
{
    if (alpha) *alpha = ia;
    if (beta)  *beta  = (ib - ic) * FM_1_SQRT3;
}

void fm_park(float alpha, float beta, float s, float c, float *d, float *q)
{
    if (d) *d =  alpha * c + beta * s;
    if (q) *q = -alpha * s + beta * c;
}

void fm_inv_park(float d, float q, float s, float c, float *alpha, float *beta)
{
    if (alpha) *alpha = d * c - q * s;
    if (beta)  *beta  = d * s + q * c;
}

/* ---- 中点注入 SVPWM ----
 * 三相 = 反 Clarke, 减去 (max+min)/2 的中点 -> 等效 SVPWM 的零序注入。
 * 与扇区法代数等价, 但只有比较、没有扇区表, 也不吃 atan2/sin。 */
void fm_svpwm(float alpha, float beta, float vbus, float *da, float *db, float *dc)
{
    float va, vb, vc;
    float vmax, vmin, voff;
    float inv_vbus;
    float a, b, c;

    if (vbus < 1.0f) vbus = 1.0f;              /* 防除零; 母线没起来时占空比算出来也没意义 */
    inv_vbus = 1.0f / vbus;

    va = alpha;
    vb = -0.5f * alpha + FM_SQRT3_2 * beta;
    vc = -0.5f * alpha - FM_SQRT3_2 * beta;

    vmax = va; if (vb > vmax) vmax = vb; if (vc > vmax) vmax = vc;
    vmin = va; if (vb < vmin) vmin = vb; if (vc < vmin) vmin = vc;

    voff = -0.5f * (vmax + vmin);

    a = 0.5f + (va + voff) * inv_vbus;
    b = 0.5f + (vb + voff) * inv_vbus;
    c = 0.5f + (vc + voff) * inv_vbus;

    if (a < 0.0f) a = 0.0f; else if (a > 1.0f) a = 1.0f;
    if (b < 0.0f) b = 0.0f; else if (b > 1.0f) b = 1.0f;
    if (c < 0.0f) c = 0.0f; else if (c > 1.0f) c = 1.0f;

    if (da) *da = a;
    if (db) *db = b;
    if (dc) *dc = c;
}