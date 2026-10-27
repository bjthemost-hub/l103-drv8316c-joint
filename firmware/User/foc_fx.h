#ifndef __FOC_FX_H
#define __FOC_FX_H

#include <stdint.h>
#include "foc_tab_q15.h"

/* =============================================================================
 * 定点 FOC 数学核 (纯函数 / 内联, 不碰硬件)
 * -----------------------------------------------------------------------------
 * 为什么定点: 这颗是 RV32IMAC, **没有 FPU**。gdb 抓过现场 —— 20kHz 谷值中断里
 * 的 PC 落在 __mulsf3, 谷值 ISR 实测 39us/50us = 78% CPU, 串口 460800 下整字节丢。
 * 定点后一次乘加 = 1 条 mul, 单拍成本从"几十周期"降到"1 周期"。
 *
 * 约定 (全部 int32, 左移位数写在名字里):
 *   角度  theta   uint16, 65536 = 2*pi   (1 LSB = 9.587e-5 rad)
 *   sin/cos       int16  Q15 (32768 = 1.0)
 *   电流  I       Q12 A  (1 LSB = 0.244 mA; 4 A = 16384)
 *   电压  V       Q12 V  (1 LSB = 0.244 mV)
 *   占空比        Q15 (32768 = 1.0)
 * 量程自检 (算术): 最大相电流 4.667 A -> 19116; Park 里 19116*32768 = 6.3e8,
 *   两项相加 1.25e9 < 2^31, 不溢出。
 * ========================================================================== */

#define FX_ANG_2PI      65536u
#define FX_ANG_PI       32768u
#define FX_ANG_90       16384u

#define FX_Q15_ONE      32768
#define FX_INV_SQRT3_Q15  18919    /* 1/sqrt(3) * 32768 */
#define FX_SQRT3_2_Q15    28378    /* sqrt(3)/2  * 32768 */

#define FX_I_SHIFT      12
#define FX_V_SHIFT      12
#define FX_ONE_I        (1 << FX_I_SHIFT)   /* 1 A */
#define FX_ONE_V        (1 << FX_V_SHIFT)   /* 1 V */

/* sin / cos, 入参 uint16 电角度。
 * +32 是"四舍五入到最近表点"的一半索引 (64/2): 直接 >>6 是截断, 最坏会差满一格,
 * 实测 (scripts\model_foc_fx.py) 把误差从 6.04e-3 降到 3.08e-3 rad。
 * uint16 加法自然回绕, 不用额外取模。 */
static inline int32_t fx_sin(uint16_t a) { return (int32_t)foc_sin_q15[(uint16_t)(a + 32u) >> FOC_TAB_SHIFT]; }
static inline int32_t fx_cos(uint16_t a) { return (int32_t)foc_sin_q15[(uint16_t)(a + FX_ANG_90 + 32u) >> FOC_TAB_SHIFT]; }

/* 幅值不变 Clarke (输入 Q12 A) */
static inline void fx_clarke(int32_t ia, int32_t ib, int32_t ic, int32_t *alpha, int32_t *beta)
{
    *alpha = ia;
    *beta  = ((ib - ic) * FX_INV_SQRT3_Q15) >> 15;
}

/* Park: (alpha,beta) -> (d,q), s/c 为 Q15 */
static inline void fx_park(int32_t alpha, int32_t beta, int32_t s, int32_t c, int32_t *d, int32_t *q)
{
    *d = (alpha * c + beta * s) >> 15;
    *q = (beta * c - alpha * s) >> 15;
}

/* 反 Park: (d,q) -> (alpha,beta) */
static inline void fx_inv_park(int32_t d, int32_t q, int32_t s, int32_t c, int32_t *alpha, int32_t *beta)
{
    *alpha = (d * c - q * s) >> 15;
    *beta  = (d * s + q * c) >> 15;
}

/* 无符号整数开方 (逐位法, 无除法无浮点)。只在上限饱和时调用。 */
static inline uint32_t fx_isqrt32(uint32_t x)
{
    uint32_t res = 0u, bit = 1u << 30;

    while(bit > x) bit >>= 2;
    while(bit)
    {
        if(x >= res + bit) { x -= res + bit; res = (res >> 1) + bit; }
        else               { res >>= 1; }
        bit >>= 2;
    }
    return res;
}

#endif
