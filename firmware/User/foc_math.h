#ifndef __FOC_MATH_H
#define __FOC_MATH_H

/* =============================================================================
 * FOC 数学核心 (纯函数, 不碰任何硬件)
 * -----------------------------------------------------------------------------
 * 约定:
 *   电角度  theta_e  单位 rad, 保持在 [0, 2*pi)
 *   电流    iabc / id / iq      单位 A (幅值不变 Clarke, 即相电流真值)
 *   电压    valpha/vbeta/vd/vq  单位 V
 *   占空比  da/db/dc            单位 1 (0..1)
 * 为什么不用 sinf/sqrtf: RV32IMAC 无 FPU, newlib 的 sinf 一帧上千周期,
 *   sqrtf 在 newlib 里还是个 double 壳 (~2.5KB Flash)。这里全部用表 + 牛顿。
 * ========================================================================== */

void  fm_sincos(float theta, float *ps, float *pc);
float fm_sin(float theta);
float fm_cos(float theta);
float fm_wrap_2pi(float a);           /* 归一到 [0, 2*pi); 输入 |a| < 1e6 */
float fm_wrap_pi(float a);            /* 归一到 [-pi, pi) */
float fm_abs(float a);
float fm_sqrt(float x);               /* x<=0 返回 0; 牛顿迭代, 不用 libm */

void  fm_clarke(float ia, float ib, float ic, float *alpha, float *beta);
void  fm_park(float alpha, float beta, float s, float c, float *d, float *q);
void  fm_inv_park(float d, float q, float s, float c, float *alpha, float *beta);

/* 中点注入 SVPWM (与扇区法等价, 但少一半分支):
 *   vbus 单位 V; 输出 da/db/dc 已钳到 [0,1]
 *   线性调制区: |V| <= vbus/sqrt(3) 时全程不削顶 */
void  fm_svpwm(float alpha, float beta, float vbus, float *da, float *db, float *dc);

#define FM_TWO_PI   6.28318530718f
#define FM_PI       3.14159265359f
#define FM_PI_2     1.57079632679f
#define FM_1_SQRT3  0.57735026919f
#define FM_SQRT3_2  0.86602540378f

#include <stdint.h>

#endif