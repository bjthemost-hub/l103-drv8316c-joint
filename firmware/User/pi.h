#ifndef __PI_H
#define __PI_H

/* =============================================================================
 * 一个小 PI (带反算抗饱和) —— 电流环两路 (d/q) 与速度环共用
 * -----------------------------------------------------------------------------
 * 抗饱和用的是"反算/条件积分": 输出被钳住时, 把积分器直接改写成
 *       integ = 钳位值 - Kp*err
 * 也就是让积分器**从饱和边界往回站**, 而不是继续往里灌。
 * 旧固件的速度环是"先钳积分再钳输出再斜率限幅", 没有这一步 —— 低速时
 * 电压环饱和 + 积分攒满 = 松脱瞬间冲, 是抖/冲的一个结构性原因。
 * ========================================================================== */

typedef struct {
    float kp;          /* 输出/误差 */
    float ki;          /* 输出/(误差*秒) */
    float ts;          /* 采样周期 s */
    float integ;       /* 积分累积, 与输出同量纲 */
    float out_min;
    float out_max;
} PI_t;

void  PI_Init(PI_t *p, float kp, float ki, float ts, float lo, float hi);
void  PI_Reset(PI_t *p);
void  PI_SetGains(PI_t *p, float kp, float ki);
float PI_Run(PI_t *p, float err);
/* 本拍用动态限幅 (电流环的 q 轴要按电压圆剩余量限, 每拍都不一样) */
float PI_RunLim(PI_t *p, float err, float lo, float hi);

#include <stdint.h>

#endif