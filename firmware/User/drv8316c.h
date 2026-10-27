#ifndef __DRV8316C_H
#define __DRV8316C_H

#include "ch32l103.h"

/* =============================================================================
 * DRV8316C 硬件层 (CH32L103) —— 由 drv8313.c/h 移植
 * -----------------------------------------------------------------------------
 * 真值表 (3x PWM 模式; MODE 经 R42 47k 接 AVDD = 3x, 手册 SLVSH07 Table 8-4 / L1634-1638)
 *   INLx = 0              -> OUTx = 高阻 Z   (INHx 无关)
 *   INLx = 1, INHx = 0    -> OUTx = 低边导通
 *   INLx = 1, INHx = 1    -> OUTx = 高边导通
 *   => EN(INLx) 常高, 在 PWM(INHx) 上打 PWM; 桥内自带死区, MCU 不出互补 PWM。
 *   与旧板 DRV8313 的 Table 1 完全同构 —— FOC / SVPWM 的算法一行没动。
 *
 * 引脚 (网表核对 2026-10-08, work\fw_prep_2026-10-08\sch\out.net):
 *   PWM_C = PA6  (pin12) -> U6.31 INHC   TIM3_CH1
 *   PWM_B = PA7  (pin13) -> U6.29 INHB   TIM3_CH2
 *   PWM_A = PB0  (pin14) -> U6.27 INHA   TIM3_CH3
 *   EN_C  = PB3  (pin26) -> U6.32 INLC
 *   EN_B  = PB4  (pin27) -> U6.30 INLB
 *   EN_A  = PB5  (pin28) -> U6.28 INLA
 *   nSLEEP= PB15 (pin17) -> U6.23        (高 = 工作)
 *   nFAULT= PA8  (pin18) <- U6.22        (R46 5.1k 上拉到 +3V3; 开漏)
 * ========================================================================== */

void     DRV8316C_Init(void);                        /* TIM3 三相 PWM + EN + nSLEEP + nFAULT */
void     DRV8316C_Enable(void);                      /* EN_A/B/C = 1 */
void     DRV8316C_Disable(void);                     /* EN_A/B/C = 0 (输出高阻), 占空比清 0 */
int      DRV8316C_IsEnabled(void);                   /* 直接读 EN_A/B/C 引脚输出锁存 */
void     DRV8316C_SetDuty(float ta, float tb, float tc);  /* a/b/c = 相名(驱动腿), 0.0~1.0 已限幅 */
uint8_t  DRV8316C_Faulted(void);                     /* 1 = nFAULT 被拉低 */
void     DRV8316C_GetDuty(float *ta, float *tb, float *tc);  /* 最近一次的相占空比 */
uint8_t  DRV8316C_nSLEEP(void);                      /* 1 = nSLEEP 引脚当前为高(工作态) */

/* 让 TIM3 的每次上溢/下溢都进中断 —— 电流采样要和 PWM 载波对齐时用 (见 csense.c 的
 * TIM3_IRQHandler)。只在 csense 的 PWM 同步采样打开时调用。 */
void     DRV8316C_PwmIT_Enable(void);
void     DRV8316C_PwmIT_Disable(void);

/* 1MHz 自由运行计数器 (TIM4, 32bit) 用于 FOC 的 Ts 计算 */
void     Micros_Init(void);
uint32_t Micros(void);

#endif
