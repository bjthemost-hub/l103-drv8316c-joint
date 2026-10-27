#ifndef __CSENSE_H
#define __CSENSE_H

#include "ch32l103.h"

/* =============================================================================
 * 三相低边电流采样 (DRV8316C 内置 CSA + ADC1 注入组), 与 PWM 同步
 * -----------------------------------------------------------------------------
 * 时序 (中心对齐 + PWM 模式1):
 *   OCxREF 的高电平段以**谷值 CNT=0** 为中心, 于是"三相同时为低 = 三相下管全开"
 *   的那一段以**峰值 CNT=ARR** 为中心 -> 低边采样只能放在峰值。
 *   峰值 UEV: CS_StartSampleIsr() 起三路注入转换 (顺序扫, 每路 1.667us, 共约 5us)
 *   谷值 UEV: CS_LatchIsr() 取数 (离峰值 25us, 早转完了)
 * 这两次都在 TIM3 的同一个 ISR 里 (isr.c), 本模块只负责寄存器动作。
 *
 * ★ 2026-10-14 旧工程踩过的坑, 原样保留在这里当护栏:
 *   - ADC 的 SCAN 位必须开, 否则三路注入只转第一路, JEOC 永不置位
 *     (应用手册 CH32L103RM V1.9 表12-3 / 12.3.1);
 *   - 起转换时 JEXTTRIG 必须和 JSWSTART 一起置 1;
 *   - 判断"这次 UEV 是峰值还是谷值"看 CNT, 不要看 DIR 位 (旧代码按 DIR 判反过,
 *     结果每周期都在谷值起转换 -> 三相读数恒 0)。
 * ========================================================================== */

#define CS_SYNC_MISS_RUN_MAX   64u   /* 连续丢这么多拍就撤 valid (设计值) */

void     CS_Init(void);
void     CS_CalZero(void);            /* 必须在 DRV 未使能、电机不转时做 */

void     CS_StartSampleIsr(void);     /* 峰值 UEV 里调 */
int      CS_LatchIsr(void);           /* 谷值 UEV 里调: 1 = 取到新一帧 */
void     CS_GetIsr(float *ia, float *ib, float *ic);   /* A, 已减零点/换算/乘符号 */

int      CS_Valid(void);
uint32_t CS_Count(void);
uint32_t CS_Miss(void);
uint16_t CS_SyncRaw(uint8_t i);
float    CS_ZeroVolt(uint8_t i);
void     CS_SetSign(uint8_t i, float s);   /* 板级极性现场改正用 */
float    CS_Sign(uint8_t i);

/* ---- 定点电流环用的原始通路 (2026-10-18 起) ----
 * 相电流在 20kHz 中断里必须走整型: 定点核心直接拿 ADC 计数减零点, 一次乘就成 Q12。 */
void    CS_GetRawIsr(uint16_t *out);   /* out[3] = A/B/C 原始计数 */
int32_t CS_ZeroRaw(uint8_t i);        /* 零点, ADC 计数 */
int32_t CS_SignI(uint8_t i);          /* 板级极性 +1/-1 */
int      CS_ZeroOk(void);

uint16_t CS_Raw(uint8_t i);           /* 单次软件触发 (慢速上下文, 诊断/标定用) */
float    CS_Volt(uint8_t i);

uint16_t CS_VM_Raw(void);
float    CS_VM_Volt(void);

/* 上电一次性自检 (证明"软件起注入 -> 出数"这条链通了) */
int      CS_SelfTestOk(void);
uint16_t CS_SelfTestJeoc(void);
uint16_t CS_SelfTestJdr(uint8_t i);

#endif