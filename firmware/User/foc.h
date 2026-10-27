#ifndef __FOC_H
#define __FOC_H

#include "ch32l103.h"

/* =============================================================================
 * 电流环 (d/q 双 PI + 中点注入 SVPWM) —— 2026-10-18 换成整型/定点核心
 * -----------------------------------------------------------------------------
 * 结构照 AT32M416 mclib (mc_isr.c 的 Id/Iq 双 PI) + ODrive (foc.cpp 的
 * 电压圆饱和与积分回退) 的骨架, 但**数值层全部重写**:
 *
 *   Id_ref ──► PI_d ──► Vd ─┐
 *                           ├─ 电压圆 ─► 反Park ─► SVPWM ─► CCR
 *   Iq_ref ──► PI_q ──► Vq ─┘
 *
 * 为什么不用 float: 这颗 RV32IMAC 没有 FPU, 20kHz 中断里每一条浮点乘除都是
 * __mulsf3/__divsf3 调用 (gdb 实测 PC 就落在 __mulsf3)。改成 Q12 定点后,
 * 一次乘加 = 1 条 mul 指令, 中断预算从 39us 掉到个位数 us 级 —— 这是能在
 * 20kHz 上同时照顾"低速平滑"和"3000rpm 角度补偿"的前提。
 *
 * 单位 (详见 foc_fx.h):
 *   电流/电压 Q12 (1 LSB = 0.244 mA / 0.244 mV); 电角度 uint16 (65536 = 2pi)
 * ========================================================================== */

typedef struct {
    /* ---- 定点状态 (只在 ISR 里写) ---- */
    int32_t  i_q12[3];                    /* A x 4096, idx 0/1/2 = A/B/C */
    int32_t  id_q12, iq_q12;              /* A x 4096 */
    int32_t  vd_q12, vq_q12;              /* V x 4096 */
    int32_t  id_ref_q12, iq_ref_q12;
    int32_t  i_abs_max_q12;               /* 本拍三相最大 |i| */
    uint16_t duty_q15[3];                 /* 本拍占空比 Q15 (16384 = 0.5) */
    uint16_t ccr[3];                      /* 定时器比较值, 调试用 */
    uint8_t  valid;                       /* 1 = 本拍拿到了有效相电流 */
    uint8_t  oc;                          /* 1 = 相电流超过 oc_trip_q12 (运行时阈值) */
    int32_t  oc_trip_q12;                 /* 跳闸阈值 A x 4096, 由 Motor_SetOcTrip 写 */
    uint16_t oc_trip_ms;                  /* 连续超限多久才跳闸 (ms), 0 = 立即 */
    uint16_t oc_ms_cnt;                   /* 已经连续超限多少 ms */
    uint16_t oc_win_acc;                  /* 当前 1ms 窗口里采了几拍 */
    int32_t  oc_win_max;                  /* 当前 1ms 窗口内的 |i| 峰值 */
    uint32_t recon_cnt;                   /* 因占空比过大而"补一相"的次数 */
    uint32_t sat_cnt;                     /* 电压圆饱和次数 */

    /* ---- 配置 / 打印 (慢速上下文) ---- */
    float    vbus;                        /* V, 当前用的母线 */
    float    v_limit;                     /* 电压圆半径 V (0.95*vbus/sqrt3) */
    float    kp, ki;                      /* 电流环增益 (V/A, V/(A*s)) */
    uint8_t  enabled;
} Foc_t;

extern Foc_t foc;

void  Foc_Init(void);
void  Foc_Reset(void);
void  Foc_SetGains(float kp, float ki);
void  Foc_GetGains(float *kp, float *ki);
void  Foc_SetGainsFromRL(float r_ohm, float l_h, float bw_hz);
void  Foc_SetVbus(float vbus);
float Foc_GetVbus(void);
void  Foc_SetIqRefs(float iq, float id);
void  Foc_SetOcTrip(float a);
void  Foc_SetOcTripMs(uint16_t ms);
void  Foc_SetIqRefsQ12(int32_t iq_q12, int32_t id_q12);   /* 20kHz 热路径: 不做浮点转换 */
int   Foc_MeasureCurrent(void);                       /* 取三相电流 + 重构 + 过流判决 */
void  Foc_RunVolt(uint16_t theta_e, float vd, float vq);  /* 开环电压矢量 */
void  Foc_RunVoltQ12(uint16_t theta_e, int32_t vd_q12, int32_t vq_q12);
void  Foc_Run(uint16_t theta_e);                      /* 谷值 UEV 里调, 20kHz */
void  Foc_ZeroVector(void);

#endif
