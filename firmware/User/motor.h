#ifndef __MOTOR_H
#define __MOTOR_H

#include "ch32l103.h"

/* =============================================================================
 * 电机控制: 状态机 + 角度/速度估计 + 速度环 + 位置环
 * -----------------------------------------------------------------------------
 * 三层节拍 (全部由 TIM3 UEV 起算):
 *    20 kHz  电流环 (定点)  foc.c      Foc_Run()
 *    20 kHz  编码器 + PLL   enc.c      Enc_PollIsr()
 *     1 kHz  速度环 -> Iq
 *     200 Hz 位置环 -> 速度给定
 *
 * 全速域怎么兼顾 (家人 2026-10-18 提的: 云台低速大扭矩 / 关节高速小扭矩):
 *   - **估计器不分档**: 二阶 PLL 对匀速输入零稳态误差, 3rpm 和 3000rpm 用同一组方程。
 *   - **角度按"电压生效时刻"外推**: 编码器在峰值采, 电压在下一个周期中心才真正作用,
 *     所以外推 1 个 PWM 周期。3000rpm/11 极对 = 550Hz 电频率, 50us = 9.9 度电角度,
 *     不补这一下, 高速端会明显掉力矩。
 *   - **增益/限幅全部可设**: 不同工况 (云台 vs 关节) 换参数, 不换代码。
 *   - **弱磁可选**: 高速端母线顶不住反电势时用负 Id 换转速 (FW1)。
 * ========================================================================== */

typedef enum {
    MOT_OFF = 0,
    MOT_ALIGN,
    MOT_TORQUE,
    MOT_VELOCITY,
    MOT_POSITION,
    MOT_OPENLOOP
} Mot_Mode;

typedef struct {
    Mot_Mode mode;
    Mot_Mode mode_req;

    float iq_cmd;               /* A  力矩模式给定 */
    float vel_cmd;              /* rad/s 机械 */
    float pos_cmd;              /* rad 机械 */
    float i_limit;              /* A */
    float oc_trip;              /* A 相电流跳闸阈值 (可调, 存 Flash) */
    uint16_t oc_trip_ms;        /* ms 连续超限多久才跳闸 (可调, 存 Flash; 0 = 立即) */
    float vel_limit;            /* rad/s */
    float accel_limit;          /* rad/s^2 */
    float pos_kp;               /* 1/s */
    float id_fw;                /* A, 弱磁电流 (<=0) */
    uint8_t fw_enable;

    float r_ohm;                /* 相电阻 Ohm —— 存档/显示用, 增益仍以 cur kp/ki 为准 */
    float l_h;                  /* 相电感 H */

    float v_align;              /* V */
    uint32_t align_ms;

    /* ---- 反馈 (只读) ---- */
    uint16_t theta_e_u16;       /* 本拍实际下发给电流环的电角度 */
    float theta_e;              /* 同上, rad (只为打印) */
    float omega_m;              /* 机械角速度 rad/s (PLL) */
    float pos_m;                /* 机械位置 rad */
    float vel_ref;              /* 速度环给定 */
    float iq_ref;               /* 速度环输出 = 电流环给定 */
    float vbus;

    uint8_t enabled;
    uint8_t aligned;
    uint8_t fault;
    uint32_t fault_code;

    uint32_t isr_cnt;        /* 谷值中断计数 (= 电流环拍数) */
    uint32_t isr_pk_cnt;     /* 峰值中断计数 (编码器/采样起点) */
    uint16_t isr_us_max;
    uint16_t isr_us_last;
    uint16_t isr_pk_us_max;
    uint16_t isr_pk_us_last;
} Motor_t;

extern Motor_t motor;

void Motor_Init(void);
void Motor_PeakIsr(void);
void Motor_ValleyIsr(void);

void Motor_RequestMode(Mot_Mode m);
void Motor_Enable(void);
void Motor_Disable(void);
void Motor_ClearCommands(void);   /* 失能/上电用: 目标贴回当前位置, 清 pending */
void Motor_SetIq(float a);
void Motor_SetVelocity(float rad_s);
void Motor_SetPosition(float rad);
void Motor_StartAlign(void);
void Motor_StartOpenloop(float volts, float hz);
void Motor_ClearFault(void);
void Motor_SetFault(uint32_t code);

void Motor_SetPolePairs(int pp);
int  Motor_PolePairs(void);
void Motor_SetVbus(float v);
void Motor_SetCurGains(float kp, float ki);
void Motor_GetCurGains(float *kp, float *ki);
void Motor_SetGainsFromRL(float r_ohm, float l_henry, float bw_hz);
void Motor_GetRL(float *r_ohm, float *l_h);
void Motor_SetSpeedGains(float kp, float ki);
void Motor_GetSpeedGains(float *kp, float *ki);
void Motor_SetPosKp(float kp);
void Motor_SetILimit(float a);
void  Motor_SetOcTrip(float a);
float Motor_GetOcTrip(void);
void  Motor_SetOcTripMs(uint16_t ms);
uint16_t Motor_GetOcTripMs(void);
void Motor_SetVelLimit(float v);
void Motor_SetAccelLimit(float a);
void Motor_SetAlignVolt(float v);
void Motor_SetPosHold(uint8_t en);   /* 到位静止锁定开关 (串口 PH0/PH1) */
uint8_t Motor_GetPosHold(void);
uint8_t Motor_HoldActive(void);      /* 当前是否正处在到位锁定 (只读, Q 诊断用) */
float   Motor_VelLp(void);           /* 低通后的速度 (只读, Q 诊断用) */
void Motor_SetFw(uint8_t en);
float Motor_GetIsrUsMax(void);
float Motor_VbusMeas(void);

#endif
