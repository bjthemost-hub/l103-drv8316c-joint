#ifndef __ENC_H
#define __ENC_H

#include "ch32l103.h"

/* =============================================================================
 * MT6701 磁编码器, 软件 SSI (PB6=CLK / PB8=DO / PB9=CSN)
 * -----------------------------------------------------------------------------
 * 为什么还是软件 SPI: 原理图 2026-10-18 已冻结, 两根硬件 SPI 的脚组
 * (PA4..PA7 与 PB3..PB5) 全被 PWM / EN 占满 (依据见旧工程 foc_sensor.c 抬头)。
 *
 * 与旧工程的区别 —— **变快了, 并且搬进了 PWM 中断**:
 *   旧: 每一位前后各 Delay_Us(1), 一帧 32 位 ≈ 70us, 只能在主循环里读,
 *       而且主循环被打印/诊断拖住时角度就断供 (这是闭环节拍不固定的原因之一)。
 *   新: 位间延时改成 NOP 数 (ENC_HALF_NOP), 一帧 ≈ 10us, 放在 PWM 峰值 UEV
 *       里每 4 拍读一次 (5kHz)。手册的时序下限是 T_CLKL/T_CLKH >= 30ns、
 *       T_L >= 100ns (见旧 foc_sensor.c 注释), 这里每位半周期约 100ns 级,
 *       有数倍余量。**上板第一件事: 看 'Q' 里 enc_err 是否为 0**。
 *
 * 帧格式 (2026-10-14 旧工程实测修正): 第 1 个下降沿采到的位无效, 从第 2 位起是
 * D13, 第 25 位是 CRC0。所以打 32 个时钟, 取 bits[30:7] 当 24 位帧,
 * 其中 [23:10] = 角度 14 位。
 * ========================================================================== */

/* 每 N 个 PWM 峰值读一帧编码器。10kHz 是刻意的: 20kHz 软件 SSI 一帧要 ~10us,
 * 峰值中断 21us, 和谷值(电流环+速度环)叠起来就把 50us 周期占满了。 */
#define ENC_DECIM      2u

void     Enc_Init(void);
void     Enc_PollIsr(void);         /* PWM 峰值 UEV 里调 (20kHz): 读一帧 + 跑 PLL */

/* ---- PLL 输出 (全速域同一套方程, 见 enc.c 抬头) ---- */
uint32_t Enc_TurnQ16(void);         /* 机械位置 Q16 圈 (0..65535), 电流环用 */
int32_t  Enc_StepQ16(void);         /* 一个 PWM 周期的角增量 Q16 圈 (带符号) */
float    Enc_TurnPos(void);         /* 机械圈 [0,1) */
float    Enc_TurnVel(void);         /* 机械圈/s, 低速走 20ms 窗口, 高速走 PLL */
int32_t  Enc_VelQ24(void);         /* 同上, Q24 圈/s (速度环拿走做定点用) */
void     Enc_SetPllBandwidth(float hz);
void     Enc_SetVelSrc(uint8_t s);   /* 0=自动(窗口/PLL) 1=只用PLL 2=只用窗口 */
uint8_t  Enc_VelSrc(void);
float    Enc_PllBandwidth(void);

float    Enc_MechRad(void);         /* 累计机械角, rad (含多圈, 已减 mech_zero) */
float    Enc_MechDelta(void);       /* 最近一次 Enc_PollIsr 的机械角增量 rad */
uint16_t Enc_Raw(void);             /* 最近一次的 14 位原始角 */
uint8_t  Enc_Mg(void);              /* 最近一帧有效帧的 Mg[3:0] 状态位 */
uint8_t  Enc_MgValid(void);         /* 是否已拿到过 CRC 正确的 Mg */
uint32_t Enc_Frames(void);
uint32_t Enc_ErrCnt(void);          /* 全 0 / 全 1 / 丢帧数 */
uint32_t Enc_CrcErrCnt(void);      /* CRC6 校验失败帧数 */
void     Enc_VelSpan(int32_t *mn, int32_t *mx, int reset);
float    Enc_PeriodVel(void);     /* 20ms 窗口计数测速 (圈/s), 低速用, 诊断看 */
void     Enc_SetMechZero(float rad);
float    Enc_ZeroRad(void);            /* mech_zero 在累计坐标里的位置 rad (= raw - mech) */
/* 掉电恢复用 (posstore.c): 直接写零位与圈数, 不重新推算 ——
 * Enc_SetMechZero() 会用"此刻角度"反算零位, 那会把断电期间转过的角度抹掉。 */
void     Enc_SetZeroRad(float rad);
void     Enc_SetRev(int32_t rev);
int32_t  Enc_Rev(void);

/* ---- 原始角捕获 (诊断: 上位机自己算速度判平滑, 不经任何估计) ---- */
#define ENC_CAP_DIV   20u          /* 10kHz -> 500Hz */
void     Enc_CapEnable(uint8_t on);
uint8_t  Enc_CapEnabled(void);
int      Enc_CapRead(uint16_t *raw, uint32_t *seq);/* 把"当前"位置定义为 mech = rad */
int      Enc_ReadBlocking(uint16_t *raw, uint8_t *crc_ok); /* 慢速上下文自检 */

#endif
