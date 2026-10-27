#include "enc.h"
#include "board.h"
#include "ch32l103_usbpd.h"
#include <stdint.h>

/* =============================================================================
 * MT6701 (14 位绝对角, 软件 SSI) + ODrive 式二阶 PLL 测速
 * -----------------------------------------------------------------------------
 * 为什么换 PLL (2026-10-18):
 *   旧做法是"每 200us 两次读数直接差分"。1 count = 2*pi/16384 = 3.83e-4 rad,
 *   200us 内差 1 count 就等于 1.92 rad/s => 低速端基本只有 0/1 两种取值,
 *   速度环拿到的是一串台阶 —— 这就是"低速卡顿"的量化来源。
 *   二阶 PLL 把"位置"当被测对象、"速度"当状态积分出来: 对匀速输入**稳态误差为零**,
 *   对量化噪声是一个二阶低通。同一套方程在 3rpm 和 3000rpm 都成立, 不需要分档。
 *
 * 结构 (与 ODrive encoder.cpp 的 pll 段同构):
 *     pos += dt*vel
 *     err  = meas - pos            (缠绕到 +-1/2 圈)
 *     pos += dt*kp*err
 *     vel += dt*ki*err
 *   临界阻尼: kp = 2*wn, ki = wn^2, wn = 2*pi*bw
 *
 * 采样率 = 20kHz (每个 PWM 峰值都读一帧)。旧版每 4 拍读一次是为了省中断时间;
 * 现在电流环已经改成定点, 峰值中断有富余, 每拍都读就把"角度外推"窗口压到
 * 半拍 (25us), 3000rpm (电频率 550Hz) 下也只差 9.9 度电角度。
 * ========================================================================== */

#define ENC_HALF_NOP   4u          /* 半周期 nop 数; 'Q' 里 enc_err 涨就加到 8 */

#define ENC_CLK_PIN    GPIO_Pin_6
#define ENC_DO_PIN     GPIO_Pin_8
#define ENC_CS_PIN     GPIO_Pin_9

#define ENC_BITS       32u
#define ENC_SHIFT      7u
#define ENC_TICK_HZ    (PWM_FREQ_HZ / ENC_DECIM)          /* PLL 实际更新率 10kHz */
#define ENC_DT         (PWM_TS_S * (float)ENC_DECIM)
#define ENC_PLL_BW_HZ_DEF  150.0f      /* 2026-10-18 实测: 10/30/100rpm 都用这一档 */

static inline void enc_clk_hi(void) { GPIOB->BSHR = ENC_CLK_PIN; }
static inline void enc_clk_lo(void) { GPIOB->BCR  = ENC_CLK_PIN; }
static inline void enc_cs_hi(void)  { GPIOB->BSHR = ENC_CS_PIN;  }
static inline void enc_cs_lo(void)  { GPIOB->BCR  = ENC_CS_PIN;  }
static inline uint32_t enc_do(void) { return (GPIOB->INDR & ENC_DO_PIN) ? 1u : 0u; }

/* 半周期延时。原来写成 while(k--) 循环, -Os 下每 bit 要跑两遍循环 + 调用开销,
 * 32 bit 一帧实测 40us (2026-10-18 上板测的 isr_pk_us)。改成展开的 nop 序列后
 * 每 bit 只剩 2 次 GPIO 写 + 1 次读 + 8 个 nop。
 * 手册下限: T_CLKL/T_CLKH >= 30ns, T_L >= 100ns; 4 个 nop @96MHz = 41.7ns, 够。 */
#define ENC_NOP4()  __asm__ __volatile__("nop; nop; nop; nop")

static uint16_t enc_raw_last;
static uint8_t  enc_mg_last;      /* MT6701 SSI 的 Mg[3:0] 状态位 (最近一帧有效帧) */
static uint8_t  enc_mg_valid;     /* 至少拿到过一帧 CRC 正确的 Mg */
static uint32_t enc_frames;
static uint32_t enc_err;
static uint32_t enc_crc_err;
static int32_t  vel_span_min, vel_span_max;   /* 诊断: 速度状态极值 */

/* ---- 20ms 窗口计数测速 (低速) ---------------------------------------------
 * 14 位编码器在 20kHz 下 1 个计数 = 1.22 圈/s。逐拍差分在低速端只有台阶,
 * 速度环必然跟着台阶抖 (2026-10-18 实测: 0.29 rad/s 时 vel 报 0~0.39 乱跳)。
 * 这里数"固定 20ms 窗口内走了多少个计数":
 *     v[Q24 圈/s] = counts * 2^24 * 20000 / (CPR * ticks) = counts * 20480000 / ticks
 * 0.29 rad/s 时窗口内约 15 个计数, 分辨率 ~7%; 窗口 400 拍 = 20ms, 对应约 25Hz。
 * 高速 (>= 1 圈/s) 交回 PLL, 那边差分本身就够细。 */
#define ENC_WIN_TICKS    200u          /* 20 ms @10kHz */
#define ENC_PER_COEF     10240000      /* 2^24 * 10000 / 16384 */
#define ENC_MID_Q24      16777216      /* 1 圈/s, 高低速切换门限 */
static int32_t  win_cnt;               /* 窗口内累计计数 */
static uint32_t win_tick;              /* 窗口内累计拍数 */
static int32_t  win_vel_q24;           /* 窗口测速结果, Q24 圈/s */
/* 速度环默认只吃 PLL (2026-10-18 实测定的)。
 * 原来默认是"低速走 20ms 窗口", 但窗口值 20ms 才更新一次、并且被 0 计数台阶
 * 拉低, 速度环看见的是"停->冲"的阶梯, 只能把增益压到 0.02/0.2, 结果就是
 * 走走停停 (MT6701 原始计数实测: 51% 的 2ms 采样零位移、5s 内 134 次回退)。
 * 换成纯 PLL 后同一台机、同样 10rpm: 零位移 0.2%、回退 0 次。
 * VS<n> 仍可在线切回去做对照。 */
static uint8_t  vel_src = 1u;          /* 0=自动 1=PLL 2=窗口 (默认 PLL) */
static uint16_t enc_raw_per;           /* 上一次采样的原始角 */
static uint8_t  enc_ok;

/* ---- 原始角捕获 (诊断用: 给上位机判"转得平不平滑") -------------------------
 * 直接从 MT6701 每帧的原始 14 位计数里按 ENC_CAP_DIV 分频抽一个, 不经过 PLL、
 * 不经过窗口测速 —— 上位机拿到的就是传感器的原样数据, 用相邻采样的计数差
 * 自己算速度, 判抖动/回退/停顿。默认关闭, 'SD1' 打开, 'SD0' 关闭。 */
static uint16_t cap_raw;
static uint32_t cap_seq;
static uint8_t  cap_div;
static uint8_t  cap_en;

/* PLL 状态 —— 全整型 (Q16 圈 / Q16 圈每秒)。
 * 为什么不用 float: 2026-10-18 上板实测峰值中断 pk=61us(max 67us), 已经超过
 * 50us 的 PWM 周期。反汇编 + 计时对照确认大头是这里的 __mulsf3/__addsf3
 * (RV32IMAC 没有 FPU)。改成 Q16 后, 20kHz 里只剩乘法/移位/常数除法。
 * 方程没变: 位置是状态, 速度是位置的积分, 对匀速输入零稳态误差 ——
 * 3rpm 和 3000rpm 用同一组方程, 只是增益不同。 */
static uint32_t enc_turn_q16;   /* 机械位置 Q16 圈 (0..65535), 给电流环用 */
static int32_t  enc_step_q16;   /* 一个 PWM 周期的角增量 Q16 圈 (带符号) */
static int32_t  pll_pos_q24;    /* 机械位置, Q24 圈 (0..2^24); Q16 会因增益量化把速度抬偏 */
static int32_t  pll_vel_q24;    /* 机械速度, Q24 圈/s (50 圈/s = 8.4e8, 装得下) */
static int32_t  pll_kp_q16;     /* 每拍位置增益 dt*kp, Q16 */
static int32_t  pll_ki_q8;      /* 每拍速度增益 dt*ki, Q8 (Q16 会溢出) */
static int32_t  pll_snap_q24;   /* 静止咬零阈值, Q24 圈/s */
static float    pll_bw_hz = ENC_PLL_BW_HZ_DEF;
static int32_t  enc_rev;        /* 多圈整数部分 (rev) */
static float    enc_zero_rad;

/* 一帧: 32 个时钟, 返回原始位流 */
static uint32_t enc_read_bits(void)
{
    uint32_t bits = 0u;
    int      k;

    enc_cs_hi(); enc_clk_hi(); ENC_NOP4();
    enc_cs_lo();               ENC_NOP4();     /* T_L >= 100ns */

    for(k = 0; k < (int)ENC_BITS; k++)
    {
        enc_clk_lo();                          /* 下降沿 = 采样点 */
        ENC_NOP4();
        bits = (bits << 1) | enc_do();
        enc_clk_hi();
        ENC_NOP4();
    }

    enc_cs_hi();
    return bits;
}

/* 6 位 CRC (X^6+X+1), 只在慢速自检里算 */
static uint8_t enc_crc6(uint32_t data18)
{
    uint8_t crc = 0u;
    int     i;

    for(i = 17; i >= 0; i--)
    {
        uint8_t inbit = (uint8_t)((data18 >> i) & 1u);
        uint8_t msb   = (uint8_t)((crc >> 5) & 1u);

        crc = (uint8_t)((crc << 1) & 0x3Fu);
        if(msb ^ inbit) crc ^= 0x03u;
    }
    return crc;
}

static void pll_tune(float bw_hz)
{
    float wn, kp, ki;

    if(bw_hz < 5.0f)     bw_hz = 5.0f;
    if(bw_hz > 800.0f)   bw_hz = 800.0f;          /* 上限: 再高就该降采样率了 */
    pll_bw_hz = bw_hz;
    wn = 6.28318530718f * bw_hz;
    kp = 2.0f * wn;                                /* 临界阻尼 */
    ki = wn * wn;

    /* 换成"每拍增量", dt = 50us。
     * kp 用 Q16: 1256.6 * 50e-6 * 65536 = 4118。
     * ki 用 Q8 : 394784 * 50e-6 * 256  = 5053; 用 Q16 会和 err 相乘溢出 int32。
     * 位置/速度状态用 Q24 —— 2026-10-18 实测: Q16 时 kp 项对 1 LSB 误差截成 0,
     * 而 ki 项每 LSB 推 19.7, 速度估计被量化噪声抬到真值的 2.4 倍。 */
    pll_kp_q16   = (int32_t)(ENC_DT * kp * 65536.0f + 0.5f);
    pll_ki_q8    = (int32_t)(ENC_DT * ki * 256.0f + 0.5f);
    pll_snap_q24 = (int32_t)(0.5f * ENC_DT * ki / (float)ENC_CPR * 16777216.0f);
    if(pll_snap_q24 < 1) pll_snap_q24 = 1;
}

static void pll_reset_q16(int32_t turn_q16)
{
    pll_pos_q24  = (turn_q16 & 0xFFFF) << 8;
    pll_vel_q24  = 0;
    if(pll_vel_q24 < vel_span_min) vel_span_min = pll_vel_q24;
    if(pll_vel_q24 > vel_span_max) vel_span_max = pll_vel_q24;
    enc_turn_q16 = (uint32_t)pll_pos_q24 >> 8;
    enc_step_q16 = 0;
}

void Enc_SetPllBandwidth(float hz) { pll_tune(hz); }
float Enc_PllBandwidth(void)       { return pll_bw_hz; }

void Enc_Init(void)
{
    GPIO_InitTypeDef gpio;
    uint16_t raw = 0u;
    uint8_t  crc_ok = 0u;

    /* PB6 = USBPD CC1, 复位后内置 5.1k Rd 是接通的 (实测复位值 0x0003),
     * 会把 CLK 线拖慢。这里清掉。 */
    RCC->HBPCENR |= RCC_USBPDEN;
    USBPD->PORT_CC1 &= (uint16_t)~(uint16_t)CC_PD;
    USBPD->PORT_CC2 &= (uint16_t)~(uint16_t)CC_PD;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin   = ENC_CLK_PIN;  GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin   = ENC_CS_PIN;   GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Pin   = ENC_DO_PIN;   GPIO_Init(GPIOB, &gpio);

    enc_clk_hi();
    enc_cs_hi();

    enc_rev      = 0;
    enc_raw_last = 0u;
    enc_mg_last  = 0u;
    enc_mg_valid = 0u;
    enc_zero_rad = 0.0f;
    enc_err      = 0u;
    enc_crc_err  = 0u;
    vel_span_min = 0x7FFFFFFF; vel_span_max = (int32_t)0x80000000;
    win_cnt      = 0;
    win_tick     = 0u;
    win_vel_q24  = 0;
    enc_raw_per  = enc_raw_last;
    enc_frames   = 0u;
    enc_ok       = 0u;
    pll_tune(ENC_PLL_BW_HZ_DEF);

    if(Enc_ReadBlocking(&raw, &crc_ok) == 0)
    {
        enc_raw_last = raw;
        enc_raw_per  = raw;
        pll_reset_q16((int32_t)raw << 2);      /* 14 位 -> Q16 圈, 正好 <<2 */
        enc_ok = 1u;
    }
}

int Enc_ReadBlocking(uint16_t *raw, uint8_t *crc_ok)
{
    uint32_t bits = enc_read_bits();
    uint32_t frame24 = (bits >> ENC_SHIFT) & 0x00FFFFFFu;
    uint16_t ang  = (uint16_t)((frame24 >> 10) & 0x3FFFu);
    uint8_t  crc_rd = (uint8_t)(frame24 & 0x3Fu);
    uint8_t  crc_ca = enc_crc6((frame24 >> 6) & 0x3FFFFu);

    if(raw)    *raw    = ang;
    if(crc_ok) *crc_ok = (uint8_t)((crc_rd == crc_ca) ? 1u : 0u);

    if(crc_rd == crc_ca)
    {
        enc_mg_last  = (uint8_t)((frame24 >> 6) & 0x0Fu);
        enc_mg_valid = 1u;
    }

    if(bits == 0u || bits == 0xFFFFFFFFu) return -1;
    return 0;
}

/* =============================================================================
 * 20kHz 峰值中断里调: 读一帧 + 跑 PLL
 * -----------------------------------------------------------------------------
 * 位置用"圈"做单位 (0..1), 速度用"圈/s"。这样 PLL 的增益与 CPR 无关,
 * 换编码器只改一个分母。
 * ========================================================================== */
void Enc_PollIsr(void)
{
    uint32_t bits;
    bits = enc_read_bits();
    uint32_t f24  = (bits >> ENC_SHIFT) & 0x00FFFFFFu;
    uint16_t raw  = (uint16_t)((f24 >> 10) & 0x3FFFu);
    int32_t  meas, err;

    if(bits == 0u || bits == 0xFFFFFFFFu)
    {
        if(enc_err < 0xFFFFFFF0u) enc_err++;
        return;                       /* 丢掉这一帧, PLL 靠预测继续走 */
    }

    /* CRC6 校验: MT6701 每帧 14 位角度 + 6 位 CRC。20kHz 下 SSI 时钟偏快时
     * 会出现"不是全 0/全 1、但个别位翻掉"的坏帧, 这种帧会把 PLL 的速度踢成
     * ±25 rad/s 的噪声 (2026-10-18 实测)。CRC 不对就整帧丢掉。 */
    {
        uint8_t  crc_rd = (uint8_t)(f24 & 0x3Fu);
        uint8_t  crc_ca = enc_crc6((f24 >> 6) & 0x3FFFFu);

        if(crc_rd != crc_ca)
        {
            if(enc_crc_err < 0xFFFFFFF0u) enc_crc_err++;
            return;
        }
    }

    enc_raw_last = raw;
    enc_mg_last  = (uint8_t)((f24 >> 6) & 0x0Fu);
    enc_mg_valid = 1u;
    enc_frames++;
    enc_ok = 1u;

    /* ---- 原始角捕获 (诊断流) ---- */
    if(cap_en)
    {
        if(++cap_div >= ENC_CAP_DIV)
        {
            cap_div = 0u;
            cap_raw = raw;
            cap_seq++;
        }
    }

    /* ---- 20ms 窗口测速 ---- */
    {
        int32_t dcount = (int32_t)raw - (int32_t)enc_raw_per;

        if(dcount >  (int32_t)(ENC_CPR / 2)) dcount -= (int32_t)ENC_CPR;
        if(dcount < -(int32_t)(ENC_CPR / 2)) dcount += (int32_t)ENC_CPR;

        enc_raw_per = raw;
        win_cnt += dcount;

        if(++win_tick >= ENC_WIN_TICKS)
        {
            win_vel_q24 = (int32_t)(((int64_t)win_cnt * ENC_PER_COEF) / (int32_t)ENC_WIN_TICKS);
            win_cnt  = 0;
            win_tick = 0;
        }
    }

    meas = (int32_t)raw << 10;                /* 14 位 -> Q24 圈 */

    /* ---- PLL (全整型) ----
     * 预测: pos += dt*vel = vel/20000  (Q24 圈)
     * 校正: pos += dt*kp*err, vel += dt*ki*err
     * 误差先缠绕到 +-半圈, 多圈时不会因为绝对值大而失锁。 */
    /* 预测步长必须跟实际采样率一致: 降到 10kHz 后这里还写 20000 的话, 每拍只推一半,
     * 位置永远滞后于实测 -> 速度估计自激 (实测 vel 喊 +17 rad/s 而位置在倒退)。 */
    /* 2026-10-18 修: ENC_TICK_HZ 是 unsigned, 写成 "/ ENC_TICK_HZ" 整段就变成
     * 无符号除法 —— 速度一旦为负, 位置每拍跳 +0.0256 圈 (256 圈/s), PLL 速度状态
     * 直接被推到 +-128 圈/s 的 int32 天花板, 电角度乱跳, 电流环饱和。
     * 现象: 静止时一切正常, 一使能 pos 狂涨、th 却几乎不变。 */
    pll_pos_q24 += (int32_t)(((int64_t)pll_vel_q24
                   + (pll_vel_q24 >= 0 ? ((int32_t)ENC_TICK_HZ / 2) : -((int32_t)ENC_TICK_HZ / 2)))
                   / (int32_t)ENC_TICK_HZ);

    err = meas - pll_pos_q24;
    if(err >  (1 << 23)) err -= (1 << 24);
    if(err < -(1 << 23)) err += (1 << 24);

    /* 限幅到 +-1/64 圈: 保证 err*增益不溢出 int32, 大误差时按饱和处理 */
    if(err >  (1 << 18)) err =  (1 << 18);
    if(err < -(1 << 18)) err = -(1 << 18);

    /* int64 相乘: err 钳到 +-2^18 时 2^18*10106 = 2.65e9 已超 int32,
     * 溢出会让"大误差"把速度推错方向, 一旦跑飞就拉不回来。 */
    pll_pos_q24 += (int32_t)(((int64_t)err * (int64_t)pll_kp_q16) >> 16);
    pll_vel_q24 += (int32_t)(((int64_t)err * (int64_t)pll_ki_q8) >> 8);

    /* 静止时把速度咬死到 0, 免得量化噪声让速度环一直输出微小电流 (低速发热/嗡声)。
     * 阈值 = 半个 LSB/拍 对应的速度, 与 ODrive 的 snap_to_zero 同口径:
     *   0.5 * dt * ki / CPR  (圈/s)。 */
    if(pll_vel_q24 < pll_snap_q24 && pll_vel_q24 > -pll_snap_q24) pll_vel_q24 = 0;

    /* 圈数回绕 -> 多圈计数 */
    if(pll_pos_q24 >= (1 << 24))      { pll_pos_q24 -= (1 << 24); enc_rev++; }
    else if(pll_pos_q24 < 0)          { pll_pos_q24 += (1 << 24); enc_rev--; }

    if(pll_vel_q24 < vel_span_min) vel_span_min = pll_vel_q24;
    if(pll_vel_q24 > vel_span_max) vel_span_max = pll_vel_q24;
    enc_turn_q16 = (uint32_t)pll_pos_q24 >> 8;
    /* 编码器现在每 ENC_DECIM 拍更新一次, 电角度外推按 2 拍(=100us)算 */
    enc_step_q16 = ((pll_vel_q24 + (pll_vel_q24 >= 0 ? 5000 : -5000)) / 10000) >> 8;
}

uint32_t Enc_TurnQ16(void)  { return enc_turn_q16; }
int32_t  Enc_StepQ16(void)  { return enc_step_q16; }
float Enc_TurnPos(void)     { return (float)pll_pos_q24 * (1.0f / 16777216.0f); }
void    Enc_SetVelSrc(uint8_t s) { vel_src = (uint8_t)(s > 2u ? 0u : s); }
uint8_t Enc_VelSrc(void)          { return vel_src; }
int32_t Enc_VelQ24(void)
{
    int32_t v = pll_vel_q24;

    if(vel_src == 1u) return pll_vel_q24;
    if(vel_src == 2u) return win_vel_q24;

    /* 低速用 20ms 窗口计数 (逐拍差分只有台阶), 高速交回 PLL (差分本身够细) */
    if(win_vel_q24 < ENC_MID_Q24 && win_vel_q24 > -ENC_MID_Q24) v = win_vel_q24;
    return v;
}

float Enc_TurnVel(void)     { return (float)Enc_VelQ24() * (1.0f / 16777216.0f); }

float Enc_PeriodVel(void)   { return (float)win_vel_q24 * (1.0f / 16777216.0f); }

/* ---- 原始角捕获接口 (见文件上部 cap_* 说明) ---- */
void    Enc_CapEnable(uint8_t on) { cap_en = (uint8_t)(on ? 1u : 0u); cap_div = 0u; }
uint8_t Enc_CapEnabled(void)      { return cap_en; }

/* 有新的捕获样本返回 1: *seq 自增计数, 上位机据此发现丢样 */
int Enc_CapRead(uint16_t *raw, uint32_t *seq)
{
    static uint32_t seen;
    if(cap_seq == seen) return 0;
    seen = cap_seq;
    if(raw) *raw = cap_raw;
    if(seq) *seq = cap_seq;
    return 1;
}
float Enc_MechRad(void)     { return ((float)enc_rev + Enc_TurnPos()) * 6.28318530718f - enc_zero_rad; }
float Enc_MechRadSlow(void) { return Enc_MechRad(); }
uint16_t Enc_Raw(void)      { return enc_raw_last; }
uint8_t  Enc_Mg(void)       { return enc_mg_last; }
uint8_t  Enc_MgValid(void)  { return enc_mg_valid; }
uint32_t Enc_Frames(void)   { return enc_frames; }
uint32_t Enc_ErrCnt(void)   { return enc_err; }
uint32_t Enc_CrcErrCnt(void) { return enc_crc_err; }

void Enc_VelSpan(int32_t *mn, int32_t *mx, int reset)
{
    *mn = vel_span_min; *mx = vel_span_max;
    if(reset) { vel_span_min = 0x7FFFFFFF; vel_span_max = (int32_t)0x80000000; }
}

float Enc_ZeroRad(void) { return enc_zero_rad; }

void Enc_SetMechZero(float rad)
{
    enc_zero_rad = ((float)enc_rev + Enc_TurnPos()) * 6.28318530718f - rad;
}
void Enc_SetZeroRad(float rad) { enc_zero_rad = rad; }
void Enc_SetRev(int32_t rev)   { enc_rev = rev; }
int32_t Enc_Rev(void)          { return enc_rev; }
