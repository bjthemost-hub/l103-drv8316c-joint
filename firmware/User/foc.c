#include "foc.h"
#include "board.h"
#include "csense.h"
#include "drv8316c.h"
#include "foc_fx.h"

Foc_t foc;

/* ---- 定标常数 (全部是算术值, 推导写在注释里) ---- */
#define PWM_PER_COUNTS   ((uint32_t)PWM_ARR() + 1u)      /* 2400 */
#define ADC_TO_I_Q12     11                              /* (3.3/4095/0.3)A/count * 4096 = 11.00 */
/* 2026-10-23: 跳闸阈值改成运行时变量 foc.oc_trip_q12 (原来编译期常量 MOTOR_OC_TRIP_A)。 */
#define DUTY_RECON_Q15   ((uint16_t)(CS_DUTY_RECON_TH * 32768.0f))

static int32_t  s_vbus_q12;
static int32_t  s_kccr;              /* (PER<<16)/vbus : 电压 -> 比较值 */
static int32_t  s_half;              /* 零矢量对应的比较值 */
static int32_t  s_duty_k_q12;        /* 比较值 -> Q15 占空比 */
static int32_t  s_vlim_q12;
static int32_t  s_vlim2_q8;          /* (vlim>>4)^2, 防 int32 平方溢出 */
static int32_t  s_kp_q14, s_ki_q14;
static int32_t  s_integ_d, s_integ_q;   /* Q12 V */

/* -----------------------------------------------------------------------------
 * 一拍 PI。反算抗饱和: 输出被钳住时把积分器拉回到"刚好等于钳位值"的位置,
 * 而不是让它继续积分 (旧固件那种"积分照积"是低速起停时第一下硬冲的来源)。
 * -------------------------------------------------------------------------- */
static inline int32_t pi_step(int32_t *integ, int32_t err, int32_t lo, int32_t hi)
{
    int32_t vp  = (s_kp_q14 * err) >> 14;
    int32_t out = vp + *integ;

    if(out > hi)      { out = hi; *integ = hi - vp; }
    else if(out < lo) { out = lo; *integ = lo - vp; }
    else              { *integ += (s_ki_q14 * err) >> 14; }
    return out;
}

static inline uint16_t clamp_ccr(int32_t v)
{
    if(v < 0) v = 0;
    if(v > (int32_t)PWM_ARR()) v = (int32_t)PWM_ARR();
    return (uint16_t)v;
}

/* 中点注入 SVPWM: 反 Clarke -> 减 (max+min)/2 -> 直接算比较值。
 * 与扇区法等价, 但没有扇区表/atan2; 线性调制区 |V| <= vbus/sqrt3。 */
static void svm_write(int32_t alpha, int32_t beta)
{
    int32_t v2   = (beta * FX_SQRT3_2_Q15) >> 15;
    int32_t va   = alpha;
    int32_t vb   = (-alpha) / 2 + v2;
    int32_t vc   = (-alpha) / 2 - v2;
    int32_t vmax = va, vmin = va, voff;

    if(vb > vmax) vmax = vb;
    if(vc > vmax) vmax = vc;
    if(vb < vmin) vmin = vb;
    if(vc < vmin) vmin = vc;
    voff = -((vmax + vmin) / 2);

    foc.ccr[0] = clamp_ccr(s_half + (((va + voff) * s_kccr) >> 16));
    foc.ccr[1] = clamp_ccr(s_half + (((vb + voff) * s_kccr) >> 16));
    foc.ccr[2] = clamp_ccr(s_half + (((vc + voff) * s_kccr) >> 16));

    foc.duty_q15[0] = (uint16_t)(((int32_t)foc.ccr[0] * s_duty_k_q12) >> 12);
    foc.duty_q15[1] = (uint16_t)(((int32_t)foc.ccr[1] * s_duty_k_q12) >> 12);
    foc.duty_q15[2] = (uint16_t)(((int32_t)foc.ccr[2] * s_duty_k_q12) >> 12);

    /* A -> CH3(PB0) / B -> CH2(PA7) / C -> CH1(PA6); 见 board.h 相名对照表 */
    TIM_SetCompare3(TIM3, foc.ccr[0]);
    TIM_SetCompare2(TIM3, foc.ccr[1]);
    TIM_SetCompare1(TIM3, foc.ccr[2]);
}

static void gains_apply(void)
{
    s_kp_q14 = (int32_t)(foc.kp * 16384.0f);
    s_ki_q14 = (int32_t)(foc.ki * PWM_TS_S * 16384.0f);
}

void Foc_Init(void)
{
    s_vbus_q12 = (int32_t)(MOTOR_VBUS_DEF * 4096.0f);
    s_half     = (int32_t)(PWM_PER_COUNTS / 2u);
    /* CCR -> Q15 占空比。Q15 的"1.0"是 32768, 所以:
     *   duty_q15 = ccr / 2400 * 32768,  这里先乘 k 再 >>12
     *   => k = 32768 * 4096 / 2400 = 55924
     * 2026-10-18 上板两次踩这个数: 先写成 65536 (4x 大), 又改成 16384*4096/2400
     * (2x 小, ccr=1200 印成 0.25)。2x 小的后果不是显示难看 ——
     * DUTY_RECON_Q15 (=0.79*32768) 永远触发不了, 真占空比 0.97 时低边窗口只剩 3%,
     * 相电流读数已经是假的, 电流环会去追这个假值。 */
    s_duty_k_q12 = (int32_t)((32768u * 4096u) / PWM_PER_COUNTS);

    Foc_Reset();
    foc.oc_trip_q12 = (int32_t)(MOTOR_OC_TRIP_A * 4096.0f);   /* 运行时阈值, 先给默认 */
    Foc_SetOcTripMs(MOTOR_OC_TRIP_MS_DEF);                    /* 跳闸延时, 先给默认 */
    Foc_SetVbus(MOTOR_VBUS_DEF);
    Foc_SetGainsFromRL(CUR_R_OHM_DEF, CUR_L_H_DEF, CUR_BW_HZ_DEF);
    foc.enabled = 0u;
}

void Foc_Reset(void)
{
    s_integ_d = s_integ_q = 0;
    foc.id_q12 = foc.iq_q12 = 0;
    foc.vd_q12 = foc.vq_q12 = 0;
    foc.recon_cnt = 0u;
    foc.sat_cnt   = 0u;
    foc.oc        = 0u;
    foc.oc_ms_cnt = 0u;
    foc.oc_win_acc = 0u;
    foc.oc_win_max = 0;
    foc.i_abs_max_q12 = 0;
    Foc_ZeroVector();
}

void Foc_SetGains(float kp, float ki)
{
    if(kp < 0.0f) kp = 0.0f;
    if(kp > 8.0f) kp = 8.0f;          /* kp_q14*16384 必须留在 int32 内 */
    if(ki < 0.0f) ki = 0.0f;
    if(ki > 100000.0f) ki = 100000.0f;

    foc.kp = kp;
    foc.ki = ki;
    gains_apply();
}

void Foc_GetGains(float *kp, float *ki) { if(kp) *kp = foc.kp; if(ki) *ki = foc.ki; }

/* 零极点对消整定: Kp = L*wc, Ki = R*wc, wc = 2*pi*bw。
 * R/L 是**设计值/现场填的实测值**, 默认值见 board.h。 */
void Foc_SetGainsFromRL(float r_ohm, float l_h, float bw_hz)
{
    float wc;

    if(bw_hz <= 0.0f) bw_hz = CUR_BW_HZ_DEF;
    wc = 6.28318530718f * bw_hz;
    Foc_SetGains(l_h * wc, r_ohm * wc);
}

void Foc_SetOcTrip(float a)        { foc.oc_trip_q12 = (int32_t)(a * 4096.0f + 0.5f); }

/* 跳闸延时 (ms)。改值顺手清计数, 免得半途改小/改大时带着旧进度。
 * 判定用 1ms 窗口峰值: PWM 纹波、换相尖峰会把逐拍判断打断, 取窗口峰值才数得下去。 */
void Foc_SetOcTripMs(uint16_t ms)
{
    if(ms > MOTOR_OC_TRIP_MS_MAX) ms = MOTOR_OC_TRIP_MS_MAX;
    foc.oc_trip_ms  = ms;
    foc.oc_ms_cnt   = 0u;
    foc.oc_win_acc  = 0u;
    foc.oc_win_max  = 0;
}

void Foc_SetVbus(float vbus)
{
    int32_t v;

    /* vbus 只是"电压->占空比"的标尺, 超范围只会让线性调制区算错。
     * 上限取 60V (本板设计输入 12~24V, 留一倍余量), 下限 0.5V 防除零。 */
    if(vbus > 60.0f)  vbus = 60.0f;
    if(vbus < 0.5f)   vbus = 0.5f;

    foc.vbus = vbus;
    v  = (int32_t)(vbus * 4096.0f);
    s_vbus_q12  = v;
    s_kccr      = (int32_t)(((uint32_t)PWM_PER_COUNTS << 16) / (uint32_t)v);
    foc.v_limit = 0.95f * vbus * 0.57735026919f;
    s_vlim_q12  = (int32_t)(foc.v_limit * 4096.0f);
    s_vlim2_q8  = (s_vlim_q12 >> 4) * (s_vlim_q12 >> 4);
}

float Foc_GetVbus(void) { return foc.vbus; }

void Foc_SetIqRefs(float iq, float id)
{
    Foc_SetIqRefsQ12((int32_t)(iq * 4096.0f), (int32_t)(id * 4096.0f));
}

void Foc_SetIqRefsQ12(int32_t iq_q12, int32_t id_q12)
{
    foc.iq_ref_q12 = iq_q12;
    foc.id_ref_q12 = id_q12;
}

/* -----------------------------------------------------------------------------
 * 相电流: ADC 计数 -> Q12 安。全整型, 一次减法一次乘。
 * 占空比过大那一相此刻低边窗口不够, 用 ia+ib+ic=0 补回来 (判据见 board.h)。
 * 返回 1 = 本拍拿到了有效相电流。
 * -------------------------------------------------------------------------- */
int Foc_MeasureCurrent(void)
{
    uint16_t raw[3];
    int32_t  i, amax = 0;

    if(!CS_Valid()) { foc.valid = 0u; return 0; }

    CS_GetRawIsr(raw);
    for(i = 0; i < 3; i++)
    {
        int32_t d = ((int32_t)raw[i] - CS_ZeroRaw((uint8_t)i)) * ADC_TO_I_Q12;
        if(CS_SignI((uint8_t)i) < 0) d = -d;
        foc.i_q12[i] = d;
        if(d < 0) d = -d;
        if(d > amax) amax = d;
    }

    /* 占空比最大的一相读数不可信 -> 用另外两相按 KCL 补 */
    {
        uint16_t dmax = foc.duty_q15[0];
        uint8_t  im   = 0u;

        if(foc.duty_q15[1] > dmax) { dmax = foc.duty_q15[1]; im = 1u; }
        if(foc.duty_q15[2] > dmax) { dmax = foc.duty_q15[2]; im = 2u; }

        if(dmax > DUTY_RECON_Q15)
        {
            int32_t oth = 0;
            for(i = 0; i < 3; i++) if(i != (int32_t)im) oth += foc.i_q12[i];
            foc.i_q12[im] = -oth;
            amax = 0;
            for(i = 0; i < 3; i++) { int32_t t = foc.i_q12[i] < 0 ? -foc.i_q12[i] : foc.i_q12[i]; if(t > amax) amax = t; }
            if(foc.recon_cnt < 0xFFFFFFFFu) foc.recon_cnt++;
        }
    }

    foc.valid = 1u;
    foc.i_abs_max_q12 = amax;
    /* 2026-10-23: 过流要"连续顶住 oc_trip_ms 毫秒"才跳。
     * 逐拍判会被 PWM 纹波打断 (实测 20kHz 下峰值在阈值上下跳, 计数每 1~2ms 就归零),
     * 所以先按 1ms 窗口取 |i| 峰值, 再拿窗口峰值跟阈值比; 窗口内没人超限才清零重计。 */
    if(foc.oc_trip_ms == 0u)
    {
        if(amax > foc.oc_trip_q12) foc.oc = 1u;          /* 0ms = 老行为: 一拍就跳 */
    }
    else
    {
        if(amax > foc.oc_win_max) foc.oc_win_max = amax;
        if(++foc.oc_win_acc >= (uint16_t)(PWM_FREQ_HZ / 1000u))
        {
            foc.oc_win_acc = 0u;
            if(foc.oc_win_max > foc.oc_trip_q12)
            {
                if(foc.oc_ms_cnt < 0xFFFFu) foc.oc_ms_cnt++;
                if(foc.oc_ms_cnt >= foc.oc_trip_ms) foc.oc = 1u;
            }
            /* 回差 12.5%: 已在计数时, 峰值没掉到阈值的 87.5% 以下就继续数,
             * 别让纹波在阈值边上把计时来回清零。 */
            else if(foc.oc_win_max < (foc.oc_trip_q12 - (foc.oc_trip_q12 >> 3)))
            {
                foc.oc_ms_cnt = 0u;
            }
            foc.oc_win_max = 0;
        }
    }
    return 1;
}

void Foc_ZeroVector(void)
{
    foc.ccr[0] = foc.ccr[1] = foc.ccr[2] = (uint16_t)s_half;
    foc.duty_q15[0] = foc.duty_q15[1] = foc.duty_q15[2] = 16384u;
    TIM_SetCompare3(TIM3, foc.ccr[0]);
    TIM_SetCompare2(TIM3, foc.ccr[1]);
    TIM_SetCompare1(TIM3, foc.ccr[2]);
}

/* 开环电压矢量: 不走 PI, 直接 反Park -> SVPWM。用于对齐与 V/f 自检。 */
void Foc_RunVoltQ12(uint16_t theta_e, int32_t d, int32_t q)
{
    int32_t al, be;

    (void)Foc_MeasureCurrent();       /* 开环也做同一份过流判决 */

    if(d >  s_vlim_q12) d =  s_vlim_q12;
    if(d < -s_vlim_q12) d = -s_vlim_q12;
    if(q >  s_vlim_q12) q =  s_vlim_q12;
    if(q < -s_vlim_q12) q = -s_vlim_q12;

    fx_inv_park(d, q, fx_sin(theta_e), fx_cos(theta_e), &al, &be);
    svm_write(al, be);
    foc.vd_q12 = d;
    foc.vq_q12 = q;
}

void Foc_RunVolt(uint16_t theta_e, float vd, float vq)
{
    Foc_RunVoltQ12(theta_e, (int32_t)(vd * 4096.0f), (int32_t)(vq * 4096.0f));
}

/* -----------------------------------------------------------------------------
 * 一拍电流环, 20kHz。theta_e = 本拍要用的电角度 (motor.c 已做外推补偿)。
 * -------------------------------------------------------------------------- */
void Foc_Run(uint16_t theta_e)
{
    int32_t s, c, alpha, beta, id, iq, vd, vq, err;

    if(!Foc_MeasureCurrent())
    {
        /* 采样链没通: 不出力矩, 清积分免得恢复时带着旧积分冲一下 */
        s_integ_d = s_integ_q = 0;
        Foc_ZeroVector();
        return;
    }

    s = fx_sin(theta_e);
    c = fx_cos(theta_e);

    fx_clarke(foc.i_q12[0], foc.i_q12[1], foc.i_q12[2], &alpha, &beta);
    fx_park(alpha, beta, s, c, &id, &iq);
    foc.id_q12 = id;
    foc.iq_q12 = iq;

    err = foc.id_ref_q12 - id;
    vd  = pi_step(&s_integ_d, err, -s_vlim_q12, s_vlim_q12);

    err = foc.iq_ref_q12 - iq;
    vq  = pi_step(&s_integ_q, err, -s_vlim_q12, s_vlim_q12);

    /* 电压圆: 合成矢量顶出去时等比缩回来, 并让两个积分器各退 1%
     * (ODrive foc.cpp 的 mod_scalefactor < 1 分支, 防饱和时积分继续爬) */
    {
        int32_t d8 = vd >> 4, q8 = vq >> 4;
        int32_t m2 = d8 * d8 + q8 * q8;

        if(m2 > s_vlim2_q8)
        {
            uint32_t m = fx_isqrt32((uint32_t)m2);
            int32_t  vl8 = s_vlim_q12 >> 4;

            if(m == 0u) m = 1u;
            vd = (int32_t)(((int64_t)vd * vl8) / (int64_t)m);
            vq = (int32_t)(((int64_t)vq * vl8) / (int64_t)m);
            s_integ_d -= s_integ_d / 100;
            s_integ_q -= s_integ_q / 100;
            if(foc.sat_cnt < 0xFFFFFFFFu) foc.sat_cnt++;
        }
    }

    foc.vd_q12 = vd;
    foc.vq_q12 = vq;

    fx_inv_park(vd, vq, s, c, &alpha, &beta);
    svm_write(alpha, beta);
}
