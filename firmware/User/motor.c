#include "motor.h"
#include "board.h"
#include "enc.h"
#include "csense.h"
#include "drv8316c.h"
#include "foc.h"
#include "pi.h"
#include "debug.h"
#include "motion.h"

Motor_t motor;

/* 分频必须能整除, 否则 20k/1k/200Hz 的相位会漂 */
typedef char motor_div_check_1[(SPD_LOOP_DIV % CUR_LOOP_DIV == 0u) ? 1 : -1];
typedef char motor_div_check_2[(POS_LOOP_DIV % SPD_LOOP_DIV == 0u) ? 1 : -1];

#define MOTOR_TS        PWM_TS_S
#define ANGLE_ADV_S     (1.0f * PWM_TS_S)     /* 见 motor.h 抬头: 电压在下一个周期中心生效 */
#define ALIGN_RAMP_MS   200u                  /* 对齐电压爬升时间 (设计值) */

/* "先下目标、下一拍再使能"这条 pending 的寿命。
 * 正常路径一拍 (50us) 之内就被 ISR 吃掉; 留到下一次使能才生效的, 一定是上一条
 * 遗留命令 —— 100ms 还没被消费就作废, 不许它把轴拽走。 */
#define POS_PEND_TTL_TICKS  2000u             /* 2000 x 50us = 100ms */

static PI_t  pi_spd;
static float pp_f;
static int   pole_pairs;
static float vbus_meas;
static int32_t phase_off_q16;                 /* 电角度零点 (电 Q16 圈) */

static uint32_t peak_cnt;
static uint32_t valley_cnt;
static uint16_t isr_us;

static uint32_t theta_ol_q32;         /* 开环电角度, Q32 圈: 低频也不会被整数步进截断 */
static uint32_t ol_step_q32;
static float    v_ol;
static int32_t  v_ol_q12;             /* v_ol 的 Q12 镜像: 20kHz 里不再做浮点转换 */
static float    f_ol;
static uint8_t  align_then_run;
static uint32_t align_ticks;
static float    align_ramp_v;
static uint8_t  en_req;

static uint32_t sat_seen;                     /* 上次看到的电压圆饱和计数 */
static float    fw_decay_acc;                 /* 弱磁回退累加器 (s) */

/* ---- 速度环 (全整型) ----
 * 2026-10-18 实测: 这段原来用浮点, 单次在谷值中断里吃掉 ~20us, 加上编码器读取的
 * 21us 峰值中断, 21+29=50us 正好把 PWM 周期占满 —— 主循环收不到串口字节,
 * 命令会静默丢失, 速度环也跟着自激。改成 Q12/Q16 整型后不再有软浮点调用。
 *   rad/s 用 Q12, 电流用 Q12 (A x 4096)
 */
static int32_t spd_kp_q16;        /* A per (rad/s) */
static int32_t spd_ki_q16;        /* A per (rad/s) 每速度拍 */
static int32_t spd_integ_q16;     /* 积分项 (A x 65536)。
                                  * 2026-10-18 实测: 用 Q12 时 ki*ts 的每拍增量
                                  * (ki=0.3, err=0.5rad/s -> 0.6 LSB) 被右移截成 0,
                                  * 积分器根本不长 (5s 后 iq 仍 0.01A, 电机不动)。 */
static int32_t spd_ref_q12;       /* 速度给定 (rad/s) */
static int32_t spd_acc_step_q12;  /* 每个速度拍允许的变化量 (rad/s) */
static int32_t s_i_lim_q12;       /* 电流限幅 (A) */
static int32_t s_vel_lim_q12;     /* 速度限幅 (rad/s) */

/* 2026-10-23: 到位静止锁定 + 速度反馈低通 (口径见 board.h 的 POS_HOLD_* / SPD_VEL_LP_HZ) */
static float   vel_lp;            /* 低通后的机械角速度 rad/s (只给环路与判定用) */
static float   vel_lp_alpha;      /* 一阶低通系数, spd_int_apply() 里按 SPD_VEL_LP_HZ 算 */
static uint8_t pos_hold;          /* 1 = 到位锁定中 (冻结速度环积分) */
static uint8_t pos_hold_cnt;      /* 200Hz 连续满足进入条件的计数 */
#if POS_HOLD_EN
static uint8_t pos_hold_en = 1u;  /* 运行时可关: 串口 PH0 / PH1 */
#else
static uint8_t pos_hold_en = 0u;
#endif

static void spd_int_apply(void)
{
    spd_kp_q16       = (int32_t)(pi_spd.kp * 65536.0f + 0.5f);
    spd_ki_q16       = (int32_t)(pi_spd.ki * ((float)SPD_LOOP_DIV * MOTOR_TS) * 65536.0f + 0.5f);
    spd_acc_step_q12 = (int32_t)(motor.accel_limit * ((float)SPD_LOOP_DIV * MOTOR_TS) * 4096.0f + 0.5f);
    s_i_lim_q12      = (int32_t)(motor.i_limit * 4096.0f + 0.5f);
    s_vel_lim_q12    = (int32_t)(motor.vel_limit * 4096.0f + 0.5f);

    {
        float w  = 6.28318530718f * SPD_VEL_LP_HZ;
        float ts = (float)SPD_LOOP_DIV * MOTOR_TS;
        vel_lp_alpha = (w * ts) / (1.0f + w * ts);
    }
}

/* 与 PI_RunLim 同口径 (含抗饱和: 顶住时积分被拉回到边界) */
static inline int32_t spd_pi_q12(int32_t err_q12)
{
    int32_t vp  = (int32_t)(((int64_t)spd_kp_q16 * (int64_t)err_q12) >> 16);
    int32_t out = vp + (spd_integ_q16 >> 4);

    if(out > s_i_lim_q12)      { out = s_i_lim_q12;  spd_integ_q16 = (int32_t)((int64_t)(s_i_lim_q12 - vp) << 4); }
    else if(out < -s_i_lim_q12){ out = -s_i_lim_q12; spd_integ_q16 = (int32_t)((int64_t)(-s_i_lim_q12 - vp) << 4); }
    else                       { spd_integ_q16 += (int32_t)(((int64_t)spd_ki_q16 * (int64_t)err_q12) >> 12); }
    return out;
}

/* ---- 速度环整定 ----
 * 一阶被控对象 + PI 的常规口径: Kp = J*wc/Kt。J / Kt 没有实测值, 所以不用模型算,
 * 直接用 (i_limit, vel_limit, 带宽) 给一个起点, 现场用 'S' 改。
 * 这里都是**设计值**, 不是实测值。 */
static void speed_pi_tune(float bw_hz)
{
    float kp, ki;

    if(bw_hz <= 0.0f) bw_hz = SPD_BW_HZ_DEF;
    kp = (motor.i_limit / (motor.vel_limit > 0.0f ? motor.vel_limit : 1.0f)) * (bw_hz / 3.0f);
    ki = kp * (6.28318530718f * bw_hz);
    PI_Init(&pi_spd, kp, ki, (float)SPD_LOOP_DIV * MOTOR_TS, -motor.i_limit, motor.i_limit);
    spd_integ_q16 = 0;
    spd_int_apply();
}

static void drv_on(void)
{
    DRV8316C_SetDuty(0.5f, 0.5f, 0.5f);
    DRV8316C_Enable();
}

static void drv_off(void)
{
    DRV8316C_Disable();
    Foc_Reset();
}

static void latch_fault(uint32_t code)
{
    motor.fault      = 1u;
    motor.fault_code = code;
    motor.enabled    = 0u;
    motor.mode       = MOT_OFF;
    motor.iq_ref     = 0.0f;
    motor.vel_ref    = 0.0f;
    motor.id_fw      = 0.0f;
    en_req           = 0u;
    drv_off();
}

/* 电角度 = (机械圈 * 极对数 + 零点) 取小数, 再外推一个 PWM 周期。
 * 用"圈"做中间量: 多圈之后不会像 mech*pp 那样丢精度。 */
static inline uint16_t elec_angle_u16(void)
{
    /* 全整型: (机械 Q16 圈 + 一个周期外推) * 极对数 + 零点, 取低 16 位就是小数部分。
     * 量程: 65536 * 64 = 4.2e6, int32 装得下。 */
    int32_t tq = (int32_t)(Enc_TurnQ16() & 0xFFFFu) + Enc_StepQ16();

    return (uint16_t)(tq * pole_pairs + phase_off_q16);
}

void Motor_Init(void)
{
    motor.mode      = MOT_OFF;
    motor.mode_req  = MOT_OFF;
    motor.iq_cmd    = 0.0f;
    motor.vel_cmd   = 0.0f;
    motor.pos_cmd   = 0.0f;
    motor.i_limit   = CUR_LIMIT_A_DEF;
    motor.oc_trip   = MOTOR_OC_TRIP_A;
    motor.oc_trip_ms = MOTOR_OC_TRIP_MS_DEF;
    motor.r_ohm     = CUR_R_OHM_DEF;
    motor.l_h       = CUR_L_H_DEF;
    motor.vel_limit = VEL_LIMIT_DEF;
    motor.accel_limit = 20.0f * VEL_LIMIT_DEF;
    motor.pos_kp    = POS_KP_DEF;
    motor.v_align   = ALIGN_VOLT_DEF;
    motor.align_ms  = ALIGN_TIME_MS_DEF;
    motor.id_fw     = 0.0f;
    motor.fw_enable = 0u;

    motor.theta_e_u16 = 0u;
    motor.theta_e = 0.0f;
    motor.omega_m = 0.0f;
    /* 2026-10-23: 上电就显示恢复出来的机械坐标 (posstore 已把零点/圈数灌进 enc)。
     * 以前这里写 0, 使能前 ? 和上位机都显示 0, 看着像没恢复。 */
    motor.pos_m = Enc_MechRad();
    motor.vel_ref = 0.0f;
    motor.iq_ref = 0.0f;
    motor.vbus = MOTOR_VBUS_DEF;
    motor.enabled = 0u;
    motor.aligned = 0u;
    motor.fault = 0u;
    motor.fault_code = 0u;
    motor.isr_us_max = 0u;
    motor.isr_pk_us_max = 0u;
    motor.isr_pk_us_last = 0u;

    pole_pairs = MOTOR_POLE_PAIRS;
    pp_f       = (float)pole_pairs;
    vbus_meas  = MOTOR_VBUS_DEF;
    phase_off_q16 = 0;

    theta_ol_q32 = (uint32_t)elec_angle_u16() << 16; ol_step_q32 = 0u; v_ol = 0.0f; v_ol_q12 = 0; f_ol = 0.0f;
    align_then_run = 0u; align_ticks = 0u; align_ramp_v = 0.0f;
    en_req = 0u;
    peak_cnt = valley_cnt = 0u;
    sat_seen = 0u; fw_decay_acc = 0.0f;
    spd_ref_q12 = 0; spd_integ_q16 = 0;

    /* 2026-10-18: 默认增益改成直接给的数值 (见 board.h). 带宽公式值在
     * 50Hz 窗口测速下自激; 需要按带宽重算时再显式调 speed_pi_tune()。 */
    Motor_SetSpeedGains(SPD_KP_DEF, SPD_KI_DEF);
    Foc_Init();
}

void Motor_SetPolePairs(int pp)
{
    if(pp < 1) pp = 1;
    if(pp > 64) pp = 64;

    /* 2026-10-23: 同值不清对齐。GUI 每次"写参数 / 存 Flash"都把 16 项全写一遍
     * (含极对数 11), 以前无条件 motor.aligned=0 -> 一写参数就丢掉对齐,
     * 之后点速度/走位全被 "not aligned" 拒掉。 */
    if(pp == pole_pairs) return;

    pole_pairs = pp;
    pp_f = (float)pp;
    motor.aligned = 0u;
}

int   Motor_PolePairs(void) { return pole_pairs; }
void  Motor_SetVbus(float v) { Foc_SetVbus(v); motor.vbus = Foc_GetVbus(); }
float Motor_VbusMeas(void)   { return vbus_meas; }

void Motor_SetCurGains(float kp, float ki) { Foc_SetGains(kp, ki); }

void Motor_GetCurGains(float *kp, float *ki)
{
    float a, b;
    Foc_GetGains(&a, &b);
    if(kp) *kp = a;
    if(ki) *ki = b;
}

void Motor_SetGainsFromRL(float r_ohm, float l_henry, float bw_hz)
{
    if(r_ohm   > 0.0f) motor.r_ohm = r_ohm;
    if(l_henry > 0.0f) motor.l_h   = l_henry;
    Foc_SetGainsFromRL(r_ohm, l_henry, bw_hz);
}

void Motor_GetRL(float *r_ohm, float *l_h)
{
    if(r_ohm) *r_ohm = motor.r_ohm;
    if(l_h)   *l_h   = motor.l_h;
}

void Motor_SetSpeedGains(float kp, float ki)
{
    pi_spd.kp = kp;
    pi_spd.ki = ki;
    pi_spd.ts = (float)SPD_LOOP_DIV * MOTOR_TS;
    pi_spd.out_min = -motor.i_limit;
    pi_spd.out_max =  motor.i_limit;
    spd_int_apply();
}

void Motor_GetSpeedGains(float *kp, float *ki)
{
    if(kp) *kp = pi_spd.kp;
    if(ki) *ki = pi_spd.ki;
}

void Motor_SetPosKp(float kp)     { if(kp > 0.0f) motor.pos_kp = kp; }
void Motor_SetILimit(float a)     { if(a > 0.0f) { motor.i_limit = a; pi_spd.out_min = -a; pi_spd.out_max = a; spd_int_apply(); } }

/* 相电流立即跳闸阈值。范围在 can_bus/cmd 那一层卡, 这里只做钳位与下发。 */
void Motor_SetOcTrip(float a)
{
    if(a < MOTOR_OC_TRIP_MIN) a = MOTOR_OC_TRIP_MIN;
    if(a > MOTOR_OC_TRIP_MAX) a = MOTOR_OC_TRIP_MAX;
    motor.oc_trip = a;
    Foc_SetOcTrip(a);
}
float Motor_GetOcTrip(void)       { return motor.oc_trip; }

/* 过流跳闸延时: 连续超限多久才关桥。范围在 can_bus/cmd 那一层卡, 这里只钳位。 */
void Motor_SetOcTripMs(uint16_t ms)
{
    if(ms > MOTOR_OC_TRIP_MS_MAX) ms = (uint16_t)MOTOR_OC_TRIP_MS_MAX;
    motor.oc_trip_ms = ms;
    Foc_SetOcTripMs(ms);
}
uint16_t Motor_GetOcTripMs(void)  { return motor.oc_trip_ms; }
void Motor_SetVelLimit(float v)   { if(v > 0.0f) { motor.vel_limit = v; spd_int_apply(); } }
void Motor_SetAccelLimit(float a) { if(a > 0.0f) { motor.accel_limit = a; spd_int_apply(); } }
void Motor_SetAlignVolt(float v)  { if(v > 0.0f) motor.v_align = v; }
void Motor_SetPosHold(uint8_t en)
{
    pos_hold_en = (uint8_t)(en ? 1u : 0u);
    if(!pos_hold_en) { pos_hold = 0u; pos_hold_cnt = 0u; }
}
uint8_t Motor_GetPosHold(void)    { return pos_hold_en; }
uint8_t Motor_HoldActive(void)     { return pos_hold; }
float   Motor_VelLp(void)          { return vel_lp; }
void Motor_SetFw(uint8_t en)      { motor.fw_enable = (uint8_t)(en ? 1u : 0u); if(!en) motor.id_fw = 0.0f; }
float Motor_GetIsrUsMax(void)     { return (float)motor.isr_us_max; }

void Motor_RequestMode(Mot_Mode m)
{
    if((int)m < 0 || (int)m > (int)MOT_OPENLOOP) return;
    motor.mode_req = m;
}

/* 2026-10-18: CAN 的 PV 命令会"先请求 POSITION 再下目标", 而 ISR 切进 POSITION 时
 * 会把 pos_cmd 改成当前角度 —— 两条命令之间只隔一拍 (50us), 顺序没保证。
 * 这里的 pending 让"目标"跟着模式切换一起在 ISR 里落定:
 *   调用时已经在 POSITION -> 直接生效, 清 pending;
 *   调用时还没切过去      -> 记进 pending, ISR 切模式时用它代替 Enc_MechRad()。
 * 两种情况都不会丢掉目标, 也不会让 ISR 覆盖主循环刚下的值。
 *
 * 2026-10-19 加寿命 (见 POS_PEND_TTL_TICKS): 只认"这一次动作刚下的目标"。 */
static volatile float   pos_pend_rad;
static volatile uint8_t pos_pend;
static volatile uint16_t pos_pend_ttl;

void Motor_SetPosition(float r)
{
    motor.pos_cmd = r;
    if(motor.mode == MOT_POSITION)
    {
        pos_pend = 0u;
    }
    else
    {
        pos_pend_rad = r;
        pos_pend_ttl = POS_PEND_TTL_TICKS;
        pos_pend     = 1u;
    }
}

/* 2026-10-19 (家人: "不能乱跑啊 会出人命的"): 把命令侧一次清干净, 目标贴回当前位置。
 * 以前 Motor_Disable() 只清 en_req, motor.pos_cmd 还留着上一条目标 (实测: 轴在
 * -19.72 rad, 里面躺着 tgt=-55.52 rad, 差 5.7 圈)。谁在"使能"和"下发新目标"之间
 * 插进来一拍, 位置环就朝那个旧目标冲出去。 */
void Motor_ClearCommands(void)
{
    pos_pend      = 0u;
    pos_pend_ttl  = 0u;
    pos_pend_rad  = 0.0f;
    motor.pos_cmd = Enc_MechRad();
    motor.vel_cmd = 0.0f;
    motor.iq_cmd  = 0.0f;
    motor.vel_ref = 0.0f;
    pos_hold      = 0u;
    pos_hold_cnt  = 0u;
    vel_lp        = 0.0f;
    spd_integ_q16 = 0;
}

void Motor_Enable(void)            { en_req = 1u; }
void Motor_Disable(void)           { en_req = 0u; Motor_ClearCommands(); }
void Motor_SetIq(float a)          { motor.iq_cmd = a; }
void Motor_SetVelocity(float r)    { motor.vel_cmd = r; }
void Motor_ClearFault(void)        { motor.fault = 0u; motor.fault_code = 0u; foc.oc = 0u; }

void Motor_SetFault(uint32_t code)
{
    if(!motor.fault) { motor.fault = 1u; motor.fault_code = code; }
    en_req = 0u;
}

void Motor_StartAlign(void)
{
    align_then_run = 0u;
    motor.mode_req = MOT_ALIGN;
    en_req = 1u;
}

void Motor_StartOpenloop(float volts, float hz)
{
    float vmax = 0.90f * Foc_GetVbus() * 0.57735026919f;

    if(volts < 0.0f) volts = -volts;
    if(volts > vmax) volts = vmax;
    if(hz < 0.0f)    hz = 0.0f;
    if(hz > 1000.0f) hz = 1000.0f;      /* 电频率上限: 3000rpm x 11 极对 = 550Hz */

    v_ol = volts;
    v_ol_q12 = (int32_t)(volts * 4096.0f);
    f_ol = hz;
    /* 从实测电角度接着走: 清 0 会在换频/重进开环时甩一个矢量跳变 (实测 5->8Hz 冲到 3.0A) */
    theta_ol_q32 = (uint32_t)elec_angle_u16() << 16;
    /* Q32: 2Hz 时每拍增量 = 2*50us*2^32 = 429497, 精度远好于 Q16 的 6.55->6 */
    ol_step_q32  = (uint32_t)(hz * MOTOR_TS * 4294967296.0f + 0.5f);
    motor.mode_req = MOT_OPENLOOP;
    en_req = 1u;
}

/* =============================================================================
 * 峰值 UEV: 起三相注入转换 + 读编码器 + PLL (20kHz)
 * ========================================================================== */
void Motor_PeakIsr(void)
{
    uint32_t t0 = Micros();

    peak_cnt++;
    motor.isr_pk_cnt = peak_cnt;
    CS_StartSampleIsr();
    if((peak_cnt % ENC_DECIM) == 0u) Enc_PollIsr();

    {
        uint16_t us = (uint16_t)(Micros() - t0);
        motor.isr_pk_us_last = us;
        if(us > motor.isr_pk_us_max) motor.isr_pk_us_max = us;
    }
}

/* =============================================================================
 * 谷值 UEV: 电流环 (20kHz) + 速度环 (1kHz) + 位置环 (200Hz)
 * ========================================================================== */
void Motor_ValleyIsr(void)
{
    uint32_t t0 = Micros();
    uint32_t tick;
    uint16_t theta;

    valley_cnt++;
    tick = valley_cnt;
    CS_LatchIsr();

    /* ---- 使能 / 失能 / 模式切换 ---- */
    if(!en_req)
    {
        /* 还没落地的目标: 过期就作废, 不许它等到下一次使能才复活 */
        if(pos_pend)
        {
            if(pos_pend_ttl > 0u) pos_pend_ttl--;
            if(pos_pend_ttl == 0u) pos_pend = 0u;
        }
        if(motor.enabled || DRV8316C_IsEnabled())
        {
            drv_off();
            motor.enabled = 0u;
            motor.mode    = MOT_OFF;
            motor.id_fw   = 0.0f;
        }
        /* 失能期间目标一直贴住当前位置: '?' 里的 tgt 就是"现在在哪",
         * 下一次使能也没有旧目标可捡 */
        motor.pos_cmd = Enc_MechRad();
        motor.vel_ref = 0.0f;
    }
    else if(!motor.enabled)
    {
        if(motor.fault || DRV8316C_Faulted())
        {
            if(!motor.fault) { motor.fault = 1u; motor.fault_code = 2u; }
            en_req = 0u;
        }
        else if(motor.mode_req == MOT_ALIGN || motor.mode_req == MOT_OPENLOOP)
        {
            motor.mode = motor.mode_req;
            motor.enabled = 1u;
            align_ticks = 0u; align_ramp_v = 0.0f;
            Foc_Reset();
            drv_on();
        }
        else if(!motor.aligned)
        {
            motor.mode = MOT_ALIGN;
            motor.enabled = 1u;
            align_then_run = 1u;
            align_ticks = 0u; align_ramp_v = 0.0f;
            Foc_Reset();
            drv_on();
        }
        else
        {
            motor.mode = motor.mode_req;
            motor.enabled = 1u;
            Foc_Reset();
            PI_Reset(&pi_spd);
            motor.omega_m = Enc_TurnVel() * 6.28318530718f;
            /* 新使能 = 从"停住"起: 速度给定 0, 不拿测速值当给定。
             * 2026-10-19 实测 '?' 里 vel=137.7 rad/s 而轴是静止的 —— 拿它当给定
             * 会在合闸瞬间顶一下电流。"接着跑"的是运行中换模式那条, 不是这里。 */
            motor.vel_ref = 0.0f;
            spd_ref_q12   = 0;
            spd_integ_q16 = 0;
            motor.pos_cmd = pos_pend ? pos_pend_rad : Enc_MechRad();
            pos_pend      = 0u;
            motor.id_fw   = 0.0f;
            drv_on();
        }
    }
    else if(motor.mode_req != motor.mode &&
            motor.mode != MOT_ALIGN &&
            motor.mode_req != MOT_OFF)
    {
        /* 运行中换模式: 速度给定从当前速度接着走, 不要从 0 硬起 */
        motor.mode = motor.mode_req;
        PI_Reset(&pi_spd);
        motor.omega_m = Enc_TurnVel() * 6.28318530718f;
        motor.vel_ref = motor.omega_m;
        spd_ref_q12   = (int32_t)(motor.omega_m * 4096.0f);
        spd_integ_q16 = 0;
        if(motor.mode == MOT_POSITION)
            motor.pos_cmd = pos_pend ? pos_pend_rad : Enc_MechRad();
        if(motor.mode == MOT_TORQUE)   motor.id_fw = 0.0f;
        pos_pend = 0u;
    }

    /* ---- 本拍电角度 ---- */
    if(motor.mode == MOT_ALIGN || motor.mode == MOT_OPENLOOP) theta = (uint16_t)(theta_ol_q32 >> 16);
    else                                                      theta = elec_angle_u16();

    motor.theta_e_u16 = theta;

    /* ---- 按模式跑 ---- */
    switch(motor.mode)
    {
    case MOT_OFF:
        Foc_ZeroVector();
        break;

    case MOT_ALIGN:
    {
        uint32_t ramp_ticks = (ALIGN_RAMP_MS * PWM_FREQ_HZ) / 1000u;
        uint32_t hold_ticks = (motor.align_ms * PWM_FREQ_HZ) / 1000u;

        align_ticks++;
        if(align_ticks < ramp_ticks)
            align_ramp_v = motor.v_align * (float)align_ticks / (float)ramp_ticks;
        else
            align_ramp_v = motor.v_align;

        Foc_RunVoltQ12(0u, (int32_t)(align_ramp_v * 4096.0f), 0);

        if(align_ticks >= (ramp_ticks + hold_ticks))
        {
            /* 对齐结束: 把此刻的电角度定义成 0 */
            phase_off_q16 = -(int32_t)((float)Enc_TurnQ16() * pp_f) & 0xFFFF;
            /* 机械零点是操作员用"设置机械 0 点"定的坐标概念, 对齐只管电角度
             * (phase_off 用的是编码器本圈 Q16, 跟机械零点无关)。已经设过零位的
             * 板子, 对齐不许把它挪走 —— 否则 GUI 上的机械坐标会整体平移。 */
            if(!Motion_HomeSet()) Enc_SetMechZero(0.0f);
            motor.aligned = 1u;
            Foc_ZeroVector();
            motor.id_fw = 0.0f;

            if(align_then_run)
            {
                align_then_run = 0u;
                motor.enabled  = 1u;
                motor.mode     = motor.mode_req;
                if(motor.mode == MOT_OFF) motor.mode = MOT_TORQUE;
                Foc_Reset();
                PI_Reset(&pi_spd);
                motor.omega_m = 0.0f;
                motor.vel_ref = 0.0f;
                spd_ref_q12   = 0;
                spd_integ_q16 = 0;
                /* 2026-10-23: 对齐把电角度零点重算了一遍, 对齐前记下的目标已经
                 * 不属于同一个坐标系 (实测: 对齐出来 pos_cmd 与当前位置差 113 rad,
                 * 随即跑了约 18 圈)。目标一律贴回当前角度, 不许再捡 pos_pend。 */
                pos_pend      = 0u;
                motor.pos_cmd = Enc_MechRad();
            }
            else
            {
                motor.mode = MOT_OFF;
                motor.mode_req = MOT_OFF;   /* 别留着 ALIGN: 再按 'e' 会重新对齐 */
                en_req     = 0u;
                motor.enabled = 0u;
            }
        }
        break;
    }

    case MOT_OPENLOOP:
        theta_ol_q32 += ol_step_q32;
        if((tick % SPD_LOOP_DIV) == 0u)
        {
            /* 开环时也把 PLL 的实测速度/位置写出来, 方便对着 '?' 核转速 */
            motor.omega_m = Enc_TurnVel() * 6.28318530718f;
            motor.pos_m   = Enc_MechRad();
        }
        Foc_RunVoltQ12((uint16_t)(theta_ol_q32 >> 16), 0, v_ol_q12);
        break;

    case MOT_TORQUE:
    case MOT_VELOCITY:
    case MOT_POSITION:
    default:
    {
        /* ---- 200Hz 位置环 -> 速度给定 (浮点, 200Hz 一次, 不占预算) ---- */
        if((tick % POS_LOOP_DIV) == 0u)
        {
            motor.pos_m = Enc_MechRad();
            if(motor.mode == MOT_POSITION)
            {
                float err = motor.pos_cmd - motor.pos_m;
                float v;

                if(err >  POS_ERR_LIMIT_RAD) err =  POS_ERR_LIMIT_RAD;
                if(err < -POS_ERR_LIMIT_RAD) err = -POS_ERR_LIMIT_RAD;
                /* 到位判定: 误差小 + 速度小, 连续 50ms 才认 (见 board.h 说明) */
                if(pos_hold_en)
                {
                    float ae = (err < 0.0f) ? -err : err;
                    float av = (vel_lp < 0.0f) ? -vel_lp : vel_lp;

                    if(!pos_hold)
                    {
                        if(ae < POS_HOLD_ENTER_RAD && av < POS_HOLD_VEL_RAD_S)
                        {
                            if(++pos_hold_cnt >= POS_HOLD_TICKS)
                            {
                                pos_hold     = 1u;
                                pos_hold_cnt = 0u;
                            }
                        }
                        else
                        {
                            pos_hold_cnt = 0u;
                        }
                    }
                    else if(ae > POS_HOLD_EXIT_RAD)
                    {
                        pos_hold = 0u;
                    }
                }
                v = pos_hold ? 0.0f : (motor.pos_kp * err);
                if(v >  motor.vel_limit) v =  motor.vel_limit;
                if(v < -motor.vel_limit) v = -motor.vel_limit;
                spd_ref_q12 = (int32_t)(v * 4096.0f + (v >= 0.0f ? 0.5f : -0.5f));
                motor.vel_ref = v;
            }
        }

        /* ---- 1kHz 速度环 -> Iq (全整型, 见文件头说明) ---- */
        if((tick % SPD_LOOP_DIV) == 0u)
        {
            int32_t vel_q12 = (int32_t)(((int64_t)Enc_VelQ24() * 25736) >> 24);

            motor.omega_m = (float)vel_q12 * (1.0f / 4096.0f);

            /* 速度反馈低通: 只影响环路与到位判定, 状态帧照旧报原始 PLL 值 */
            vel_lp += (((float)vel_q12 * (1.0f / 4096.0f)) - vel_lp) * vel_lp_alpha;

            if(motor.mode == MOT_VELOCITY)
            {
                int32_t tgt = (int32_t)(motor.vel_cmd * 4096.0f + (motor.vel_cmd >= 0.0f ? 0.5f : -0.5f));

                if(tgt >  s_vel_lim_q12) tgt =  s_vel_lim_q12;
                if(tgt < -s_vel_lim_q12) tgt = -s_vel_lim_q12;

                /* 给定斜率限幅 (rad/s^2), 起步不硬冲 */
                if(tgt > spd_ref_q12 + spd_acc_step_q12)      spd_ref_q12 += spd_acc_step_q12;
                else if(tgt < spd_ref_q12 - spd_acc_step_q12) spd_ref_q12 -= spd_acc_step_q12;
                else                                          spd_ref_q12  = tgt;

                motor.vel_ref = (float)spd_ref_q12 * (1.0f / 4096.0f);
            }

            if(motor.mode == MOT_TORQUE)
            {
                int32_t iq = (int32_t)(motor.iq_cmd * 4096.0f + (motor.iq_cmd >= 0.0f ? 0.5f : -0.5f));

                if(iq >  s_i_lim_q12) iq =  s_i_lim_q12;
                if(iq < -s_i_lim_q12) iq = -s_i_lim_q12;
                motor.iq_ref  = (float)iq * (1.0f / 4096.0f);
                spd_integ_q16 = 0;
            }
            else if(pos_hold && motor.mode == MOT_POSITION)
            {
                /* 到位锁定: 直接断开输出 (iq=0) 并清积分 —— 这条轴上没有重力负载,
                 * 松手比"冻住一个电流值"安全: 冻结值一旦是饱和的 1.8A, 轴会一直加速。 */
                motor.iq_ref  = 0.0f;
                spd_integ_q16 = 0;
                spd_ref_q12   = 0;
                motor.vel_ref = 0.0f;
            }
            else
            {
                int32_t vel_use_q12 = (int32_t)(vel_lp * 4096.0f + (vel_lp >= 0.0f ? 0.5f : -0.5f));

                motor.iq_ref = (float)spd_pi_q12(spd_ref_q12 - vel_use_q12) * (1.0f / 4096.0f);
            }

            /* ---- 弱磁 (可选, 默认关) ----
             * 只在电压圆真的饱和时往负 Id 走一步; 不饱和就慢慢退回来。
             * id 上限取 0.8*i_limit, 给 q 轴留电压余量。 */
            if(motor.fw_enable)
            {
                float dt_spd = (float)SPD_LOOP_DIV * MOTOR_TS;

                if(foc.sat_cnt != sat_seen)
                {
                    sat_seen = foc.sat_cnt;
                    motor.id_fw -= 0.05f * motor.i_limit;
                    fw_decay_acc = 0.0f;
                }
                else
                {
                    fw_decay_acc += dt_spd;
                    if(fw_decay_acc > 0.20f)          /* 200ms 没饱和就慢慢退 */
                    {
                        fw_decay_acc = 0.20f;
                        motor.id_fw += 0.02f * motor.i_limit;
                        if(motor.id_fw > 0.0f) motor.id_fw = 0.0f;
                    }
                }
                if(motor.id_fw < -0.8f * motor.i_limit) motor.id_fw = -0.8f * motor.i_limit;
            }
            else
            {
                motor.id_fw = 0.0f;
            }

            /* 浮点 -> Q12 只在这里 (1kHz) 做一次; 20kHz 每拍只搬整数。 */
            Foc_SetIqRefs(motor.iq_ref, motor.id_fw);
        }

        Foc_Run(theta);
        break;
    }
    }

    if(foc.oc) latch_fault(1u);

    isr_us = (uint16_t)(Micros() - t0);
    motor.isr_us_last = isr_us;
    if(isr_us > motor.isr_us_max) motor.isr_us_max = isr_us;
    motor.isr_cnt = valley_cnt;
}
