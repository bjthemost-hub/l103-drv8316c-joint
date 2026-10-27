#include "motion.h"
#include "board.h"
#include "debug.h"
#include "fmt.h"
#include "motor.h"
#include "enc.h"
#include "foc.h"
#include "csense.h"
#include "nvcfg.h"
#include "posstore.h"
#include "drv8316c.h"
#include <math.h>

/* =============================================================================
 * CAN 动作层 —— 语义与坐标系见 motion.h 抬头
 * ========================================================================== */

#define _2PI               6.28318530718f
#define SEQ_R2D            (180.0f / 3.14159265359f)
#define SEQ_D2R            (3.14159265359f / 180.0f)

#define PV_VLIM_MIN        0.05f        /* rad/s; 再小的"速度上限"等于不走, 抬到这个值 */
#define PV_ARRIVE_RAD      0.02f        /* 到位判据 ~1.15 度 (同旧工程 SEQ_SETTLE_RAD) */
#define PV_HOME_VLIM       3.1416f      /* 回零默认速度 = 30rpm (协议: 0 -> 默认 30rpm) */

/* 一条 PV 命令最多走多少 (设计值, 只防手滑/单位错/帧解错, 不是性能上限)。
 * 2026-10-19 家人: "不能乱跑啊 会出人命的" —— 目标离当前位置远得离谱时宁可不走。 */
#define MOTION_PV_MAX_TURNS 30.0f
#define MOTION_PV_MAX_RAD   (MOTION_PV_MAX_TURNS * _2PI)

#define SEQ_WAIT_MAX_US    6553500L
#define SEQ_DWELL_MAX_MS   655350L
#define SEQ_DEG_MAX        36000.0f     /* 单段 100 圈上限 (设计值, 只防手滑) */
#define SEQ_SETTLE_RAD     0.02f
#define SEQ_SETTLE_MAX_US  800000u      /* 参考到位后最多再等 800ms 让轴停稳 */
#define SEQ_MOVE_TMAX_S    20.0f
#define SEQ_MOVE_EXTRA_US  200000u

typedef enum { MV_IDLE = 0, MV_PV, MV_VEL } MoveMode_t;

static MoveMode_t m_mode;

static NvCfg_t   cfg;
static uint8_t   cfg_from_flash;
static uint8_t   home_set;
/* "业务执行过程中" —— 这个值为 1 时一律不许写 Flash。
 * 由 Motion_Task() 每拍刷新; 判据 = 脚本在跑 / PV 没到位 / 恒速还在使能 /
 * 开环在转 / 正在对齐。 */
static uint8_t   biz_busy;

/* ---- 板载动作脚本 ---- */
typedef struct {
    uint32_t wait_us;
    uint32_t dwell_us;
    float    delta_rad;
    float    vlim_rad_s;
} SeqSeg_t;

typedef enum { SEQ_PH_IDLE = 0, SEQ_PH_WAIT, SEQ_PH_MOVE, SEQ_PH_DWELL } SeqPhase_t;

static SeqSeg_t  seq[MOTION_SEQ_MAX_SEG];
static uint8_t   seq_n;
static uint8_t   seq_head;
static uint8_t   seq_run;
static uint8_t   seq_loops;
static uint8_t   seq_pass;
static SeqPhase_t seq_phase;
static uint32_t  seq_t_us;
static uint32_t  seq_move_to_us;
static uint32_t  seq_arr_us;
static uint32_t  seq_seg_us;
static float     seq_tgt_rad;           /* 命令链上的目标 (raw 坐标), 不是测量值 */

static void motion_runpv_internal(float target_raw_rad, float vlim_rad_s);
static void cfg_fill_from_ram(NvCfg_t *c);
static void cfg_apply(const NvCfg_t *c);

/* =============================================================================
 * 坐标 / 状态小工具
 * ========================================================================== */

/* raw 坐标 = mech + home。home 就存在 enc 的 zero 里, 跟串口 'Z' 共用一套。 */
float    Motion_PosMechRad(void) { return Enc_MechRad(); }
float    Motion_HomeRad(void)    { return Enc_ZeroRad(); }
float    Motion_PosRad(void)     { return Enc_MechRad() + Enc_ZeroRad(); }
float    Motion_VelRad(void)     { return motor.omega_m; }
int      Motion_IsEnabled(void)  { return motor.enabled ? 1 : 0; }
int      Motion_HomeSet(void)    { return home_set ? 1 : 0; }
int      Motion_PolePairs(void)  { return Motor_PolePairs(); }
float    Motion_Vbus(void)       { return motor.vbus; }
int      Motion_CfgFromFlash(void) { return cfg_from_flash ? 1 : 0; }

float Motion_StallAmp(void)      { return motor.i_limit; }

float Motion_PeakAmp(void)
{
    float a = (float)foc.i_abs_max_q12 * (1.0f / 4096.0f);

    if(a < 0.0f) a = -a;
    return a;
}

int Motion_IsStalled(void)
{
    /* V2 的跳闸码: 1 = 相电流超过 MOTOR_OC_TRIP_A (foc.c 里的 foc.oc), 2 = 8316C nFAULT */
    return (motor.fault && motor.fault_code == 1u) ? 1 : 0;
}

void Motion_ClearFault(void) { Motor_ClearFault(); }

int Motion_Arrived(void)
{
    float d = motor.pos_cmd - Enc_MechRad();

    if(d < 0.0f) d = -d;
    return (d < PV_ARRIVE_RAD) ? 1 : 0;
}

int Motion_IsRunning(void)
{
    if(m_mode == MV_PV)  return Motion_Arrived() ? 0 : 1;
    if(m_mode == MV_VEL) return motor.enabled ? 1 : 0;
    return 0;
}

uint8_t Motion_StatusBits(void)
{
    uint8_t b = 0u;

    if(Motion_IsRunning() || seq_run) b |= 0x01u;              /* RUN */
    if(Motion_Arrived())              b |= 0x02u;              /* ARRIVED */
    if(Motion_IsStalled())            b |= 0x04u;              /* STALL */
    if(Enc_ErrCnt() || Enc_CrcErrCnt()) b |= 0x08u;            /* SENSOR_ERR (本次上电累计) */
    if(motor.aligned)                 b |= 0x10u;              /* ALIGNED */
    if(PosStore_Restored())           b |= 0x20u;              /* POS_RESTORED (2026-10-23: 位置/零位来自 Flash) */
    if(motor.enabled)                 b |= 0x40u;              /* ENABLED */
    if(home_set)                      b |= 0x80u;              /* ZERO_SET */
    return b;
}

void Motion_PrintStatus(void)
{
    printf("MOT: pos=%s mech=%s home=%s vel=%s mode=%s%s\r\n",
           FmtF(Motion_PosRad(), 4), FmtF(Motion_PosMechRad(), 4), FmtF(Motion_HomeRad(), 4),
           FmtF(Motion_VelRad(), 3),
           (m_mode == MV_PV) ? "PV" : (m_mode == MV_VEL) ? "VEL" : "IDLE",
           seq_run ? " +SEQ" : "");
    printf("MOT: tgt=%s arrived=%d stall=%d aligned=%u enabled=%u home=%d seq=%u/%u pass=%u\r\n",
           FmtF(motor.pos_cmd, 4), Motion_Arrived(), Motion_IsStalled(),
           (unsigned)motor.aligned, (unsigned)motor.enabled,
           home_set, (unsigned)seq_n, (unsigned)MOTION_SEQ_MAX_SEG, (unsigned)seq_pass);
}

/* =============================================================================
 * 运动
 * ========================================================================== */

static void motion_runpv_internal(float target_raw_rad, float vlim_rad_s)
{
    if(!motor.aligned)
    {
        printf("MOT: PV refused - not aligned\r\n");
        return;
    }

    if(vlim_rad_s < 0.0f) vlim_rad_s = -vlim_rad_s;
    if(vlim_rad_s < PV_VLIM_MIN) vlim_rad_s = PV_VLIM_MIN;

    m_mode = MV_PV;
    Motor_SetVelLimit(vlim_rad_s);
    /* 先请求模式再下目标: Motor_SetPosition 内部会处理"正要切进 POSITION 时
     * ISR 把 pos_cmd 改成当前角度"的覆盖 (见 motor.c 的 pos_pend)。 */
    Motor_RequestMode(MOT_POSITION);
    Motor_SetPosition(target_raw_rad - Enc_ZeroRad());
    Motor_Enable();
}

int Motion_RunPV(float target_raw_rad, float vlim_rad_s)
{
    float d;

    if(seq_run) Motion_Seq_Abort();          /* 两边抢 pos_cmd, 手动命令先让脚本让位 */

    /* 2026-10-19 护栏: 单位错 (圈当弧度) / 帧解错 / 上一条遗留目标, 都表现为
     * "目标离当前位置很远"。这种命令一律不执行, 也不偷偷钳位 —— 让主机看到拒绝。 */
    d = (target_raw_rad - Enc_ZeroRad()) - Enc_MechRad();
    if(d > MOTION_PV_MAX_RAD || d < -MOTION_PV_MAX_RAD)
    {
        printf("MOT: PV refused - %s turns away (limit %d turns)\r\n",
               FmtF(d / _2PI, 2), (int)MOTION_PV_MAX_TURNS);
        return -1;
    }

    motion_runpv_internal(target_raw_rad, vlim_rad_s);
    return 0;
}

void Motion_RunVEL(float vel_rad_s)
{
    if(seq_run) Motion_Seq_Abort();
    if(!motor.aligned)
    {
        printf("MOT: VEL refused - not aligned\r\n");
        return;
    }
    m_mode = MV_VEL;
    Motor_SetVelocity(vel_rad_s);
    Motor_RequestMode(MOT_VELOCITY);
    Motor_Enable();
}

void Motion_Stop(uint8_t release)
{
    (void)release;

    Motion_Seq_Abort();
    m_mode = MV_IDLE;

    /* 2026-10-23: 停就是停, 不许再去"恢复到什么位置"。
     * 以前 release=0 走的是"停并保持": Motor_RequestMode(MOT_POSITION) +
     * Motor_SetPosition(当前角) + Motor_Enable() —— 名义上是停, 实际是
     * 重新使能并进位置环。使能那一刻 pos_cmd 只要离当前位置有一圈, 位置环
     * 误差就被钳到 POS_ERR_LIMIT_RAD(6.2832), 输出 25 x 6.2832 = 157.08 rad/s
     * (实测 '?' 里 ref vel=-157.08); CAN 的清故障帧没有 RUN 位, 也会掉进这条路
     * —— 点一次清故障就重新使能一次, 于是"位置一路涨、停不下来"。
     * 现在: 不管谁调、带什么参数, 一律失能 + 桥臂高阻; 要再动必须显式使能。 */
    Motor_Disable();
    Motor_RequestMode(MOT_OFF);
    printf("MOT: stop (bridge high-Z)\r\n");
}

void Motion_SaveNow(void)
{
    /* V2 不存位置 (enc_rev 上电从 0 起, 存了也是错的) —— 协议位收下但什么也不做。 */
}

int Motion_SetEnabled(uint8_t on)
{
    if(on)
    {
        if(!motor.aligned) return -1;
        m_mode = MV_IDLE;
        Motor_RequestMode(MOT_POSITION);
        Motor_SetPosition(Enc_MechRad());    /* 使能并保持当前位置 */
        Motor_Enable();
    }
    else
    {
        Motion_Seq_Abort();
        m_mode = MV_IDLE;
        Motor_Disable();
        Motor_RequestMode(MOT_OFF);
    }
    return 0;
}

/* =============================================================================
 * 机械原点
 * ========================================================================== */

void Motion_SetHomeHere(void)
{
    if(seq_run) Motion_Seq_Abort();     /* 脚本按旧坐标在跑, 先让它让位 */
    Enc_SetMechZero(0.0f);      /* 当前位置 = 机械 0 */
    /* 坐标一平移, 位置环手里的 pos_cmd 就成了旧坐标系的数 —— 必须跟着换成新坐标,
     * 否则手一松轴就朝旧位置走 (2026-10-19 家人: "设置机械0点好像也不好用了")。 */
    Motor_SetPosition(Enc_MechRad());
    motor.pos_m = Enc_MechRad();       /* 失能状态下 pos_m 不会自己刷, 这里补一拍 */
    home_set = 1u;
    PosStore_RequestSave();        /* 零点变了: 尽快存 (仍要求不忙、不动) */
    printf("MOT: home set here (mech=0, 已排队写 Flash)\r\n");
}

void Motion_ClearHome(void)
{
    if(home_set)
    {
        /* home := 0, 保持 raw 坐标连续: 让 mech 读数不变 */
        Enc_SetMechZero(Enc_MechRad() + Enc_ZeroRad());
        motor.pos_m = Enc_MechRad();
        home_set = 0u;
        PosStore_RequestSave();
    }
    printf("MOT: home cleared (mech = raw)\r\n");
}

int Motion_GotoHome(float vlim_rad_s)
{
    if(!home_set) return -1;
    if(vlim_rad_s <= 0.0f) vlim_rad_s = PV_HOME_VLIM;
    /* 目标是 raw 坐标里的零位 -> mech = 0 */
    motion_runpv_internal(Enc_ZeroRad(), vlim_rad_s);
    return 0;
}

/* =============================================================================
 * 配置: 极对数 / 电流上限 / 母线 / 节点号  (存 Flash, 见 nvcfg.h)
 * ========================================================================== */

/* 把一条记录里的有效字段应用到 RAM，并同步 cfg (后面 SAVE 时以 cfg 为准)。
 * 0 = 这一条没设过 -> 保留当前默认值，不拿 0 去覆盖。 */
static void cfg_apply(const NvCfg_t *c)
{
    if(c->node >= (int)CAN_NODE_MIN && c->node <= (int)CAN_NODE_MAX)
        cfg.node = c->node;

    if(c->pp >= MOTOR_CFG_PP_MIN && c->pp <= MOTOR_CFG_PP_MAX)
    {
        cfg.pp = c->pp;
        Motor_SetPolePairs(c->pp);
    }

    if(c->ilim_ca > 0)
    {
        cfg.ilim_ca = c->ilim_ca;
        Motor_SetILimit((float)c->ilim_ca * 0.01f);
    }

    if(c->vbus_cv > 0)
    {
        cfg.vbus_cv = c->vbus_cv;
        Motor_SetVbus((float)c->vbus_cv * 0.01f);
    }

    if(c->spd_kp_e5 > 0 && c->spd_ki_e2 > 0)
    {
        cfg.spd_kp_e5 = c->spd_kp_e5;
        cfg.spd_ki_e2 = c->spd_ki_e2;
        Motor_SetSpeedGains((float)c->spd_kp_e5 * 0.00001f,
                            (float)c->spd_ki_e2 * 0.01f);
    }

    if(c->pos_kp_e2 > 0)
    {
        cfg.pos_kp_e2 = c->pos_kp_e2;
        Motor_SetPosKp((float)c->pos_kp_e2 * 0.01f);
    }

    if(c->cur_kp_e4 > 0 && c->cur_ki_e2 > 0)
    {
        cfg.cur_kp_e4 = c->cur_kp_e4;
        cfg.cur_ki_e2 = c->cur_ki_e2;
        Motor_SetCurGains((float)c->cur_kp_e4 * 0.0001f,
                          (float)c->cur_ki_e2 * 0.01f);
    }

    if(c->r_mohm > 0)
    {
        cfg.r_mohm = c->r_mohm;
        motor.r_ohm = (float)c->r_mohm * 0.001f;
    }

    if(c->l_uh > 0)
    {
        cfg.l_uh = c->l_uh;
        motor.l_h = (float)c->l_uh * 0.000001f;
    }

    if(c->vel_lim_e1 > 0)
    {
        cfg.vel_lim_e1 = c->vel_lim_e1;
        Motor_SetVelLimit((float)c->vel_lim_e1 * 0.1f);
    }

    if(c->accel_e1 > 0)
    {
        cfg.accel_e1 = c->accel_e1;
        Motor_SetAccelLimit((float)c->accel_e1 * 0.1f);
    }

    if(c->align_mv > 0)
    {
        cfg.align_mv = c->align_mv;
        Motor_SetAlignVolt((float)c->align_mv * 0.001f);
    }

    /* 相电流跳闸阈值 (2026-10-23 加): 老记录里这块是 0 -> 用默认, 不覆盖 */
    if(c->oc_trip_ca >= (int)(MOTOR_OC_TRIP_MIN * 100.0f) &&
       c->oc_trip_ca <= (int)(MOTOR_OC_TRIP_MAX * 100.0f))
    {
        cfg.oc_trip_ca = c->oc_trip_ca;
        Motor_SetOcTrip((float)c->oc_trip_ca * 0.01f);
    }

    /* 过流跳闸延时 (2026-10-23 加): 老记录 -1 -> 用默认, 不覆盖 */
    if(c->oc_trip_ms >= 0 && c->oc_trip_ms <= (int)MOTOR_OC_TRIP_MS_MAX)
    {
        cfg.oc_trip_ms = c->oc_trip_ms;
        Motor_SetOcTripMs((uint16_t)c->oc_trip_ms);
    }

    if(c->pll_bw_e1 > 0)
    {
        cfg.pll_bw_e1 = c->pll_bw_e1;
        Enc_SetPllBandwidth((float)c->pll_bw_e1 * 0.1f);
    }

    if(c->vsrc >= 0 && c->vsrc <= 2)
    {
        cfg.vsrc = c->vsrc;
        Enc_SetVelSrc((uint8_t)c->vsrc);
    }
}

/* 把 RAM 里的全部可调值装进 cfg 再写 Flash。
 * 2026-10-23: 以前只存 4 项, 现在速度环/位置环/电流环/限幅/对齐/PLL 都存。 */
static void cfg_fill_from_ram(NvCfg_t *c)
{
    float kp, ki, r, l;

    Motor_GetSpeedGains(&kp, &ki);
    c->spd_kp_e5 = (int)(kp * 100000.0f + 0.5f);
    c->spd_ki_e2 = (int)(ki * 100.0f + 0.5f);

    c->pos_kp_e2 = (int)(motor.pos_kp * 100.0f + 0.5f);

    Motor_GetCurGains(&kp, &ki);
    c->cur_kp_e4 = (int)(kp * 10000.0f + 0.5f);
    c->cur_ki_e2 = (int)(ki * 100.0f + 0.5f);

    Motor_GetRL(&r, &l);
    c->r_mohm = (int)(r * 1000.0f + 0.5f);
    c->l_uh   = (int)(l * 1000000.0f + 0.5f);

    c->vel_lim_e1 = (int)(motor.vel_limit * 10.0f + 0.5f);
    c->accel_e1   = (int)(motor.accel_limit * 10.0f + 0.5f);
    c->align_mv   = (int)(motor.v_align * 1000.0f + 0.5f);
    c->oc_trip_ca = (int)(motor.oc_trip * 100.0f + 0.5f);
    c->oc_trip_ms = (int)motor.oc_trip_ms;
    c->pll_bw_e1  = (int)(Enc_PllBandwidth() * 10.0f + 0.5f);
    c->vsrc       = (int)Enc_VelSrc();

    c->ilim_ca = (int)(motor.i_limit * 100.0f + 0.5f);
    c->vbus_cv = (int)(motor.vbus * 100.0f + 0.5f);
    c->pp      = Motor_PolePairs();
    c->node    = cfg.node;
}

static int cfg_save(void)
{
    if(!NvCfg_Enabled()) return 0;
    cfg_fill_from_ram(&cfg);
    return NvCfg_Save(&cfg);
}

int Motion_SaveCfg(void) { return cfg_save(); }

int Motion_SetMotorCfg(int pp, float amp, float vbus)
{
    int   ca = 0, cv = 0;

    if(pp != 0)
    {
        if(pp < MOTOR_CFG_PP_MIN || pp > MOTOR_CFG_PP_MAX)
        {
            printf("MOT: pp %d out of range %d..%d\r\n", pp, MOTOR_CFG_PP_MIN, MOTOR_CFG_PP_MAX);
            return -1;
        }
        if(pp != Motor_PolePairs() && motor.enabled)
        {
            printf("MOT: pp refused - disable the driver first ('d' / CAN ENABLE 0)\r\n");
            return -1;
        }
    }
    if(amp != 0.0f)
    {
        if(amp < MOTOR_CFG_AMP_MIN || amp > MOTOR_CFG_AMP_MAX)
        {
            printf("MOT: amp %s out of range %s..%s A\r\n",
                   FmtF(amp, 2), FmtF(MOTOR_CFG_AMP_MIN, 2), FmtF(MOTOR_CFG_AMP_MAX, 2));
            return -1;
        }
        ca = (int)(amp * 100.0f + 0.5f);
    }
    if(vbus > 0.0f)
    {
        if(vbus < MOTOR_CFG_VBUS_MIN || vbus > MOTOR_CFG_VBUS_MAX)
        {
            printf("MOT: vbus %s out of range %s..%s V\r\n",
                   FmtF(vbus, 2), FmtF(MOTOR_CFG_VBUS_MIN, 1), FmtF(MOTOR_CFG_VBUS_MAX, 1));
            return -1;
        }
        cv = (int)(vbus * 100.0f + 0.5f);
    }

    if(pp != 0)
    {
        Motor_SetPolePairs(pp);            /* 会作废对齐 —— 协议里要求主机重跑 'A' */
        cfg.pp = pp;
    }
    if(ca != 0) { Motor_SetILimit(amp); cfg.ilim_ca = ca; }
    if(cv != 0) { Motor_SetVbus(vbus);  cfg.vbus_cv = cv; }

    cfg_from_flash = cfg_save();
    printf("MOT: cfg pp=%d ilim=%s A vbus=%s V %s\r\n",
           Motor_PolePairs(), FmtF(motor.i_limit, 2), FmtF(motor.vbus, 2),
           cfg_from_flash ? "(saved)" : "(NOT saved)");
    return 0;
}

int Motion_Node(void) { return cfg.node; }
int Motion_NodeFromFlash(void) { return cfg_from_flash ? 1 : 0; }

int Motion_SetNode(int n)
{
    if(n < (int)CAN_NODE_MIN || n > (int)CAN_NODE_MAX)
    {
        printf("MOT: node %d out of range %u..%u\r\n",
               n, (unsigned)CAN_NODE_MIN, (unsigned)CAN_NODE_MAX);
        return -1;
    }
    cfg.node = n;
    cfg_from_flash = cfg_save();
    printf("MOT: node=%d %s\r\n", cfg.node, cfg_from_flash ? "(saved to Flash)" : "(NOT saved)");
    return 0;
}

void Motion_PrintNode(void)
{
    printf("NODE: %d (%s)\r\n", cfg.node, cfg_from_flash ? "from Flash" : "default");
}

/* =============================================================================
 * 板载动作脚本 (CAN SEG / SEG_HR / SEGCTL)
 * -----------------------------------------------------------------------------
 * 语义照旧 (2026-09-29 定, 旧工程实测过):
 *   段 = 等 wait -> 走 deg (方向取 rpm 符号) -> 停 dwell; 角度是**相对本遍起点**。
 *   每块板各跑自己那条时间轴, 多节点同步靠主机"算好每段的 wait + 最后一帧广播开始"。
 *   Flash 一次都不写 (V2 本来也不存位置)。
 * ========================================================================== */

int  Motion_Seq_Running(void) { return seq_run ? 1 : 0; }
int  Motion_Seq_Remain(void)  { return seq_run ? (int)(seq_n - seq_head) : 0; }
int  Motion_Seq_Pass(void)    { return seq_run ? (int)seq_pass : 0; }

void Motion_Seq_Clear(void)
{
    seq_n    = 0u;
    seq_head = 0u;
    printf("SEQ: queue cleared\r\n");
}

void Motion_Seq_Abort(void)
{
    if(!seq_run) return;

    seq_run   = 0u;
    seq_phase = SEQ_PH_IDLE;
    seq_head  = 0u;
    seq_n     = 0u;                    /* 停下就把脚本丢掉: 免得"以为停了又自己跑" */
    printf("SEQ: aborted, script dropped\r\n");
}

static int seq_push(float wait_us, float rpm, float deg, float dwell_ms)
{
    if(seq_n >= (uint8_t)MOTION_SEQ_MAX_SEG)
    {
        printf("SEQ ERR: queue full (%d segs max)\r\n", MOTION_SEQ_MAX_SEG);
        return -1;
    }
    if(wait_us < 0.0f || wait_us > (float)SEQ_WAIT_MAX_US) { printf("SEQ ERR: wait range\r\n"); return -1; }
    if(dwell_ms < 0.0f || dwell_ms > (float)SEQ_DWELL_MAX_MS) { printf("SEQ ERR: dwell range\r\n"); return -1; }
    if(deg <= 0.0f) { printf("SEQ ERR: deg must be POSITIVE (direction = sign of rpm)\r\n"); return -1; }
    if(deg > SEQ_DEG_MAX) { printf("SEQ ERR: deg too big\r\n"); return -1; }
    if(rpm == 0) { printf("SEQ ERR: rpm 0\r\n"); return -1; }
    if(fabsf((float)rpm) * _2PI / 60.0f > (float)VEL_LIMIT_DEF)
    {
        printf("SEQ ERR: rpm too big (max %s rpm)\r\n", FmtF((float)VEL_LIMIT_DEF * 60.0f / _2PI, 0));
        return -1;
    }

    seq[seq_n].wait_us    = (uint32_t)wait_us;
    seq[seq_n].dwell_us   = (uint32_t)(dwell_ms * 1000.0f + 0.5f);
    seq[seq_n].delta_rad  = (rpm < 0) ? -(deg * SEQ_D2R) : (deg * SEQ_D2R);
    seq[seq_n].vlim_rad_s = fabsf((float)rpm) * _2PI / 60.0f;
    seq_n++;
    return 0;
}

int Motion_Seq_AddSeg(long wait_us, int rpm, float deg, long dwell_ms)
{
    int rc = seq_push((float)wait_us, (float)rpm, deg, (float)dwell_ms);

    if(rc == 0) printf("SEQ: +1 seg (CAN) -> queue %d/%d\r\n", (int)seq_n, MOTION_SEQ_MAX_SEG);
    return rc;
}

int Motion_Seq_Start(int loops)
{
    if(seq_n == 0u)
    {
        printf("SEQ: nothing loaded\r\n");
        return -1;
    }
    if(!motor.aligned) { printf("SEQ: refused - not aligned\r\n"); return -1; }
    if(!motor.enabled) { printf("SEQ: refused - driver disabled\r\n"); return -1; }
    if(loops < 1)  loops = 1;
    if(loops > 99) loops = 99;

    seq_loops   = (uint8_t)loops;
    seq_pass    = 1u;
    seq_head    = 0u;
    seq_run     = 1u;
    seq_phase   = SEQ_PH_WAIT;
    seq_arr_us  = 0u;
    seq_tgt_rad = Motion_PosRad();      /* 本遍起点 = 按下开始这一刻的实际位置 */
    seq_t_us    = Micros() + seq[0].wait_us;

    printf("SEQ: RUN %d seg(s) x %d pass(es), anchor=%s rad\r\n",
           (int)seq_n, loops, FmtF(seq_tgt_rad, 4));
    return 0;
}

void Motion_Seq_Print(void)
{
    int   i;
    float tot = 0.0f;

    printf("SEQ: %d/%d segs, %s", (int)seq_n, MOTION_SEQ_MAX_SEG,
           seq_run ? "RUNNING" : (seq_n ? "loaded" : "empty"));
    if(seq_run) printf(", pass %d/%d, next seg %d", (int)seq_pass, (int)seq_loops, (int)seq_head + 1);
    printf("\r\n");

    for(i = 0; i < (int)seq_n; i++)
    {
        tot += seq[i].delta_rad;
        printf("  %2d: wait %luus, %s deg @ %s rpm, dwell %lums\r\n",
               i + 1, (unsigned long)seq[i].wait_us,
               FmtF(seq[i].delta_rad * SEQ_R2D, 1),
               FmtF(seq[i].vlim_rad_s * 60.0f / _2PI, 0),
               (unsigned long)(seq[i].dwell_us / 1000u));
    }
    if(seq_n) printf("  total from anchor: %s deg = %s turns\r\n",
                     FmtF(tot * SEQ_R2D, 1), FmtF(tot / _2PI, 3));
}

/* 紧跟其后的段是"同向 + 不等 + 不停"吗? 是就不进定点窗口, 直接接下一次运动。 */
static int seq_next_is_immediate(void)
{
    const SeqSeg_t *cur = &seq[seq_head];
    const SeqSeg_t *nxt;

    if((int)seq_head + 1 < (int)seq_n)  nxt = &seq[seq_head + 1u];
    else if(seq_pass < seq_loops)       nxt = &seq[0];
    else                                return 0;

    if(nxt->wait_us != 0u || nxt->dwell_us != 0u) return 0;
    return ((nxt->delta_rad > 0.0f) == (cur->delta_rad > 0.0f)) ? 1 : 0;
}

static void Motion_Seq_Task(void)
{
    uint32_t now;

    if(!seq_run) return;
    now = Micros();

    switch(seq_phase)
    {
    case SEQ_PH_WAIT:
    {
        SeqSeg_t *s;
        float     t_est;
        float     d_rad;

        if((int32_t)(now - seq_t_us) < 0) return;

        s = &seq[seq_head];
        d_rad = s->delta_rad;

        /* 同向 + 同速 + 不等 + 不停: 并进一次运动, 中途不减速停车 */
        while((int)seq_head + 1 < (int)seq_n)
        {
            SeqSeg_t *n = &seq[seq_head + 1u];
            if(n->wait_us != 0u || n->dwell_us != 0u) break;
            if((n->delta_rad > 0.0f) != (s->delta_rad > 0.0f)) break;
            if(fabsf(n->vlim_rad_s - s->vlim_rad_s) >= 0.001f) break;
            seq_head++;
            d_rad += n->delta_rad;
        }

        seq_tgt_rad += d_rad;
        motion_runpv_internal(seq_tgt_rad, s->vlim_rad_s);

        t_est = fabsf(d_rad) / s->vlim_rad_s;
        if(t_est > SEQ_MOVE_TMAX_S) t_est = SEQ_MOVE_TMAX_S;
        seq_move_to_us = now + (uint32_t)(t_est * 2.0f * 1e6f) + SEQ_MOVE_EXTRA_US;

        seq_seg_us = now;
        seq_arr_us = 0u;
        seq_phase  = SEQ_PH_MOVE;

        printf("SEQ: pass %d/%d seg %d/%d  %s deg @ %s rpm -> %s rad\r\n",
               (int)seq_pass, (int)seq_loops, (int)seq_head + 1, (int)seq_n,
               FmtF(d_rad * SEQ_R2D, 1), FmtF(s->vlim_rad_s * 60.0f / _2PI, 0),
               FmtF(seq_tgt_rad, 4));
        break;
    }

    case SEQ_PH_MOVE:
        if(Motion_Arrived())
        {
            if(seq_arr_us == 0u) seq_arr_us = now ? now : 1u;
            if(seq_next_is_immediate() ||
               fabsf(Enc_MechRad() - motor.pos_cmd) < SEQ_SETTLE_RAD ||
               (uint32_t)(now - seq_arr_us) >= SEQ_SETTLE_MAX_US)
            {
                printf("SEQ:   done in %lums, rest err %s rad (dwell %lums)\r\n",
                       (unsigned long)((now - seq_seg_us) / 1000u),
                       FmtF(Enc_MechRad() - motor.pos_cmd, 4),
                       (unsigned long)(seq[seq_head].dwell_us / 1000u));
                seq_t_us  = now + seq[seq_head].dwell_us;
                seq_phase = SEQ_PH_DWELL;
            }
        }
        else
        {
            seq_arr_us = 0u;
        }

        if((int32_t)(now - seq_move_to_us) >= 0)
        {
            printf("SEQ: !! segment timeout, %s rad away -> stop + hold, script dropped\r\n",
                   FmtF(Enc_MechRad() - motor.pos_cmd, 4));
            Motion_Stop(0);                /* 内部会把脚本丢掉 */
        }
        break;

    case SEQ_PH_DWELL:
        if((int32_t)(now - seq_t_us) < 0) return;

        seq_head++;
        if(seq_head < seq_n)
        {
            seq_phase = SEQ_PH_WAIT;
            seq_t_us  = now + seq[seq_head].wait_us;
        }
        else if(seq_pass < seq_loops)
        {
            seq_pass++;
            seq_head  = 0u;
            /* seq_tgt_rad 不动: 锚点必须是命令链上的量, 不能拿实测值 (旧工程踩过, 越走越远) */
            seq_phase = SEQ_PH_WAIT;
            seq_t_us  = now + seq[0].wait_us;
            printf("SEQ: pass %d/%d done, next from %s rad\r\n",
                   (int)seq_pass - 1, (int)seq_loops, FmtF(seq_tgt_rad, 4));
        }
        else
        {
            seq_run   = 0u;
            seq_phase = SEQ_PH_IDLE;
            seq_head  = 0u;
            printf("SEQ: all done (%d pass(es)) - queue kept\r\n", (int)seq_loops);
        }
        break;

    default:
        break;
    }
}

/* =============================================================================
 * 上电初始化 / 主循环
 * ========================================================================== */

void Motion_Init(void)
{
    NvCfg_t c;

    m_mode = MV_IDLE;
    home_set = 0u;
    seq_n = seq_head = seq_run = seq_pass = 0u;
    seq_phase = SEQ_PH_IDLE;
    seq_tgt_rad = 0.0f;

    /* 2026-10-19: 上电把命令侧显式清一次。脚本队列本来就在 RAM (nvcfg 只存
     * 节点号/极对数/电流上限/母线, 不存位置也不存脚本), 但"清"要做在明面上 ——
     * 家人 2026-10-19 怀疑是 Flash 没清, 实际是 motor.pos_cmd 里躺着上一条目标。 */
    Motor_RequestMode(MOT_OFF);
    Motor_Disable();
    printf("MOT: boot state cleared (mode=OFF, tgt=current, seq empty)\r\n");

    cfg.node = (int)CAN_NODE_DEFAULT;
    cfg_from_flash = 0u;

    /* 2026-10-23: 先把 Motor_Init()/Foc_Init() 装好的 RAM 值做成一条完整的
     * 默认记录。这样 Flash 里没有记录、记录损坏、或 magic 不认时，cfg 与
     * 电机状态都停在编译默认值上，而不是留下 pp/ilim/vbus = 0 的半套配置。 */
    cfg_fill_from_ram(&cfg);

    NvCfg_Init();

    if(NvCfg_Load(&c))
    {
        cfg_from_flash = 1u;
        cfg_apply(&c);
    }

    printf("MOT: node=%d pp=%d ilim=%s A vbus=%s V cfg=%s\r\n",
           cfg.node, Motor_PolePairs(), FmtF(motor.i_limit, 2), FmtF(motor.vbus, 2),
           cfg_from_flash ? "Flash" : "default");
    {
        float skp, ski, ckp, cki;

        Motor_GetSpeedGains(&skp, &ski);
        Motor_GetCurGains(&ckp, &cki);
        printf("MOT: spd %s/%s pos %s\r\n",
               FmtF(skp, 5), FmtF(ski, 2), FmtF(motor.pos_kp, 2));
        printf("MOT: cur %s/%s R=%s L=%smH\r\n",
               FmtF(ckp, 4), FmtF(cki, 1), FmtF(motor.r_ohm, 2), FmtF(motor.l_h * 1000.0f, 3));
        printf("MOT: vlim=%s acc=%s align=%sV pll=%sHz\r\n",
               FmtF(motor.vel_limit, 1), FmtF(motor.accel_limit, 1),
               FmtF(motor.v_align, 2), FmtF(Enc_PllBandwidth(), 1));
    }
}

/* 每拍刷新"业务执行过程中"标志 —— 定义见上面 biz_busy */
static void biz_update(void)
{
    uint8_t b = 0u;

    if(seq_run)                                    b = 1u;   /* 脚本在跑 */
    else if(m_mode == MV_PV && !Motion_Arrived())   b = 1u;   /* 走位没到位 */
    else if(m_mode == MV_VEL && motor.enabled)      b = 1u;   /* 恒速还在转 */
    else if(motor.mode == MOT_POSITION && !Motion_Arrived()) b = 1u; /* P 走位没到位 */
    else if(motor.mode == MOT_OPENLOOP)             b = 1u;   /* 开环在转 */
    else if(motor.mode == MOT_ALIGN)                b = 1u;   /* 对齐中 */
    biz_busy = b;
}

int Motion_Busy(void)        { return biz_busy ? 1 : 0; }
void Motion_SetHomeFlag(int on) { home_set = (uint8_t)(on ? 1u : 0u); }

void Motion_Task(void)
{
    Motion_Seq_Task();
    biz_update();
}
