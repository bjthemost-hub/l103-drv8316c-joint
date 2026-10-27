#include "can_bus.h"
#include "board.h"
#include "debug.h"
#include "cmd.h"
#include "motion.h"
#include "motor.h"
#include "csense.h"
#include "drv8316c.h"
#include "faults.h"
#include "enc.h"
#include "nvcfg.h"
#include "fmt.h"
#include <string.h>

/* 2026-10-18: 从 CH32L103_DRV8316C_FOC_CAN 搬过来 (协议一个字没改),
 * 只换了下层 —— 旧的 foc_motor/foc_sensor/motion_pv 换成 V2 的 motor/enc/motion。
 * 与旧工程的差别 (都在 motion.c 那边说明):
 *   - 位置/机械零位由 posstore.c 掉电保存 (停稳 3 s 写), 恢复成功时状态帧 POS_RESTORED=1;
 *   - "堵转阈值" 映射到 V2 的电流上限 motor.i_limit (CAN 0x184 照旧上报);
 *   - 母线电压用 CS_VM_Volt() 实测 (本板有分压), 配置帧的 CS_VM_Available 位恒 1。 */

/* =============================================================================
 * CAN1 (U4 = MAX3051EKA+T) 收发 + 报文协议 + 10ms 反馈
 * -----------------------------------------------------------------------------
 * ID = (节点号 << 5) | 功能号 —— 完整的协议/寻址/多节点脚本约定见 can_bus.h 头部。
 * 引脚/速率定案的依据见 board.h 里 CAN_PRESCALER 那一段注释。
 * 中断向量名 USB_LP_CAN1_RX0_IRQn 与 USB 共用(见 ch32l103.h 枚举),
 * 本工程不用 USB, 所以只判 CAN 的 IT 位即可。
 * ========================================================================== */

#define CAN_RXQ_LEN     16

typedef struct {
    uint16_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} CanRxItem_t;

static volatile CanRxItem_t rxq[CAN_RXQ_LEN];
static volatile uint8_t  rxq_head;
static volatile uint8_t  rxq_tail;

static volatile uint32_t n_rx, n_overrun;
static volatile uint16_t last_id;
static volatile uint8_t  last_dlc;
static uint32_t n_tx, n_txfail;
static uint32_t next_fb_us;
static uint8_t  fb_on = 1;    /* 周期反馈帧的子开关 ('K0'/'K1') */
/* 主动上报总闸 (2026-09-29 新增)。
 * 0 = **被动设备**: 除非上位机来问, 一条都不主动发。
 *   管住: 10ms 周期反馈 0x180 + 故障事件 0x1A0 + 周期故障位图 0x183。
 *   不管: "被问就答" 的 0x181/0x182/0x183/0x184 (0x113 询问 / 各条命令的 ACK)。
 * 理由: 一块 CAN 上挂十几个设备时谁都不许自己刷屏; 而且这里刷屏的代价不是"多占点带宽",
 *   是**把控制环拖死** (发帧忙等会堵主循环, 实测闭环 FOC 从 ~2000 次/秒 掉到 3~7 次/秒)。
 * 默认 0; **不存 Flash**, 每次上电都是被动的; 串口 'K1' 打开, 'K0' 关掉。 */
static uint8_t  dbg_on = 0;
static uint8_t  ready;

static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static int16_t rd_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void CAN_Bus_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    CAN_InitTypeDef  can  = {0};
    NVIC_InitTypeDef nvic = {0};

    rxq_head = rxq_tail = 0;
    n_rx = n_overrun = 0; n_tx = n_txfail = 0;
    last_id = 0xFFFFu; last_dlc = 0;

    fb_on  = 1;          /* 子开关默认可用 */
    dbg_on = 0;          /* 但总闸默认关: 上电即被动, 不主动发任何帧 */

    /* CAN 挪到 PD0(RX) / PD1(TX) —— 旧板的 PB8/PB9 本板给了编码器 (ENC_DO/ENC_CS)。
     * 依据 (2026-10-08 从应用手册 AFIO_PCFR1 位域原文核过, 手册 V1.9 L5182-5199):
     *   bit15   PD0PD1_RM : 0 = PD0/PD1 当晶振脚 / 1 = 当 GPIO 用 (复位值 0)
     *   [14:13] CAN_RM    : 00 = PA11/PA12, 10 = PB8/PB9, 01 = 保留,
     *                       11 = CAN_RX -> PD0, CAN_TX -> PD1
     * 本板 PD0/PD1 上没挂晶振 (PD0->U4.4 RXD, PD1->U4.1 TXD), 时钟走 HSI
     * (User\system_ch32l103.c SetSysClockTo96_HSI), 所以两位都要写。
     * 直接写寄存器, 不用 GPIO_Remap* 宏 —— 宏的编码跟位域位置绑着, 写位域不歧义。 */
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_AFIO | RCC_PB2Periph_GPIOD, ENABLE);

    /* 2026-10-18: 重射先写, CAN 时钟后开。原来是先开 CAN 时钟再写 AFIO,
     * 实测这台板子上 CAN 核没拿到 PD0/PD1 —— TX 不出现在引脚、RX 收到的是乱码
     * (依据: 发一帧 REC 从 9 跳到 129, 而 TXD->RXD 的环路上 'Y' 自检是通的)。
     * 重射位同样是 家规里那两个: b15 PD0PD1_RM=1, CAN_RM[14:13]=11b。 */
    AFIO->PCFR1 |=  (1u << 15);
    AFIO->PCFR1 &= ~(0x3u << 13);
    AFIO->PCFR1 |=  (0x3u << 13);

    RCC_PB1PeriphClockCmd(RCC_PB1Periph_CAN1, ENABLE);

    /* PD1 = CAN1_TX: 复用推挽; PD0 = CAN1_RX: 输入上拉 */
    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &gpio);

    CAN_DeInit(CAN1);

    /* 不做软件滤波: 默认全通, 标准帧/扩展帧都收进来, 在软件里按 ID 分派。
       (参考同板子跑通过的 CH32L103_USB2CAN_DM/User/can_bsp.c, 它也没配滤波) */
    can.CAN_TTCM   = DISABLE;
    can.CAN_ABOM   = ENABLE;            /* 自动 bus-off 恢复 */
    can.CAN_AWUM   = DISABLE;
    can.CAN_NART   = DISABLE;           /* 仲裁失败自动重传 */
    can.CAN_RFLM   = DISABLE;
    can.CAN_TXFP   = ENABLE;            /* 按请求顺序发 */
    can.CAN_Mode   = (CAN_LOOPBACK ? CAN_Mode_LoopBack : CAN_Mode_Normal);
    can.CAN_SJW    = CAN_SJW_1tq;
    can.CAN_BS1    = CAN_BS1_TQ;
    can.CAN_BS2    = CAN_BS2_TQ;
    can.CAN_Prescaler = CAN_PRESCALER;
    /* 注意: 这个库的返回值是 CAN_InitStatus_Success = 1 (ch32l103_can.h),
       不是"0 表示成功"。 */
    ready = (CAN_Init(CAN1, &can) == CAN_InitStatus_Success) ? 1 : 0;

    /* CAN_DeInit 里有一次 CAN1 复位, 它过完再把引脚重射确认一次。 */
    AFIO->PCFR1 |= (1u << 15);
    AFIO->PCFR1 |= (0x3u << 13);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    nvic.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    next_fb_us = Micros() + CAN_FB_PERIOD_US;
}

int CAN_Bus_Ready(void) { return (int)ready; }

/* 本机节点地址 (1..63)。真值在 motion_pv (它管 Flash 里那条快照 + 是否来自 Flash),
 * 这里只做一层转发: 每次发帧现读, 运行期改节点号不会留下"缓存没跟上"的坑。 */
uint8_t CAN_Bus_Node(void) { return (uint8_t)Motion_Node(); }

int CAN_Bus_SendStd(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    CanTxMsg tx = {0};
    uint8_t  mbox;
    uint32_t t0;

    if(dlc > 8u) dlc = 8u;

    tx.StdId = id;
    tx.IDE   = CAN_Id_Standard;
    tx.RTR   = CAN_RTR_Data;
    tx.DLC   = dlc;
    if(dlc) memcpy(tx.Data, data, dlc);

    /* 有界等待: 按**时间**算, 不按圈数算 (理由见 board.h 的 CAN_TX_TIMEOUT_US) */
    t0 = Micros();

    for(;;)
    {
        mbox = CAN_Transmit(CAN1, &tx);
        if(mbox != CAN_NO_MB) break;
        if((uint32_t)(Micros() - t0) > CAN_TX_TIMEOUT_US) { n_txfail++; return 1; }
    }

    while(CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Pending)
    {
        if((uint32_t)(Micros() - t0) > CAN_TX_TIMEOUT_US)
        {
            CAN_CancelTransmit(CAN1, mbox);   /* 别空转、别堵主循环: 宁可丢这一帧 */
            n_txfail++;
            return 1;
        }
    }

    if(CAN_TransmitStatus(CAN1, mbox) != CAN_TxStatus_Ok) { n_txfail++; return 1; }
    n_tx++;
    return 0;
}

void CAN_Bus_SetFeedback(uint8_t on) { fb_on = on ? 1u : 0u; }
uint8_t CAN_Bus_FeedbackOn(void)     { return fb_on; }

/* 主动上报总闸 (2026-09-29 新增): 0 = 被动 (默认) */
void    CAN_Bus_SetDebug(uint8_t on) { dbg_on = on ? 1u : 0u; }
uint8_t CAN_Bus_DebugOn(void)        { return dbg_on; }

/* ---- 收到命令: PV ---- */
static void send_status_frame(uint16_t id);
static void send_cfg_frame(void);
static void send_info_frame(void);
static void send_motorcfg_frame(void);

static void handle_pv(const uint8_t *d, uint8_t dlc)
{
    uint8_t f;
    float   tgt, vlim;
    int32_t mrad;
    int16_t vc;

    if(dlc < 7u) return;

    f    = d[6];
    mrad = rd_i32(d + 0);
    vc   = rd_i16(d + 4);

    if(f & 0x08u) Motion_ClearFault();

    if(!(f & 0x02u))
    {
        /* 2026-10-23: 清故障帧(只有 bit3, 没有 RUN 位)不许再被当成"停止"。
         * 以前它掉进这个分支, 执行 Motion_Stop() -> 重新使能并进位置环,
         * 于是"点一次清故障, 电机就转起来"。清完故障, 模式与使能一概不动。 */
        if(!(f & 0x08u)) Motion_Stop(1u);
        return;
    }

    if(!Cmd_IsAligned())
    {
        printf("CAN: PV rejected - not aligned (run 'A' first)\r\n");
        Fault_ReportReject(CAN_FN_PV, CAN_FAULT_NOT_ALIGNED);
        return;
    }

    tgt  = (float)mrad * 0.001f;
    vlim = (float)vc * 0.01f;
    if(vlim < 0.0f) vlim = -vlim;

    if(f & 0x01u)
    {
        tgt += Motion_PosRad();            /* 相对: 相对收到命令这一刻 (raw 坐标) */
    }
    else if(f & 0x20u)
    {
        /* 机械坐标: 目标 = 机械位置 + 机械零位 */
        if(!Motion_HomeSet())
        {
            printf("CAN: PV rejected - no mechanical zero (send 0x112 first)\r\n");
            Fault_ReportReject(CAN_FN_PV, CAN_FAULT_CMD_REJECT);
            return;
        }
        tgt += Motion_HomeRad();
    }

    if(Motion_RunPV(tgt, vlim) != 0)
    {
        Fault_ReportReject(CAN_FN_PV, CAN_FAULT_CMD_REJECT);
        return;
    }
    if(f & 0x10u) Motion_SaveNow();
}

/* ---- 收到命令: 动作脚本加一段 (0x120, 2026-09-29) ----
 *   b0..1 uint16 开始前等待 [100us]   b2..3 int16 速度 [rpm, 符号 = 方向]
 *   b4..5 int16 角度 [0.1 度] (必须正数)   b6..7 uint16 跑完停留 [10ms]
 *   例: 100rpm 正转 90 度, 停 0.5s, 不等 -> 00 00 64 00 84 03 32 00
 *   上限: rpm 1432 / 角度 +-9.1 圈 (int16 x0.1 度) / 等待 6.5535s / 停留 655.35s
 *   要单段 >3276.7 度 -> 用 0x122 (同布局, 角度 1 度/LSB) */
static void handle_seg(const uint8_t *d, uint8_t dlc)
{
    long  wait_us, dwell_ms;
    int   rpm;
    float deg;

    if(dlc < 8u) return;

    wait_us  = (long)((uint16_t)(d[0] | ((uint16_t)d[1] << 8))) * 100L;
    rpm      = (int)rd_i16(d + 2);
    deg      = (float)rd_i16(d + 4) * 0.1f;
    dwell_ms = (long)((uint16_t)(d[6] | ((uint16_t)d[7] << 8))) * 10L;

    if(Motion_Seq_AddSeg(wait_us, rpm, deg, dwell_ms) != 0)
    {
        Fault_ReportReject(CAN_FN_SEG, CAN_FAULT_CMD_REJECT);
        return;
    }
    send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY));
}

/* ---- 收到命令: 动作脚本加一段 · 大角度版 (0x122, 2026-10-06) ----
 *   布局与 0x120 **逐字节相同**, 只有角度字段的刻度不同:
 *     0x120: b4..5 int16 角度 [0.1 度] -> 单段上限 3276.7 度 (细, 适合短段)
 *     0x122: b4..5 int16 角度 [1   度] -> 单段上限 32767 度 (粗, 适合长段 >3276.7 度)
 *   8 个字节已经被 wait/rpm/角度/dwell 占满, 角度要更大就只能牺牲分辨率 ——
 *   长程段用 1 度粒度没有实际损失(1 度 = 0.0028 圈), 短段继续用 0x120。
 *   两条最终都进同一个 Motion_Seq_AddSeg(): 落点/同向接续行为完全一致。
 *   上限: rpm 1432 / 角度 +-32767 度 / 等待 6.5535s / 停留 655.35s */
static void handle_seg_hr(const uint8_t *d, uint8_t dlc)
{
    long  wait_us, dwell_ms;
    int   rpm;
    float deg;

    if(dlc < 8u) return;

    wait_us  = (long)((uint16_t)(d[0] | ((uint16_t)d[1] << 8))) * 100L;
    rpm      = (int)rd_i16(d + 2);
    deg      = (float)rd_i16(d + 4) * 1.0f;
    dwell_ms = (long)((uint16_t)(d[6] | ((uint16_t)d[7] << 8))) * 10L;

    if(Motion_Seq_AddSeg(wait_us, rpm, deg, dwell_ms) != 0)
    {
        Fault_ReportReject(CAN_FN_SEG_HR, CAN_FAULT_CMD_REJECT);
        return;
    }
    send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY));
}

/* ---- 收到命令: 动作脚本控制 (0x121) ----
 *   b0 0=清空队列  1=开始  2=打印队列;   b1 = 遍数 (0 或 1 = 一遍) */
static void handle_segctl(const uint8_t *d, uint8_t dlc)
{
    uint8_t sub   = (dlc > 0u) ? d[0] : 0u;
    int     loops = (dlc > 1u) ? (int)d[1] : 1;

    if(sub == 0u)
    {
        Motion_Seq_Clear();
    }
    else if(sub == 1u)
    {
        if(Motion_Seq_Start(loops) != 0)
        {
            Fault_ReportReject(CAN_FN_SEGCTL, CAN_FAULT_CMD_REJECT);
            return;
        }
    }
    else
    {
        Motion_Seq_Print();
    }
    send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY));
}

/* ---- 收到命令: 设节点号 (CAN_FN_SETNODE, 2026-10-16 新增) ----
 *   b0 = 节点号 (1..63);  b1 = 0x5A (防呆位, 不对就拒)
 * 立刻存 Flash。答复用**改完之后**的节点号发状态帧, 主机据此确认改到了哪一台。
 * ⚠ 用广播发这条 = 在场每一台都改成同一个号; 首次编号请用串口 '%<n>'(一次只接一块)。 */
static void handle_setnode(const uint8_t *d, uint8_t dlc)
{
    if(dlc < 2u || d[1] != 0x5Au)
    {
        printf("CAN: SETNODE rejected - need b0=node(1..63), b1=0x5A\r\n");
        Fault_ReportReject(CAN_FN_SETNODE, CAN_FAULT_CMD_REJECT);
        return;
    }
    if(Motion_SetNode((int)d[0]) != 0)
    {
        Fault_ReportReject(CAN_FN_SETNODE, CAN_FAULT_CMD_REJECT);
        return;
    }
    send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY));
}

/* ---- 收到命令: 纯速度 ---- */
static void handle_vel(const uint8_t *d, uint8_t dlc)
{
    uint8_t f;
    int16_t vc;

    if(dlc < 3u) return;
    f  = d[2];
    vc = rd_i16(d + 0);

    if(f & 0x08u) Motion_ClearFault();

    if(!(f & 0x02u))
    {
        /* 同 handle_pv: 纯清故障帧不动模式/不使能 */ 
        if(!(f & 0x08u)) Motion_Stop(1u);
        return;
    }

    if(!Cmd_IsAligned())
    {
        printf("CAN: VEL rejected - not aligned (run 'A' first)\r\n");
        Fault_ReportReject(CAN_FN_VEL, CAN_FAULT_NOT_ALIGNED);
        return;
    }

    Motion_RunVEL((float)vc * 0.01f);
}

/* ---- 收到命令: 设角度环速度上限 (0x115) ----
 * b0..b1 = int16 x0.01 rad/s。这就是 PID_Angle 输出的钳位值,
 * 也是 CAN 的 vlim 给再大也没用的那个天花板。 */
static void handle_vlim(const uint8_t *d, uint8_t dlc)
{
    int16_t vc;

    if(dlc < 2u) return;
    vc = rd_i16(d + 0);
    if(vc <= 0) return;

    Motor_SetVelLimit((float)vc * 0.01f);
    printf("CAN: VLIM(angle) = %s rad/s\r\n", FmtF(motor.vel_limit, 3));
    send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY));
}

/* ---- 收到命令: 设电机参数 (0x116) ----
 * b0 = 极对数 (0 = 不改), b1..2 = int16 堵转阈值 0.01A (0 = 不改),
 * b4..5 = int16 母线电压 0.01V (0 = 不改; dlc < 6 时当没给),
 * b3 bit0 = 1 才是"确认要改" —— 这几个参数会一直留在 Flash 里, 加个防呆位。
 * 范围校验 / 越界拒绝 / 改极对数要求先失能 / 作废对齐, 全在 Motion_SetMotorCfg()。 */
static void handle_motorcfg(const uint8_t *d, uint8_t dlc)
{
    int     pp;
    float   amp, vbus;
    int16_t ca, cv;

    if(dlc < 4u)
    {
        printf("CAN: MOTORCFG rejected - dlc=%u < 4\r\n", (unsigned)dlc);
        Fault_ReportReject(CAN_FN_MOTORCFG, CAN_FAULT_CMD_REJECT);
        return;
    }

    if(!(d[3] & 0x01u))
    {
        printf("CAN: MOTORCFG rejected - b3 bit0=0 (set 1 to confirm a param write)\r\n");
        Fault_ReportReject(CAN_FN_MOTORCFG, CAN_FAULT_CMD_REJECT);
        return;
    }

    pp  = (int)d[0];
    ca  = rd_i16(d + 1);
    amp = (float)ca * 0.01f;

    /* 母线电压只有 dlc>=6 才带; 0/负数都当"不改" */
    vbus = 0.0f;
    if(dlc >= 6u)
    {
        cv = rd_i16(d + 4);
        if(cv > 0) vbus = (float)cv * 0.01f;
    }

    if(Motion_SetMotorCfg(pp, amp, vbus) != 0)
    {
        printf("CAN: MOTORCFG rejected (see reason line above)\r\n");
        Fault_ReportReject(CAN_FN_MOTORCFG, CAN_FAULT_CMD_REJECT);
        return;
    }

    send_motorcfg_frame();
    send_cfg_frame();
    send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY));   /* 改极对数会清 aligned, 让主机立刻看到 */
}

/* ---- 收到命令: 电角度对齐 (CAN_FN_ALIGN, 2026-10-18 新增) ----
 *   b0 = 0x5A 防呆 —— 对齐会硬拽转子, 不接受裸帧误触发。
 *   Motor_StartAlign() 是**非阻塞**的 (只置状态位), 真正的对齐跑在 motor.c 的状态机里,
 *   所以在这里调用不会把 CAN 主循环卡住 ALIGN_TIME_MS_DEF。
 *   回一帧状态帧当 ACK; 那一刻 aligned 还是 0, 主机要轮询到 aligned=1 才算完成。 */
static void handle_align(const uint8_t *d, uint8_t dlc)
{
    if(dlc < 1u || d[0] != 0x5Au)
    {
        printf("CAN: ALIGN rejected - need b0=0x5A\r\n");
        Fault_ReportReject(CAN_FN_ALIGN, CAN_FAULT_CMD_REJECT);
        return;
    }

    Motor_StartAlign();
    printf("CAN: align start (约 %ums)\r\n", (unsigned)ALIGN_TIME_MS_DEF);
    send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY));
}

/* ---- 收到命令: 使能 / 失能 (0x110) ---- */
static void handle_enable(const uint8_t *d, uint8_t dlc)
{
    uint8_t on = (dlc >= 1u) ? (d[0] ? 1u : 0u) : 1u;

    if(Motion_SetEnabled(on) != 0)
    {
        Fault_ReportReject(CAN_FN_ENABLE, CAN_FAULT_NOT_ALIGNED);
        return;
    }

    send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY));
}

/* ---- 收到命令: 回机械原点 (0x111) ---- */
static void handle_home(const uint8_t *d, uint8_t dlc)
{
    float vlim = 0.0f;

    if(dlc >= 2u) vlim = (float)rd_i16(d) * 0.01f;

    if(Motion_GotoHome(vlim) != 0)
    {
        Fault_ReportReject(CAN_FN_HOME, CAN_FAULT_CMD_REJECT);
        return;
    }

    send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY));
}

/* ---- 收到命令: 这里设为机械 0 点 (0x112) ---- */
static void handle_setzero(const uint8_t *d, uint8_t dlc)
{
    (void)d; (void)dlc;

    /* 2026-10-19: 设机械零点只是改坐标原点 (不动电机), 不再要求先做电角度对齐 ——
     * 原来这条让"刚上电/刚重烧"的板子点这个按钮只回一个 NOT_ALIGNED, 看着就是坏了。 */
    Motion_SetHomeHere();
    send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY));
    send_cfg_frame();
}

/* ---- 收到命令: 询问 (0x113) ---- */
static void handle_query(const uint8_t *d, uint8_t dlc)
{
    uint8_t what = (dlc >= 1u) ? d[0] : 0u;

    if(what == 0xFFu)
    {
        send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY));
        send_cfg_frame();
        Fault_SendBitmap(CAN_ID_TX(CAN_FN_FB_FAULT));
        send_motorcfg_frame();
        return;
    }

    switch(what)
    {
        case 1u:  send_cfg_frame();                  break;
        case 2u:  Fault_SendBitmap(CAN_ID_TX(CAN_FN_FB_FAULT)); break;
        case 3u:  send_motorcfg_frame();             break;
        case 4u:  send_info_frame();                 break;
        default:  send_status_frame(CAN_ID_TX(CAN_FN_FB_QUERY)); break;
    }
}


/* ---- 收到命令: 读板型/固件版本 (功能号 14; 2026-10-18) ---- */
static void handle_info(const uint8_t *d, uint8_t dlc)
{
    (void)d;
    (void)dlc;                 /* 无载荷, 怎么发都答 */
    send_info_frame();
}

/* ---- 收到命令: 故障上报配置 (0x114) ---- */
static void handle_faultcfg(const uint8_t *d, uint8_t dlc)
{
    uint8_t on = 1u, per = 10u;

    Fault_GetReport(&on, &per);
    if(dlc >= 1u) on  = d[0] ? 1u : 0u;
    if(dlc >= 2u) per = d[1];
    Fault_SetReport(on, per);

    printf("CAN: fault report %s period=%u (x100ms)\r\n", on ? "ON" : "OFF", (unsigned)per);
    send_cfg_frame();
}

/* 即时状态帧 0x181 (布局与 0x180 相同; status 见 can_bus.h) */
static void send_status_frame(uint16_t id)
{
    uint8_t d[8];
    int32_t pos_m;
    int16_t vel_c;
    float   amp;
    int32_t cur;

    pos_m = (int32_t)(Motion_PosMechRad() * 1000.0f);   /* 机械坐标 */
    vel_c = (int16_t)(Motion_VelRad() * 100.0f);
    amp   = Motion_PeakAmp();
    cur   = (int32_t)(amp * 10.0f);
    if(cur >  127) cur =  127;
    if(cur < -127) cur = -127;

    d[0] = (uint8_t)(pos_m & 0xFF);
    d[1] = (uint8_t)((pos_m >> 8) & 0xFF);
    d[2] = (uint8_t)((pos_m >> 16) & 0xFF);
    d[3] = (uint8_t)((pos_m >> 24) & 0xFF);
    d[4] = (uint8_t)(vel_c & 0xFF);
    d[5] = (uint8_t)((vel_c >> 8) & 0xFF);
    d[6] = Motion_StatusBits();
    d[7] = (uint8_t)(int8_t)cur;

    (void)CAN_Bus_SendStd(id, d, 8);
}

/* 配置帧 0x182 */
static void send_cfg_frame(void)
{
    uint8_t  d[8];
    uint8_t  on = 1u, per = 10u;
    int32_t  home_m;
    float    v;
    uint16_t vv;

    home_m = (int32_t)(Motion_HomeRad() * 1000.0f);
    Fault_GetReport(&on, &per);

    v = CS_VM_Volt() * 10.0f;
    if(v < 0.0f)      v = 0.0f;
    if(v > 65535.0f)  v = 65535.0f;
    vv = (uint16_t)v;

    d[0] = (uint8_t)(home_m & 0xFF);
    d[1] = (uint8_t)((home_m >> 8) & 0xFF);
    d[2] = (uint8_t)((home_m >> 16) & 0xFF);
    d[3] = (uint8_t)((home_m >> 24) & 0xFF);
    d[4] = (uint8_t)(vv & 0xFF);
    d[5] = (uint8_t)((vv >> 8) & 0xFF);

    d[6] = 0u;
    if(Motion_HomeSet())   d[6] |= 0x01u;
    if(Cmd_IsAligned())    d[6] |= 0x02u;
    if(VM_SENSE_ENABLE)    d[6] |= 0x04u;      /* 本板有母线分压 */
    if(Motion_IsEnabled()) d[6] |= 0x08u;
    if(on)                 d[6] |= 0x10u;

    d[7] = per;

    (void)CAN_Bus_SendStd(CAN_ID_TX(CAN_FN_FB_CFG), d, 8);
}

/* 电机参数帧 0x184 (2026-09-28 新增; b4..5 母线电压是同日后来加的) */
static void send_motorcfg_frame(void)
{
    uint8_t  d[8];
    int16_t  ca, cv;
    float    a;

    a  = Motion_StallAmp();
    if(a < 0.0f) a = 0.0f;
    ca = (int16_t)(a * 100.0f + 0.5f);

    cv = (int16_t)(Motion_Vbus() * 100.0f + 0.5f);

    d[0] = (uint8_t)Motion_PolePairs();
    d[1] = (uint8_t)(ca & 0xFF);
    d[2] = (uint8_t)((ca >> 8) & 0xFF);

    d[3] = 0u;
    if(Motion_CfgFromFlash()) d[3] |= 0x01u;
    if(Cmd_IsAligned())       d[3] |= 0x02u;
    if(Motion_IsEnabled())    d[3] |= 0x04u;

    d[4] = (uint8_t)(cv & 0xFF);
    d[5] = (uint8_t)((cv >> 8) & 0xFF);
    d[6] = (uint8_t)Motion_Seq_Remain();   /* 脚本: 这一遍还剩几段 */
    d[7] = (uint8_t)Motion_Seq_Pass();     /* 脚本: 正在跑第几遍 (0 = 没跑) */

    (void)CAN_Bus_SendStd(CAN_ID_TX(CAN_FN_FB_MOTORCFG), d, 8);
}

/* 板型/版本帧 0x186 (功能号 22; 2026-10-18 新增) */
/* =============================================================================
 * 功能号 15 参数读写 (2026-10-23) —— 与 CH32V303_5010 板**同一套字节布局**,
 * 上位机同一段解析就能认两种板子。逐字节口径见 can_bus.h 的 CAN_FN_PARAM 段。
 * 索引与 SAVE 进 Flash 的 16 个字段一一对应 (见 motion.c cfg_fill_from_ram)。
 * ========================================================================== */
#define CAN_PARAM_MAGIC   0x5Au
#define CAN_PARAM_SIG     0xA5u
#define L103_PARAM_N      18u     /* 2026-10-23: 16 = 跳闸阈值 A, 17 = 跳闸延时 ms */
#define CAN_PQ_LEN        24u            /* 一次读全部 16 帧, 留点余量 */

static uint8_t  pq[CAN_PQ_LEN][8];
static uint8_t  pq_head, pq_tail;
static uint32_t n_pq_drop;
static uint8_t  param_dirty;             /* bit1 用: 改过 RAM, 还没 SAVE */
static uint8_t  flash_ok;                /* bit0 用: Flash 里有有效记录 */
static uint8_t  flash_ok_known;

static float rd_f32(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static void param_q_push(uint8_t op, uint8_t idx, float val, uint8_t fail)
{
    uint8_t  d[8];
    uint32_t u;
    uint8_t  fl = 0u;
    uint8_t  nx = (uint8_t)((pq_head + 1u) % CAN_PQ_LEN);

    if(nx == pq_tail) { n_pq_drop++; return; }

    if(!flash_ok_known)
    {
        NvCfg_t c;
        flash_ok       = NvCfg_Load(&c) ? 1u : 0u;
        flash_ok_known = 1u;
        if(flash_ok) param_dirty = 0u;
    }

    memcpy(&u, &val, 4);
    if(flash_ok)    fl |= 0x01u;
    if(param_dirty) fl |= 0x02u;
    if(fail)        fl |= 0x04u;

    d[0] = (uint8_t)(op | 0x80u);
    d[1] = idx;
    d[2] = (uint8_t)(u & 0xFFu);
    d[3] = (uint8_t)((u >> 8) & 0xFFu);
    d[4] = (uint8_t)((u >> 16) & 0xFFu);
    d[5] = (uint8_t)((u >> 24) & 0xFFu);
    d[6] = fl;
    d[7] = CAN_PARAM_SIG;

    memcpy(pq[pq_head], d, 8);
    pq_head = nx;
}

static int param_q_pop(uint8_t *d)
{
    if(pq_tail == pq_head) return 0;
    memcpy(d, pq[pq_tail], 8);
    pq_tail = (uint8_t)((pq_tail + 1u) % CAN_PQ_LEN);
    return 1;
}

static float param_get(int i)
{
    float a, b;

    switch(i)
    {
    case 0:  return (float)Motor_PolePairs();
    case 1:  return motor.i_limit;
    case 2:  return motor.vbus;
    case 3:  Motor_GetSpeedGains(&a, &b); return a;
    case 4:  Motor_GetSpeedGains(&a, &b); return b;
    case 5:  return motor.pos_kp;
    case 6:  Motor_GetCurGains(&a, &b);   return a;
    case 7:  Motor_GetCurGains(&a, &b);   return b;
    case 8:  return motor.vel_limit;
    case 9:  return motor.accel_limit;
    case 10: return motor.v_align;
    case 11: return Enc_PllBandwidth();
    case 12: return (float)Enc_VelSrc();
    case 13: return motor.r_ohm;
    case 14: return motor.l_h * 1000.0f;      /* H -> mH */
    case 15: return (float)CAN_Bus_Node();
    case 16: return Motor_GetOcTrip();     /* 相电流跳闸阈值 A (2026-10-23 加) */
    case 17: return (float)Motor_GetOcTripMs();   /* 连续超限多久才跳闸 ms (2026-10-23 加) */
    default: return 0.0f;
    }
}

/* 写完立即生效 (RAM), 只有 op=0x03 SAVE 才进 Flash。
 * 极对数在使能状态拒绝改 (改完必须重跑对齐); 节点号只读, 走功能号 12 SETNODE。 */
static int param_set(int i, float v)
{
    float a, b;

    switch(i)
    {
    case 0:
        if(v < (float)MOTOR_CFG_PP_MIN || v > (float)MOTOR_CFG_PP_MAX) return 0;
        if((int)(v + 0.5f) != Motor_PolePairs() && Motion_IsEnabled())   return 0;
        Motor_SetPolePairs((int)(v + 0.5f));
        return 1;
    case 1:
        if(v < MOTOR_CFG_AMP_MIN || v > MOTOR_CFG_AMP_MAX) return 0;
        Motor_SetILimit(v);
        return 1;
    case 2:
        if(v < MOTOR_CFG_VBUS_MIN || v > MOTOR_CFG_VBUS_MAX) return 0;
        Motor_SetVbus(v);
        return 1;
    case 3:
        if(v <= 0.0f || v > 10.0f) return 0;
        Motor_GetSpeedGains(&a, &b); Motor_SetSpeedGains(v, b);  return 1;
    case 4:
        if(v < 0.0f || v > 1000.0f) return 0;
        Motor_GetSpeedGains(&a, &b); Motor_SetSpeedGains(a, v);  return 1;
    case 5:
        if(v <= 0.0f || v > 1000.0f) return 0;
        Motor_SetPosKp(v);
        return 1;
    case 6:
        if(v <= 0.0f || v > 1000.0f) return 0;
        Motor_GetCurGains(&a, &b); Motor_SetCurGains(v, b);      return 1;
    case 7:
        if(v < 0.0f) return 0;
        Motor_GetCurGains(&a, &b); Motor_SetCurGains(a, v);      return 1;
    case 8:
        if(v <= 0.0f || v > 5000.0f) return 0;
        Motor_SetVelLimit(v);
        return 1;
    case 9:
        if(v <= 0.0f) return 0;
        Motor_SetAccelLimit(v);
        return 1;
    case 10:
        if(v < 0.5f || v > 12.0f) return 0;
        Motor_SetAlignVolt(v);
        return 1;
    case 11:
        if(v < 5.0f || v > 800.0f) return 0;
        Enc_SetPllBandwidth(v);
        return 1;
    case 12:
        if(v < 0.0f || v > 2.0f) return 0;
        Enc_SetVelSrc((uint8_t)(v + 0.5f));
        return 1;
    case 13:
        if(v <= 0.0f || v > 100.0f) return 0;
        Motor_GetRL(&a, &b); Motor_SetGainsFromRL(v, b, CUR_BW_HZ_DEF);
        return 1;
    case 16:
        if(v < MOTOR_OC_TRIP_MIN || v > MOTOR_OC_TRIP_MAX) return 0;
        Motor_SetOcTrip(v);
        return 1;
    case 17:
        if(v < 0.0f || v > (float)MOTOR_OC_TRIP_MS_MAX) return 0;
        Motor_SetOcTripMs((uint16_t)(v + 0.5f));
        return 1;
    case 14:
        if(v <= 0.0f || v > 1000.0f) return 0;
        Motor_GetRL(&a, &b); Motor_SetGainsFromRL(a, v * 0.001f, CUR_BW_HZ_DEF);
        return 1;
    case 15:
    default:
        return 0;                     /* 节点号请用功能号 12 */
    }
}

static void handle_param(const uint8_t *d, uint8_t dlc)
{
    uint8_t op, magic;

    if(dlc < 1u) return;
    op    = d[0];
    magic = (dlc >= 7u) ? d[6] : 0u;

    switch(op)
    {
    case 0x00u:                       /* INFO: 参数个数 */
        param_q_push(op, 0xFFu, (float)L103_PARAM_N, 0u);
        break;

    case 0x01u:                       /* READ: b1=起始索引 b2=个数 */
    {
        int i, start, cnt;

        if(dlc < 2u || (int)d[1] >= (int)L103_PARAM_N)
        {
            Fault_ReportReject(CAN_FN_PARAM, CAN_FAULT_CMD_REJECT);
            param_q_push(op, (uint8_t)d[1], 0.0f, 1u);
            break;
        }
        start = (int)d[1];
        cnt   = (dlc >= 3u && d[2] != 0u) ? (int)d[2] : 1;
        for(i = start; i < start + cnt && i < (int)L103_PARAM_N; i++)
            param_q_push(op, (uint8_t)i, param_get(i), 0u);
        break;
    }

    case 0x02u:                       /* WRITE 一项 */
    {
        int idx = (int)d[1];
        float v;

        if(dlc < 7u || magic != CAN_PARAM_MAGIC || idx >= (int)L103_PARAM_N)
        {
            printf("CAN: PARAM write rejected - need b6=0x5A, dlc>=7, idx<%u\r\n", (unsigned)L103_PARAM_N);
            Fault_ReportReject(CAN_FN_PARAM, CAN_FAULT_CMD_REJECT);
            param_q_push(op, (uint8_t)idx, 0.0f, 1u);
            break;
        }
        v = rd_f32(&d[2]);
        if(!param_set(idx, v))
        {
            printf("CAN: PARAM[%d]=%s rejected (range / pp while enabled / read-only)\r\n",
                   idx, FmtF(v, 4));
            Fault_ReportReject(CAN_FN_PARAM, CAN_FAULT_CMD_REJECT);
            param_q_push(op, (uint8_t)idx, param_get(idx), 1u);
            break;
        }
        param_dirty = 1u;
        param_q_push(op, (uint8_t)idx, param_get(idx), 0u);
        break;
    }

    case 0x03u:                       /* SAVE -> Flash */
    {
        int ok;

        if(dlc < 7u || magic != CAN_PARAM_MAGIC)
        {
            Fault_ReportReject(CAN_FN_PARAM, CAN_FAULT_CMD_REJECT);
            param_q_push(op, 0xFDu, 0.0f, 1u);
            break;
        }
        if(Motion_IsEnabled())        /* 擦/写 Flash 会打断电流环, 不许在受力时存 */
        {
            printf("CAN: PARAM save refused - driver enabled\r\n");
            Fault_ReportReject(CAN_FN_PARAM, CAN_FAULT_CMD_REJECT);
            param_q_push(op, 0xFDu, 0.0f, 1u);
            break;
        }
        ok = Motion_SaveCfg();
        if(ok) { param_dirty = 0u; flash_ok_known = 0u; }
        printf("CAN: PARAM save -> %s\r\n", ok ? "OK" : "FAIL");
        param_q_push(op, 0xFDu, ok ? 1.0f : 0.0f, ok ? 0u : 1u);
        break;
    }

    case 0x04u:                       /* DEFAULTS: 擦掉 Flash 记录 (RAM 不动) */
    {
        int ok;

        if(dlc < 7u || magic != CAN_PARAM_MAGIC)
        {
            Fault_ReportReject(CAN_FN_PARAM, CAN_FAULT_CMD_REJECT);
            param_q_push(op, 0xFCu, 0.0f, 1u);
            break;
        }
        if(Motion_IsEnabled())
        {
            printf("CAN: PARAM defaults refused - driver enabled\r\n");
            Fault_ReportReject(CAN_FN_PARAM, CAN_FAULT_CMD_REJECT);
            param_q_push(op, 0xFCu, 0.0f, 1u);
            break;
        }
        ok = NvCfg_Erase();
        flash_ok_known = 0u;
        param_q_push(op, 0xFCu, ok ? 1.0f : 0.0f, ok ? 0u : 1u);
        break;
    }

    default:
        Fault_ReportReject(CAN_FN_PARAM, CAN_FAULT_CMD_REJECT);
        break;
    }
}

static void send_info_frame(void)
{
    uint8_t d[8];

    d[0] = (uint8_t)BOARD_ID;
    d[1] = (uint8_t)FW_VER_MAJOR;
    d[2] = (uint8_t)FW_VER_MINOR;
    d[3] = (uint8_t)FW_VER_PATCH;
    d[4] = (uint8_t)CAN_PROTO_VER;
    d[5] = 0u;
    d[6] = 0u;
    d[7] = 0u;

    (void)CAN_Bus_SendStd(CAN_ID_TX(CAN_FN_FB_INFO), d, 8);
}

static void send_feedback(void) { send_status_frame(CAN_ID_TX(CAN_FN_FB)); }

void CAN_Bus_Task(void)
{
    /* 1) 先处理收到的命令 */
    while(rxq_tail != rxq_head)
    {
        CanRxItem_t it;
        uint8_t i;
        uint8_t me;
        uint8_t node;

        it.id  = rxq[rxq_tail].id;
        it.dlc = rxq[rxq_tail].dlc;
        for(i = 0; i < 8u; i++) it.data[i] = rxq[rxq_tail].data[i];
        rxq_tail = (uint8_t)((rxq_tail + 1u) % CAN_RXQ_LEN);

        /* ① 节点过滤: 广播(0) 或 本机节点号 才执行, 别人的帧直接丢。
         *    me 每帧现读 —— 刚被 SETNODE 改过号也能立刻按新号收命令。 */
        me   = CAN_Bus_Node();
        node = CAN_ID_NODE(it.id);
        if(node != CAN_NODE_BCAST && node != me) continue;

        /* ② 功能号分派 (主机 -> 板那 0..15 段) */
        switch(CAN_ID_FUNC(it.id))
        {
            case CAN_FN_PV:       handle_pv(it.data, it.dlc);       break;
            case CAN_FN_VEL:      handle_vel(it.data, it.dlc);      break;
            case CAN_FN_ENABLE:   handle_enable(it.data, it.dlc);   break;
            case CAN_FN_HOME:     handle_home(it.data, it.dlc);     break;
            case CAN_FN_SETZERO:  handle_setzero(it.data, it.dlc);  break;
            case CAN_FN_QUERY:    handle_query(it.data, it.dlc);    break;
            case CAN_FN_FAULTCFG: handle_faultcfg(it.data, it.dlc); break;
            case CAN_FN_VLIM:     handle_vlim(it.data, it.dlc);     break;
            case CAN_FN_MOTORCFG: handle_motorcfg(it.data, it.dlc); break;
            case CAN_FN_SEG:      handle_seg(it.data, it.dlc);      break;
            case CAN_FN_SEGCTL:   handle_segctl(it.data, it.dlc);   break;
            case CAN_FN_SEG_HR:   handle_seg_hr(it.data, it.dlc);   break;
            case CAN_FN_SETNODE:  handle_setnode(it.data, it.dlc);  break;
            case CAN_FN_ALIGN:    handle_align(it.data, it.dlc);    break;
            case CAN_FN_INFO:     handle_info(it.data, it.dlc);     break;
            case CAN_FN_PARAM:    handle_param(it.data, it.dlc);    break;
            default: break;   /* 板->主机那半 (16..31) 与保留号一律忽略 */
        }
    }

    /* 2) 参数应答队列: 每次任务只发一帧, 别把主循环堵住 */
    {
        uint8_t q[8];

        if(param_q_pop(q))
            (void)CAN_Bus_SendStd(CAN_ID_TX(CAN_FN_FB_PARAM), q, 8);
    }

    /* 3) 10ms 反馈 —— **只在 debug 打开时发** (默认被动: 上位机不问就不发) */
    if(dbg_on && fb_on)
    {
        uint32_t now = Micros();
        if((int32_t)(now - next_fb_us) >= 0)
        {
            next_fb_us += CAN_FB_PERIOD_US;
            /* 落后太多 (比如刚做完 flash 擦写) 就重新对齐, 免得追着补发一串 */
            if((int32_t)(now - next_fb_us) > (int32_t)CAN_FB_PERIOD_US)
                next_fb_us = now + CAN_FB_PERIOD_US;
            send_feedback();
        }
    }
    else
    {
        next_fb_us = Micros() + CAN_FB_PERIOD_US;
    }
}

void USB_LP_CAN1_RX0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        while(CAN_MessagePending(CAN1, CAN_FIFO0))
        {
            CanRxMsg rx;
            uint8_t  next;

            CAN_Receive(CAN1, CAN_FIFO0, &rx);

            next = (uint8_t)((rxq_head + 1u) % CAN_RXQ_LEN);
            if(next != rxq_tail)
            {
                rxq[rxq_head].id  = (uint16_t)(rx.StdId & 0x7FFu);
                rxq[rxq_head].dlc = (uint8_t)rx.DLC;
                memcpy((void *)rxq[rxq_head].data, rx.Data, 8);
                rxq_head = next;
            }
            else
            {
                n_overrun++;
            }

            n_rx++;
            last_id  = (uint16_t)(rx.StdId & 0x7FFu);
            last_dlc = (uint8_t)rx.DLC;
        }
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

uint32_t CAN_Bus_RxCount(void)  { return n_rx; }
uint32_t CAN_Bus_TxCount(void)  { return n_tx; }
uint32_t CAN_Bus_TxFail(void)   { return n_txfail; }
uint32_t CAN_Bus_Overrun(void)  { return n_overrun; }
uint32_t CAN_Bus_LastId(void)   { return (uint32_t)last_id; }
int      CAN_Bus_LastRxDlc(void) { return (int)last_dlc; }

void CAN_Bus_ResetCounters(void)
{
    n_rx = 0; n_tx = 0; n_txfail = 0; n_overrun = 0;
}

void CAN_Bus_SendTest(void)
{
    uint8_t  d[8] = { 0xA5, 0x5A, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
    uint16_t id   = CAN_ID_TX(CAN_FN_FB);
    int r = CAN_Bus_SendStd(id, d, 8);

    printf("CAN: test frame id=0x%03X (node=%u) -> %s (tx=%lu fail=%lu)\r\n",
           (unsigned)id, (unsigned)CAN_Bus_Node(), r ? "FAIL" : "sent",
           (unsigned long)n_tx, (unsigned long)n_txfail);
}

void CAN_Bus_PrintStatus(void)
{
    uint32_t errsr = CAN1->ERRSR;
    uint32_t statr = CAN1->STATR;

    printf("CAN: node=%u (%s, 0=广播 1..63=设备) | ID=(node<<5)|fn\r\n",
           (unsigned)CAN_Bus_Node(), Motion_NodeFromFlash() ? "from Flash" : "default");
    printf("CAN: ready=%d fb=%s upstream(debug)=%s n_rx=%lu n_tx=%lu tx_fail=%lu overrun=%lu\r\n",
           (int)ready, fb_on ? "ON" : "OFF", dbg_on ? "ON" : "OFF",
           (unsigned long)n_rx, (unsigned long)n_tx,
           (unsigned long)n_txfail, (unsigned long)n_overrun);
    printf("CAN: bitrate=%luHz presc=%u bs1=%u bs2=%u (PCLK1=%luHz) | last rx id=0x%03lX dlc=%d\r\n",
           (unsigned long)(SystemCoreClock / 2u / CAN_PRESCALER / 24u),
           (unsigned)CAN_PRESCALER, 15u, 8u,
           (unsigned long)(SystemCoreClock / 2u),
           (unsigned long)last_id, (int)last_dlc);
    printf("CAN: STATR=0x%08lX ERRSR=0x%08lX TEC=%lu REC=%lu LEC=0x%lX\r\n",
           (unsigned long)statr, (unsigned long)errsr,
           (unsigned long)((errsr >> 16) & 0xFFu), (unsigned long)((errsr >> 24) & 0xFFu),
           (unsigned long)(errsr & 0x07u));
    /* BTIMR 直接读出来核对波特率: 位定义见 ch32l103_can.c CAN_Init() 里那两行 */
    printf("CAN: BTIMR=0x%08lX -> mode=%lu sjw=%lu bs2=%lu(+1) bs1=%lu(+1) presc=%lu  -> %lu bit/s\r\n",
           (unsigned long)CAN1->BTIMR,
           (unsigned long)((CAN1->BTIMR >> 30) & 0x3u),
           (unsigned long)((CAN1->BTIMR >> 24) & 0x3u),
           (unsigned long)((CAN1->BTIMR >> 20) & 0x7u),
           (unsigned long)((CAN1->BTIMR >> 16) & 0xFu),
           (unsigned long)((CAN1->BTIMR & 0x3FFu) + 1u),
           (unsigned long)((SystemCoreClock / 2u) /
                           (((CAN1->BTIMR & 0x3FFu) + 1u) *
                            (1u + ((CAN1->BTIMR >> 20) & 0x7u) + 1u + ((CAN1->BTIMR >> 16) & 0xFu) + 1u))));
    printf("CAN: AFIO_PCFR1=0x%08lX (CAN_RM[14:13]=%lu 应为3, PD0PD1_RM(b15)=%lu 应为1)\r\n",
           (unsigned long)AFIO->PCFR1,
           (unsigned long)((AFIO->PCFR1 >> 13) & 0x3u),
           (unsigned long)((AFIO->PCFR1 >> 15) & 0x1u));
}

/* ---- 收发器(MAX3051)回路自测 (2026-10-18) ----
 * 做法: 把 PD1 临时当普通 GPIO 拉高/拉低, 从 PD0(RXD) 读回总线电平。
 *   通过 = PD1 拉低(显性) 时 PD0 也变 0, 拉高(隐性) 时 PD0 = 1,
 *          说明 MCU 引脚 -> U4.TXD -> CANH/CANL -> U4.RXD -> MCU 这一整圈是通的。
 * 不动运动/驱动状态, 只在串口手动敲 'Y' 时跑。 */
int CAN_Bus_XcvLoopTest(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t idle, hi, lo, hi_ok = 1u;
    volatile int spin;
    int i, k;

    idle = (uint8_t)((GPIOD->INDR & 0x0001u) ? 1u : 0u);

    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    GPIO_SetBits(GPIOD, GPIO_Pin_1);              /* TXD = 1 -> 隐性 */
    for(spin = 0; spin < 4000; spin++) { }
    hi = 0u;
    for(k = 0; k < 5; k++)
    {
        if(GPIOD->INDR & 0x0001u) { hi = 1u; }
        else { hi_ok = 0u; }
        for(spin = 0; spin < 800; spin++) { }
    }

    GPIO_ResetBits(GPIOD, GPIO_Pin_1);            /* TXD = 0 -> 显性 */
    for(spin = 0; spin < 4000; spin++) { }
    lo = (uint8_t)((GPIOD->INDR & 0x0001u) ? 1u : 0u);

    GPIO_SetBits(GPIOD, GPIO_Pin_1);
    for(spin = 0; spin < 800; spin++) { }

    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;            /* 还原成 CAN_TX 复用 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    printf("XCV: PD0 idle=%u | TXD=1 -> PD0=%u%s | TXD=0 -> PD0=%u\r\n",
           (unsigned)idle, (unsigned)hi, hi_ok ? "" : "(抖动)", (unsigned)lo);

    if(!hi_ok || hi != 1u)
    {
        printf("XCV: FAIL - 隐性时 RXD 被拉住 (总线被别处拉低或短路)\r\n");
        return 1;
    }
    if(lo != 0u)
    {
        printf("XCV: FAIL - 拉显性后 RXD 不跟着变 (PD1 焊点 / U4.TXD / 驱动级不通)\r\n");
        return 2;
    }
    printf("XCV: OK - 收发器回路通 (PD1 -> U4 -> CANH/L -> U4 -> PD0)\r\n");
    return 0;
}

/* ---- 引脚强制拉低 (2026-10-18) ----
 * 'V0': PD0(4脚) / PD1(5脚) 都配成推挽输出并拉低, 用来量 MCU 焊盘与走线;
 * 'V1': 还原 CAN 用法 (PD1 复用推挽, PD0 输入上拉)。
 * V0 期间总线被拉成显性且与 U4.RXD 对顶, 只在台面上短时间用, 别挂别的节点。 */
void CAN_Bus_PinsLow(int on)
{
    GPIO_InitTypeDef gpio = {0};

    if(on)
    {
        gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
        gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
        gpio.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOD, &gpio);
        GPIO_ResetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1);
        printf("PIN: PD0(4)=0 PD1(5)=0 推挽拉低 保持中 | INDR=0x%04lX CFGLR=0x%08lX\r\n",
               (unsigned long)GPIOD->INDR, (unsigned long)GPIOD->CFGLR);
    }
    else
    {
        gpio.GPIO_Pin   = GPIO_Pin_1;
        gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOD, &gpio);
        gpio.GPIO_Pin   = GPIO_Pin_0;
        gpio.GPIO_Mode  = GPIO_Mode_IPU;
        GPIO_Init(GPIOD, &gpio);
        printf("PIN: 还原 CAN 用法 (PD1=AF_PP PD0=IPU)\r\n");
    }
}
