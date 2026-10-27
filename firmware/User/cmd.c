#include "cmd.h"
#include "board.h"
#include "debug.h"
#include "fmt.h"
#include "motor.h"
#include "foc.h"
#include "enc.h"
#include "csense.h"
#include "drv8316c.h"
#include "motion.h"
#include "can_bus.h"
#include "faults.h"
#include "nvcfg.h"
#include "posstore.h"

#define RX_RING_SIZE   192
#define LINE_MAX       64

static volatile char rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head, rx_tail;
static char line[LINE_MAX];
static uint16_t line_len;

static uint8_t st_auto;
static uint32_t st_next_us;

/* -----------------------------------------------------------------------------
 * 轻量十进制解析 —— 不用 atof/strtod。
 * mcu-flash-slimming 实测过: newlib 的 atof 会拉进 strtod 全家约 18.7KB Flash,
 * 这块板 63KB 根本装不下。
 * -------------------------------------------------------------------------- */
int Cmd_ParseFloat(const char **pp, float *out)
{
    const char *p = *pp;
    float v = 0.0f, sc;
    int   neg = 0, digits = 0;

    while(*p == ' ' || *p == '\t') p++;
    if(*p == '+' || *p == '-') { neg = (*p == '-'); p++; }

    while(*p >= '0' && *p <= '9') { v = v * 10.0f + (float)(*p - '0'); p++; digits++; }
    if(*p == '.')
    {
        p++; sc = 0.1f;
        while(*p >= '0' && *p <= '9') { v += (float)(*p - '0') * sc; sc *= 0.1f; p++; digits++; }
    }
    if(!digits) return 0;

    *out = neg ? -v : v;
    *pp  = p;
    return 1;
}

int Cmd_ParseInt(const char **pp, int *out)
{
    float f;
    if(!Cmd_ParseFloat(pp, &f)) return 0;
    *out = (int)(f + (f < 0.0f ? -0.5f : 0.5f));
    return 1;
}

/* ---- USART1 接收 ---- */
void Cmd_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_USART1 | RCC_PB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate            = DEBUG_BAUD_DEFAULT;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* 抢占优先级必须低于 TIM3 (=0): 串口收字节晚一点没关系, 电流环一拍也不许晚。
     * 注意 WCH 的 NVIC_Init 只对抢占值 0 和 1 真正写寄存器 (2026-10-18 审计),
     * 所以这里只能填 0 或 1, 填 2/3 会静默不写。 */
    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    rx_head = rx_tail = 0;
    line_len = 0;
    st_auto  = 0u;
    st_next_us = Micros();

    printf("CMD: ready (baud=%lu)\r\n", (unsigned long)DEBUG_BAUD_DEFAULT);
}

void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        char     ch   = (char)(USART_ReceiveData(USART1) & 0xFF);
        uint16_t next = (uint16_t)((rx_head + 1u) % RX_RING_SIZE);

        if(next != rx_tail)
        {
            rx_ring[rx_head] = ch;
            rx_head = next;
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void Cmd_Poll(void)
{
    while(rx_tail != rx_head)
    {
        char ch = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % RX_RING_SIZE);

        if(ch == '\r' || ch == '\n')
        {
            if(line_len > 0u)
            {
                line[line_len] = '\0';
                Cmd_Exec(line);
                line_len = 0u;
            }
        }
        else if(line_len < (LINE_MAX - 1u))
        {
            line[line_len++] = ch;
        }
    }

    if(st_auto)
    {
        uint32_t now = Micros();
        if((int32_t)(now - st_next_us) >= 0)
        {
            /* 累加而不是 rebase: 让 ST 行严格落在 100ms 栅格上, 不被打印耗时推后。
             * 落后超过一整拍 (主循环被卡) 才重新对齐, 免得连打补齐。 */
            st_next_us += 100000u;
            if((int32_t)(now - st_next_us) >= 0) st_next_us = now + 100000u;
            Cmd_Exec("?");
        }
    }
}

/* ---- 输出 ---- */
static const char *mode_name(Mot_Mode m)
{
    switch(m)
    {
        case MOT_OFF:      return "OFF";
        case MOT_ALIGN:    return "ALIGN";
        case MOT_TORQUE:   return "TORQUE";
        case MOT_VELOCITY: return "VELOCITY";
        case MOT_POSITION: return "POSITION";
        case MOT_OPENLOOP: return "OPENLOOP";
        default:           return "?";
    }
}

/* 定点量到人读值 (Q12 -> 物理量) */
#define Q12F(x)  ((float)(x) * (1.0f/4096.0f))

static void cmd_status(void)
{
    printf("ST mode=%s en=%u al=%u flt=%u/%lu pos=%s vel=%s iq=%s id=%s th=%s\r\n",
           mode_name(motor.mode), (unsigned)motor.enabled, (unsigned)motor.aligned,
           (unsigned)motor.fault, (unsigned long)motor.fault_code,
           FmtF(motor.pos_m, 3), FmtF(motor.omega_m, 2),
           FmtF(Q12F(foc.iq_q12), 3), FmtF(Q12F(foc.id_q12), 3),
           FmtF((float)motor.theta_e_u16 * (6.28318530718f/65536.0f), 3));
    printf("   ref vel=%s iq=%s vd=%s vq=%s idfw=%s imax=%s vbus=%s isr=%uus(max %uus) pk=%uus(max %uus)\r\n",
           FmtF(motor.vel_ref, 2), FmtF(motor.iq_ref, 3),
           FmtF(Q12F(foc.vd_q12), 2), FmtF(Q12F(foc.vq_q12), 2),
           FmtF(motor.id_fw, 3), FmtF(Q12F(foc.i_abs_max_q12), 3), FmtF(motor.vbus, 1),
           (unsigned)motor.isr_us_last, (unsigned)motor.isr_us_max,
           (unsigned)motor.isr_pk_us_last, (unsigned)motor.isr_pk_us_max);
    /* 临时诊断 (2026-10-23): 过流延时计数 */
    printf("   oc ms=%u cnt=%u winmax=%ld trip=%ld\r\n",
           (unsigned)foc.oc_trip_ms, (unsigned)foc.oc_ms_cnt,
           (long)foc.oc_win_max, (long)foc.oc_trip_q12);
    /* 位置掉电保存状态 (posstore): 停稳 3s 写一次 Flash */
    printf("   pos seq=%lu ok=%lu fail=%lu home=%u busy=%u saved=%s\r\n",
           (unsigned long)PosStore_Seq(), (unsigned long)PosStore_WriteOk(),
           (unsigned long)PosStore_WriteFail(), (unsigned)PosStore_HomeSet(),
           (unsigned)Motion_Busy(), FmtF(PosStore_SavedMech(), 3));
}

static void cmd_diag(void)
{
    float ia, ib, ic;
    uint8_t mg = Enc_Mg();

    CS_GetIsr(&ia, &ib, &ic);
    printf("DIAG csync=%lu miss=%lu valid=%d recon=%lu\r\n",
           (unsigned long)CS_Count(), (unsigned long)CS_Miss(),
           CS_Valid(), (unsigned long)foc.recon_cnt);
    printf("     ia=%s ib=%s ic=%s (A)  sum=%s\r\n",
           FmtF(ia, 3), FmtF(ib, 3), FmtF(ic, 3), FmtF(ia + ib + ic, 3));
    printf("     duty=%s/%s/%s vlim=%s sat=%lu\r\n",
           FmtF((float)foc.duty_q15[0] * (1.0f/32768.0f), 3),
           FmtF((float)foc.duty_q15[1] * (1.0f/32768.0f), 3),
           FmtF((float)foc.duty_q15[2] * (1.0f/32768.0f), 3),
           FmtF(foc.v_limit, 2), (unsigned long)foc.sat_cnt);
    printf("     enc frames=%lu err=%lu crc=%lu raw=%u pp=%d\r\n",
           (unsigned long)Enc_Frames(), (unsigned long)Enc_ErrCnt(),
           (unsigned long)Enc_CrcErrCnt(), (unsigned)Enc_Raw(), Motor_PolePairs());
    printf("     enc mg=0x%X field=%s push=%u overspeed=%u valid=%u\r\n",
           (unsigned)mg,
           ((mg & 3u) == 0u) ? "NORMAL" : (((mg & 3u) == 1u) ? "STRONG" : (((mg & 3u) == 2u) ? "WEAK" : "RESERVED")),
           (unsigned)((mg >> 2) & 1u), (unsigned)((mg >> 3) & 1u), (unsigned)Enc_MgValid());
    {
        int32_t vmn, vmx;
        Enc_VelSpan(&vmn, &vmx, 1);
        printf("     vel span=%s..%s rad/s (peak-to-peak since last Q)\r\n",
               FmtF((float)vmn * (6.28318530718f / 16777216.0f), 2),
               FmtF((float)vmx * (6.28318530718f / 16777216.0f), 2));
        printf("     win vel=%s rad/s (20ms window)\r\n",
               FmtF(Enc_PeriodVel() * 6.28318530718f, 3));
    }
    {
        float skp, ski;
        Motor_GetSpeedGains(&skp, &ski);
        printf("     cur kp=%s ki=%s | spd kp=%s ki=%s | poskp=%s hold=%u vlp=%s\r\n",
               FmtF(foc.kp, 4), FmtF(foc.ki, 1),
               FmtF(skp, 5), FmtF(ski, 2), FmtF(motor.pos_kp, 2),
               (unsigned)Motor_HoldActive(), FmtF(Motor_VelLp(), 2));
    }
    printf("     vm=%s nFAULT=%u\r\n", FmtF(CS_VM_Volt(), 2), (unsigned)DRV8316C_Faulted());
}

static void cmd_mg(void)
{
    uint8_t mg = Enc_Mg();

    printf("MG raw=%u mg=0x%X field=%s push=%u overspeed=%u valid=%u frames=%lu crc_err=%lu\r\n",
           (unsigned)Enc_Raw(), (unsigned)mg,
           ((mg & 3u) == 0u) ? "NORMAL" : (((mg & 3u) == 1u) ? "STRONG" : (((mg & 3u) == 2u) ? "WEAK" : "RESERVED")),
           (unsigned)((mg >> 2) & 1u), (unsigned)((mg >> 3) & 1u), (unsigned)Enc_MgValid(),
           (unsigned long)Enc_Frames(), (unsigned long)Enc_CrcErrCnt());
}

int Cmd_IsAligned(void) { return motor.aligned ? 1 : 0; }

static int starts(const char *s, const char *pre)
{
    while(*pre) { if(*s != *pre) return 0; s++; pre++; }
    return 1;
}

void Cmd_Exec(const char *ln)
{
    const char *p = ln;
    float a = 0.0f, b = 0.0f;
    int   n;

    if(*p == '\0') return;

    /* ---- 两字母命令先判 ---- */
    if(starts(p, "PP"))      { p += 2; if(Cmd_ParseInt(&p, &n)) { Motor_SetPolePairs(n); printf("OK pp=%d（对齐作废，需重跑 'A'）\r\n", Motor_PolePairs()); } return; }
    if(starts(p, "RL"))      { p += 2; if(Cmd_ParseFloat(&p, &a)) { if(*p == ',') p++; if(Cmd_ParseFloat(&p, &b)) { Motor_SetGainsFromRL(a, b * 0.001f, CUR_BW_HZ_DEF); Motor_GetCurGains(&a, &b); printf("OK cur kp=%s ki=%s\r\n", FmtF(a, 4), FmtF(b, 1)); } } return; }
    if(starts(p, "KP"))      { p += 2; if(Cmd_ParseFloat(&p, &a)) { Motor_SetPosKp(a); printf("OK poskp=%s\r\n", FmtF(motor.pos_kp, 2)); } return; }
    if(starts(p, "VA"))      { p += 2; if(Cmd_ParseFloat(&p, &a)) { Motor_SetAlignVolt(a); printf("OK valign=%s\r\n", FmtF(motor.v_align, 2)); } return; }
    if(starts(p, "CS"))      { int i; p += 2; if(Cmd_ParseInt(&p, &i)) { if(*p == ',') p++; if(Cmd_ParseFloat(&p, &a)) { CS_SetSign((uint8_t)i, a); printf("OK sign[%d]=%s\r\n", i, FmtF(CS_Sign((uint8_t)i), 0)); } } return; }
    if(starts(p, "CZ"))      { CS_CalZero(); printf("OK zero A=%s B=%s C=%s V（此时桥必须没通电）\r\n",
                                FmtF(CS_ZeroVolt(0), 4), FmtF(CS_ZeroVolt(1), 4), FmtF(CS_ZeroVolt(2), 4)); return; }
    if(starts(p, "FW"))      { if(p[2] == '1') Motor_SetFw(1); else Motor_SetFw(0); printf("OK fw=%u\r\n", (unsigned)motor.fw_enable); return; }
    if(starts(p, "PB"))      { p += 2; if(Cmd_ParseFloat(&p, &a)) { Enc_SetPllBandwidth(a); printf("OK pll bw=%s Hz\r\n", FmtF(Enc_PllBandwidth(), 1)); } return; }
    if(starts(p, "PH"))      { p += 2; if(Cmd_ParseInt(&p, &n)) Motor_SetPosHold((uint8_t)(n != 0));
                               printf("OK pos_hold=%u\r\n", (unsigned)Motor_GetPosHold()); return; }
    if(starts(p, "MG"))      { cmd_mg(); return; }
    if(starts(p, "VS"))      { p += 2; if(Cmd_ParseInt(&p, &n)) { Enc_SetVelSrc((uint8_t)n); printf("OK velsrc=%u (0自动 1PLL 2窗口)\r\n", (unsigned)Enc_VelSrc()); } return; }
    if(starts(p, "SD"))      { p += 2; if(Cmd_ParseInt(&p, &n)) { Enc_CapEnable((uint8_t)(n != 0)); printf("OK SD=%u (原始角 %uHz, 每行 seq,raw 十六进制)\r\n", (unsigned)Enc_CapEnabled(), (unsigned)(PWM_FREQ_HZ / ENC_DECIM / ENC_CAP_DIV)); } return; }
    if(starts(p, "SAVE"))    { if(Motion_SaveCfg()) printf("OK 参数已存 Flash\r\n");
                               else printf("ERR 存 Flash 失败\r\n"); return; }
    if(starts(p, "CFG"))     { NvCfg_PrintInfo(); return; }
    if(starts(p, "DFLT"))    { if(NvCfg_Erase()) printf("OK 已擦掉保存的参数, 断电重上电回默认\r\n");
                               else printf("ERR 擦除失败\r\n"); return; }
    if(starts(p, "POS"))     { PosStore_PrintInfo(); return; }
    if(starts(p, "RESET"))   { printf("OK reset (先失能再重启)\r\n"); Motion_Stop(1); Motor_Disable();
                               Delay_Ms(100); NVIC_SystemReset(); return; }
    if(starts(p, "ST"))      { if(p[2] == '0') st_auto = 0u; else if(p[2] == '1') st_auto = 1u; printf("OK auto=%u\r\n", (unsigned)st_auto); return; }

    switch(*p)
    {
    case '?': cmd_status(); break;
    case 'Q': cmd_diag();   break;

    /* 'K'        看 CAN/故障/动作层总状态
     * 'K1' 'K0'  主动上报总闸 (10ms 反馈 / 故障事件), 默认 0 = 被动, 不存 Flash
     * 'J'        发一帧测试帧 (看总线上有没有人 ACK) */
    case 'K':
        if(p[1] == '0') { CAN_Bus_SetDebug(0); printf("OK upstream=0 (被动: 不问不发)\r\n"); }
        else if(p[1] == '1') { CAN_Bus_SetDebug(1); printf("OK upstream=1 (10ms 反馈/故障事件)\r\n"); }
        else { CAN_Bus_PrintStatus(); Fault_Print(); Motion_PrintStatus(); }
        break;

    case 'J': CAN_Bus_SendTest(); break;

    case 'Y': CAN_Bus_XcvLoopTest(); break;   /* 收发器回路自测 */

    case 'v': CAN_Bus_PinsLow((p[1]=='1') ? 0 : 1); break;   /* v0=PD0/PD1推挽拉低 v1=还原 */

    /* '%' 看节点号; '%3' 设成 3 号并立刻存 Flash (改完建议断电重上电核对) */
    case '%':
    {
        const char *q = ln + 1;
        if(*q == '\0') { Motion_PrintNode(); break; }
        if(Cmd_ParseInt(&q, &n)) { if(Motion_SetNode(n) == 0) printf("OK node=%d\r\n", Motion_Node()); }
        else printf("ERR 节点号要 1..63\r\n");
        break;
    }

    /* '*' 板载动作脚本 (与 CAN 的 SEG/SEGCTL 同一套引擎, 没有 CAN 硬件时用它验):
     *   *{wait_us,rpm,deg,dwell_ms}{...}   追加若干段
     *   *!   跑一遍        *!3  跑 3 遍
     *   *0   清空          *?   打印队列 */
    case '*':
    {
        const char *q = ln + 1;

        if(*q == '\0' || *q == '?') { Motion_Seq_Print(); break; }
        if(*q == '0') { Motion_Seq_Clear(); break; }
        if(*q == '!')
        {
            int loops = 1;
            q++;
            if(Cmd_ParseInt(&q, &loops)) { /* 用了给的值 */ }
            if(Motion_Seq_Start(loops) != 0) printf("ERR seq start 被拒 (见上面原因)\r\n");
            break;
        }

        {
            int added = 0;
            while(*q)
            {
                float w, rpm, dg, dw;

                while(*q == ' ' || *q == '\t' || *q == ';' || *q == ',') q++;
                if(*q == '\0') break;
                if(*q != '{') { printf("ERR seq: 这里要 '{', 实际 '%c'\r\n", *q); break; }
                q++;
                if(!Cmd_ParseFloat(&q, &w)   || *q != ',') { printf("ERR seq 第1段 wait\r\n"); break; }
                q++;
                if(!Cmd_ParseFloat(&q, &rpm) || *q != ',') { printf("ERR seq 第2段 rpm\r\n"); break; }
                q++;
                if(!Cmd_ParseFloat(&q, &dg)  || *q != ',') { printf("ERR seq 第3段 deg\r\n"); break; }
                q++;
                if(!Cmd_ParseFloat(&q, &dw)  || *q != '}') { printf("ERR seq 第4段 dwell\r\n"); break; }
                q++;
                if(Motion_Seq_AddSeg((long)w, (int)rpm, dg, (long)dw) != 0) break;
                added++;
            }
            if(added) printf("OK seq +%d 段\r\n", added);
        }
        break;
    }

    /* 'T': 同一时刻的时基/转角快照, 给上位机算"每拍多少 us"和"极对数"用。
     * 两帧相隔数十秒做差: dcs = PWM 拍数, dus = Micros 增量, dpos = 机械角增量。
     *   us_per_tick = dus/dcs   (两路都在板上, 与串口时序无关)
     *   pole_pairs  = (dcs * f_cmd/20000) / (dpos/2pi)   (f_cmd 是 'R' 给的电频率) */
    case 'T':
        printf("T us=%lu cs=%lu pk=%lu vc=%lu pos=%s vel=%s\r\n",
               (unsigned long)Micros(), (unsigned long)CS_Count(),
               (unsigned long)motor.isr_pk_cnt, (unsigned long)motor.isr_cnt,
               FmtF(motor.pos_m, 4), FmtF(motor.omega_m, 4));
        break;

    /* 2026-10-19: 'e' 一律"使能并保持当前位置", 不许它把上一条遗留目标捡起来。
     * 大写 E/D/F/N/B 是上位机 UART 那半边一直在发的字母 (GUI 的使能/设零点/回零/停),
     * 做别名收下 —— 以前这里只有小写, 界面上那几个按钮按下去只会回"未知命令"。 */
    case 'e':
    case 'E':
        Motor_RequestMode(MOT_POSITION);
        Motor_SetPosition(Enc_MechRad());
        Motor_Enable();
        printf("OK enable req (hold %s rad)\r\n", FmtF(Enc_MechRad(), 4));
        break;
    case 'd':
    case 'D': Motor_Disable(); printf("OK disable req\r\n"); break;
    case 'B':
        p++;
        Motion_Stop(1u);                        /* 2026-10-23: B0/B1 都是停+失能, 不再'停并保持' */
        break;
    case 'A': Motor_StartAlign(); printf("OK align start (约 %ums)\r\n", (unsigned)ALIGN_TIME_MS_DEF); break;
    case 'X': Motor_ClearFault(); printf("OK fault cleared\r\n"); break;

    /* O<A>[,<ms>]: 相电流跳闸阈值 + 连续超限多久才跳 (2026-10-23 加;
     * 以前 O 是"清故障", 现在清故障用 X)。不给参数 = 只回报当前值。 */
    case 'O': p++;
        if(Cmd_ParseFloat(&p, &a))
        {
            if(a < MOTOR_OC_TRIP_MIN || a > MOTOR_OC_TRIP_MAX)
            {
                printf("ERR oc_trip range %s..%s A\r\n",
                       FmtF(MOTOR_OC_TRIP_MIN, 2), FmtF(MOTOR_OC_TRIP_MAX, 2));
                break;
            }
            Motor_SetOcTrip(a);
            if(*p == ',') p++;
            if(Cmd_ParseInt(&p, &n))
            {
                if(n < 0 || n > (int)MOTOR_OC_TRIP_MS_MAX)
                {
                    printf("ERR oc_trip_ms range 0..%u ms\r\n", (unsigned)MOTOR_OC_TRIP_MS_MAX);
                    break;
                }
                Motor_SetOcTripMs((uint16_t)n);
            }
            printf("OK oc_trip=%s A delay=%ums\r\n",
                   FmtF(motor.oc_trip, 3), (unsigned)motor.oc_trip_ms);
        }
        else printf("OK oc_trip=%s A delay=%ums\r\n",
                    FmtF(motor.oc_trip, 3), (unsigned)motor.oc_trip_ms);
        break;

    case 'M': p++; if(Cmd_ParseInt(&p, &n)) {
        /* M<n> 顺手把使能也点上, 免得设了模式却没输出 (M0 = 停并使能) */
        Motor_RequestMode((Mot_Mode)n);
        if(n == 0) Motor_Disable(); else Motor_Enable();
        printf("OK mode=%s\r\n", mode_name((Mot_Mode)n));
    } break;

    case 'I': p++; if(Cmd_ParseFloat(&p, &a)) { Motor_SetIq(a); printf("OK iq_cmd=%s A\r\n", FmtF(a, 3)); } break;
    case 'L': p++; if(Cmd_ParseFloat(&p, &a)) { Motor_SetILimit(a); printf("OK i_limit=%s A\r\n", FmtF(motor.i_limit, 3)); } break;
    case 'V': p++; if(Cmd_ParseFloat(&p, &a)) { Motor_SetVelocity(a); printf("OK vel_cmd=%s rad/s\r\n", FmtF(a, 3)); } break;
    case 'P': p++; if(Cmd_ParseFloat(&p, &a)) { Motor_SetPosition(a); printf("OK pos_cmd=%s rad\r\n", FmtF(a, 3)); } break;
    case 'W': p++; if(Cmd_ParseFloat(&p, &a)) { Motor_SetVelLimit(a); printf("OK vel_limit=%s\r\n", FmtF(motor.vel_limit, 2)); } break;
    case 'G': p++; if(Cmd_ParseFloat(&p, &a)) { Motor_SetAccelLimit(a); printf("OK accel=%s\r\n", FmtF(motor.accel_limit, 1)); } break;
    case 'C': p++; if(Cmd_ParseFloat(&p, &a)) { if(*p == ',') p++; if(Cmd_ParseFloat(&p, &b)) { Motor_SetCurGains(a, b); Motor_GetCurGains(&a, &b); printf("OK cur kp=%s ki=%s\r\n", FmtF(a, 4), FmtF(b, 1)); } } break;
    case 'S': p++; if(Cmd_ParseFloat(&p, &a)) { if(*p == ',') p++; if(Cmd_ParseFloat(&p, &b)) { Motor_SetSpeedGains(a, b); printf("OK spd kp=%s ki=%s\r\n", FmtF(a, 5), FmtF(b, 2)); } } break;
    case 'R': p++; if(Cmd_ParseFloat(&p, &a)) { if(*p == ',') p++; if(Cmd_ParseFloat(&p, &b)) { Motor_StartOpenloop(a, b); printf("OK openloop %sV %sHz\r\n", FmtF(a, 2), FmtF(b, 2)); } } break;

    case 'F':
    case 'Z':
        Motion_SetHomeHere();          /* 设零位 + 目标跟着新坐标走 + 记 home_set */
        printf("OK 当前位置 = 机械 0\r\n");
        break;

    case 'N':
        p++;
        /* 'N' / 'N<rpm>': 回机械原点 (GUI 的「回机械 0 点」) */
        if(!Cmd_ParseFloat(&p, &a) || a <= 0.0f) a = 0.0f;
        if(Motion_GotoHome(a * 6.28318530718f / 60.0f) != 0)
            printf("ERR 还没设机械零点 (先 F / Z / CAN 功能3)\r\n");
        break;

    default:
        printf("ERR 未知命令 '%s'（CAP 里是命令表）\r\n", ln);
        break;
    }
}
