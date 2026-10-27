#include "faults.h"
#include "board.h"
#include "debug.h"
#include "fmt.h"
#include "can_bus.h"
#include "motion.h"
#include "motor.h"
#include "enc.h"
#include "csense.h"
#include "drv8316c.h"
#include "nvcfg.h"
#include "foc.h"

/* =============================================================================
 * 故障位图 + 主动上报 —— 采集来源见 faults.h
 * ========================================================================== */

#define FAULT_EVT_GAP_US        20000u      /* 两次事件帧最小间隔 20ms (设计值: 防止抖起来刷屏) */
#define FAULT_REJECT_HOLD_US    1000000u    /* "命令被拒"脉冲 1s */
#define FAULT_REPORT_DEF_100MS  10u         /* 默认 1s 重报一次位图 */
#define FAULT_OC_CLEAR_RATIO    0.8f

static uint16_t active, latched, prev_active;
static uint32_t n_events;
static uint8_t  rep_on = 1u, rep_period = FAULT_REPORT_DEF_100MS;
static uint32_t next_period_us, next_evt_us;

static uint8_t  was_enabled;
static uint32_t flash_fail_seen;
static uint8_t  sens_seen;
static uint32_t reject_until_us;
static int16_t  reject_cmd;
static uint8_t  align_latch;

/* 被限速挡下的那一拍: 把要报的位攒着, 下一拍接着发, 不丢 */
static uint16_t pend_set, pend_clr;

static const char *fault_name(uint16_t bit)
{
    switch(bit)
    {
        case CAN_FAULT_STALL:       return "STALL";
        case CAN_FAULT_OVERCURRENT: return "OVERCURRENT";
        case CAN_FAULT_DISABLED:    return "DISABLED";
        case CAN_FAULT_VM_LOW:      return "VM_LOW";
        case CAN_FAULT_SENSOR:      return "SENSOR";
        case CAN_FAULT_NOT_ALIGNED: return "NOT_ALIGNED";
        case CAN_FAULT_FLASH_FAIL:  return "FLASH_FAIL";
        case CAN_FAULT_DRV_NFAULT:  return "DRV_NFAULT";
        case CAN_FAULT_CMD_REJECT:  return "CMD_REJECT";
        default:                    return "?";
    }
}

static int send_event(uint16_t bit, uint8_t is_clear, uint8_t kind, int16_t info)
{
    uint8_t  d[8];
    uint32_t now = Micros();
    uint16_t ms;

    if((int32_t)(now - next_evt_us) < 0) return 0;
    next_evt_us = now + FAULT_EVT_GAP_US;

    ms = (uint16_t)(now / 1000u);

    d[0] = (uint8_t)(bit & 0xFFu);
    d[1] = (uint8_t)((bit >> 8) & 0xFFu);
    d[2] = is_clear;
    d[3] = kind;
    d[4] = (uint8_t)((uint16_t)info & 0xFFu);
    d[5] = (uint8_t)(((uint16_t)info >> 8) & 0xFFu);
    d[6] = (uint8_t)(ms & 0xFFu);
    d[7] = (uint8_t)((ms >> 8) & 0xFFu);

    if(CAN_Bus_SendStd(CAN_ID_TX(CAN_FN_EVT_FAULT), d, 8u) != 0) return 0;

    n_events++;
    return 1;
}

void Fault_SendBitmap(uint16_t id)
{
    uint8_t d[8];

    d[0] = (uint8_t)(active & 0xFFu);
    d[1] = (uint8_t)((active >> 8) & 0xFFu);
    d[2] = (uint8_t)(latched & 0xFFu);
    d[3] = (uint8_t)((latched >> 8) & 0xFFu);
    d[4] = (uint8_t)(n_events & 0xFFu);
    d[5] = (uint8_t)((n_events >> 8) & 0xFFu);
    d[6] = (uint8_t)((n_events >> 16) & 0xFFu);
    d[7] = (uint8_t)((n_events >> 24) & 0xFFu);

    (void)CAN_Bus_SendStd(id, d, 8u);
}

void Fault_Init(void)
{
    active = latched = prev_active = 0u;
    pend_set = pend_clr = 0u;
    n_events = 0u;
    rep_on = 1u;
    rep_period = FAULT_REPORT_DEF_100MS;
    next_period_us = Micros() + 1000000u;
    next_evt_us = Micros();

    was_enabled = (uint8_t)DRV8316C_IsEnabled();
    flash_fail_seen = NvCfg_WriteFail();
    sens_seen = (uint8_t)((Enc_ErrCnt() || Enc_CrcErrCnt()) ? 1u : 0u);
    reject_until_us = 0u;
    reject_cmd = 0;
    align_latch = 0u;
}

void Fault_ReportReject(uint16_t cmd_fn, uint16_t reason_bit)
{
    reject_until_us = Micros() + FAULT_REJECT_HOLD_US;
    reject_cmd      = (int16_t)cmd_fn;

    if(reason_bit & CAN_FAULT_NOT_ALIGNED) align_latch = 1u;
}

void Fault_Task(void)
{
    uint32_t now = Micros();
    uint16_t want = 0u;
    uint16_t i;

    /* ---------------- 1) 采集 ---------------- */
    if(Motion_IsStalled()) want |= (uint16_t)(CAN_FAULT_STALL | CAN_FAULT_OVERCURRENT);

    if((uint32_t)(now - reject_until_us) < 0x80000000u && reject_until_us != 0u)
        want |= CAN_FAULT_CMD_REJECT;

    if(DRV8316C_Faulted()) want |= CAN_FAULT_DRV_NFAULT;

    if(Enc_ErrCnt() || Enc_CrcErrCnt()) sens_seen = 1u;
    if(sens_seen) want |= CAN_FAULT_SENSOR;

    if(NvCfg_WriteFail() != flash_fail_seen)
    {
        flash_fail_seen = NvCfg_WriteFail();
        want |= CAN_FAULT_FLASH_FAIL;
    }

    if(motor.aligned) align_latch = 0u;
    if(align_latch)   want |= CAN_FAULT_NOT_ALIGNED;

    /* 使能 -> 失能 的下降沿 */
    if(was_enabled && !motor.enabled) want |= CAN_FAULT_DISABLED;
    was_enabled = (uint8_t)(motor.enabled ? 1u : 0u);

    latched |= want;

    /* ---------------- 2) 变化 -> 事件帧 ---------------- */
    if(want != active)
    {
        uint16_t risen  = (uint16_t)(want & ~active);
        uint16_t fallen = (uint16_t)(active & ~want);
        int16_t  amp    = (int16_t)(Motion_PeakAmp() * 100.0f);

        /* 主动上报总闸关着 (默认被动: 不问就不发) 时只维护 active/latched,
         * 一帧不发, 也不往 pend 里攒 —— 免得下次 K1 一开就把上电以来的历史
         * 故障一股脑倒到总线上。被动设备照样记账: 被问就答走 Fault_SendBitmap。 */
        if(CAN_Bus_DebugOn() && rep_on)
        {
            for(i = 0; i < 16u; i++)
            {
                uint16_t b = (uint16_t)(1u << i);

                if(risen & b)
                {
                    if(send_event(b, 0u, (b == CAN_FAULT_CMD_REJECT) ? FAULT_INFO_CMDID : FAULT_INFO_AMP,
                                  (b == CAN_FAULT_CMD_REJECT) ? reject_cmd : amp))
                    { /* sent */ }
                    else pend_set |= b;
                }
                if(fallen & b)
                {
                    if(send_event(b, 1u, FAULT_INFO_NONE, 0)) { /* sent */ }
                    else pend_clr |= b;
                }
            }
        }
        else
        {
            pend_set = pend_clr = 0u;
        }

        active = want;
    }

    /* 被限速挡下的补发 */
    if(pend_set || pend_clr)
    {
        uint16_t b;

        for(i = 0; i < 16u; i++)
        {
            b = (uint16_t)(1u << i);
            if(pend_set & b)
            {
                if(send_event(b, 0u, FAULT_INFO_AMP, (int16_t)(Motion_PeakAmp() * 100.0f))) pend_set &= (uint16_t)~b;
            }
            if(pend_clr & b)
            {
                if(send_event(b, 1u, FAULT_INFO_NONE, 0)) pend_clr &= (uint16_t)~b;
            }
        }
    }

    /* ---------------- 3) 周期重报 ---------------- */
    if((int32_t)(now - next_period_us) >= 0)
    {
        next_period_us = now + (uint32_t)rep_period * 100000u;
        /* 只报"真有故障"的设备: 健康的板子不占总线 (与旧工程同口径) */
        if(rep_on && CAN_Bus_DebugOn() && active)
            Fault_SendBitmap(CAN_ID_TX(CAN_FN_FB_FAULT));
    }

    prev_active = active;
}

uint16_t Fault_Active(void)  { return active; }
uint16_t Fault_Latched(void) { return latched; }
uint32_t Fault_EventCount(void) { return n_events; }

void Fault_Clear(uint16_t bits)
{
    latched &= (uint16_t)~bits;
}

void Fault_SetReport(uint8_t on, uint8_t period_100ms)
{
    rep_on     = on ? 1u : 0u;
    rep_period = period_100ms ? period_100ms : 1u;
}

void Fault_GetReport(uint8_t *on, uint8_t *period_100ms)
{
    if(on) *on = rep_on;
    if(period_100ms) *period_100ms = rep_period;
}

void Fault_Print(void)
{
    printf("FLT: active=0x%04X latched=0x%04X events=%lu report=%u per=%u(x100ms)\r\n",
           (unsigned)active, (unsigned)latched, (unsigned long)n_events,
           (unsigned)rep_on, (unsigned)rep_period);

    if(active)
    {
        uint16_t i;
        printf("FLT: ");
        for(i = 0; i < 16u; i++)
        {
            uint16_t b = (uint16_t)(1u << i);
            if(active & b) printf("%s ", fault_name(b));
        }
        printf("\r\n");
    }
}
