#include "ch32l103.h"
#include "board.h"          /* 必须在 debug.h 之前: 它定义了 DEBUG_BAUD_DEFAULT */
#include "debug.h"
#include "fmt.h"
#include "drv8316c.h"
#include "csense.h"
#include "enc.h"
#include "motor.h"
#include "foc.h"
#include "cmd.h"
#include "motion.h"
#include "can_bus.h"
#include "faults.h"
#include "posstore.h"

/* =============================================================================
 * CH32L103 + DRV8316C  FOC V2  (按 AT32M416 mclib 的骨架重做)
 * -----------------------------------------------------------------------------
 * 主循环只管三件事: 收命令 / 每 10ms 采一次母线电压 / 看 nFAULT。
 * **控制全部在 TIM3 中断里** (20kHz), 主循环再怎么卡也影响不到环路节拍。
 * ========================================================================== */

/* ---- 状态灯 (口径与旧固件 CH32L103_DRV8316C_FOC_CAN 一致) --------------------
 * 绿 D1 (LED_OK , PC14): 阳极->+3V3, 阴极->R17 860R->PC14  —— 灌电流接法, 拉低才亮
 * 红 D2 (LED_ERR, PC15): 阳极->+3V3, 阴极->R21     ->PC15  —— 灌电流接法, 拉低才亮
 *
 *   绿灯 (LED_OK) —— 心跳 / 出力指示
 *     上电        : 立刻常亮 (电和程序都起来了)
 *     停机 / 空闲 : 1 秒翻转一次 (2 秒一个周期)
 *     驱动使能中  : 常亮不闪 (一眼看出正在出力)
 *
 *   红灯 (LED_ERR) —— 状态指示
 *     有故障 / 堵转 : 快闪 (100 ms 翻转一次)
 *     没使能        : 慢闪 (500 ms 翻转一次)
 *     使能且一切正常: 灭
 *     硬故障(MCU崩): 快闪 (见 ch32l103_it.c 的 HardFault_Handler)
 */
/* 红灯当"故障"看的位: 堵转/过流/驱动 nFAULT/编码器/存 Flash 失败/母线低。
 * DISABLED(正常失能)、NOT_ALIGNED(还没对齐)、CMD_REJECT(命令被拒) 不算故障。 */
#define LED_ERR_FAULT_MASK  (CAN_FAULT_STALL | CAN_FAULT_OVERCURRENT | CAN_FAULT_DRV_NFAULT | \
                             CAN_FAULT_SENSOR | CAN_FAULT_FLASH_FAIL | CAN_FAULT_VM_LOW)

static void led_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &gpio);
    GPIO_ResetBits(GPIOC, GPIO_Pin_14);                 /* 绿灯: 上电即亮 */
    GPIO_SetBits(GPIOC, GPIO_Pin_15);                   /* 红灯: 灭 */
}

static void banner(void)
{
    uint16_t raw = 0;
    uint8_t  crc_ok = 0;
    float ia, ib, ic;

    printf("\r\n=== CH32L103 + DRV8316C FOC V2 (build %s %s) ===\r\n", __DATE__, __TIME__);
    printf("CLK:%lu  PWM:%luHz  Ts:%sus\r\n",
           (unsigned long)SystemCoreClock, (unsigned long)PWM_FREQ_HZ,
           FmtF(PWM_TS_S * 1e6f, 2));
    printf("LOOP: cur=%luHz spd=%luHz pos=%luHz enc=%luHz  pll_bw=%sHz\r\n",
           (unsigned long)(PWM_FREQ_HZ / CUR_LOOP_DIV),
           (unsigned long)(PWM_FREQ_HZ / SPD_LOOP_DIV),
           (unsigned long)(PWM_FREQ_HZ / POS_LOOP_DIV),
           (unsigned long)PWM_FREQ_HZ, FmtF(Enc_PllBandwidth(), 0));

    printf("DRV: nSLEEP=%u nFAULT=%u\r\n", (unsigned)DRV8316C_nSLEEP(), (unsigned)DRV8316C_Faulted());
    (void)Enc_ReadBlocking(&raw, &crc_ok);
    printf("ENC: raw=%u crc=%s\r\n", (unsigned)raw, crc_ok ? "OK" : "MISMATCH");
    printf("CSA: zerov A=%s B=%s C=%s | selftest jeoc=%u ok=%u JDR=%u/%u/%u\r\n",
           FmtF(CS_ZeroVolt(0), 4), FmtF(CS_ZeroVolt(1), 4), FmtF(CS_ZeroVolt(2), 4),
           (unsigned)CS_SelfTestJeoc(), (unsigned)CS_SelfTestOk(),
           (unsigned)CS_SelfTestJdr(0), (unsigned)CS_SelfTestJdr(1), (unsigned)CS_SelfTestJdr(2));
    CS_GetIsr(&ia, &ib, &ic);
    printf("CSA: 当前 ia=%s ib=%s ic=%s A (DRV 未使能时应≈0)\r\n",
           FmtF(ia, 3), FmtF(ib, 3), FmtF(ic, 3));
    printf("VM: %s V (分压比 %s)\r\n", FmtF(CS_VM_Volt(), 2), FmtF(VM_DIVIDER_RATIO, 3));
    {
        float kp, ki;
        Motor_GetCurGains(&kp, &ki);
        printf("GAIN: cur kp=%s V/A ki=%s V/(A*s) | pp=%d\r\n",
               FmtF(kp, 4), FmtF(ki, 1), Motor_PolePairs());
    }
}

int main(void)
{
    uint32_t t_led, t_red, t_vm, t_try = 0;
    uint8_t  led_on = 1u;               /* 1 = 绿灯亮 */
    uint8_t  red_on = 0u;               /* 1 = 红灯亮 */

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(DEBUG_BAUD_DEFAULT);
    UsartTx_Init();
    led_init();

    Micros_Init();

    DRV8316C_Init();       /* TIM3 起振, nSLEEP 拉高, EN 保持低 (桥高阻) */
    CS_Init();             /* ADC 注入组 + 上电自检 */
    CS_CalZero();          /* 此刻 DRV 没使能, 相电流为 0 -> 正是零点 */
    Enc_Init();
    PosStore_Init();       /* 读 Flash 里的机械零点/圈数, 恢复掉电前的位置 */
    Motor_Init();
    Motion_Init();         /* 读 Flash 里的节点号/极对数/电流上限/母线, 覆盖编译默认 */
    Motion_SetHomeFlag(PosStore_HomeSet());
    Cmd_Init();
    CAN_Bus_Init();        /* CAN1: PD0/PD1, 1Mbps */
    Fault_Init();

    DRV8316C_SetDuty(0.5f, 0.5f, 0.5f);   /* 零矢量, 等中断接管 */
    banner();
    printf("CAN: %s node=%d (1..63, 0=广播) | '%' 看/设节点号, 'K' 看 CAN 状态, 'J' 发测试帧\r\n",
           CAN_Bus_Ready() ? "ready" : "NOT ready", Motion_Node());
    printf("SAFE: 驱动未使能, 桥高阻。输入 'e' 使能, '?' 看状态, 'Q' 看诊断\r\n");

    /* 打开 TIM3 更新中断 —— 从这一刻起控制环才开始跑 */
    DRV8316C_PwmIT_Enable();

    t_led = Micros() + 1000000u;
    t_red = Micros();
    t_vm  = Micros();

    while(1)
    {
        uint32_t now;

        Cmd_Poll();
        CAN_Bus_Task();     /* 收命令 + (打开时) 周期反馈 */
        Motion_Task();      /* 板载动作脚本状态机 */
        Fault_Task();       /* 故障位图 + 主动上报 */
        PosStore_Tick(Micros());          /* 停稳 3s 存一次位置 (写 Flash 只在这里发生) */

        /* 原始角诊断流: 'SD1' 打开后主循环把 MT6701 的原始计数直出给上位机 */
        if(Enc_CapEnabled())
        {
            uint16_t cr;
            uint32_t cs;

            if(Enc_CapRead(&cr, &cs)) printf("%X,%X\r\n", (unsigned)(cs & 0xFFFu), (unsigned)cr);
        }

        now = Micros();

        /* 母线电压: 10ms 一次就够 (主循环卡一下也不影响环路) */
        if((int32_t)(now - t_vm) >= 0)
        {
            float vm;
            t_vm = now + 10000u;
            vm = CS_VM_Volt();
            if(vm > 1.0f) Motor_SetVbus(vm);
        }

        /* nFAULT 低 -> 失能, 但不在中断/主循环里死等 (旧固件那个 while(1) 会把 CAN 一起弄哑) */
        if(DRV8316C_Faulted())
        {
            if((int32_t)(now - t_try) >= 0)
            {
                t_try = now + 1000000u;
                Motor_SetFault(2u);        /* 2 = 8316C 的 nFAULT 被拉低 */
                Motor_Disable();
                printf("ERR nFAULT 低 -> 已失能; 查故障后 'X' 清除再 'e'\r\n");
            }
        }

        /* 绿灯: 停机时 1 秒翻转一次; 使能中常亮 (与旧固件一致) */
        if((int32_t)(now - t_led) >= 0)
        {
            t_led = now + 1000000u;
            if(Motion_IsEnabled())
            {
                if(!led_on) { GPIO_ResetBits(GPIOC, GPIO_Pin_14); led_on = 1u; }
            }
            else
            {
                led_on = (uint8_t)!led_on;
                if(led_on) GPIO_ResetBits(GPIOC, GPIO_Pin_14);
                else       GPIO_SetBits(GPIOC, GPIO_Pin_14);
            }
        }

        /* 红灯: 故障/堵转 -> 快闪; 没使能 -> 慢闪; 使能且正常 -> 灭 */
        {
            uint32_t period;

            if(Motion_IsStalled() || (Fault_Active() & LED_ERR_FAULT_MASK) != 0u)
                period = 100000u;      /* 快闪: 100ms 翻一次 */
            else if(!Motion_IsEnabled())
                period = 500000u;      /* 慢闪: 500ms 翻一次 */
            else
                period = 0u;           /* 正常: 灭 */

            if(period == 0u)
            {
                if(red_on) { GPIO_SetBits(GPIOC, GPIO_Pin_15); red_on = 0u; }
                t_red = now;           /* 灭着的时候把计时从头算, 恢复时不会先蹦半拍 */
            }
            else if((int32_t)(now - t_red) >= 0)
            {
                t_red = now + period;
                red_on = (uint8_t)!red_on;
                if(red_on) GPIO_ResetBits(GPIOC, GPIO_Pin_15);
                else       GPIO_SetBits(GPIOC, GPIO_Pin_15);
            }
        }
    }
}