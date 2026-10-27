#include "drv8316c.h"
#include "board.h"
#include "debug.h"

/* =============================================================================
 * DRV8316C 硬件层 (CH32L103) —— 从 drv8313.c 移植
 *   PWM   : TIM3 CH1/CH2/CH3 -> PA6 / PA7 / PB0, 中心对齐 20kHz
 *   EN    : PB5 / PB4 / PB3 (EN_A / EN_B / EN_C)
 *   nSLEEP: PB15 推挽输出, 高 = 工作 (内部 150/200/300k 下拉 -> 复位期间天然睡眠)
 *   nFAULT: PA8 输入上拉 (R46 5.1k -> +3V3)
 *
 * 相名与定时器通道的对应 (网表实据 2026-10-08, 旧板 A=CH1/B=CH2/C=CH3, 本板 A/C 对调):
 *   A 相: PB0 = TIM3_CH3   U6.27 INHA -> OUTA;  U6.40 SOA -> MOT_A_V -> PA1 = ADC_Channel_1
 *   B 相: PA7 = TIM3_CH2   U6.29 INHB -> OUTB;  U6.39 SOB -> MOT_B_V -> PA2 = ADC_Channel_2
 *   C 相: PA6 = TIM3_CH1   U6.31 INHC -> OUTC;  U6.38 SOC -> MOT_C_V -> PA3 = ADC_Channel_3
 * 这张表写错的话: PWM 会与采样的相错位 -> 力矩方向错/FOC 抖, 且电流会读错相。
 * ========================================================================== */

void DRV8316C_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB |
                          RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM3, ENABLE);

    /* ---- PWM 输出: PA6(TIM3_CH1=C) / PA7(TIM3_CH2=B) / PB0(TIM3_CH3=A), 复用推挽 ---- */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &gpio);

    /* ---- EN_A/B/C: 推挽输出, 先拉低 (高阻) ---- */
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);

    /* ---- nSLEEP (PB15): 输出, 先低 = 睡眠 ---- */
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin  = GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);

    /* ---- nFAULT (PA8): 输入上拉 ---- */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin  = GPIO_Pin_8;
    GPIO_Init(GPIOA, &gpio);

    /* ---- TIM3: 中心对齐, 20kHz (96MHz / (2*2400)) ---- */
    TIM_DeInit(TIM3);
    tb.TIM_Prescaler         = 0;
    tb.TIM_Period            = (uint16_t)PWM_ARR();
    tb.TIM_CounterMode       = TIM_CounterMode_CenterAligned1;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &tb);

    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = 0;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM3, &oc);
    TIM_OC2Init(TIM3, &oc);
    TIM_OC3Init(TIM3, &oc);

    /* CH4 不出脚 (对应引脚没配复用), 只借它的比较匹配当"峰值"标记。
     * CCR4 = ARR -> 中心对齐下每个载波周期只在顶部匹配一次, CC4IF 一拍只置一次。
     * 为什么要它: 原来靠读 CNT (> ARR/2 判峰值) 分峰/谷, 一旦某拍中断被上一拍
     * 拖晚 (1kHz 速度环那拍实测 44us > 半个周期 25us), CNT 已经落到下半区,
     * 峰值被判成谷值 —— 2026-10-18 上板量到 5.26% 的周期因此丢了编码器采样和
     * ADC 触发, 谷值处理多跑 11% (开环转速看着偏高 10% 就是它)。 */
    TIM_OC4Init(TIM3, &oc);
    TIM_SetCompare4(TIM3, (uint32_t)PWM_ARR());

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_Cmd(TIM3, ENABLE);

    /* ---- 唤醒 8316C: nSLEEP 拉高 + 等 tWAKE ----
     * tWAKE = 1ms (nSLEEP=1 -> outputs ready 且 nFAULT 释放; 手册 SLVSH07 L598-599)。
     * 必须在 Delay_Init() 之后调 DRV8316C_Init()。 */
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    /* 2026-10-13 改成 2ms: 手册 SLVSH07 电气表 tWAKE = 1ms(typ, 无 max 列) 是
     * "nSLEEP=1 到 outputs ready 且 nFAULT released" 的时间, 而且手册 §8.4.1
     * 明确写 "nFAULT 在 nSLEEP 上电期间会被拉低, 时长不超过 tWAKE" —— 也就是说
     * 刚唤醒时读到 nFAULT=1 是**正常现象**, 不是故障。原来等 1ms 正好压在 typ 值上,
     * 实测 banner 那行时不时读到 nFAULT=1 (运行起来再读就是 0), 就是压线造成的。
     * 2ms = 2 倍 typ 余量 (设计取值, 未实测最小需要多久)。 */
    Delay_Ms(2);
}

void DRV8316C_Enable(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
}

void DRV8316C_Disable(void)
{
    /* 注意: 清 CCR 只是让 INHx=0(低边); 真正把桥变成高阻的是 INLx=0 */
    TIM_SetCompare1(TIM3, 0);
    TIM_SetCompare2(TIM3, 0);
    TIM_SetCompare3(TIM3, 0);
    GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
}

int DRV8316C_IsEnabled(void)
{
    return (int)((GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3) &&
                  GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4) &&
                  GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5)) ? 1 : 0);
}

static float last_duty[3];

void DRV8316C_SetDuty(float ta, float tb, float tc)
{
    uint32_t arr = (uint32_t)PWM_ARR();

    if(ta < 0.0f) ta = 0.0f; else if(ta > 1.0f) ta = 1.0f;
    if(tb < 0.0f) tb = 0.0f; else if(tb > 1.0f) tb = 1.0f;
    if(tc < 0.0f) tc = 0.0f; else if(tc > 1.0f) tc = 1.0f;

    last_duty[0] = ta; last_duty[1] = tb; last_duty[2] = tc;

    /* A -> CH3(PB0) / B -> CH2(PA7) / C -> CH1(PA6); 见文件头相名对照表 */
    TIM_SetCompare3(TIM3, (uint32_t)(ta * (float)arr));
    TIM_SetCompare2(TIM3, (uint32_t)(tb * (float)arr));
    TIM_SetCompare1(TIM3, (uint32_t)(tc * (float)arr));
}

void DRV8316C_GetDuty(float *ta, float *tb, float *tc)
{
    *ta = last_duty[0]; *tb = last_duty[1]; *tc = last_duty[2];
}

uint8_t DRV8316C_Faulted(void)
{
    return (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) ? 1 : 0;
}

uint8_t DRV8316C_nSLEEP(void)
{
    return (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15) != 0) ? 1u : 0u;
}

void DRV8316C_PwmIT_Enable(void)
{
    NVIC_InitTypeDef nvic = {0};

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update | TIM_IT_CC4);
    TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_CC4, ENABLE);

    /* 抢占优先级给最高: 电流采样必须落在载波顶那一小段窗口里, 不能被别的 ISR 挤晚。
     * (TIM1 的 3kHz 驱动中断是 1, CAN 是 3; NVIC 分组见 main.c) */
    nvic.NVIC_IRQChannel                   = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

void DRV8316C_PwmIT_Disable(void)
{
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update | TIM_IT_CC4);
    TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_CC4, DISABLE);
}

/* ---- TIM4: 32 位自由运行计数器, 1MHz, 供 FOC 计算 Ts ---- */
void Micros_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;

    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM4, ENABLE);

    TIM_DeInit(TIM4);
    tb.TIM_Prescaler         = (uint16_t)(SystemCoreClock / 1000000u - 1u);
    tb.TIM_Period            = 0xFFFFFFFFu;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &tb);

    TIM_Cmd(TIM4, ENABLE);
}

uint32_t Micros(void)
{
    return TIM4->CNT_TIM4;
}
