#include "csense.h"
#include "board.h"
#include <stdint.h>

/* idx 0/1/2 = A/B/C = PA1/PA2/PA3 (网表核对 2026-10-08) */
static const uint8_t cs_ch[CS_NCH] = { ADC_Channel_1, ADC_Channel_2, ADC_Channel_3 };

static float   cs_zero_v[CS_NCH];
static int32_t cs_zero_cnt[CS_NCH];   /* 零点换成 ADC 计数, 给定点电流环用 */
static uint8_t cs_zero_ok;
static float   cs_sign[CS_NCH] = { CS_SIGN_A, CS_SIGN_B, CS_SIGN_C };
static int8_t  cs_sign_i[CS_NCH] = { 1, 1, 1 };   /* cs_sign 的整型镜像: 20kHz 里不做浮点比较 */

static volatile uint16_t cs_raw[CS_NCH];
static volatile uint8_t  cs_valid;
static volatile uint32_t cs_cnt;
static volatile uint32_t cs_miss;
static volatile uint16_t cs_miss_run;
static volatile uint8_t  cs_waiting;

static uint16_t cs_st_jdr[CS_NCH];
static uint8_t  cs_st_jeoc;
static uint8_t  cs_st_ok;

/* 采样时间: 相电流源阻抗 330R + 22p, 7.5 周期够; 母线源阻抗约 4.6k, 必须用 28.5 周期
 * (数据手册 V2.1 表3-30: 14MHz 下 28.5 周期允许 R_AIN<=50k, 1.5 周期只允许 1.2k) */
#define CS_SMP_PHASE   ADC_SampleTime_CyclesMode1
#define CS_SMP_VM      ADC_SampleTime_CyclesMode3

void CS_Init(void)
{
    ADC_InitTypeDef  adc  = {0};
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;

    for(i = 0; i < CS_NCH; i++)
    {
        cs_zero_v[i]   = CS_VREF_NOM;
        cs_zero_cnt[i] = (int32_t)(CS_VREF_NOM * CS_FULLSCALE / CS_VDDA + 0.5f);
        cs_raw[i]      = 0u;
        cs_sign_i[i]   = (cs_sign[i] < 0.0f) ? -1 : 1;
    }
    cs_zero_ok = 0u; cs_valid = 0u; cs_cnt = 0u; cs_miss = 0u;
    cs_miss_run = 0u; cs_waiting = 0u;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);            /* ADCCLK = 96/8 = 12MHz */

    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode  = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &gpio);

    ADC_DeInit(ADC1);
    (void)Get_CalibrationValue(ADC1);

    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = ENABLE;         /* ★ 必须开, 否则三路注入只转一路 */
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = 1;
    adc.ADC_OutputBuffer       = ADC_OutputBuffer_Disable;
    adc.ADC_Pga                = ADC_Pga_1;
    ADC_Init(ADC1, &adc);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1) != RESET);
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) != RESET);

    /* 注入组: 三路相电流, rank1=A -> rank2=B -> rank3=C */
    ADC_InjectedSequencerLengthConfig(ADC1, CS_NCH);
    for(i = 0; i < CS_NCH; i++)
    {
        ADC_InjectedChannelConfig(ADC1, cs_ch[i], (uint8_t)(i + 1u), CS_SMP_PHASE);
    }
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None); /* JSWSTART */
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);                           /* JEXTTRIG 一起开 */

    /* 上电自检: 软件起一次注入, 看 JEOC 与三个 JDR */
    {
        uint32_t guard = 200000u;

        ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
        ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET)
        {
            if(--guard == 0u) break;
        }
        cs_st_jeoc = (uint8_t)((guard != 0u) ? 1u : 0u);
        if(cs_st_jeoc)
        {
            cs_st_jdr[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
            cs_st_jdr[1] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
            cs_st_jdr[2] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
            ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
            cs_st_ok = 1u;
            /* 自检这一拍本来就是零点 (DRV 未使能), 直接种进 cs_raw[]: 中断还没跑起来时
             * banner 拿 0 当 raw 会算出一个假的负电流 */
            cs_raw[0] = cs_st_jdr[0];
            cs_raw[1] = cs_st_jdr[1];
            cs_raw[2] = cs_st_jdr[2];
        }
    }
}

/* ---- 峰值 UEV: 起三路注入转换 (全程整型, 不碰浮点) ---- */
void CS_StartSampleIsr(void)
{
    ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
    cs_waiting = 1u;
}

/* ---- 谷值 UEV: 取数。返回 1 = 取到新一帧 ---- */
int CS_LatchIsr(void)
{
    if(!cs_waiting) return 0;
    cs_waiting = 0u;

    if(ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) != RESET)
    {
        cs_raw[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        cs_raw[1] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
        cs_raw[2] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
        ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
        cs_valid = 1u;
        cs_cnt++;
        cs_miss_run = 0u;
        return 1;
    }

    cs_miss++;
    if(cs_miss_run < 0xFFFFu) cs_miss_run++;
    if(cs_miss_run >= CS_SYNC_MISS_RUN_MAX) cs_valid = 0u;
    return 0;
}

void CS_GetIsr(float *ia, float *ib, float *ic)
{
    const float k = CS_VDDA / CS_FULLSCALE / CS_GAIN_V_PER_A;   /* A/LSB */

    /* 用 volatile 快照, 避免三个数被下一拍换掉一半 */
    uint16_t r0 = cs_raw[0], r1 = cs_raw[1], r2 = cs_raw[2];

    if(ia) *ia = ((float)r0 * k - cs_zero_v[0] / CS_GAIN_V_PER_A) * cs_sign[0];
    if(ib) *ib = ((float)r1 * k - cs_zero_v[1] / CS_GAIN_V_PER_A) * cs_sign[1];
    if(ic) *ic = ((float)r2 * k - cs_zero_v[2] / CS_GAIN_V_PER_A) * cs_sign[2];
}

void CS_GetRawIsr(uint16_t out[CS_NCH])
{
    out[0] = cs_raw[0]; out[1] = cs_raw[1]; out[2] = cs_raw[2];
}

int32_t  CS_ZeroRaw(uint8_t i) { return (i < CS_NCH) ? cs_zero_cnt[i] : 2048; }
int32_t  CS_SignI(uint8_t i)   { return (i < CS_NCH) ? (int32_t)cs_sign_i[i] : 1; }
int      CS_ZeroOk(void)       { return (int)cs_zero_ok; }

int      CS_Valid(void)        { return (int)cs_valid; }
uint32_t CS_Count(void)        { return cs_cnt; }
uint32_t CS_Miss(void)         { return cs_miss; }
uint16_t CS_SyncRaw(uint8_t i) { return (i < CS_NCH) ? cs_raw[i] : 0u; }
float    CS_ZeroVolt(uint8_t i){ return (i < CS_NCH) ? cs_zero_v[i] : 0.0f; }
void     CS_SetSign(uint8_t i, float s)
{
    if(i < CS_NCH)
    {
        cs_sign[i]   = (s < 0.0f) ? -1.0f : 1.0f;
        cs_sign_i[i] = (s < 0.0f) ? -1 : 1;
    }
}
float    CS_Sign(uint8_t i)    { return (i < CS_NCH) ? cs_sign[i] : 0.0f; }

int      CS_SelfTestOk(void)   { return (int)cs_st_ok; }
uint16_t CS_SelfTestJeoc(void) { return (uint16_t)cs_st_jeoc; }
uint16_t CS_SelfTestJdr(uint8_t i) { return (i < CS_NCH) ? cs_st_jdr[i] : 0u; }

/* ---- 慢速路径: 单次软件触发 (零点标定 / 诊断) ---- */
uint16_t CS_Raw(uint8_t i)
{
    uint32_t guard = 200000u;

    if(i >= CS_NCH) return 0u;

    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    ADC_RegularChannelConfig(ADC1, cs_ch[i], 1, CS_SMP_PHASE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
        if(--guard == 0u) return 0u;
    }
    return ADC_GetConversionValue(ADC1);
}

float CS_Volt(uint8_t i)
{
    return (float)CS_Raw(i) * CS_VDDA / CS_FULLSCALE;
}

void CS_CalZero(void)
{
    uint32_t i, n = 64u;
    float a0 = 0.0f, a1 = 0.0f, a2 = 0.0f;

    for(i = 0; i < n; i++)
    {
        a0 += CS_Volt(0);
        a1 += CS_Volt(1);
        a2 += CS_Volt(2);
    }
    cs_zero_v[0] = a0 / (float)n;
    cs_zero_v[1] = a1 / (float)n;
    cs_zero_v[2] = a2 / (float)n;
    for(i = 0; i < CS_NCH; i++)
    {
        cs_zero_cnt[i] = (int32_t)(cs_zero_v[i] * CS_FULLSCALE / CS_VDDA + 0.5f);
    }
    cs_zero_ok = 1u;
}

uint16_t CS_VM_Raw(void)
{
#if VM_SENSE_ENABLE
    uint32_t guard = 200000u;

    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    ADC_RegularChannelConfig(ADC1, VM_ADC_CHANNEL, 1, CS_SMP_VM);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
        if(--guard == 0u) return 0u;
    }
    return ADC_GetConversionValue(ADC1);
#else
    return 0u;
#endif
}

float CS_VM_Volt(void)
{
#if VM_SENSE_ENABLE
    return (float)CS_VM_Raw() * CS_VDDA / CS_FULLSCALE * VM_DIVIDER_RATIO;
#else
    return MOTOR_VBUS_DEF;
#endif
}
