#include "ch32l103.h"
#include "board.h"
#include "drv8316c.h"
#include "motor.h"

/* =============================================================================
 * TIM3 中断 —— 整个快速环路的唯一时基
 * -----------------------------------------------------------------------------
 * 中心对齐 + PWM 模式1 下每个载波周期有两次 UEV (峰值 + 谷值), 合计 40kHz。
 * 峰谷**用 CC4 匹配标志判**, 不读 CNT:
 *   CCR4 = ARR, 每个周期只在顶部匹配一次 -> CC4IF 就代表"这一拍是峰值"。
 *   读 CNT 的写法在中断被上一拍拖晚时会判反 (2026-10-18 实测: 5.26% 的周期
 *   峰值落到下半区, 被当成谷值处理, 丢编码器采样+ADC 触发)。CC4IF 与 UEV 同时
 *   置位, 延迟多久都不会认错。
 *
 *   峰值 (CC4): 起三相注入转换 (低边窗口正中) + 读编码器 + PLL
 *   谷值 (UEV): 取电流 -> 电流环 -> (分频) 速度环 / 位置环 -> 写占空比
 * ========================================================================== */

void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void TIM3_IRQHandler(void)
{
    uint16_t fl = TIM3->INTFR;

    if(fl & TIM_FLAG_CC4)
    {
        /* 峰值的 UEV 也在同一时刻置位, 一起清掉, 别再当成谷值跑一遍 */
        TIM3->INTFR = (uint16_t)~(TIM_FLAG_CC4 | TIM_FLAG_Update);
        Motor_PeakIsr();
    }
    else if(fl & TIM_FLAG_Update)
    {
        TIM3->INTFR = (uint16_t)~TIM_FLAG_Update;
        Motor_ValleyIsr();
    }
}