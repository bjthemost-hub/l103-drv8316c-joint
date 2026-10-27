/********************************** (C) COPYRIGHT  *******************************
 * File Name          : debug.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/06
 * Description        : This file contains all the functions prototypes for UART
 *                      Printf , Delay functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __DEBUG_H
#define __DEBUG_H

#include "stdio.h"
#include "ch32l103.h"

/* 极简 printf (User/mini_fmt.c): 替掉 newlib 的 printf 家族, 省 ~6 KB Flash + ~1 KB RAM。
 * 本头文件里把 printf 定义成 mini_printf —— 所以下面对 printf 的调用不用改。 */
#include "mini_fmt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* UART Printf Definition */
#define DEBUG_UART1    1
#define DEBUG_UART2    2
#define DEBUG_UART3    3

/* DEBUG UATR Definition */
#ifndef DEBUG
#define DEBUG   DEBUG_UART1
#endif

/* UART1 波特率 —— **上位机必须用同一个值** (脚本里是 -Baud, 默认同此)。
 * 只有这一个宏, cmd.c 的 Cmd_Init 和 main.c 都取它, 不会再出现两处不一致。
 * 460800 在本板 (PCLK2 = 96MHz) 下: SPL 算出 BRR = 13 (整数分频 13 + 小数 0),
 * 实际 = 96e6/(16*13) = 461538 baud -> 误差 +0.16% (按 SPL 公式算的, 未实测眼图)。
 * 想临时换: 'SWB<baud>' (两边同时改), 掉电/复位回到本宏。 */
#ifndef DEBUG_BAUD_DEFAULT
#define DEBUG_BAUD_DEFAULT   115200u
#endif

extern uint32_t OPA_Trim;
extern uint16_t ADC_Trim;
extern uint32_t TS_Val;
extern uint32_t CHIPID;
extern uint16_t USBPD_CFG;

void Delay_Init(void);
void Delay_Us(uint32_t n);
void Delay_Ms(uint32_t n);
void USART_Printf_Init(uint32_t baudrate);

/* ---- UART1 非阻塞发送 (环形缓冲 + DMA1 通道 4 搬字节) ----
 * 原版 _write 是每字符死等 TC: 一次 1Hz 打印要 ~30ms, 主循环整段停住。
 * 第二版: 环形缓冲 + TXE 中断 (每字节一次中断)。
 * 第三版 (2026-10-14): 环形缓冲 + DMA —— CPU 只在**整段发完**时中断一次。
 *   CH32L103 应用手册 V2.2 §11.2.3: USART1_TX = DMA1 通道 4 (固定映射)。
 * 用法: USART_Printf_Init() 之后立刻 UsartTx_Init(); 接收 (RXNE) 仍走 USART1 中断。
 * 顺序无关: Cmd_Init()/改波特率会再调 USART_Init() 并清掉 CTLR3 的 DMAT 位,
 *           tx_kick() 每次起段都会把它补回来。 */
void     UsartTx_Init(void);          /* 清空缓冲 + 配 DMA1_CH4 + 开 NVIC */
void     UsartTx_DmaIsr(void);        /* DMA1_Channel4_IRQHandler 里调 */
uint32_t UsartTx_Drop(void);          /* 缓冲满而丢掉的字节数 (0 = 没丢过) */
uint32_t UsartTx_Segs(void);          /* 起过多少段 DMA (每段 <= 2048 B) */
uint32_t UsartTx_Isr (void);          /* 进过多少次 DMA 完成中断 */
uint32_t UsartTx_Prem(void);          /* 其中通道没搬完就进来的次数 (应为 0) */
uint32_t UsartTx_Spur(void);          /* 空转中断次数 (应为 0) */
uint32_t UsartTx_Sent(void);          /* 交给 DMA 的字节总数 */
int      UsartTx_Busy(void);          /* 还有字节没发完 */
int      UsartTx_PutBytes(const uint8_t *buf, int n);   /* 二进制流用: 非阻塞塞 n 字节 */

#if(DEBUG)
  #define PRINT(format, ...)    printf(format, ##__VA_ARGS__)
#else
  #define PRINT(X...)
#endif

#ifdef __cplusplus
}
#endif

#endif
