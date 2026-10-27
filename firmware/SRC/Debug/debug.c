/********************************** (C) COPYRIGHT  *******************************
 * File Name          : debug.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/07/08
 * Description        : This file contains all the functions prototypes for UART
 *                      Printf , Delay functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "debug.h"

static uint16_t  p_us = 0;
static uint16_t p_ms = 0;
/*********************************************************************
 * @fn      Delay_Init
 *
 * @brief   Initializes Delay Funcation.
 *
 * @return  none
 */
void Delay_Init(void)
{
    p_us = SystemCoreClock / 8000;
    p_ms = (uint16_t)p_us * 1;
}

/*********************************************************************
 * @fn      Delay_Us
 *
 * @brief   Microsecond Delay Time.
 *
 * @param   n - Microsecond number.
 *
 * @return  None
 */
void Delay_Us(uint32_t n)
{
    uint32_t i;

    SysTick->SR &= ~(1 << 0);
    i = (uint32_t)(n * p_us)/1000;

    SysTick->CMP = i;
    SysTick->CTLR |= (1 << 4);
    SysTick->CTLR |= (1 << 5) | (1 << 0);

    while((SysTick->SR & (1 << 0)) != (1 << 0));
    SysTick->CTLR &= ~(1 << 0);
}

/*********************************************************************
 * @fn      Delay_Ms
 *
 * @brief   Millisecond Delay Time.
 *
 * @param   n - Millisecond number.
 *
 * @return  None
 */
void Delay_Ms(uint32_t n)
{
    uint32_t i;

    SysTick->SR &= ~(1 << 0);
    i = (uint32_t)n * p_ms;

    SysTick->CMP = i;
    SysTick->CTLR |= (1 << 4);
    SysTick->CTLR |= (1 << 5) | (1 << 0);

    while((SysTick->SR & (1 << 0)) != (1 << 0));
    SysTick->CTLR &= ~(1 << 0);
}

/*********************************************************************
 * @fn      USART_Printf_Init
 *
 * @brief   Initializes the USARTx peripheral.
 *
 * @param   baudrate - USART communication baud rate.
 *
 * @return  None
 */
void USART_Printf_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

#if(DEBUG == DEBUG_UART1)
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_USART1 | RCC_PB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

#elif(DEBUG == DEBUG_UART2)
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_USART2, ENABLE);
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

#elif(DEBUG == DEBUG_UART3)
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_USART3, ENABLE);
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

#endif

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;

#if(DEBUG == DEBUG_UART1)
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

#elif(DEBUG == DEBUG_UART2)
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);

#elif(DEBUG == DEBUG_UART3)
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);

#endif
}

/*********************************************************************
 * @fn      _write
 *
 * @brief   Support Printf Function
 *
 * @param   *buf - UART send Data.
 *          size - Data length
 *
 * @return  size: Data length
 */
/* =============================================================================
 * UART1 非阻塞发送 —— 环形缓冲 + DMA 搬字节 (2026-10-14 改)
 * -----------------------------------------------------------------------------
 *   _write / UsartTx_PutBytes 只往环形缓冲塞, 立刻返回; 真正把字节写进 DATAR 的是
 *   **DMA1 通道 4** —— CH32L103 应用手册 V2.2 §11.2.3 "DMA 请求映射":
 *       USART1_TX -> DMA1 通道 4        USART1_RX -> DMA1 通道 5
 *   通道号是硬件固定映射 (手册那张表: "Fixed hardware priority Channel No."),
 *   没有"请求选择"寄存器可配, 所以 USART1 发送只能挂通道 4。
 *
 *   为什么换 DMA (2026-10-14 家人要求 "串口输出日志用 DMA 送出来"):
 *   上一版是每发一个字节进一次 TXE 中断 —— 115200 下 87us 一次, 460800 下 21.7us
 *   一次, 每次都要保存/恢复现场; 波特率一提, 这份开销按比例涨。改 DMA 后 CPU 只在
 *   **整段发完**时进一次中断去接下一段。
 *
 *   生产者只有一个 (主循环里的 printf / 波形流), 消费者只有一个 (DMA 完成中断):
 *   tx_head 只由主循环推进, tx_tail 只由中断推进 —— 单生产者单消费者, 不需要临界区。
 *   中断频率: 只有尾指针撞到缓冲区末尾才切段, 一段最多 TX_RING_SZ 字节 ->
 *   460800 下最坏 ~44ms 一次, 115200 下最坏 ~178ms 一次 (按波特率算的, 未实测)。
 * ========================================================================== */
#if(DEBUG == DEBUG_UART1)
#define TX_RING_SZ   2048u                  /* 必须是 2 的幂 */
#define TX_RING_MSK  (TX_RING_SZ - 1u)

static volatile uint8_t  tx_ring[TX_RING_SZ];
static volatile uint16_t tx_head  = 0;      /* 生产者写位置 (主循环推进) */
static volatile uint16_t tx_tail  = 0;      /* DMA 正在读的位置 (中断推进) */
static volatile uint16_t tx_chunk = 0;      /* 本段交给 DMA 的字节数 (0 = 没有段在途) */
static volatile uint32_t tx_drop  = 0;      /* 缓冲满, 丢掉的字节数 */
static volatile uint32_t tx_seg   = 0;      /* 一共起过多少段 (诊断用) */
static volatile uint32_t tx_isr   = 0;      /* 进过多少次 DMA 完成中断 (诊断用) */
static volatile uint32_t tx_prem  = 0;      /* 通道还没搬完就进来的次数 (诊断用) */
static volatile uint32_t tx_spur  = 0;      /* 空转时来的中断 (诊断用, 正常应为 0) */
static volatile uint32_t tx_sent  = 0;      /* 交给 DMA 的字节总数 (诊断用) */

/* 起下一段。主循环 (塞完数据) 和 DMA 完成中断 (接下一段) 都会调。
 *
 * 2026-10-14 修 (整段日志凭空消失的根因): 写 CNTR/MADDR 必须先停通道
 * (手册要求通道非使能时才能改), 而且要先把上一段的完成标志清掉再改 CNTR。
 * 原来顺序是 tx_chunk=n; MADDR=..; CNTR=n; ENABLE; —— 上一次的 TC 若还挂着,
 * 新段一使能就可能立刻被判成已发完, 中断里按 tx_chunk 推进尾指针,
 * 于是尾指针跳过了整段从未发出去的字节: 症状就是整段日志消失、txdrop 却为 0。 */
static void tx_kick(void)
{
    uint16_t n, run;

    if(tx_chunk != 0u) return;                          /* 已经有一段在途 */

    n = (uint16_t)((tx_head - tx_tail) & TX_RING_MSK);  /* 攒了多少字节 */
    if(n == 0u) return;

    run = (uint16_t)(TX_RING_SZ - tx_tail);             /* 到尾为止能连着发多少 */
    if(n > run) n = run;

    /* DMAT 位可能被后来的 USART_Init() 清掉 (Cmd_Init / 改波特率都会重配 CTLR3),
     * 起段前补一次, 这样与调用顺序无关。 */
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    DMA_Cmd(DMA1_Channel4, DISABLE);        /* 改 CNTR/MADDR 前必须停通道 */
    DMA_ClearITPendingBit(DMA1_IT_TC4);     /* 清掉上一段的完成标志 */
    DMA1_Channel4->MADDR = (uint32_t)&tx_ring[tx_tail];
    DMA1_Channel4->CNTR  = n;
    tx_chunk = n;                           /* 记长度, 再使能 */
    DMA_Cmd(DMA1_Channel4, ENABLE);
    tx_seg++;
}

void UsartTx_Init(void)
{
    DMA_InitTypeDef  dma;
    NVIC_InitTypeDef nvic;

    tx_head  = 0;
    tx_tail  = 0;
    tx_chunk = 0;
    tx_drop  = 0;
    tx_seg   = 0;
    tx_isr   = 0;
    tx_prem  = 0;
    tx_spur  = 0;
    tx_sent  = 0;

    /* 关掉旧的 TXE 中断路子: DMA 接管后它只会来抢字节 (把缓冲里的字节提前写进 DATAR) */
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

    /* DMA1 时钟 + 通道 4: 外设 = USART1->DATAR (地址固定), 存储器 = tx_ring (递增),
     * 每次 1 字节; **普通模式** —— 一段发完通道自己停, 由 TC 中断接下一段。 */
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel4);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DATAR;
    dma.DMA_MemoryBaseAddr     = (uint32_t)tx_ring;
    dma.DMA_DIR                = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize         = 0u;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Normal;
    dma.DMA_Priority           = DMA_Priority_Medium;
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &dma);

    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    /* 这里就把 NVIC 打开, 否则 Cmd_Init 之前打的字只会堆在缓冲里发不出去。
     * 开两个通道: USART1 (收) 和 DMA1_CH4 (发完)。 */
    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;      /* 比电机中断(1)低 */
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel                   = DMA1_Channel4_IRQn;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&nvic);
}

/* DMA 一段发完 (中断里调): 推进尾指针, 再把下一段接上。
 *
 * 2026-10-14 修: 不再无条件信任 tx_chunk, 改用 CNTR 反算真正搬出去多少字节:
 *     done = tx_chunk - CNTR(剩余)
 * 这样即便中断提前进来 (或来个空转中断), 也只推进确实搬走的那部分,
 * 剩下的字节下一段接着发, 不会整段丢掉。tx_prem 专门记"提前进来"的次数。 */
void UsartTx_DmaIsr(void)
{
    uint16_t left, done;

    if(DMA_GetITStatus(DMA1_IT_TC4) == RESET) return;

    tx_isr++;

    if(tx_chunk == 0u){                     /* 空转时来的中断: 清掉走人 */
        tx_spur++;
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        return;
    }

    DMA_Cmd(DMA1_Channel4, DISABLE);
    left = (uint16_t)DMA1_Channel4->CNTR;   /* 还剩几个没搬 */
    if(left != 0u) tx_prem++;               /* 通道没搬完就进来了 (诊断) */
    DMA_ClearITPendingBit(DMA1_IT_TC4);

    done     = (uint16_t)(tx_chunk - left); /* 真正搬进去的字节数 */
    tx_sent += done;
    tx_tail  = (uint16_t)((tx_tail + done) & TX_RING_MSK);
    tx_chunk = 0u;
    tx_kick();
}

uint32_t UsartTx_Drop(void) { return tx_drop; }
uint32_t UsartTx_Segs(void) { return tx_seg;  }
uint32_t UsartTx_Isr (void) { return tx_isr;  }
uint32_t UsartTx_Prem(void) { return tx_prem; }
uint32_t UsartTx_Spur(void) { return tx_spur; }
uint32_t UsartTx_Sent(void) { return tx_sent; }

/* 还有东西没发完 (缓冲里有货, 或有一段正在 DMA) */
int UsartTx_Busy(void) { return (tx_chunk != 0u) || (tx_head != tx_tail) ? 1 : 0; }

/* 供二进制波形流用: 一次塞 n 个字节, 满了就丢并计数, **绝不死等**。
 * 和 _write 走同一个环形缓冲 / 同一个 DMA, 所以主循环里调用它不会阻塞 FOC。
 * 返回真正塞进去的字节数 (可能 < n, 差额进了 tx_drop)。 */
int UsartTx_PutBytes(const uint8_t *buf, int n)
{
    int i, put = 0;

    if(buf == 0 || n <= 0) return 0;

    for(i = 0; i < n; i++){
        uint16_t next = (uint16_t)((tx_head + 1u) & TX_RING_MSK);

        if(next == tx_tail) { tx_drop++; continue; }   /* 满了丢字节, 绝不死等 */
        tx_ring[tx_head] = buf[i];
        tx_head = next;
        put++;
    }

    tx_kick();
    return put;
}

/* DMA 一段发完的中断入口 (启动文件里是 .weak 死循环桩, 这里给它真身) */
void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void DMA1_Channel4_IRQHandler(void)
{
    UsartTx_DmaIsr();
}
#endif

__attribute__((used))
int _write(int fd, char *buf, int size)
{
    int i;

    for(i = 0; i < size; i++){
#if(DEBUG == DEBUG_UART1)
        uint16_t next = (uint16_t)((tx_head + 1u) & TX_RING_MSK);

        if(next == tx_tail) { tx_drop++; continue; }   /* 满了丢字符, 绝不死等 */
        tx_ring[tx_head] = (uint8_t)buf[i];
        tx_head = next;
#elif(DEBUG == DEBUG_UART2)
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        USART_SendData(USART2, *buf++);
#elif(DEBUG == DEBUG_UART3)
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
        USART_SendData(USART3, *buf++);
#endif
    }

#if(DEBUG == DEBUG_UART1)
    /* 塞完叫一次 DMA 就够了 —— 循环里叫没用 (有一段在途时 tx_kick 直接返回) */
    if(size > 0) tx_kick();
#endif

    return size;
}

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
__attribute__((used))
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
    return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}

/*********************************************************************
 * Minimal newlib syscall stubs.
 * The link uses no nosys.specs (self-contained): _write/_sbrk above back
 * printf + heap; the rest are no-ops (mirrors libgloss/nosys).
 */
struct stat;  /* incomplete - stubs do not inspect it */

__attribute__((used))
int _read(int fd, char *buf, int len)
{
    (void)fd; (void)buf; (void)len;
    return 0;                 /* no stdin -> EOF */
}

__attribute__((used))
int _close(int fd)
{
    (void)fd;
    return -1;
}

__attribute__((used))
int _fstat(int fd, struct stat *st)
{
    (void)fd; (void)st;
    return 0;
}

__attribute__((used))
int _isatty(int fd)
{
    (void)fd;
    return 0;
}

__attribute__((used))
int _lseek(int fd, int ptr, int dir)
{
    (void)fd; (void)ptr; (void)dir;
    return 0;
}
