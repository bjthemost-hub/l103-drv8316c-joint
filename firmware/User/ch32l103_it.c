#include "ch32l103_it.h"
#include "board.h"

/* 异常处理。TIM3 / USART1 / DMA1_CH4 的中断各自写在 isr.c / cmd.c / debug.c 里
 * (与旧工程一致: 哪里有硬件就在哪里处理, 不集中到这个文件)。 */

void NMI_Handler(void)        __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));

void NMI_Handler(void)
{
    while(1) { }
}

/* 硬故障: PC15 (LED_ERR) 快闪, 不复位 —— 复位会让人误以为"上电没反应" */
void HardFault_Handler(void)
{
    GPIO_InitTypeDef gpio;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_15;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &gpio);

    while(1)
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_15);
        { volatile uint32_t i; for(i = 0; i < 200000u; i++) { } }
        GPIO_ResetBits(GPIOC, GPIO_Pin_15);
        { volatile uint32_t i; for(i = 0; i < 200000u; i++) { } }
    }
}