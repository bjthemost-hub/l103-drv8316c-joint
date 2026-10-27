#ifndef __CMD_H
#define __CMD_H

#include "ch32l103.h"

/* 命令口: UART1 (PA9/PA10, 115200/460800 见 DEBUG_BAUD_DEFAULT)
 * 这是一个**全新**的命令集, 与旧固件 (CH32L103_DRV8316C_FOC_CAN) 不兼容 ——
 * 家人 2026-10-18 定的: 旧命令格式可以整个丢掉。
 * 完整命令表见 docs\命令表.md。 */

void Cmd_Init(void);
void Cmd_Poll(void);
void Cmd_Exec(const char *line);
int  Cmd_IsAligned(void);          /* = motor.aligned (can_bus.c 也用它) */
int  Cmd_ParseFloat(const char **pp, float *out);
int  Cmd_ParseInt(const char **pp, int *out);

#endif