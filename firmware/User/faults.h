#ifndef __FAULTS_H
#define __FAULTS_H

#include "ch32l103.h"

/* =============================================================================
 * 故障位图 + 主动上报 (帧格式照旧工程 faults.h, 未改)
 * -----------------------------------------------------------------------------
 * 帧:
 *   0x1A0 (CAN_FN_EVT_FAULT) 事件帧: 某一位"发生/消失"的瞬间各发一帧
 *         b0..1 故障位  b2 0=发生/1=消失  b3 附加量类型  b4..5 附加量  b6..7 上电起 ms
 *   0x183 (CAN_FN_FB_FAULT) 位图帧: b0..1 活动  b2..3 锁存  b4..7 累计事件数
 *
 * 本固件的采集来源 (跟旧工程一一对应):
 *   STALL / OVERCURRENT <- motor.fault_code == 1 (foc.c 的 foc.oc 跳闸, >MOTOR_OC_TRIP_A)
 *   DRV_NFAULT          <- DRV8316C_Faulted() 实时
 *   SENSOR              <- Enc_ErrCnt()/Enc_CrcErrCnt() 本次上电累计非 0
 *   NOT_ALIGNED         <- 有命令因"没对齐"被拒, 对齐成功后自动撤
 *   CMD_REJECT          <- 命令被拒的 1s 脉冲
 *   FLASH_FAIL          <- NvCfg 写失败计数变了
 *   DISABLED            <- 使能过又被失能 (上电本来就没使能, 不报)
 *   VM_LOW              <- 本版**不实现** (恒 0): 母线欠压阈值得按实际电源定, 没定之前不乱报
 * ========================================================================== */

#define CAN_FAULT_STALL         0x0001u
#define CAN_FAULT_OVERCURRENT   0x0002u
#define CAN_FAULT_DISABLED      0x0004u
#define CAN_FAULT_VM_LOW        0x0008u
#define CAN_FAULT_SENSOR        0x0010u
#define CAN_FAULT_NOT_ALIGNED   0x0020u
#define CAN_FAULT_FLASH_FAIL    0x0040u
#define CAN_FAULT_DRV_NFAULT    0x0080u
#define CAN_FAULT_CMD_REJECT    0x0100u

#define FAULT_INFO_NONE         0u
#define FAULT_INFO_AMP          1u
#define FAULT_INFO_VOLT         2u
#define FAULT_INFO_CMDID        3u

void     Fault_Init(void);
void     Fault_Task(void);
uint16_t Fault_Active(void);
uint16_t Fault_Latched(void);
uint32_t Fault_EventCount(void);
void     Fault_Clear(uint16_t bits);
void     Fault_SetReport(uint8_t on, uint8_t period_100ms);
void     Fault_GetReport(uint8_t *on, uint8_t *period_100ms);
void     Fault_SendBitmap(uint16_t id);
void     Fault_ReportReject(uint16_t cmd_fn, uint16_t reason_bit);
void     Fault_Print(void);

#endif
