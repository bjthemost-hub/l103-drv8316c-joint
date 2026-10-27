#ifndef __MOTION_H
#define __MOTION_H

#include "ch32l103.h"

/* =============================================================================
 * CAN 命令的"动作层": 把协议要的语义 (PV / 恒速 / 使能 / 回零 / 板载脚本)
 * 翻译成 V2 的 motor.c 那套 (20k 电流环 + 1k 速度环 + 200Hz 位置环)。
 * -----------------------------------------------------------------------------
 * 这是**新写的一层**, 不是把旧工程 motion_pv.c 搬过来:
 *   旧的: 自己算梯形/三角参考轨迹, 再喂给 Type_angle 角度环
 *   新的: 位置环就是 V2 的 `pos_kp * (pos_cmd - pos_m)`, 目标直接下给 motor.pos_cmd,
 *         上限由 `vel_limit` 钳 (PV 的 vlim 就落在这一层)
 * 为什么这么换: V2 的位置环本来就是给"低速大扭矩"调的 (2026-10-18 台架: 10rpm
 *   零位移 0.5%), 再叠一套参考轨迹只会多一层要调的东西。旧轨迹那套参数
 *   (MOTION_PV_DECEL 300 等) 是为旧电压模式 FOC 定的, 搬过来反而要重调。
 *
 * 坐标系 (两套, 分清楚):
 *   raw  = 本次上电以来累计的机械角 = Enc_MechRad() + home
 *          (CAN 协议里的"绝对位置"就是这一套 —— 旧工程叫 shaft_angle)
 *   mech = raw - home = Enc_MechRad()  (设过机械零位之后就是"机械坐标")
 *   home 存在 enc 的 zero 里 (Enc_SetMechZero), 所以 mech 读数是免费的,
 *   而且跟串口 'Z' 是同一套零位, 不会出现"两处各记一个零"。
 *
 * ★ 位置/机械零位**掉电保存**在 posstore.c (2026-10-23):
 *   -> 停稳 3 s 写一次 Flash, "业务执行过程中"不写, 也不在 FOC 中断里写;
 *   -> 上电按"存的机械坐标 + 零点 + 单圈绝对角"反推圈数恢复;
 *   -> CAN 状态帧 POS_RESTORED 位 = 本次确实从 Flash 恢复了。
 *   节点号/极对数/电流上限/母线电压这些**配置**是存 Flash 的 (见 nvcfg.h)。
 * ========================================================================== */

#define MOTION_SEQ_MAX_SEG   16

void     Motion_Init(void);
void     Motion_Task(void);            /* 主循环每拍调一次 (板载脚本状态机) */

/* ---- 运动 ---- */
int      Motion_RunPV(float target_raw_rad, float vlim_rad_s);   /* 0 = 接受, -1 = 目标太远被拒 */
void     Motion_RunVEL(float vel_rad_s);
void     Motion_Stop(uint8_t release); /* release=1 松轴, 0 = 停下保持原位 */
void     Motion_SaveNow(void);
int      Motion_SaveCfg(void);     /* 把当前 RAM 里的全部可调参数写进 Flash (1=成功) */         /* 协议里 PV b6 bit4 "存位置" —— V2 没有位置存储, 空实现 */

/* ---- 使能 / 机械原点 ---- */
int      Motion_SetEnabled(uint8_t on);   /* 1 = 使能并保持当前位置; -1 = 没对齐, 拒 */
int      Motion_IsEnabled(void);
void     Motion_SetHomeHere(void);        /* 当前位置设成机械 0 点 (存 Flash, 掉电保留) */
void     Motion_ClearHome(void);          /* 取消机械零位: 机械坐标 = 原始坐标 */
void     Motion_SetHomeFlag(int on);      /* 上电把 Flash 里恢复的零位标志灌回来 */
int      Motion_HomeSet(void);
float    Motion_HomeRad(void);            /* 零位在 raw 坐标里的位置 rad */
float    Motion_PosMechRad(void);         /* 机械坐标 */
int      Motion_GotoHome(float vlim_rad_s); /* 回机械原点; -1 = 没设过零位 */

/* ---- 状态 ---- */
int      Motion_IsRunning(void);
int      Motion_Busy(void);           /* 1 = "业务执行过程中" (脚本/走位/恒速/开环/对齐) */
int      Motion_Arrived(void);
int      Motion_IsStalled(void);
void     Motion_ClearFault(void);
uint8_t  Motion_StatusBits(void);         /* CAN_ST_* 位 (定义在 can_bus.h) */
float    Motion_PosRad(void);             /* raw 坐标 */
float    Motion_VelRad(void);             /* 机械角速度 rad/s */
float    Motion_PeakAmp(void);            /* 本拍三相电流绝对值最大 */
void     Motion_PrintStatus(void);

/* ---- 配置 (存 Flash) ---- */
int      Motion_PolePairs(void);
float    Motion_StallAmp(void);           /* = 电流上限 i_limit (见 docs 里的映射说明) */
float    Motion_Vbus(void);
int      Motion_CfgFromFlash(void);
int      Motion_SetMotorCfg(int pp, float amp, float vbus);  /* 0 = 成功, -1 = 越界/使能中改极对数 */

int      Motion_Node(void);
int      Motion_NodeFromFlash(void);
int      Motion_SetNode(int n);           /* 1..63; 0 = 成功 */
void     Motion_PrintNode(void);

/* ---- 板载动作脚本 (CAN SEG / SEG_HR / SEGCTL) ---- */
int      Motion_Seq_AddSeg(long wait_us, int rpm, float deg, long dwell_ms);
int      Motion_Seq_Start(int loops);
void     Motion_Seq_Clear(void);
void     Motion_Seq_Abort(void);
void     Motion_Seq_Print(void);
int      Motion_Seq_Running(void);
int      Motion_Seq_Remain(void);
int      Motion_Seq_Pass(void);

#endif
