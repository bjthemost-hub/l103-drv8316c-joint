#ifndef __POSSTORE_H
#define __POSSTORE_H

#include "ch32l103.h"

/* =============================================================================
 * 位置 / 机械零点的掉电保存 (2026-10-23)
 * -----------------------------------------------------------------------------
 * 需求:
 *   1) 机械 0 点要存 Flash, 掉电再上电还在;
 *   2) 运动停止且**完全不动** 3 秒以上, 把当前位置写一次 Flash;
 *   3) 不允许在 FOC/中断里写 Flash (擦写会让 CPU 停几毫秒, 电流环会丢拍);
 *   4) 有一个"业务执行过程中"的变量, 期间一律不写 Flash。
 *
 * 传感器前提 (MT6701 手册 v1.9, 2026-10-23 查):
 *   - 单圈绝对 0~360°, SSI 14 位, 1 LSB = 360/16384 = 0.022°;
 *   - INL 典型 ±1.0°, 最大 ±1.5° (手册 5. 电气性能参数) —— 所以"同一个零点"
 *     复现到 1° 级是芯片本身的极限, 不是软件能压掉的;
 *   - 多圈靠软件 enc_rev 记, 上电从 0 起 -> 这里把 rev 一起存, 本产品最多 720°,
 *     只要**断电期间轴转动 < 180°**, 恢复出来的位置就是唯一的。
 *
 * 记录 (每槽 64B, 每页 4 槽, 两页轮换):
 *   w0 magic 'VP21'   w1 seq (越大越新)   w2 mech x1e4 rad   w3 zero x1e4 rad
 *   w4 flags (bit0 = 设过机械零位)        w5 crc (FNV-1a over w0..w4)
 *   w6..w15 = 0
 *
 * 页: 0x0800FD00 / 0x0800FE00 —— 都在链接脚本给 pos_log 留的最后 1KB 里
 *   (0xFC00 nvcfg / 0xFD00 + 0xFE00 posstore / 0xFF00 空闲)。
 *   两页轮换: A 写满 -> 擦 B 再写, 擦的时候 A 还留着最新的记录。
 *
 * 擦写次数: 一页 4 槽, 两页 8 槽 -> 每 8 次保存擦一次页。
 * ========================================================================== */

void     PosStore_Init(void);          /* Enc_Init() 之后调: 读记录并恢复 zero / rev */
void     PosStore_Tick(uint32_t now_us);/* 主循环每拍调一次 (Micros()) */
void     PosStore_RequestSave(void);   /* 零位/坐标被显式改过 -> 尽快写 (仍要求不忙、不动) */

int      PosStore_Usable(void);
int      PosStore_Restored(void);    /* 1 = 本次上电的位置/零位来自 Flash */
int      PosStore_HomeSet(void);       /* 上电恢复出来的"设过机械零位"标志 */
float    PosStore_SavedMech(void);     /* 记录里的机械坐标 rad */
uint32_t PosStore_Seq(void);
uint32_t PosStore_WriteOk(void);
uint32_t PosStore_WriteFail(void);
void     PosStore_PrintInfo(void);

#endif