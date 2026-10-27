#ifndef __CAN_BUS_H
#define __CAN_BUS_H

#include "ch32l103.h"

/* =============================================================================
 * ★ 2026-10-18 起这份跑在 **CH32L103_DRV8316C_FOC_V2** 上 (协议与旧工程逐字一致)
 * -----------------------------------------------------------------------------
 * 从 CH32L103_DRV8316C_FOC_CAN 搬过来的, 功能号/帧格式/位定义**一个都没改**,
 * 换的只是下层 (旧 foc_motor/foc_sensor/motion_pv -> V2 的 motor/enc/motion)。
 * 跟旧工程**行为上**的差别, 主机侧要知道的三条:
 *   1) **没有位置掉电恢复**: 状态帧的 POS_RESTORED(0x20) 恒 0;
 *      PV 的 b6 bit4 "存位置" 收下但什么也不做;
 *      SETZERO 的机械零位只在**本次上电内**有效 (掉电后重设)。
 *      -> 主机要绝对位置时, 自己用 QUERY 读回来当基准, 不要假设掉电后还在原位。
 *   2) "堵转阈值" 在 V2 里就是 **Iq 电流上限 motor.i_limit** (0.01A 刻度照旧),
 *      超限判据仍是 MOTOR_OC_TRIP_A 的硬跳闸 -> 状态位 STALL + 故障 STALL/OVERCURRENT。
 *   3) 母线电压这一版是**实测**(CS_VM_Volt), 配置帧的 VM 可用位恒 1。
 * ========================================================================== */

/* =============================================================================
 * CAN1 应用层 (收发器 U4 = MAX3051EKA+T, PD0=RX / PD1=TX, 1Mbps)
 * -----------------------------------------------------------------------------
 * ★ 2026-10-16 起: **ID 带节点号** —— 一条总线上可以挂多台 (最多 63 台)
 *
 *     ID = (节点号 << 5) | 功能号          (标准帧 11bit, 数据小端)
 *
 *       节点号 0     = **广播**: 在场每一台都执行
 *       节点号 1..63 = 设备地址 (每块板一个, 存 Flash)
 *       功能号 0..15 = 主机 -> 板 ;  16..31 = 板 -> 主机
 *
 *   编解码: CAN_ID_MAKE(node, fn) / CAN_ID_NODE(id) / CAN_ID_FUNC(id);
 *   本机发应答一律用 CAN_ID_TX(fn) (= 自己的节点号 + 功能号)。
 *   收帧只看 (id >> 5): 等于本机节点号、或 0(广播) 才执行, 其余直接丢。
 *
 *   为什么是 6 位节点 + 5 位功能: 标准帧 ID 只有 11 位 (0x000~0x7FF),
 *   64 x 32 = 2048 正好用满。要再留余量就只能上扩展帧 (29 位), 本板用不上。
 *
 *   多台同时应答不会撞车: 应答帧带的是**自己的**节点号, 主机分得清是谁答的。
 *   点名(总线扫描): 广播发 功能号 QUERY -> 在场每一台各回自己的一帧。
 *
 *   ⚠ 节点号怎么设: 新板默认 CAN_NODE_DEFAULT (board.h, 现在是 1)。
 *      串口 '%<n>' (例 '%3'), 或 CAN 功能号 SETNODE (b0=节点号 1..63, b1=0x5A 防呆)。
 *      两种都**立刻存 Flash**; 串口只敲一个 '%' = 看当前值。
 *      **首次编号时总线上最好只有这一台** —— 两块默认号相同的板子同时在场会被一起改。
 *
 *   ---- 主机 -> 板 (功能号 0..15) ----
 *   实际 ID = CAN_ID_MAKE(节点号, 功能号); 下面用"功能号 + 宏名"记。
 *
 *   功能号 0 (CAN_FN_PV) : 位置-速度命令 (PV)
 *           b0..3  int32  目标位置, 单位 mrad (0.001 rad), 可正可负, 可多圈
 *           b4..5  int16  目标速度上限, 单位 0.01 rad/s
 *           b6     flags  bit0 1=相对位置(相对收到命令那一刻) 0=绝对位置
 *                         bit1 1=运行 0=停止
 *                         bit2 1=停止时释放(松轴, 断 EN) 0=停止时保持在原位
 *                         bit3 1=清除堵转故障(顺便重新使能)
 *                         bit4 1=立刻把当前位置存进 Flash (V2: 空操作, 不存位置)
 *                         bit5 1=目标位置用机械坐标(需先设过机械零位, 见 功能号 4 SETZERO)
 *                              bit0 和 bit5 同时置位时按 bit0 (相对) 处理
 *           b7     保留 (0)
 *
 *   功能号 1 (CAN_FN_VEL) : 纯速度命令
 *           b0..1  int16  速度, 单位 0.01 rad/s
 *           b2     flags  同 PV 的 b6 (bit0 / bit5 忽略)
 *           b3..7  保留 (0)
 *
 *   功能号 2 (CAN_FN_ENABLE) : 使能 / 失能
 *           b0     1=使能(合上 EN, 进入角度环保持当前位置) 0=失能(松轴, 输出高阻)
 *                  * 使能要求已对齐; 否则拒绝并回 故障事件(NOT_ALIGNED)
 *           b1..7  保留 (0)
 *           回复: 状态帧 (功能号 17)
 *
 *   功能号 3 (CAN_FN_HOME) : 回机械原点 (走到 功能号 4 SETZERO 设定的那个零点)
 *           b0..1  int16  速度上限, 单位 0.01 rad/s (0 -> 默认 30rpm)
 *           b2..7  保留 (0)
 *           回复: 状态帧   没设过零位 -> 拒绝, 回 故障事件(CMD_REJECT)
 *
 *   功能号 4 (CAN_FN_SETZERO) : 把当前位置设为机械 0 点 (V2: 只在本次上电内有效)
 *           b0..7  保留 (0)
 *           回复: 状态帧 + 配置帧
 *
 *   功能号 5 (CAN_FN_QUERY) : 询问
 *           b0     0=位置+速度(回状态帧)   1=配置(回配置帧)
 *                  2=故障(回故障帧)        3=电机参数(回电机参数帧)
 *                  0xFF=以上全回 (状态 + 配置 + 故障 + 电机参数)
 *           回复: 按 b0
 *
 *   功能号 6 (CAN_FN_FAULTCFG) : 故障主动上报配置
 *           b0     0=关 1=开 (默认开)
 *           b1     周期重报间隔, 单位 100ms (0 = 只在变化时报; 默认 10 = 1s)
 *           回复: 配置帧
 *
 *   功能号 7 (CAN_FN_VLIM) : 设角度环的速度上限 (PID_Angle 输出的钳位)
 *           b0..1  int16  速度上限, 单位 0.01 rad/s (<=0 忽略)
 *           回复: 状态帧
 *
 *   功能号 8 (CAN_FN_MOTORCFG) : 设**电机参数** (极对数 / 堵转阈值 / 母线电压), 立即存 Flash
 *                      (2026-09-28 新增: 换电机/换母线电源不用再改代码重编)
 *           b0     uint8  极对数 (1..64); **0 = 这一条不改**
 *           b1..2  int16  堵转/过流阈值, 单位 0.01 A; **0 = 这一条不改**
 *           b3     flags  bit0 **必须为 1** —— 这是"确认要改"的防呆位,
 *                         因为这些参数会一直留在 Flash 里; 给 0 直接拒
 *           b4..5  int16  母线电压, 单位 0.01 V; **0 = 这一条不改** (dlc < 6 时当没给)
 *           b6..7  保留 (0)
 *           几条一起给时: 先一起校验, 只要有一条越界就整条不作数 (不会改一半),
 *           通过后**只写一次 Flash**。
 *           拒绝的条件: 参数越界 / b3 bit0=0 / 要改极对数时驱动还在使能中。
 *           ★ **改极对数会把"已对齐"作废** —— zero_electric_angle 是用旧极对数
 *             算出来的 (foc_motor.c 的 AlignSensor)。所以改完必须重跑对齐 'A',
 *             否则电角度整圈都错 (没劲/抖/发热/不走)。
 *           回复: 电机参数帧 + 配置帧
 *
 *   功能号 9  (CAN_FN_SEG)    : 动作脚本: 追加一段 (2026-09-29)
 *   功能号 11 (CAN_FN_SEG_HR) : 动作脚本: 追加一段 · **大角度版** (2026-10-06)
 *           两条布局**逐字节相同**, 只有角度那个字段的刻度不同:
 *             b0..1  uint16 开始前等待 [100us]     b2..3  int16 速度 [rpm, 符号=方向]
 *             b4..5  int16  角度 (必须正数)         b6..7  uint16 跑完停留 [10ms]
 *             CAN_FN_SEG    : 角度 [0.1 度] -> 单段上限 3276.7 度 (细, 适合短段)
 *             CAN_FN_SEG_HR : 角度 [1   度] -> 单段上限 32767 度 (粗, 适合长段 >3276.7 度)
 *           例: 100rpm 正转 90 度, 停 0.5s, 不等 -> 00 00 64 00 84 03 32 00
 *           上限: rpm 1432 / 角度 +-100 圈 / 等待 6.5535s / 停留 655.35s
 *           回复: 状态帧
 *
 *   功能号 10 (CAN_FN_SEGCTL) : 动作脚本: 控制
 *           b0 0=清空队列  1=开始  2=打印队列;   b1 = 遍数 (0 或 1 = 一遍)
 *           回复: 状态帧
 *
 *   功能号 12 (CAN_FN_SETNODE) : 设节点号 (2026-10-16 新增)
 *           b0     uint8  节点号 (1..63)
 *           b1     uint8  0x5A —— 防呆位, 不对就拒 (免得一条杂帧改了板子身份)
 *           立刻存 Flash; 答复用**改完之后**的节点号发状态帧, 主机据此确认改到了谁。
 *           ⚠ 用广播(节点号 0)发这条 = 在场每一台都改成同一个号, 别这么用。
 *             首次编号请用串口 '%<n>'。
 *
 *   功能号 13 (CAN_FN_ALIGN) : 电角度对齐 (2026-10-18 新增, 原来只有串口 'A')
 *           b0     uint8  0x5A —— 防呆位, 不对就拒
 *                          (对齐会硬拽转子, 不接受裸帧误触发)
 *           非阻塞: 只启动 motor.c 的对齐状态机, 固件在后台跑完 (ALIGN_TIME_MS_DEF)。
 *           回复: 立刻回一帧状态帧 —— 注意那一刻 aligned 通常还是 0,
 *                 主机要等 QUERY 读到 aligned=1 才算对齐完成。
 *           b1..7  保留 (0)
 *
 *   功能号 14 (CAN_FN_INFO) : 读板型 / 固件版本 (2026-10-18 新增)
 *           无载荷 (dlc 0 也可以), 一律回一帧 功能号 22。
 *           另外 QUERY (功能号 5) 的 b0=4 也能单独要这一帧。
 *
 *   功能号 15 (CAN_FN_PARAM) : 全量参数表读写 (2026-10-23; 与 CH32V303_5010 板同一套布局)
 *           b0 = op
 *             0x00 INFO     : 无载荷                        -> 1 帧, index=0xFF, value=参数个数(16)
 *             0x01 READ     : b1=起始索引 b2=个数(0 当 1)    -> 每项 1 帧
 *             0x02 WRITE    : b1=索引 b2..5=float32 b6=0x5A -> 1 帧(改完的回读值)
 *             0x03 SAVE     : b6=0x5A                        -> 1 帧, index=0xFD, 1=成功
 *             0x04 DEFAULTS : b6=0x5A (擦掉 Flash 记录, RAM 不动) -> 1 帧, index=0xFC, 1=成功
 *           WRITE 只改 RAM、立即生效; 要留住必须再发 SAVE。
 *           使能中拒绝 SAVE/DEFAULTS (Flash 擦写会打断电流环); 极对数在使能中拒绝改。
 *           索引: 0 极对数 / 1 电流上限A / 2 母线V / 3 速度环kp / 4 速度环ki / 5 位置环kp /
 *                 6 电流环kp / 7 电流环ki / 8 速度上限 / 9 加速度 / 10 对齐电压V /
 *                 11 PLL带宽Hz / 12 测速源 / 13 相电阻Ω / 14 电感mH / 15 节点号(只读)
 *           节点号请走功能号 12 SETNODE。
 *
 *   ---- 板 -> 主机 (功能号 16..31) ----
 *   ★ ID 里的节点号是**发送方自己的** (CAN_ID_TX(fn)), 所以多台同时答不会撞车。
 *
 *   功能号 16 (CAN_FN_FB) : 周期反馈, 每 10ms 一帧
 *           b0..3  int32  当前机械坐标位置, 单位 mrad (多圈)
 *                         (= 原始累计位置 - 机械零位; 没设过零位时二者相同)
 *           b4..5  int16  当前速度, 单位 0.01 rad/s
 *           b6     status 见 CAN_ST_*
 *           b7     int8   相电流, 单位 0.1 A (取三相重构后 |i| 最大的那相, 带符号)
 *
 *   功能号 17 (CAN_FN_FB_QUERY) : 即时状态帧 (= QUERY 的应答, 也是使能/回原点/设零点的 ACK)
 *           布局与周期反馈**完全一样**
 *
 *   功能号 18 (CAN_FN_FB_CFG) : 配置帧
 *           b0..3  int32  机械零位在原始坐标系里的位置, 单位 mrad
 *           b4..5  uint16 母线电压, 单位 0.1 V (本版硬件没有 VM 采样 -> 配置值)
 *           b6     flags  bit0 已设机械零位  bit1 已对齐   bit2 有 VM 采样
 *                         bit3 驱动已使能      bit4 故障主动上报开
 *           b7     uint8  上报周期, 单位 100ms
 *
 *   功能号 19 (CAN_FN_FB_FAULT) : 故障帧
 *           b0..1  uint16 当前活动故障位 (CAN_FAULT_*, 定义见 faults.h)
 *           b2..3  uint16 锁存故障位 (出现过就保持 1, 直到被清)
 *           b4..7  uint32 累计故障事件帧数
 *
 *   功能号 20 (CAN_FN_FB_MOTORCFG) : **电机参数帧** (2026-09-28 新增)
 *           b0     uint8  当前极对数
 *           b1..2  int16  当前堵转/过流阈值, 单位 0.01 A
 *           b3     flags  bit0 参数来自 Flash 记录 (不是编译默认值)
 *                         bit1 已对齐        bit2 驱动已使能
 *           b4..5  int16  当前母线电压, 单位 0.01 V
 *           b6     uint8  脚本: 这一遍还剩几段
 *           b7     uint8  脚本: 正在跑第几遍 (0 = 没跑)
 *
 *   功能号 21 (CAN_FN_EVT_FAULT) : 故障事件 (主动上报)
 *           b0..1  uint16 故障位
 *           b2     0=发生  1=消失
 *           b3     b4..5 是什么: 0 无 / 1 电流(0.01A) / 2 电压(0.1V) / 3 **被拒命令的功能号**
 *           b4..5  int16  附加数值
 *           b6..7  uint16 事件时刻, 单位 ms (上电起, 会回绕)
 *
 *   功能号 22 (CAN_FN_FB_INFO) : 板型 / 固件版本帧 (功能号 14 的应答, 2026-10-18 新增)
 *           b0     uint8  板型号: 1 = L103 + DRV8316C  (以后每加一款板往后加号, 老号不许改)
 *           b1     uint8  固件主版本
 *           b2     uint8  固件次版本
 *           b3     uint8  固件修订号
 *           b4     uint8  CAN 应用层协议版本 (变 = 功能号表/帧布局不兼容)
 *           b5..7  保留 (0)
 *
 *   功能号 23..31 保留 (板 -> 主机)
 *
 *   ---- 主动上报总闸 (2026-09-29 新增) ----
 *   默认**被动**: 上电后周期反馈、故障事件、周期故障位图 **一帧都不主动发**,
 *   只有"被问才答" (询问 -> 状态/配置/故障/电机参数, 以及各条命令的 ACK 帧)。
 *   要开周期上报: 串口发 `K1` (关: `K0`)。**不存 Flash**, 掉电即回到被动。
 *   主机想要数据就自己用 询问 轮询 —— 一块 CAN 上挂十几台时, 本来就该这样。
 *   周期反馈只在 debug 打开时发 (2026-09-29 起), 布局见功能号 16。
 *
 *   ---- 多节点动作脚本 (2026-10-16 新增, 用法) ----
 *   每块板各有一份队列 (MOTION_SEQ_MAX_SEG = 16 段), 段是**相对自己这条时间轴**的
 *   (等 wait -> 走 deg -> 停 dwell), 时间轴起点 = 收到"开始"那一刻。
 *   所以多节点同步的做法是:
 *     1) 主机按节点把动作切片, 每段发到**该节点的** 功能号 SEG / SEG_HR;
 *     2) 每台的 wait 由主机按"这张共享时间表"算好 (该段起始时刻 - 本节点上一段结束时刻);
 *     3) 最后发**一帧广播**的 功能号 SEGCTL (b0=1) —— 所有板在同一个 CAN 帧上起跑,
 *        时间轴起点对齐到同一个时刻, 各关节就同步了。
 *   ⚠ 这是"共享时间表 + 各自相对等待", 不是硬时间同步: 某一段比标称耗时长,
 *     那一台后面的段整体顺延 (各台之间会有毫秒~几十毫秒级漂移)。
 * ========================================================================== */

/* ---- ID 组装 (2026-10-16 起): ID = (节点号 << 5) | 功能号 ---- */
#define CAN_ID_MAKE(node, fn) ((uint16_t)((((uint16_t)(node) & 0x3Fu) << 5) | \
                                          ((uint16_t)(fn) & 0x1Fu)))
#define CAN_ID_NODE(id)       ((uint8_t)(((uint16_t)(id) >> 5) & 0x3Fu))
#define CAN_ID_FUNC(id)       ((uint8_t)((uint16_t)(id) & 0x1Fu))

/* 本机应答用的 ID: 节点号 = 自己的 (CAN_Bus_Node, 定义在本文件尾部) */
#define CAN_ID_TX(fn)         CAN_ID_MAKE(CAN_Bus_Node(), (fn))

/* ---- 功能号: 主机 -> 板 (0..15) ---- */
#define CAN_FN_PV            0u         /* 位置-速度 */
#define CAN_FN_VEL           1u         /* 纯速度 */
#define CAN_FN_ENABLE        2u         /* 使能 / 失能 */
#define CAN_FN_HOME          3u         /* 回机械原点 */
#define CAN_FN_SETZERO       4u         /* 把当前位置设为机械 0 点 */
#define CAN_FN_QUERY         5u         /* 询问 (位置速度 / 配置 / 故障 / 电机参数) */
#define CAN_FN_FAULTCFG      6u         /* 故障主动上报开关 + 周期 */
#define CAN_FN_VLIM          7u         /* 设角度环速度上限 (rad/s) */
#define CAN_FN_MOTORCFG      8u         /* 设电机参数 (极对数/堵转阈值/母线电压) 并存 Flash */
#define CAN_FN_SEG           9u         /* 动作脚本: 追加一段 (2026-09-29) */
#define CAN_FN_SEGCTL       10u         /* 动作脚本: 控制 (b0 0=清空 1=开始 2=打印, b1=遍数) */
#define CAN_FN_SEG_HR       11u         /* 动作脚本: 追加一段·大角度版 (布局同 SEG, 角度 int16 x 1 度) */
#define CAN_FN_SETNODE      12u         /* 设节点号 (2026-10-16 新增) */
#define CAN_FN_ALIGN        13u         /* 电角度对齐 (2026-10-18 新增; b0 必须 0x5A) */
#define CAN_FN_INFO         14u         /* 读板型/固件版本 (2026-10-18 新增; 无载荷) */
#define CAN_FN_PARAM        15u         /* 全量参数表读写 (2026-10-23; 应答走 23) */

/* ---- 功能号: 板 -> 主机 (16..31) ---- */
#define CAN_FN_FB           16u         /* 10ms 周期反馈 */
#define CAN_FN_FB_QUERY     17u         /* 即时状态帧 (ACK / 询问应答) */
#define CAN_FN_FB_CFG       18u         /* 配置帧 */
#define CAN_FN_FB_FAULT     19u         /* 故障帧 (位图) */
#define CAN_FN_FB_MOTORCFG  20u         /* 电机参数帧; b6/b7 = 队列剩余段数/遍数 */
#define CAN_FN_EVT_FAULT    21u         /* 故障事件 (主动上报) */
#define CAN_FN_FB_INFO      22u         /* 板型/固件版本帧 (功能号 14 的应答) */
#define CAN_FN_FB_PARAM     23u         /* 参数表应答帧 (功能号 15 的应答; b7 = 0xA5 签名) */

/* 反馈状态位 */
#define CAN_ST_RUN           0x01u      /* 位置/速度环正在跑 */
#define CAN_ST_ARRIVED       0x02u      /* 已到目标位置 */
#define CAN_ST_STALL         0x04u      /* 堵转过流锁存, PWM 已停 */
#define CAN_ST_SENSOR_ERR    0x08u      /* MT6701 读失败 (丢帧/CRC 不过) */
#define CAN_ST_ALIGNED       0x10u      /* 有可用对齐参数 (方向/零点电角) */
#define CAN_ST_POS_RESTORED  0x20u      /* 当前位置来自 Flash 恢复 */
#define CAN_ST_ENABLED       0x40u      /* DRV8316C EN 已合上 */
#define CAN_ST_ZERO_SET      0x80u      /* 已设过机械零位 */

#define CAN_FB_PERIOD_US     10000u     /* 10ms */

void     CAN_Bus_Init(void);
int      CAN_Bus_Ready(void);
uint8_t  CAN_Bus_Node(void);                 /* 本机节点号 1..63 (真值在 motion.c, 存 Flash 见 nvcfg) */
void     CAN_Bus_Task(void);                 /* 主循环调用: 收命令 + 10ms 反馈 */
int      CAN_Bus_SendStd(uint16_t id, const uint8_t *data, uint8_t dlc);
void     CAN_Bus_SetFeedback(uint8_t on);
uint8_t  CAN_Bus_FeedbackOn(void);
void     CAN_Bus_SetDebug(uint8_t on);       /* 主动上报总闸: 0=被动(默认) 1=debug */
uint8_t  CAN_Bus_DebugOn(void);
void     CAN_Bus_PrintStatus(void);          /* 'K' */
void     CAN_Bus_SendTest(void);             /* 'J' */
int      CAN_Bus_XcvLoopTest(void);          /* 'Y': 收发器回路自测 (临时把 PD1 当 GPIO) */
uint32_t CAN_Bus_RxCount(void);
uint32_t CAN_Bus_TxCount(void);
uint32_t CAN_Bus_TxFail(void);
uint32_t CAN_Bus_Overrun(void);
uint32_t CAN_Bus_LastId(void);
void     CAN_Bus_PinsLow(int on);            /* 'v0'/'v1': PD0/PD1 当 GPIO 拉低 / 还原 (查收发器) */
void     CAN_Bus_ResetCounters(void);
int      CAN_Bus_LastRxDlc(void);

#endif
