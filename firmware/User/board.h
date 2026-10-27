#ifndef __BOARD_H
#define __BOARD_H

/* =============================================================================
 * CH32L103K8U6 + DRV8316C  ——  FOC V2 (按 AT32M416 mclib 的骨架重做)
 * -----------------------------------------------------------------------------
 * 本工程与同目录下的 CH32L103_DRV8316C_FOC_CAN **是两套固件**, 互不影响:
 *   旧的  : 电压模式 FOC, 闭环跑在主循环 (~1.6ms 一拍, 无电流环) -> 低速抖
 *   本工程: 20kHz 固定节拍电流环 + 1kHz 速度环 + 200Hz 位置环 (见 docs\)
 *
 * 引脚与网表的对应 **一个字都没改** (原理图 2026-10-18 已冻结, 不再动):
 *   PWM_A  = PB0 -> U6.27 INHA  TIM3_CH3
 *   PWM_B  = PA7 -> U6.29 INHB  TIM3_CH2
 *   PWM_C  = PA6 -> U6.31 INHC  TIM3_CH1     <-- 本板 A/C 相对旧板对调过
 *   EN_A/B/C = PB5 / PB4 / PB3 -> U6.28 / 30 / 32 (INLA/B/C)
 *   nSLEEP = PB15, nFAULT = PA8 (R46 5.1k 上拉, 开漏)
 *   MOT_A_V= PA1 <- SOA ; MOT_B_V= PA2 <- SOB ; MOT_C_V= PA3 <- SOC  (8316C 内部 CSA)
 *   ADC0   = PA0 母线分压
 *   ENC_CLK= PB6 -> U5.7 ; ENC_DO= PB8 <- U5.6 ; ENC_CS= PB9 -> U5.8  (MT6701 SSI)
 *   CAN1   = PD0(RX) / PD1(TX) ; UART1 = PA9(TX) / PA10(RX)
 * 时钟: HSI 96MHz (system_ch32l103.c SetSysClockTo96_HSI, 未改)
 * ========================================================================== */

#include "ch32l103.h"

/* ---- PWM: TIM3 中心对齐 ---- */
#define PWM_FREQ_HZ             20000u
#define PWM_ARR()               (SystemCoreClock / 2u / PWM_FREQ_HZ - 1u)
#define PWM_TS_S                (1.0f / (float)PWM_FREQ_HZ)   /* 50us */

/* ---- 控制环分频 (都从 PWM 谷值 UEV 起算, 20kHz 基准) ----
 * 电流环  : 每 1 拍        -> 20 kHz
 * 速度环  : 每 20 拍       -> 1  kHz
 * 位置环  : 每 100 拍      -> 200 Hz
 * 编码器  : 每 4 拍        -> 5  kHz  (SSI 一帧要打 25 个时钟, 20kHz 里塞不下)
 * 三个分频必须能整除, 否则相位会漂 —— 编译期 static assert 兜住 (见 motor_ctrl.c)。 */
#define CUR_LOOP_DIV            1u
#define SPD_LOOP_DIV            20u
#define POS_LOOP_DIV            100u
#define ENC_READ_DIV            4u

/* ---- 三相电流采样: DRV8316C 内置低边 sense-FET + 内部 CSA ----
 * 通道 idx 0/1/2 = A/B/C = PA1/PA2/PA3 = ADC_Channel_1/2/3
 * GAIN 脚悬空 = 0.3 V/A (R49/R50 都不贴, 依据 board.h(旧) 四电平脚表 + 手册 SLVSH07)
 * VLINEAR = 0.25..3.05V, 中点 VREF/2 = 1.65V -> 满量程 ±4.667A (算术值, 未实测)
 * 零点不用 1.65V: 上电 DRV 未使能时实测 (cs_zero_cal) */
#define CS_NCH                 3
#define CS_VDDA                3.3f
#define CS_FULLSCALE           4095.0f
#define CS_GAIN_V_PER_A        0.3f
#define CS_VREF_NOM            1.65f
#define CS_AMP_FS              4.667f

/* 占空比超过这个值, 那一相的低边窗口就容不下三路注入转换 -> 该相读数不可信,
 * 用 ia+ib+ic=0 补回来。推导见旧工程 csense.c 注释 (算术值, 未实测)。 */
/* 推导 (算术, 未实测): 峰值处低边窗口 = (1-d)*Ts/2 = (1-d)*25us;
 * 三路注入 3*(7.5+12.5) 周期 / 12MHz = 5.0us -> 要求 (1-d)*25 > 5 -> d < 0.80。
 * 取 0.79 留一点余量。旧工程写的 0.842 只对应 3.95us, 不够三路转完。 */
#define CS_DUTY_RECON_TH       0.79f

/* 三相电流极性的符号 (0/1/2 = A/B/C)。板级现场确认前先都填 +1;
 * 若发现力矩方向与 Iq 符号相反 / ia+ib+ic 明显不为 0, 先用命令 'CS<idx>,<+1|-1>' 试,
 * 试对了再回来改这里。手册 Figure 8-30 的横轴读法有歧义 (见旧工程 csense.h:20)。 */
#define CS_SIGN_A              (+1.0f)
#define CS_SIGN_B              (+1.0f)
#define CS_SIGN_C              (+1.0f)

/* ---- 母线电压 ---- */
#define VM_SENSE_ENABLE        1
#define VM_ADC_CHANNEL         ADC_Channel_0
/* 分压比: 2026-10-13 家人定案按板上实物 49k9/4k7 = 11.617
 * (旧工程 board.h 有完整推导: 实测分压节点 1.687V -> 19.60V, 与电源 19.4V 差 1%) */
#define VM_DIVIDER_RATIO       11.617f

/* ---- 电机 / 编码器 ---- */
#define MOTOR_POLE_PAIRS       11          /* MY-4621F, 24N22P = 22 极 = 11 极对
                                            * (依据见旧工程 board.h, 2026-10-14 两次实测
                                            *  estimated=11)。换电机用命令 'PP<n>' 改。 */
#define ENC_CPR                16384       /* MT6701 14 位 */
#define ENC_DIR                (+1.0f)     /* 编码器计数方向 vs 电角度正方向 */

/* ---- 控制默认值 (全部是**设计值/起点值**, 不是实测值) ----
 * 电流环: 按 R=6.7Ω / L=1mH 的假设取 200Hz 带宽, 上电就能稳;
 *         R 来自旧工程 board.h 的观察值 (3V/6.7Ω≈0.45A), L 未测。
 *         命令 'RL<ohm>,<mH>' 可以按实际参数重算, 'KP'/'KI' 可以手改。
 * 速度环: 1kHz 采样, 带宽 ~30Hz 起点。
 * 位置环: 200Hz 采样, P=20 1/s 起点。 */
#define CUR_BW_HZ_DEF          200.0f
#define CUR_R_OHM_DEF          6.7f        /* 2026-10-23 实测: 目前这颗电机 6.7Ω.
                                           * (4.2 是按对齐 3.0V/0.71A 反算的, 不是万用表量出来的,
                                           *  与厂家线电阻 10.32Ω 折合相电阻 5.2Ω 也对不上, 作废) */
#define CUR_L_H_DEF            0.00175f    /* 2026-10-23 实测: 电感 1.75 mH */
/* 速度环默认增益直接给数值, 不再由带宽公式推。
 * 速度反馈已改成纯 PLL (enc.c 的 vel_src 默认 1), 20ms 窗口只留作诊断;
 * 下面是 2026-10-18 用 MT6701 原始计数逐组打分挑出来的 (10/30/100 rpm 三档都验):
 *   kp=0.05 A/(rad/s)  ki=4.0 A/(rad/s*s)  PLL 带宽 150Hz
 *   10rpm  9.99rpm 零位移0.5% 回退0 100ms峰峰43.6%
 *   30rpm 29.98rpm 零位移0%   回退0 100ms峰峰 8.7%
 *   100rpm 99.94rpm 零位移0%  回退0 100ms峰峰 3.5%
 * 现场仍可用 'S<kp>,<ki>' 改。 */
#define SPD_KP_DEF             0.05f
#define SPD_KI_DEF             4.00f
#define SPD_BW_HZ_DEF          30.0f      /* 仅 speed_pi_tune() 用, 现不作为默认 */
#define POS_KP_DEF             20.0f

/* Iq/Id 给定上限 A。家人 2026-10-19 定: 这颗电机的**堵转电流要给到 1.8A**
 *   (原 1.0A 那一版, 台架电流表还没到 1A 就顶住失能了)。
 * 峰值 1.9A 仍是电机标称: 1.8A 是"顶得住"的余量, 秒级满载仍建议留在 1A 附近。 */
#define CUR_LIMIT_A_DEF        1.8f
#define VEL_LIMIT_DEF          330.0f      /* rad/s 机械角速度上限; 3000rpm = 314 rad/s */
#define POS_ERR_LIMIT_RAD      6.2832f     /* 位置环误差先限一圈, 防大偏差冲击 */

/* ---- 到位静止锁定 (2026-10-23) ----
 * 现象 (2026-10-23 实测): 到位静止时编码器原始计数只在相邻 2 个码之间跳 (pp=1 码 = 0.022 度),
 *   但 iq 在 0.04~0.15A 峰峰之间来回 —— 速度反馈的量化噪声被速度环放大成电流纹波,
 *   再变回微小转矩, 形成"顶在两个码之间"的自持抖动。手碰一下把转子挪过码边界, 它就停在
 *   另一个平衡点, 所以看起来"碰一下就好了"。
 * 做法: 到位且速度很小时 (持续 50ms), 冻结速度环积分、输出"进入那一刻的积分值"(保持力矩不变),
 *   不再跟着噪声调电流; 位置误差超过 EXIT 或速度变大就立刻恢复闭环。
 *   POS_HOLD_EN=0 可整体关掉, 回到原来的连续闭环。 */
#define POS_HOLD_EN            0
#define POS_HOLD_ENTER_RAD     0.0040f     /* 进入: |位置误差| < 0.004 rad (0.23 度) */
#define POS_HOLD_EXIT_RAD      0.0120f     /* 退出: |位置误差| > 0.012 rad (0.69 度) */
#define POS_HOLD_VEL_RAD_S     0.50f       /* 进入: |滤波后速度| < 0.5 rad/s */
#define POS_HOLD_TICKS         10u         /* 200Hz x 10 = 50ms 连续满足才进 */

/* 速度反馈一阶低通 (给速度环和到位判定用; 状态帧里的 vel 仍是原始 PLL 值)。
 * PLL 带宽 150Hz, 静止时会把 1 码量化噪声放大成几 rad/s 的尖峰; 30Hz 低通把尖峰压掉,
 * 相位滞后约 5.3ms, 相对 1kHz 速度环可以接受。 */
#define SPD_VEL_LP_HZ          1.0e6f      /* 1e6 = 直通(不滤波)。要试低通就改 30.0f */

#define ALIGN_VOLT_DEF         3.0f        /* 对齐电压 V。2026-10-18 本板实测:
                                            * 1.5V 只挪 0.01rad(拉不动); 3.0V 稳态相电流 0.71A, 转子吸住;
                                            * 4.0V 相电流 0.96A(已贴近 1A 连续上限, 不要用) */
#define ALIGN_TIME_MS_DEF      800u

/* 立即跳闸的相电流上限 (A)。采样链满量程是 ±4.667A (算术值), 取 4.0A 留一点余量。
 * 这是**设计值**: 8316C 自己的 OCP 在 10A 档 (手册表 8-1), 那一道太晚, 保护不了小电机。
 * 用途: 万一电流极性/编码器方向配错导致正反馈跑飞, 这一条会先把桥关掉。 */
#define MOTOR_OC_TRIP_A        2.0f        /* 默认跳闸阈值 (A); 运行时可改, 存 Flash */
#define MOTOR_OC_TRIP_MIN      0.5f
#define MOTOR_OC_TRIP_MAX      4.5f        /* 采样链算术满量程 ±4.667 A, 再高就保护不了了 */
/* 过流持续多久才跳闸 (ms)。原来超过阈值一拍就跳, 堵转不到 1A 也断;
 * 改成"连续超限累计够这个时间才跳", 让换相尖峰/瞬时冲击不误跳。0 = 一拍就跳。 */
#define MOTOR_OC_TRIP_MS_DEF   5000u
#define MOTOR_OC_TRIP_MS_MAX   10000u

#define MOTOR_VBUS_DEF         24.0f       /* 母线默认值 (设计值); 有分压采样时以实测为准 */

/* ---- CAN1 (U4 = MAX3051EKA+T, PD0=RX / PD1=TX) ----
 * 位时序沿用旧工程 2026-08-xx 在本板实测跑通的那组:
 *   PCLK1 = 48MHz (SystemCoreClock 96MHz / PPRE1_DIV2, 见 system_ch32l103.c),
 *   Prescaler=2 -> tq = 24MHz, 1 + BS1(15) + BS2(8) = 24 tq -> 1 Mbps。
 * 该工程此前已和 COM5 上的 USB-CAN 跑通过收发。 */
#define CAN_PRESCALER           2u
#define CAN_BS1_TQ              CAN_BS1_15tq
#define CAN_BS2_TQ              CAN_BS2_8tq

/* 发一帧的等待上限 (时间口径, 不按圈数)。
 * 为什么要有: 总线上没人给 ACK 时硬件 TxMailbox 会一直 Pending, 旧代码空转圈数兜底,
 * 实测一次要 162ms (最坏 480ms), 把主循环吃满 —— 控制环从 ~2000 次/秒 掉到个位数。
 * 1Mbps 下一帧最多约 130us, 给 2ms 是十几倍余量; 发不出去就丢掉这一帧, 不许堵控制环。 */
#define CAN_TX_TIMEOUT_US       2000u

/* ---- 节点地址 (协议与用法见 can_bus.h 头部) ----
 * ID = (节点号 << 5) | 功能号; 0 = 广播, 1..63 = 设备地址。存 Flash (见 nvcfg.h)。 */
#define CAN_NODE_BCAST          0u
#define CAN_NODE_DEFAULT        1u
#define CAN_NODE_MIN            1u
#define CAN_NODE_MAX            63u

/* ---- 板型 / 固件版本 (2026-10-18: CAN 功能号 14 询问, 应答功能号 22) ----
 * b0 板型号是**协议的一部分**: 以后每加一款驱动板就往后加号, 老号不许改 (主机按它认板子)。
 * 固件三个字节给人看; 协议版本一变 = 功能号表/帧布局不兼容, 主机要跟着改。
 * 版本号从 0.1.0 起, 没跟任何已发布版本挂钩 (2026-10-18 定)。 */
#define BOARD_ID_L103_DRV8316C  1u
#define BOARD_ID                BOARD_ID_L103_DRV8316C
#define FW_VER_MAJOR            0u
#define FW_VER_MINOR            1u
#define FW_VER_PATCH            0u
#define CAN_PROTO_VER           1u


/* ---- 电机参数的可写范围 (CAN MOTORCFG / 串口命令的护栏) ----
 * 这几个是**设计值**(越界就拒), 不是实测值:
 *   电流上限 0.1~4.5A —— 上限贴着采样链满量程 ±4.667A (算术值, 未实测);
 *   母线 6~60V —— 本板 12~24V 工况的两侧留量。 */
#define MOTOR_CFG_PP_MIN        1
#define MOTOR_CFG_PP_MAX        64
#define MOTOR_CFG_AMP_MIN       0.1f
#define MOTOR_CFG_AMP_MAX       4.5f
#define MOTOR_CFG_VBUS_MIN      6.0f
#define MOTOR_CFG_VBUS_MAX      60.0f

/* ---- 串口 ---- */
#define DEBUG_BAUD_DEFAULT     115200u   /* 2026-10-18 降速: 电流环 ISR 约 37us, 460800 下(21.7us/字节)接收 ISR 被压住会整字节丢; 115200 = 86.8us/字节, 稳 */


/* ---- 编译开关的默认值 (MRS 工程没有 -D, 这里必须给默认; Makefile 用 -D 覆盖) ----
 * 2026-10-21 在这里补过一次, 但改动只落在工作区、没进 git, 10-23 一次还原后被冲掉,
 * MRS 于是又报 can_bus.c:122 'CAN_LOOPBACK' undeclared。
 * 改法: 默认值放头文件 -> make 和 MRS 两边都成立; Makefile 仍可 -D 覆盖。别删。 */
#ifndef CAN_LOOPBACK
#define CAN_LOOPBACK            0       /* 1 = CAN 控制器回环 (不接总线也能自测发) */
#endif
#ifndef CMD_TIPS
#define CMD_TIPS                0       /* 1 = 打开面向人的教学式提示 (正式镜像默认关) */
#endif

#endif
