#ifndef __NVCFG_H
#define __NVCFG_H

#include "ch32l103.h"

/* =============================================================================
 * 掉电保存的配置 (V2, 2026-10-23)
 * -----------------------------------------------------------------------------
 *  可调参数 + 电机参数都做成可改、存 Flash、上电读回。
 *
 * 存什么 (16 项):
 *   节点号 / 极对数 / 电流上限 / 母线电压
 *   速度环 kp, ki / 位置环 kp
 *   电流环 kp, ki / 相电阻 / 相电感
 *   速度上限 / 加速度上限 / 对齐电压 / PLL 带宽 / 测速源
 * **这里不存位置** —— 位置/机械零位走 posstore.c 的独立两页 (2026-10-23 起),
 *   本文件只管参数。两个模块各用各的页, 互不覆盖。
 *
 * 存哪儿: 0x0800FC00 起 1 页 (256B), 切成 4 条 64B 记录, 从前往后顺写,
 *   4 条满了擦一次页从头写。上电扫一遍取 seq 最大的那条。
 *   ★ 记录头 magic = 'VN20'。老版本 ('VN10', 32B/条, 只有 4 个字段) 不再识别:
 *     页里没有 'VN20' 就当"没有记录", 第一次保存前先擦页 —— 否则从旧格式上
 *     直接编程会因为有 0->1 的位而写不进去。
 *   ★ 这个地址跟旧工程 pos_log 的页有重叠, magic 不同, 两边互不认。
 *   ★ 一次保存 = 擦页 6.2~7.5ms + 编程 2.0~2.5ms (手册 V2.1 表3-18 典型值),
 *     CPU 会停在那儿, 电流环丢几拍。所以**只在人下命令时写**, 不在运动里写。
 *
 * 记录 64B (16 x uint32, 小端):
 *   off  0  magic 'VN20'        off  4  seq (越大越新)
 *   off  8  node  1..63         off 12  pp 1..64
 *   off 16  ilim  x100  A       off 20  vbus x100 V
 *   off 24  spd_kp x1e5         off 28  spd_ki x1e2
 *   off 32  pos_kp x1e2         off 36  cur_kp x1e4
 *   off 40  cur_ki x1e2         off 44  vel_lim x10 rad/s
 *   off 48  accel x10 rad/s2    off 52  align_mV(14b) | oc_trip_ca(10b <<14) | vel_src(<<24)
 *   off 56  pll_bw_e1(低16) | (oc_trip_ms+1)(高16)   [高16=0 => 老记录, 用默认]
 *   off 60  r_mohm (低16) | l_uH (高16)
 *
 * 存 0 = 这一条"没设过", 保存时沿用上一条记录里的旧值 (跟 CAN MOTORCFG 的
 * "0 = 这一条不改" 同一套语义)。
 * ========================================================================== */

typedef struct {
    int node;          /* 1..63, 0 = 没设过 */
    int pp;            /* 1..64 */
    int ilim_ca;       /* 0.01 A */
    int vbus_cv;       /* 0.01 V */
    int spd_kp_e5;     /* x1e5 */
    int spd_ki_e2;     /* x1e2 */
    int pos_kp_e2;     /* x1e2 */
    int cur_kp_e4;     /* x1e4 */
    int cur_ki_e2;     /* x1e2 */
    int vel_lim_e1;    /* x10 rad/s */
    int accel_e1;      /* x10 rad/s^2 */
    int align_mv;      /* mV */
    int pll_bw_e1;     /* x10 Hz */
    int r_mohm;        /* mOhm */
    int l_uh;          /* uH */
    int vsrc;          /* 0 自动 / 1 PLL / 2 窗口 */
    int oc_trip_ca;    /* 0.01 A 相电流跳闸阈值; 0 = 没设过 (老记录) -> 用默认 */
    int oc_trip_ms;    /* ms 过流持续多久才跳闸; -1 = 没设过 (老记录) -> 用默认 */
} NvCfg_t;

void     NvCfg_Init(void);
int      NvCfg_Load(NvCfg_t *c);        /* 1 = 找到有效记录 */
int      NvCfg_Save(const NvCfg_t *c);  /* 1 = 写完回读一致; 0 的字段沿用旧值 */
int      NvCfg_Erase(void);             /* 1 = 擦掉所有记录 (下次上电回默认) */
int      NvCfg_Enabled(void);
uint32_t NvCfg_WriteOk(void);
uint32_t NvCfg_WriteFail(void);
uint32_t NvCfg_Seq(void);
void     NvCfg_PrintInfo(void);

#endif
