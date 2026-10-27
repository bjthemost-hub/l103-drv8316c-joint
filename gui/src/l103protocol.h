#ifndef L103PROTOCOL_H
#define L103PROTOCOL_H

/* =============================================================================
 * L103 (CH32L103 + DRV8313 FOC) 上位机协议层
 *
 * 全部字段/位定义逐条核过固件源码, 出处:
 *   ...\04_工程\CH32L103_DRV8313_FOC_CAN\User\{can_bus.h, can_bus.c, cmd.c,
 *                                            motion_pv.c, motion_pv.h, board.h, faults.h}
 *   ...\04_工程\CH32L103_DRV8313_FOC_CAN\命令表与使用说明_UART与CAN_2026-09-28.md
 *
 * CAN 数据一律**小端**。
 *
 * ★ 2026-10-16 起 CAN 改多节点: ID = (节点号 << 5) | 功能号, 节点号 0 = 广播。
 *   原来的固定 ID (0x100 位置速度 / 0x110 使能 / ...) 在总线上**不存在了** ——
 *   老上位机/老脚本发的帧会被新固件当"别的节点"丢掉。
 * ========================================================================== */

#include <QByteArray>
#include <QString>
#include <QtGlobal>

namespace L103 {

/* ---------------- CAN ID: ID = (节点号 << 5) | 功能号 ----------------
 * 2026-10-16 起固件改多节点寻址, 老的固定 ID (0x100..0x1A0) 全部作废。
 * 出处: L103_DRV8316C\firmware\CH32L103_DRV8316C_FOC_CAN\User\can_bus.h
 *         #define CAN_ID_MAKE(node, fn) (((node) & 0x3F) << 5 | ((fn) & 0x1F))
 *         #define CAN_ID_NODE(id) ((id) >> 5)   #define CAN_ID_FUNC(id) ((id) & 0x1F)
 * 11 位标准帧: 节点号 0 = 广播(在场每一台都收), 1..63 = 单台; 功能号 0..31。
 * 板 -> 主机的帧, ID 里的节点号是**发送方自己**的, 所以多台同时答不会撞车。
 */
enum CanFn : quint32 {
    /* 主机 -> 板 (0..15) */
    FN_PV          = 0,    /* 位置-速度 */
    FN_VEL         = 1,    /* 纯速度 */
    FN_ENABLE      = 2,    /* 使能/失能 */
    FN_HOME        = 3,    /* 回机械原点 */
    FN_SETZERO     = 4,    /* 把当前位置设为机械 0 点 */
    FN_QUERY       = 5,    /* 询问 */
    FN_FAULTCFG    = 6,    /* 故障主动上报配置 */
    FN_VLIM        = 7,    /* 设角度环速度上限 */
    FN_MOTORCFG    = 8,    /* 设电机参数并存 Flash */
    FN_SEG         = 9,    /* 动作脚本: 加一段 (角度 0.1 度刻度) */
    FN_SEGCTL      = 10,   /* 动作脚本: 控制 (b0 0=清空 1=开始 2=打印, b1=遍数) */
    FN_SEG_HR      = 11,   /* 动作脚本: 加一段 · 大角度版 (角度 1 度刻度) */
    FN_SETNODE     = 12,   /* 设节点号 (存 Flash) */
    FN_ALIGN       = 13,   /* 电角度对齐 (b0 必须 = ALIGN_MAGIC; 2026-10-18 新增) */
    FN_INFO        = 14,   /* 读板型/固件版本 (无载荷; 2026-10-18 新增) */
    FN_PARAM       = 15,   /* 全量参数表读写 (CH32V303 板; 2026-10-22 新增) */

    /* 板 -> 主机 (16..31) */
    FN_FB          = 16,   /* 周期反馈 (10ms, 只在 debug 打开时发) */
    FN_FB_QUERY    = 17,   /* 即时状态帧 (询问应答 / ACK) */
    FN_FB_CFG      = 18,   /* 配置帧 */
    FN_FB_FAULT    = 19,   /* 故障帧 */
    FN_FB_MOTORCFG = 20,   /* 电机参数帧 (b6/b7 = 队列剩余段数/遍数) */
    FN_EVT_FAULT   = 21,   /* 故障事件(主动上报) */
    FN_FB_INFO     = 22,   /* 板型/固件版本帧 (功能号 14 的应答) */
    FN_PARAM_RSP   = 23,   /* 参数表应答帧 (功能号 15 的应答; b7 = 0xA5 签名) */
    FN_OL          = 24    /* 开环电压/频率 (2026-10-23 新增; b0 必须 0x5A) */
};

/* ---------------- 功能号 15/23: 参数表 (CH32V303_5010 板) ----------------
 * 一帧一项 4 字节 float。逐字节布局见固件 can_bus.c 的 handle_param() 头注释:
 *   主机 -> 板: b0=op, b1=索引, b2..5=float, b6=0x5A(写/存/默认/重载要), b7=不用
 *   板 -> 主机: b0=op|0x80, b1=索引, b2..5=float, b6=flags, b7=0xA5
 * 索引 0xFF = 参数个数 / 0xFD = 存 Flash 结果 / 0xFE = 重载结果 / 0xFC = 恢复默认
 * ★ 这是本板专有的; L103 那块板不认这两个功能号 (那边还是功能号 8 的三项老格式)。 */
enum ParamOp : quint8 {
    PARAM_INFO     = 0x00,
    PARAM_READ     = 0x01,
    PARAM_WRITE    = 0x02,
    PARAM_SAVE     = 0x03,
    PARAM_DEFAULTS = 0x04,
    PARAM_RELOAD   = 0x05
};

const quint8 PARAM_MAGIC = 0x5A;   /* 写/存/默认/重载的防呆位 */
const quint8 PARAM_SIG   = 0xA5;   /* 应答帧 b7 签名 */
/* GUI 侧最多支持到 64 项 (协议 b1 本来就是 8 位, 能到 255)。
 * 实际行数不写死: 读之前先发 INFO (op 0x00), 板子回 index=0xFF / value=参数个数,
 * 界面按回报的个数建表, READ 也只读这么多项 —— 固件加项后 GUI 自动跟上。 */
const int    PARAM_MAX   = 64;

struct ParamRsp {
    bool   valid = false;
    quint8 op    = 0;
    quint8 index = 0;
    float  value = 0.0f;
    quint8 flags = 0;      /* bit0 Flash 有有效记录 / bit1 改过未存 / bit2 失败 */
};

/* 节点号范围 (固件 board.h: CAN_NODE_BCAST / CAN_NODE_MIN / CAN_NODE_MAX) */
const int NODE_BCAST = 0;
const int NODE_MIN   = 1;
const int NODE_MAX   = 63;

/* 设节点号的防呆位 (固件要求 b1 = 0x5A, 不对直接拒) */
const quint8 SETNODE_MAGIC = 0x5A;
const quint8 ALIGN_MAGIC   = 0x5A;   /* 对齐命令的防呆位 (对齐会硬拽转子) */

inline quint32 canId(int node, quint32 fn)
{
    return ((quint32)(node & 0x3F) << 5) | (fn & 0x1Fu);
}
inline int     canNode(quint32 id) { return (int)((id >> 5) & 0x3Fu); }
inline quint32 canFn(quint32 id)   { return id & 0x1Fu; }

/* ---------------- 旧协议 (2026-10-15 及以前固件): 固定 ID ----------------
 * 与 2026-10-16 起的 "(节点号<<5) | 功能号" 是**两套 ID**; 帧的载荷逐字节相同,
 * 所以 build / parse 两边共用, 只有 ID 不同。
 * 旧固件没有: CAN 对齐 (只能串口 'A')、读板型/版本 (功能14/22)、设节点号、参数表 (15/23)。
 *
 * 两套 ID 互不相交:
 *   · 新 ID 0x00..0x1F 在旧固件里按精确 ID 匹配, 一条都对不上 -> 丢掉;
 *   · 旧 ID 0x100..0x1A0 在新固件里会被算成节点 8/9/12/13 的功能号 -> 节点过滤丢掉。
 * ★ 但"丢掉"只在**那个节点号上没人**时成立。所以:
 *   · 只读问询 (0x05 / 0x113) 可以两套各发一遍 —— 用来探协议;
 *   · 动作类命令绝不双发, 免得被总线上的节点 8/9/12/13 当成给它的命令。
 */
bool    legacyIdOfFn(quint32 fn, quint32 *outId);   /* 功能号 -> 旧 ID; 没有对应返回 false */
bool    legacyFnOfId(quint32 id, quint32 *outFn);   /* 旧 ID -> 功能号; 不是旧 ID 返回 false */
QString legacyIdName(quint32 id);                   /* "0x181 ... (旧协议)" */

/* ---------------- 功能0 (PV) b6 / 功能1 (速度) b2 的 flags ---------------- */
enum PvFlag : quint8 {
    PVF_REL     = 0x01,   /* 相对收到命令那一刻 */
    PVF_RUN     = 0x02,   /* 1 = 运行, 0 = 停止(必须先有它才动) */
    PVF_RELEASE = 0x04,   /* 停止时: 1 = 松轴, 0 = 保持 */
    PVF_CLEAR   = 0x08,   /* 只清故障, 不使能 (固件 2026-10-23 起; 以前会顺带重新使能) */
    PVF_SAVE    = 0x10,   /* 立刻把当前位置存 Flash */
    PVF_MECH    = 0x20    /* 目标用机械坐标(需先设过机械零位) */
};

/* ---------------- 状态位 (功能16 / 功能17 的 b6) ---------------- */
enum StatusBit : quint8 {
    ST_RUN      = 0x01,
    ST_ARRIVED  = 0x02,
    ST_STALL    = 0x04,
    ST_SENSOR   = 0x08,
    ST_ALIGNED  = 0x10,
    ST_RESTORED = 0x20,
    ST_ENABLED  = 0x40,
    ST_ZERO_SET = 0x80
};

/* ---------------- 故障位 (功能19 / 功能21) ---------------- */
enum FaultBit : quint16 {
    FA_STALL      = 0x0001,
    FA_OVERCURR   = 0x0002,
    FA_DISABLED   = 0x0004,
    FA_VM_LOW     = 0x0008,
    FA_SENSOR     = 0x0010,
    FA_NOT_ALIGN  = 0x0020,
    FA_FLASH_FAIL = 0x0040,
    FA_DRV_NFAULT = 0x0080,
    FA_CMD_REJECT = 0x0100
};

/* ---------------- 单位换算 (固件口径) ---------------- */
const double PI_        = 3.14159265358979323846;
const double TWO_PI     = 6.28318530717958647692;
const double DEG2RAD    = 0.0174532925199433;
const double RAD2DEG    = 57.2957795130823;
const double CS_RAD_S   = 0.01;   /* 速度字段 1 LSB = 0.01 rad/s */
const double CS_VOLT    = 0.01;   /* 母线电压字段 1 LSB = 0.01 V */
const double CS_AMP     = 0.01;   /* 堵转阈值字段 1 LSB = 0.01 A */

/* 固件上限 (board.h / motion_pv.c), 用来在界面上先挡一道 */
const double STALL_A_MAX     = 3.0;
const double STALL_A_MIN     = 0.1;
const double VBUS_V_MIN      = 8.0;
const double VBUS_V_MAX      = 36.0;
const int    POLE_PAIRS_MIN  = 1;
const int    POLE_PAIRS_MAX  = 64;
const double VEL_LIMIT_RAD_S = 150.0;   /* MOTOR_VELOCITY_LIMIT_MAX = 1432 rpm */
const double SEQ_DEG_MAX_CAN    = 3276.7;   /* 功能9  角度字段 int16 x0.1 度 */
const double SEQ_DEG_MAX_CAN_HR = 32767.0;  /* 功能11 大角度版 角度字段 int16 x1 度 */
const double SEQ_DEG_MAX_UART= 36000.0; /* 文本脚本 100 圈 */
const int    SEQ_MAX_SEG     = 16;
const int    SEQ_TEXT_LINE   = 128;     /* 串口一行约 128 字符 (LINE_MAX) */
const double SEQ_RPM_MAX     = 1432.0;

/* ---------------- 解码结果 ---------------- */
struct StatusFrame {
    bool   valid     = false;
    qint32 mechMrad  = 0;
    double mechRad   = 0.0;
    double mechTurns = 0.0;
    double velRadS   = 0.0;
    double velRpm    = 0.0;
    quint8 status    = 0;
    double ampA      = 0.0;
};

struct CfgFrame {
    bool   valid            = false;
    double homeRad          = 0.0;
    bool   homeSet          = false;
    double vbusV            = 0.0;
    bool   vmAvailable      = false;
    bool   aligned          = false;
    bool   enabled          = false;
    bool   reportOn         = false;
    quint8 reportPeriod100ms= 0;
};

struct FaultFrame {
    bool    valid  = false;
    quint16 active = 0;
    quint16 latched= 0;
    quint32 events = 0;
};

struct MotorCfgFrame {
    bool   valid     = false;
    int    polePairs = 0;
    double stallA    = 0.0;
    bool   fromFlash = false;
    bool   aligned   = false;
    bool   enabled   = false;
    double vbusV     = 0.0;
    quint8 seqRemain = 0;
    quint8 seqPass   = 0;
};


/* 功能号 22 板型/固件版本帧 (固件 board.h: BOARD_ID / FW_VER_* / CAN_PROTO_VER) */
struct InfoFrame {
    bool   valid    = false;
    quint8 boardId  = 0;     /* 1 = L103 + DRV8316C; 2 = L103 + DRV8313 (V2); 以后每款板加一个号 */
    quint8 fwMajor  = 0;
    quint8 fwMinor  = 0;
    quint8 fwPatch  = 0;
    quint8 protoVer = 0;     /* CAN 应用层协议版本 */
};

struct FaultEvent {
    bool    valid   = false;
    quint16 bits    = 0;
    bool    cleared = false;
    quint8  kind    = 0;    /* 0 无 / 1 电流 0.01A / 2 电压 0.1V / 3 被拒命令 ID */
    qint16  info    = 0;
    quint16 tMs     = 0;
};

/* ---------------- 解码 (成功返回 true; id 不匹配返回 false) ---------------- */
bool parseStatus    (quint32 id, const QByteArray &d, StatusFrame   *out);
bool parseCfg       (quint32 id, const QByteArray &d, CfgFrame      *out);
bool parseFault     (quint32 id, const QByteArray &d, FaultFrame    *out);
bool parseMotorCfg  (quint32 id, const QByteArray &d, MotorCfgFrame *out);
bool parseParamRsp  (quint32 id, const QByteArray &d, ParamRsp       *out);
bool parseInfo      (quint32 id, const QByteArray &d, InfoFrame      *out);
bool parseFaultEvent(quint32 id, const QByteArray &d, FaultEvent    *out);

/* ---------------- 编码 ---------------- */
QByteArray buildPv       (qint32 targetMrad, double vlimRadS, quint8 flags);
QByteArray buildVel      (double velRadS, quint8 flags);
QByteArray buildEnable   (bool on);
QByteArray buildAlign    ();
QByteArray buildInfo     ();
QByteArray buildHome     (double vlimRadS);
QByteArray buildQuery    (quint8 what);
QByteArray buildFaultCfg (bool on, quint8 period100ms);
QByteArray buildVlim     (double vmaxRadS);
QByteArray buildMotorCfg (int polePairs, double stallA, double vbusV);
QByteArray buildParamCmd (quint8 op, quint8 idx, float val, quint8 count = 1);  /* 功能15 参数表 */
QByteArray buildSeg      (int waitUs, int rpm, double deg, int dwellMs);   /* 功能9  角度 0.1 度刻度 */
QByteArray buildSegHr    (int waitUs, int rpm, double deg, int dwellMs);   /* 功能11 角度 1   度刻度 */
QByteArray buildSegCtl   (int sub, int loops);

/* ---------------- 文本 ---------------- */
QString statusText (quint8 bits);
QString faultText  (quint16 bits);
QString canFnName  (quint32 fn);        /* 功能号 -> 人话 */
QString canIdName  (quint32 id);        /* 整条 ID -> "0x041 节点2·脚本加段" */
QString boardName  (quint8 boardId);    /* 板型号 -> "L103 + DRV8316C" / "L103 + DRV8313" */
QString infoText   (const InfoFrame &f); /* 板型 + 固件版本, 一行给人看 */

} /* namespace L103 */

#endif /* L103PROTOCOL_H */
