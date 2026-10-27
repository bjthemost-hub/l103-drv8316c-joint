#include "l103protocol.h"

#include <QtMath>
#include <QStringList>
#include <cstring>

namespace L103 {

/* ------------------------- 小端读写 ------------------------- */
static qint16  rdI16(const QByteArray &d, int o)
{
    return (qint16)((quint8)d[o] | ((quint16)(quint8)d[o + 1] << 8));
}
static qint32  rdI32(const QByteArray &d, int o)
{
    quint32 v = (quint32)(quint8)d[o]
              | ((quint32)(quint8)d[o + 1] << 8)
              | ((quint32)(quint8)d[o + 2] << 16)
              | ((quint32)(quint8)d[o + 3] << 24);
    return (qint32)v;
}
static quint16 rdU16(const QByteArray &d, int o)
{
    return (quint16)((quint8)d[o] | ((quint16)(quint8)d[o + 1] << 8));
}
static quint32 rdU32(const QByteArray &d, int o)
{
    return (quint32)(quint8)d[o]
         | ((quint32)(quint8)d[o + 1] << 8)
         | ((quint32)(quint8)d[o + 2] << 16)
         | ((quint32)(quint8)d[o + 3] << 24);
}
static void wrI16(QByteArray &b, int o, qint16 v)
{
    b[o]     = (char)(quint8)(v & 0xFF);
    b[o + 1] = (char)(quint8)((v >> 8) & 0xFF);
}
static void wrI32(QByteArray &b, int o, qint32 v)
{
    quint32 u = (quint32)v;
    b[o]     = (char)(quint8)(u & 0xFF);
    b[o + 1] = (char)(quint8)((u >> 8) & 0xFF);
    b[o + 2] = (char)(quint8)((u >> 16) & 0xFF);
    b[o + 3] = (char)(quint8)((u >> 24) & 0xFF);
}

/* ------------------------- 解码 ------------------------- */
bool parseStatus(quint32 id, const QByteArray &d, StatusFrame *out)
{
    if (canFn(id) != FN_FB && canFn(id) != FN_FB_QUERY) return false;
    if (d.size() < 8) return false;

    out->valid     = true;
    out->mechMrad  = rdI32(d, 0);
    out->mechRad   = (double)out->mechMrad / 1000.0;
    out->mechTurns = out->mechRad / TWO_PI;
    out->velRadS   = (double)rdI16(d, 4) * CS_RAD_S;
    out->velRpm    = out->velRadS * 60.0 / TWO_PI;
    out->status    = (quint8)d[6];
    out->ampA      = (double)(qint8)d[7] * 0.1;
    return true;
}

bool parseCfg(quint32 id, const QByteArray &d, CfgFrame *out)
{
    if (canFn(id) != FN_FB_CFG) return false;
    if (d.size() < 8) return false;

    quint8 f = (quint8)d[6];
    out->valid             = true;
    out->homeRad           = (double)rdI32(d, 0) / 1000.0;
    out->vbusV             = (double)rdU16(d, 4) * 0.1;
    out->homeSet           = (f & 0x01) != 0;
    out->aligned           = (f & 0x02) != 0;
    out->vmAvailable       = (f & 0x04) != 0;
    out->enabled           = (f & 0x08) != 0;
    out->reportOn          = (f & 0x10) != 0;
    out->reportPeriod100ms = (quint8)d[7];
    return true;
}

bool parseFault(quint32 id, const QByteArray &d, FaultFrame *out)
{
    if (canFn(id) != FN_FB_FAULT) return false;
    if (d.size() < 8) return false;

    out->valid   = true;
    out->active  = rdU16(d, 0);
    out->latched = rdU16(d, 2);
    out->events  = rdU32(d, 4);
    return true;
}

bool parseMotorCfg(quint32 id, const QByteArray &d, MotorCfgFrame *out)
{
    if (canFn(id) != FN_FB_MOTORCFG) return false;
    if (d.size() < 8) return false;

    quint8 f = (quint8)d[3];
    out->valid     = true;
    out->polePairs = (quint8)d[0];
    out->stallA    = (double)rdI16(d, 1) * CS_AMP;
    out->fromFlash = (f & 0x01) != 0;
    out->aligned   = (f & 0x02) != 0;
    out->enabled   = (f & 0x04) != 0;
    out->vbusV     = (double)rdI16(d, 4) * CS_VOLT;
    out->seqRemain = (quint8)d[6];
    out->seqPass   = (quint8)d[7];
    return true;
}

/* 功能号 23: 参数表应答。判 b7 = 0xA5 才认 —— L103 那块板的功能号 20 帧没有这个签名,
 * 所以同一个界面接两种板子不会互相误认。 */
bool parseParamRsp(quint32 id, const QByteArray &d, ParamRsp *out)
{
    if (canFn(id) != FN_PARAM_RSP) return false;
    if (d.size() < 8) return false;
    if ((quint8)d[7] != PARAM_SIG) return false;

    quint32 u = (quint32)(quint8)d[2]
              | ((quint32)(quint8)d[3] << 8)
              | ((quint32)(quint8)d[4] << 16)
              | ((quint32)(quint8)d[5] << 24);
    float f;
    std::memcpy(&f, &u, 4);

    out->valid = true;
    out->op    = (quint8)((quint8)d[0] & 0x7Fu);
    out->index = (quint8)d[1];
    out->value = f;
    out->flags = (quint8)d[6];
    return true;
}

bool parseInfo(quint32 id, const QByteArray &d, InfoFrame *out)
{
    if (canFn(id) != FN_FB_INFO) return false;
    if (d.size() < 5) return false;          /* 固件现在发 8 字节, 5 个有效 */

    out->valid    = true;
    out->boardId  = (quint8)d[0];
    out->fwMajor  = (quint8)d[1];
    out->fwMinor  = (quint8)d[2];
    out->fwPatch  = (quint8)d[3];
    out->protoVer = (quint8)d[4];
    return true;
}

bool parseFaultEvent(quint32 id, const QByteArray &d, FaultEvent *out)
{
    if (canFn(id) != FN_EVT_FAULT) return false;
    if (d.size() < 8) return false;

    out->valid   = true;
    out->bits    = rdU16(d, 0);
    out->cleared = ((quint8)d[2] != 0);
    out->kind    = (quint8)d[3];
    out->info    = rdI16(d, 4);
    out->tMs     = rdU16(d, 6);
    return true;
}

/* ------------------------- 编码 ------------------------- */
static QByteArray make8()
{
    QByteArray b(8, '\0');
    return b;
}

QByteArray buildPv(qint32 targetMrad, double vlimRadS, quint8 flags)
{
    QByteArray b = make8();
    wrI32(b, 0, targetMrad);
    double cs = vlimRadS / CS_RAD_S;
    if (cs < 0) cs = -cs;
    if (cs > 32767.0) cs = 32767.0;
    wrI16(b, 4, (qint16)qRound(cs));
    b[6] = (char)(quint8)flags;
    b[7] = 0;
    return b;
}

QByteArray buildVel(double velRadS, quint8 flags)
{
    QByteArray b = make8();
    double cs = velRadS / CS_RAD_S;
    if (cs >  32767.0) cs =  32767.0;
    if (cs < -32768.0) cs = -32768.0;
    wrI16(b, 0, (qint16)qRound(cs));
    b[2] = (char)(quint8)flags;
    return b.left(3);
}

QByteArray buildEnable(bool on)
{
    QByteArray b = make8();
    b[0] = (char)(quint8)(on ? 1 : 0);
    return b.left(1);
}

/* 读板型/固件版本 (CAN 功能号 14)。固件不要求载荷, 发一个 0 字节最省事;
 * 应答是功能号 22 —— 别把它当成"没答", 见 parseInfo()。 */
QByteArray buildInfo()
{
    QByteArray b = make8();
    b[0] = 0;
    return b.left(1);
}

/* 电角度对齐 (CAN 功能号 13; 固件要求 b0 = ALIGN_MAGIC 0x5A 防呆)。
 * 非阻塞: 板子收到就回一帧状态帧, 真正的对齐在后台跑 (ALIGN_TIME_MS_DEF);
 * 那一刻 aligned 通常还是 0, 要轮询到 aligned=1 才算完成。 */
QByteArray buildAlign()
{
    QByteArray b = make8();
    b[0] = (char)(quint8)ALIGN_MAGIC;
    return b.left(1);
}

QByteArray buildHome(double vlimRadS)
{
    QByteArray b = make8();
    double cs = vlimRadS / CS_RAD_S;
    if (cs < 0) cs = -cs;
    if (cs > 32767.0) cs = 32767.0;
    wrI16(b, 0, (qint16)qRound(cs));
    return b.left(2);
}

QByteArray buildQuery(quint8 what)
{
    QByteArray b = make8();
    b[0] = (char)what;
    return b.left(1);
}

QByteArray buildFaultCfg(bool on, quint8 period100ms)
{
    QByteArray b = make8();
    b[0] = (char)(quint8)(on ? 1 : 0);
    b[1] = (char)period100ms;
    return b.left(2);
}

QByteArray buildVlim(double vmaxRadS)
{
    QByteArray b = make8();
    double cs = vmaxRadS / CS_RAD_S;
    if (cs < 1.0) cs = 1.0;
    if (cs > 32767.0) cs = 32767.0;
    wrI16(b, 0, (qint16)qRound(cs));
    return b.left(2);
}

QByteArray buildMotorCfg(int polePairs, double stallA, double vbusV)
{
    QByteArray b = make8();
    b[0] = (char)(quint8)(polePairs < 0 ? 0 : polePairs);

    qint16 ca = 0;
    if (stallA > 0.0) {
        double v = stallA / CS_AMP + 0.5;
        if (v > 32767.0) v = 32767.0;
        ca = (qint16)v;
    }
    wrI16(b, 1, ca);

    b[3] = (char)0x01;                    /* bit0 = 防呆确认位, 必须 = 1 */

    qint16 cv = 0;
    if (vbusV > 0.0) {
        double v = vbusV / CS_VOLT + 0.5;
        if (v > 32767.0) v = 32767.0;
        cv = (qint16)v;
    }
    wrI16(b, 4, cv);
    return b;
}

QByteArray buildParamCmd(quint8 op, quint8 idx, float val, quint8 count)
{
    QByteArray b = make8();
    quint32 u;
    std::memcpy(&u, &val, 4);

    b[0] = (char)op;
    b[1] = (char)idx;
    b[2] = (char)(quint8)(u & 0xFFu);
    b[3] = (char)(quint8)((u >> 8) & 0xFFu);
    b[4] = (char)(quint8)((u >> 16) & 0xFFu);
    b[5] = (char)(quint8)((u >> 24) & 0xFFu);
    b[6] = (char)PARAM_MAGIC;      /* 防呆位; 读/INFO 时板子不看它 */
    b[7] = 0;

    /* READ 时 b2 是"要几项"(固件就是这么解析的), WRITE 时 b2 是 float 的最低字节 */
    if (op == PARAM_READ) b[2] = (char)count;
    return b;
}

/* 功能 9 (CAN_FN_SEG) 与功能 11 (CAN_FN_SEG_HR) 布局**逐字节相同**, 只有角度那个
 * int16 字段的刻度不同 —— 所以下面共用一个写体, 调用时喂不同的 LSB 数。 */
static QByteArray buildSegBody(int waitUs, int rpm, double degLsb, int dwellMs)
{
    QByteArray b = make8();
    int w = waitUs / 100;                 /* 1 LSB = 100 us */
    if (w < 0) w = 0;
    if (w > 65535) w = 65535;
    wrI16(b, 0, (qint16)w);

    if (rpm >  32767) rpm =  32767;
    if (rpm < -32768) rpm = -32768;
    wrI16(b, 2, (qint16)rpm);

    if (degLsb >  32767.0) degLsb =  32767.0;
    if (degLsb < -32768.0) degLsb = -32768.0;
    wrI16(b, 4, (qint16)qRound(degLsb));

    int dw = dwellMs / 10;                /* 1 LSB = 10 ms */
    if (dw < 0) dw = 0;
    if (dw > 65535) dw = 65535;
    wrI16(b, 6, (qint16)dw);
    return b;
}

QByteArray buildSeg(int waitUs, int rpm, double deg, int dwellMs)
{
    return buildSegBody(waitUs, rpm, deg * 10.0, dwellMs);   /* 1 LSB = 0.1 度, 单帧上限 3276.7 度 */
}

QByteArray buildSegHr(int waitUs, int rpm, double deg, int dwellMs)
{
    return buildSegBody(waitUs, rpm, deg, dwellMs);          /* 1 LSB = 1 度, 单帧上限 32767 度 */
}

QByteArray buildSegCtl(int sub, int loops)
{
    QByteArray b = make8();
    b[0] = (char)(quint8)sub;
    if (loops < 0)   loops = 0;
    if (loops > 255) loops = 255;
    b[1] = (char)(quint8)loops;
    return b.left(2);
}

/* ------------------------- 文本 ------------------------- */
QString statusText(quint8 bits)
{
    QStringList s;
    if (bits & ST_RUN)      s << QStringLiteral("RUN");
    if (bits & ST_ARRIVED)  s << QStringLiteral("ARRIVED");
    if (bits & ST_STALL)    s << QStringLiteral("STALL");
    if (bits & ST_SENSOR)   s << QStringLiteral("SENSOR_ERR");
    if (bits & ST_ALIGNED)  s << QStringLiteral("ALIGNED");
    if (bits & ST_RESTORED) s << QStringLiteral("POS_RESTORED");
    if (bits & ST_ENABLED)  s << QStringLiteral("ENABLED");
    if (bits & ST_ZERO_SET) s << QStringLiteral("ZERO_SET");
    if (s.isEmpty())        s << QStringLiteral("(none)");
    return s.join(QStringLiteral(" | "));
}

QString faultText(quint16 bits)
{
    QStringList s;
    if (bits & FA_STALL)      s << QStringLiteral("STALL(堵转)");
    if (bits & FA_OVERCURR)   s << QStringLiteral("OVERCURRENT(过流)");
    if (bits & FA_DISABLED)   s << QStringLiteral("DISABLED(已失能)");
    if (bits & FA_VM_LOW)     s << QStringLiteral("VM_LOW(母线低)");
    if (bits & FA_SENSOR)     s << QStringLiteral("SENSOR(编码器)");
    if (bits & FA_NOT_ALIGN)  s << QStringLiteral("NOT_ALIGNED(未对齐)");
    if (bits & FA_FLASH_FAIL) s << QStringLiteral("FLASH_FAIL");
    if (bits & FA_DRV_NFAULT) s << QStringLiteral("DRV_NFAULT");
    if (bits & FA_CMD_REJECT) s << QStringLiteral("CMD_REJECT(命令被拒)");
    if (s.isEmpty())          s << QStringLiteral("(无)");
    return s.join(QStringLiteral(" | "));
}

QString canFnName(quint32 fn)
{
    switch (fn) {
    case FN_PV:          return QStringLiteral("位置-速度(PV)");
    case FN_VEL:         return QStringLiteral("纯速度");
    case FN_ENABLE:      return QStringLiteral("使能/失能");
    case FN_HOME:        return QStringLiteral("回机械原点");
    case FN_SETZERO:     return QStringLiteral("设机械零点");
    case FN_QUERY:       return QStringLiteral("询问");
    case FN_FAULTCFG:    return QStringLiteral("故障上报配置");
    case FN_VLIM:        return QStringLiteral("角度环速度上限");
    case FN_MOTORCFG:    return QStringLiteral("电机参数");
    case FN_SEG:         return QStringLiteral("脚本加段");
    case FN_SEGCTL:      return QStringLiteral("脚本控制");
    case FN_SEG_HR:      return QStringLiteral("脚本加段(大角度)");
    case FN_SETNODE:     return QStringLiteral("设节点号");
    case FN_ALIGN:       return QStringLiteral("电角度对齐");
    case FN_FB:          return QStringLiteral("周期反馈");
    case FN_FB_QUERY:    return QStringLiteral("状态帧");
    case FN_FB_CFG:      return QStringLiteral("配置帧");
    case FN_FB_FAULT:    return QStringLiteral("故障帧");
    case FN_FB_MOTORCFG: return QStringLiteral("电机参数帧");
    case FN_EVT_FAULT:   return QStringLiteral("故障事件");
    case FN_INFO:        return QStringLiteral("读板型/版本");
    case FN_FB_INFO:     return QStringLiteral("板型/版本帧");
    case FN_OL:          return QStringLiteral("开环电压/频率");
    default:             return QStringLiteral("功能%1(保留)").arg(fn);
    }
}

/* ------------------- 旧协议 (2026-10-15 及以前固件) 的固定 ID -------------------
 * 表里只列两边**同义**的命令; 载荷布局逐字节相同 (新固件的 send_* 就是从这份抄过去的),
 * 所以收发都复用 build / parse, 只换 ID。
 * 旧固件没有的: FN_ALIGN (只能串口 'A')、FN_INFO、FN_SETNODE、FN_PARAM。 */
static const struct { quint32 fn; quint32 id; } kLegacyMap[] = {
    { FN_PV,          0x100u },   /* 位置-速度 */
    { FN_VEL,         0x101u },   /* 纯速度 */
    { FN_ENABLE,      0x110u },   /* 使能 / 失能 */
    { FN_HOME,        0x111u },   /* 回机械原点 */
    { FN_SETZERO,     0x112u },   /* 设机械 0 点 */
    { FN_QUERY,       0x113u },   /* 询问 */
    { FN_FAULTCFG,    0x114u },   /* 故障上报配置 */
    { FN_VLIM,        0x115u },   /* 角度环速度上限 */
    { FN_MOTORCFG,    0x116u },   /* 电机参数 */
    { FN_SEG,         0x120u },   /* 脚本加段 */
    { FN_SEGCTL,      0x121u },   /* 脚本控制 */
    { FN_SEG_HR,      0x122u },   /* 脚本加段(大角度) */
    { FN_FB,          0x180u },   /* 周期反馈 */
    { FN_FB_QUERY,    0x181u },   /* 即时状态帧 */
    { FN_FB_CFG,      0x182u },   /* 配置帧 */
    { FN_FB_FAULT,    0x183u },   /* 故障帧 */
    { FN_FB_MOTORCFG, 0x184u },   /* 电机参数帧 */
    { FN_EVT_FAULT,   0x1A0u }    /* 故障事件 */
};

bool legacyIdOfFn(quint32 fn, quint32 *outId)
{
    for (const auto &m : kLegacyMap)
        if (m.fn == fn) { if (outId) *outId = m.id; return true; }
    return false;
}

bool legacyFnOfId(quint32 id, quint32 *outFn)
{
    for (const auto &m : kLegacyMap)
        if (m.id == id) { if (outFn) *outFn = m.fn; return true; }
    return false;
}

QString legacyIdName(quint32 id)
{
    quint32 fn = 0;
    if (!legacyFnOfId(id, &fn))
        return QStringLiteral("0x%1 (旧协议?)").arg(QString::number(id, 16).toUpper().rightJustified(3, QLatin1Char('0')));
    return QStringLiteral("%1 · 0x%2 (旧协议)").arg(canFnName(fn),
        QString::number(id, 16).toUpper().rightJustified(3, QLatin1Char('0')));
}

/* 板型号 (固件 board.h: BOARD_ID_*)。
 * ⚠ 这是**协议约定**, 不是从板子上读出来的名字: 两边要一起改。
 *   新板往后加号, 老号不许改 —— 改了就分不清 0x01 到底是谁。 */
QString boardName(quint8 boardId)
{
    switch (boardId) {
    case 1:  return QStringLiteral("L103 + DRV8316C");
    case 2:  return QStringLiteral("L103 + DRV8313");
    case 3:  return QStringLiteral("CH32V303 + 5010");   /* 2026-10-24 从 2 挪开, 见 V303 固件 board.h */
    default: return QStringLiteral("未知板型(%1)").arg((int)boardId);
    }
}

QString infoText(const InfoFrame &f)
{
    if (!f.valid) return QStringLiteral("—");
    return QStringLiteral("%1 · 固件 %2.%3.%4 · 协议 v%5")
        .arg(boardName(f.boardId))
        .arg((int)f.fwMajor).arg((int)f.fwMinor).arg((int)f.fwPatch)
        .arg((int)f.protoVer);
}

QString canIdName(quint32 id)
{
    const int n = canNode(id);
    return QStringLiteral("0x%1 节点%2·%3")
        .arg(QStringLiteral("%1").arg(id, 3, 16, QLatin1Char('0')).toUpper())   /* 只大写数字, 前缀留 0x */
        .arg(n == 0 ? QStringLiteral("0(广播)") : QString::number(n))
        .arg(canFnName(canFn(id)));
}

} /* namespace L103 */
