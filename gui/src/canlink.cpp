#include "canlink.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QLibrary>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>

/* =============================================================================
 * USBCANx64.dll 动态绑定
 *
 * 函数原型来自 USB-CAN 适配器厂商 SDK 的 USBCAN.h。
 * x64 下 WINAPI(__stdcall) 与 __cdecl 是同一套调用约定, 所以不需要额外修饰。
 * ========================================================================== */

typedef void        (*fnDLLInit)(void);
typedef const char *(*fnCanGetDevName)(unsigned long);
typedef void       *(*fnCanOpenDev)(unsigned long, unsigned long, void *);
typedef void        (*fnCanCloseDev)(unsigned long);
typedef int         (*fnCanSendFrame)(unsigned long, unsigned char *, unsigned char,
                                      unsigned long, int, int);
typedef int         (*fnCanRecvFrame)(unsigned long, void *);

/* CanRecvData 的 MSVC 自然对齐布局 (x64):
 *   nNum        @ 0
 *   Data[i]     @ 4 + i*28
 *     nType     @ +0
 *     union     @ +4
 *       Frame: nSize @ +4   FrameId @ +8   Buf[8] @ +12  bExtend @ +20  bRemote @ +24
 *   sizeof = 116
 * 这里按偏移手工取, 不依赖编译器的结构体布局。 */
static const int RECV_STRIDE = 28;
static const int RECV_NUM_OFF = 0;
static const int RECV_NTYPE_OFF = 0;
static const int RECV_NSTATE_OFF = 4;
static const int RECV_NSIZE_OFF = 4;
static const int RECV_ID_OFF = 8;
static const int RECV_BUF_OFF = 12;

static QString locateDll()
{
    QStringList cand;
    cand << QCoreApplication::applicationDirPath() + QStringLiteral("/USBCANx64.dll");
    cand << QDir::currentPath() + QStringLiteral("/USBCANx64.dll");
    cand << QStringLiteral("E:/MyProjs/08.CHxxxx/CH9433EVT/EVT/TOOLS/Windows/lib/USBCANx64.dll");
    for (const QString &c : cand) {
        if (QFileInfo::exists(c)) return c;
    }
    return QString();
}

QString CanLink::dllPathTried()
{
    QString f = locateDll();
    return f.isEmpty() ? QStringLiteral("(没找到 USBCANx64.dll)") : f;
}

/* ------------------------- 枚举 ------------------------- */
QStringList CanLink::ch9433Devices()
{
    QStringList out;
    QString dll = locateDll();
    if (dll.isEmpty()) return out;

    QLibrary lib(dll);
    if (!lib.load()) return out;

    fnDLLInit pInit = (fnDLLInit)lib.resolve("DLLInit");
    fnCanGetDevName pName = (fnCanGetDevName)lib.resolve("CanGetDevName");
    if (!pInit || !pName) { lib.unload(); return out; }

    pInit();
    for (unsigned long i = 0; i < 16; ++i) {
        const char *nm = pName(i);
        if (!nm) break;
        out << QStringLiteral("设备%1 | %2").arg(i).arg(QString::fromLocal8Bit(nm));
    }
    return out;
}

QStringList CanLink::dmPorts()
{
    QStringList out;
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &i : infos) {
        QString d = i.description().trimmed();
        out << QStringLiteral("%1 | %2").arg(i.portName(), d);
    }
    return out;
}

/* ------------------------- 生命周期 ------------------------- */
CanLink::CanLink(QObject *parent)
    : QObject(parent)
{
    m_timer = new QTimer(this);
    m_timer->setInterval(5);
    connect(m_timer, &QTimer::timeout, this, &CanLink::poll);
}

CanLink::~CanLink()
{
    close();
}

bool CanLink::open(Kind kind, const QString &target, int baud, QString *err)
{
    close();

    if (kind == Kind_Ch9433) {
        QString dll = locateDll();
        if (dll.isEmpty()) {
            if (err) *err = QStringLiteral("没找到 USBCANx64.dll");
            return false;
        }

        m_lib = new QLibrary(dll);
        if (!m_lib->load()) {
            if (err) *err = QStringLiteral("加载 %1 失败: %2").arg(dll, m_lib->errorString());
            delete m_lib; m_lib = nullptr;
            return false;
        }

        fnDLLInit        pInit  = (fnDLLInit)       m_lib->resolve("DLLInit");
        fnCanOpenDev     pOpen  = (fnCanOpenDev)    m_lib->resolve("CanOpenDev");
        fnCanSendFrame   pSend  = (fnCanSendFrame)  m_lib->resolve("CanSendFrame");
        fnCanRecvFrame   pRecv  = (fnCanRecvFrame)  m_lib->resolve("CanRecvFrame");
        fnCanCloseDev    pClose = (fnCanCloseDev)   m_lib->resolve("CanCloseDev");

        if (!pInit || !pOpen || !pSend || !pRecv || !pClose) {
            if (err) *err = QStringLiteral("USBCANx64.dll 里缺少需要的导出函数");
            m_lib->unload(); delete m_lib; m_lib = nullptr;
            return false;
        }

        pInit();

        /* target 形如 "设备0 | ..."; 取第一个数字当设备号 */
        m_devIndex = 0;
        {
            QString t = target;
            int bar = t.indexOf('|');
            if (bar > 0) t = t.left(bar);
            bool ok = false;
            int n = t.remove(QStringLiteral("设备")).trimmed().toInt(&ok);
            if (ok) m_devIndex = n;
        }

        struct Cfg { unsigned long nWorkMode; int bOneShot; unsigned long nBaudRate; };
        Cfg cfg;
        cfg.nWorkMode  = 0;
        cfg.bOneShot   = 0;
        cfg.nBaudRate  = (unsigned long)baud;

        void *h = pOpen((unsigned long)m_devIndex, 1000, &cfg);
        if (!h || h == (void *)(qintptr)-1) {
            if (err) *err = QStringLiteral("CanOpenDev 失败 (设备 %1)").arg(m_devIndex);
            m_lib->unload(); delete m_lib; m_lib = nullptr;
            return false;
        }

        m_openDev = h;
        m_devOpen = true;
        m_kind = Kind_Ch9433;
        m_desc = QStringLiteral("CH9433 EVB 设备%1 @%2 bps").arg(m_devIndex).arg(baud);

        m_timer->start();
        emit opened(m_desc);
        emit message(QStringLiteral("[CAN] 已打开 %1 (DLL: %2)").arg(m_desc, dll));
        return true;
    }

    if (kind == Kind_DmUsb2Can) {
        QString port = target;
        int bar = port.indexOf('|');
        if (bar > 0) port = port.left(bar);
        port = port.trimmed();

        m_ser = new QSerialPort(this);
        m_ser->setPortName(port);
        m_ser->setBaudRate(baud > 0 ? baud : 921600);
        m_ser->setDataBits(QSerialPort::Data8);
        m_ser->setParity(QSerialPort::NoParity);
        m_ser->setStopBits(QSerialPort::OneStop);
        m_ser->setFlowControl(QSerialPort::NoFlowControl);

        if (!m_ser->open(QIODevice::ReadWrite)) {
            if (err) *err = m_ser->errorString();
            delete m_ser; m_ser = nullptr;
            return false;
        }

        connect(m_ser, &QSerialPort::readyRead, this, [this]() {
            feedDm(m_ser->readAll());
        });

        m_dmBuf.clear();
        m_kind = Kind_DmUsb2Can;
        m_desc = QStringLiteral("达妙 DM-USB2CAN %1 @%2").arg(port).arg(m_ser->baudRate());

        emit opened(m_desc);
        emit message(QStringLiteral("[CAN] 已打开 %1 (30 字节信封)").arg(m_desc));
        return true;
    }

    if (err) *err = QStringLiteral("未选择 CAN 类型");
    return false;
}

void CanLink::close()
{
    m_timer->stop();

    if (m_lib) {
        fnCanCloseDev pClose = (fnCanCloseDev)m_lib->resolve("CanCloseDev");
        if (pClose && m_devOpen) pClose((unsigned long)m_devIndex);
        m_lib->unload();
        delete m_lib;
        m_lib = nullptr;
        m_devOpen = false;
        m_openDev = nullptr;
    }

    if (m_ser) {
        m_ser->disconnect(this);
        if (m_ser->isOpen()) m_ser->close();
        m_ser->deleteLater();
        m_ser = nullptr;
    }

    m_dmBuf.clear();

    if (m_kind != Kind_None) {
        QString was = m_desc;
        m_kind = Kind_None;
        m_desc.clear();
        emit message(QStringLiteral("[CAN] 已关闭 %1").arg(was));
        emit closed();
    }
}

/* ------------------------- 发送 ------------------------- */
bool CanLink::sendFrame(quint32 id, const QByteArray &data, QString *err)
{
    if (!isOpen()) {
        if (err) *err = QStringLiteral("CAN 没打开");
        return false;
    }
    if (data.size() > 8) {
        if (err) *err = QStringLiteral("标准帧最多 8 字节");
        return false;
    }

    if (m_kind == Kind_Ch9433) return sendCh9433(id, data, err);
    if (m_kind == Kind_DmUsb2Can) return sendDm(id, data, err);

    if (err) *err = QStringLiteral("未知 CAN 类型");
    return false;
}

bool CanLink::sendCh9433(quint32 id, const QByteArray &data, QString *err)
{
    fnCanSendFrame pSend = (fnCanSendFrame)m_lib->resolve("CanSendFrame");
    if (!pSend) {
        if (err) *err = QStringLiteral("CanSendFrame 未解析");
        return false;
    }

    unsigned char buf[8] = {0};
    for (int i = 0; i < data.size(); ++i) buf[i] = (unsigned char)data[i];

    int rc = pSend((unsigned long)m_devIndex, buf, (unsigned char)data.size(),
                   (unsigned long)id, 0, 0);
    if (rc != 1) {
        if (err) *err = QStringLiteral("CanSendFrame 返回 %1 (1=成功 0=超时 -1=失败)").arg(rc);
        return false;
    }
    return true;
}

bool CanLink::sendDm(quint32 id, const QByteArray &data, QString *err)
{
    if (!m_ser || !m_ser->isOpen()) {
        if (err) *err = QStringLiteral("串口没打开");
        return false;
    }

    QByteArray f(30, '\0');
    f[0] = (char)0x55;
    f[1] = (char)0xAA;
    f[2] = (char)0x1E;                 /* 长度 = 30 */
    f[3] = (char)0x01;                 /* HOST_CMD_FWD */

    f[4] = 0x01; f[5] = 0; f[6] = 0; f[7] = 0;        /* times = 1 */
    f[8] = 0x05; f[9] = 0; f[10] = 0; f[11] = 0;      /* interval = 5 ms */

    f[12] = 0x00;                                     /* 标准帧 */
    f[13] = (char)(quint8)(id & 0xFF);
    f[14] = (char)(quint8)((id >> 8) & 0xFF);
    f[15] = (char)(quint8)((id >> 16) & 0xFF);
    f[16] = (char)(quint8)((id >> 24) & 0xFF);

    f[17] = 0x00;                                     /* 数据帧 */
    f[18] = 0x08;                                     /* 数据长度 */
    f[19] = 0x00;
    f[20] = 0x00;

    for (int i = 0; i < data.size() && i < 8; ++i) f[21 + i] = data[i];
    f[29] = 0x00;                                     /* CRC 模块忽略 */

    qint64 w = m_ser->write(f);
    m_ser->flush();
    if (w != f.size()) {
        if (err) *err = QStringLiteral("写串口只写了 %1/%2 字节").arg(w).arg(f.size());
        return false;
    }
    return true;
}

/* ------------------------- 接收 ------------------------- */
void CanLink::poll()
{
    if (m_kind != Kind_Ch9433 || !m_lib) return;

    fnCanRecvFrame pRecv = (fnCanRecvFrame)m_lib->resolve("CanRecvFrame");
    if (!pRecv) return;

    unsigned char raw[160];
    for (int guard = 0; guard < 8; ++guard) {
        memset(raw, 0, sizeof(raw));
        int r = pRecv((unsigned long)m_devIndex, raw);
        if (r != 1) break;

        quint32 n = *(quint32 *)(raw + RECV_NUM_OFF);
        if (n > 4) n = 4;

        for (quint32 i = 0; i < n; ++i) {
            const unsigned char *e = raw + 4 + (int)i * RECV_STRIDE;
            if (e[RECV_NTYPE_OFF] != 0) continue;             /* 1 = 状态, 不是帧 */

            unsigned char nSize = e[RECV_NSIZE_OFF];
            if (nSize > 8) nSize = 8;
            quint32 id = *(quint32 *)(e + RECV_ID_OFF);

            QByteArray d((const char *)(e + RECV_BUF_OFF), (int)nSize);
            emit frameReceived(id, d);
        }
    }
}

void CanLink::feedDm(const QByteArray &chunk)
{
    m_dmBuf.append(chunk);

    for (;;) {
        int head = m_dmBuf.indexOf((char)0xAA);
        if (head < 0) { m_dmBuf.clear(); return; }
        if (head > 0) m_dmBuf.remove(0, head);
        if (m_dmBuf.size() < 16) return;

        if ((quint8)m_dmBuf[0] != 0xAA || (quint8)m_dmBuf[15] != 0x55) {
            m_dmBuf.remove(0, 1);
            continue;
        }

        quint8 cmd = (quint8)m_dmBuf[1];
        quint8 hdr = (quint8)m_dmBuf[2];
        /* 只认 0x11 = "从总线上收到一帧"(板子来的数据)。
         * 0x12 是模块回给主机的"我发出去了"收条, ID 回显的是我们自己发的 ID,
         * 拿它当板子应答会把"回零(功能3)"误读成"状态帧(功能17)" —— 命令表 §9.1 点名的坑。 */
        if (cmd == 0x11) {
            int len = hdr & 0x3F;
            if (len > 8) len = 8;
            quint32 id = (quint8)m_dmBuf[3]
                       | ((quint32)(quint8)m_dmBuf[4] << 8)
                       | ((quint32)(quint8)m_dmBuf[5] << 16)
                       | ((quint32)(quint8)m_dmBuf[6] << 24);
            QByteArray d = m_dmBuf.mid(7, len);
            emit frameReceived(id, d);
        }
        m_dmBuf.remove(0, 16);
    }
}
