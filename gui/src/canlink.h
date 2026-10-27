#ifndef CANLINK_H
#define CANLINK_H

/* L103 CAN 通道。两种转接卡, 信封不一样 (命令表 §9):
 *
 *   CH9433 EVB (VID_1A86&PID_5610, MI_00)
 *       走 USBCANx64.dll, CanSendFrame(handle, buf, 8, id, 0, 0) —— 发**裸帧**, 没有信封。
 *       本程序用 QLibrary **动态加载**, 所以没装 DLL 也能跑 (只有 CAN 页灰掉)。
 *
 *   达妙 DM-USB2CAN (VID_2E88&PID_4603)
 *       PC 侧是个串口 COM5 @921600, 每帧要套 30 字节信封。
 *
 * 两种都是标准帧 11bit、总线 1 Mbps。
 */

#include <QByteArray>
#include <QObject>
#include <QString>
#include <QStringList>

class QLibrary;
class QSerialPort;
class QTimer;

class CanLink : public QObject
{
    Q_OBJECT
public:
    enum Kind {
        Kind_None = 0,
        Kind_Ch9433 = 1,      /* 沁恒 CH9433 EVB, 裸帧 */
        Kind_DmUsb2Can = 2    /* 达妙 DM-USB2CAN, COM + 30 字节信封 */
    };

    explicit CanLink(QObject *parent = nullptr);
    ~CanLink() override;

    /* 枚举。DLL 不在时 ch9433Devices() 返回空表。 */
    static QStringList ch9433Devices();
    static QStringList dmPorts();
    static QString     dllPathTried();

    bool open(Kind kind, const QString &target, int baud, QString *err = nullptr);
    void close();
    bool isOpen() const { return m_kind != Kind_None; }
    Kind kind() const { return m_kind; }
    QString describe() const { return m_desc; }

    bool sendFrame(quint32 id, const QByteArray &data, QString *err = nullptr);

signals:
    void opened(const QString &desc);
    void closed();
    void frameReceived(quint32 id, const QByteArray &data);
    void message(const QString &text);

private slots:
    void poll();

private:
    bool sendCh9433(quint32 id, const QByteArray &data, QString *err);
    bool sendDm(quint32 id, const QByteArray &data, QString *err);
    void feedDm(const QByteArray &chunk);

    Kind    m_kind = Kind_None;
    QString m_desc;
    QTimer *m_timer = nullptr;

    /* ---- CH9433 ---- */
    QLibrary *m_lib = nullptr;
    int       m_devIndex = 0;
    bool      m_devOpen  = false;
    void     *m_openDev  = nullptr;

    /* ---- 达妙 ---- */
    QSerialPort *m_ser = nullptr;
    QByteArray   m_dmBuf;
};

#endif /* CANLINK_H */
