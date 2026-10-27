#ifndef BRIDGELINK_H
#define BRIDGELINK_H

/* H750 UART<->CAN 桥 (帧模式) 的 PC 侧链路。
 *
 * 物理: 串口 921600 8N1, 无流控。
 * 帧:   AA 55 FLAGS ID(2/4 大端) DLC DATA... XOR  (见 bridgeframing.h)
 *
 * 与 CanLink 的对外口径保持一致: open/close/isOpen/sendFrame/describe +
 * opened/closed/frameReceived/message, 这样 MainWindow 里只换后端, 协议层不动。
 */

#include <QByteArray>
#include <QObject>
#include <QString>
#include <QStringList>

#include "bridgeframing.h"

class QSerialPort;

class BridgeLink : public QObject
{
    Q_OBJECT
public:
    explicit BridgeLink(QObject *parent = nullptr);
    ~BridgeLink() override;

    static QStringList availablePorts();       /* "COM6 | USB-SERIAL CH343" */

    bool open(const QString &portName, int baud, QString *err = nullptr);
    void close();
    bool isOpen() const;
    QString describe() const { return m_desc; }
    QString portName() const { return m_portName; }
    int baud() const { return m_baud; }

    /* 经典帧, 最多 8 字节; 失败时 err 里是原因。 */
    bool sendFrame(quint32 id, const QByteArray &data, QString *err = nullptr);

    quint64 rxFrames() const { return m_rxFrames; }
    quint64 badXor() const { return m_parser.badXor; }

signals:
    void opened(const QString &desc);
    void closed();
    void frameReceived(quint32 id, const QByteArray &data);
    void message(const QString &text);

private slots:
    void onReadyRead();

private:
    QSerialPort *m_port = nullptr;
    BridgeFraming::Parser m_parser;
    QString m_portName;
    QString m_desc;
    int     m_baud = 921600;
    quint64 m_rxFrames = 0;
    quint64 m_txFrames = 0;
};

#endif /* BRIDGELINK_H */
