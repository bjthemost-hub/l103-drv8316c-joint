#include "bridgelink.h"
#include "uartlink.h"

#include <QSerialPort>
#include <QSerialPortInfo>

BridgeLink::BridgeLink(QObject *parent)
    : QObject(parent)
{
}

BridgeLink::~BridgeLink()
{
    close();
}

QStringList BridgeLink::availablePorts()
{
    /* 和 UART 页同一套枚举口径 (COMx | 描述), 少写一份容易不一致。 */
    return UartLink::availablePorts();
}

bool BridgeLink::open(const QString &portName, int baud, QString *err)
{
    close();

    m_port = new QSerialPort(this);
    m_port->setPortName(portName);
    m_port->setBaudRate(baud);
    m_port->setDataBits(QSerialPort::Data8);
    m_port->setParity(QSerialPort::NoParity);
    m_port->setStopBits(QSerialPort::OneStop);
    m_port->setFlowControl(QSerialPort::NoFlowControl);

    if (!m_port->open(QIODevice::ReadWrite)) {
        if (err) *err = m_port->errorString();
        delete m_port;
        m_port = nullptr;
        return false;
    }

    m_portName = portName;
    m_baud = baud;
    m_parser.reset();
    m_rxFrames = 0;
    m_txFrames = 0;
    m_desc = QStringLiteral("桥接口 %1 @ %2").arg(portName).arg(baud);

    connect(m_port, &QSerialPort::readyRead, this, &BridgeLink::onReadyRead);

    emit opened(m_desc);
    emit message(QStringLiteral("[桥] 已打开 %1 @ %2 8N1 (AA 55 帧模式)").arg(portName).arg(baud));
    return true;
}

void BridgeLink::close()
{
    if (!m_port) return;

    m_port->disconnect(this);
    if (m_port->isOpen()) m_port->close();
    m_port->deleteLater();
    m_port = nullptr;

    const QString was = m_portName;
    m_portName.clear();
    m_desc.clear();
    m_parser.reset();

    emit message(QStringLiteral("[桥] 已关闭 %1").arg(was));
    emit closed();
}

bool BridgeLink::isOpen() const
{
    return m_port && m_port->isOpen();
}

bool BridgeLink::sendFrame(quint32 id, const QByteArray &data, QString *err)
{
    if (!isOpen()) {
        if (err) *err = QStringLiteral("桥接口没打开");
        return false;
    }
    if (data.size() > 8) {
        if (err) *err = QStringLiteral("经典 CAN 帧最多 8 字节");
        return false;
    }
    if (id > 0x7FFu) {
        if (err) *err = QStringLiteral("标准帧 ID 不能超过 0x7FF");
        return false;
    }

    const QByteArray f = BridgeFraming::encode(id, data);
    if (f.isEmpty()) {
        if (err) *err = QStringLiteral("帧编码失败");
        return false;
    }

    const qint64 w = m_port->write(f);
    if (w != f.size()) {
        if (err) *err = QStringLiteral("写串口只写了 %1/%2 字节").arg(w).arg(f.size());
        return false;
    }

    ++m_txFrames;
    return true;
}

void BridgeLink::onReadyRead()
{
    if (!m_port) return;
    const QByteArray chunk = m_port->readAll();
    if (chunk.isEmpty()) return;

    const QVector<BridgeFraming::Frame> frames = m_parser.feed(chunk);
    for (const BridgeFraming::Frame &f : frames) {
        ++m_rxFrames;
        emit frameReceived(f.id, f.data);
    }
}
