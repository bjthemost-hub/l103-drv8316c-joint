#include "uartlink.h"

#include <QSerialPort>
#include <QSerialPortInfo>

UartLink::UartLink(QObject *parent)
    : QObject(parent)
{
}

UartLink::~UartLink()
{
    close();
}

QStringList UartLink::availablePorts()
{
    QStringList out;
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &i : infos) {
        QString d = i.description().trimmed();
        if (d.isEmpty()) d = QStringLiteral("(无描述)");
        out << QStringLiteral("%1 | %2").arg(i.portName(), d);
    }
    return out;
}

bool UartLink::open(const QString &portName, int baud, QString *err)
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
    m_rawMode = false;
    m_buf.clear();
    m_rxBytes = 0;
    m_txBytes = 0;

    connect(m_port, &QSerialPort::readyRead, this, &UartLink::onReadyRead);

    emit opened(portName);
    emit message(QStringLiteral("[UART] 已打开 %1 @ %2 8N1").arg(portName).arg(baud));
    return true;
}

void UartLink::close()
{
    if (!m_port) return;

    m_port->disconnect(this);
    if (m_port->isOpen()) m_port->close();
    m_port->deleteLater();
    m_port = nullptr;

    QString was = m_portName;
    m_portName.clear();
    m_buf.clear();

    emit message(QStringLiteral("[UART] 已关闭 %1").arg(was));
    emit closed();
}

bool UartLink::isOpen() const
{
    return m_port && m_port->isOpen();
}

void UartLink::sendRaw(const QByteArray &bytes)
{
    if (!isOpen() || bytes.isEmpty()) return;

    m_port->write(bytes);
    m_txBytes += (quint64)bytes.size();

    QString pretty = QString::fromLatin1(bytes).trimmed();
    emit message(QStringLiteral("[UART→] %1").arg(pretty));
}

void UartLink::sendCommand(const QString &line)
{
    QByteArray payload = line.toLatin1();
    payload.append("\r\n");
    sendRaw(payload);
}

void UartLink::onReadyRead()
{
    QByteArray chunk = m_port->readAll();
    if (chunk.isEmpty()) return;

    m_rxBytes += (quint64)chunk.size();
    emit bytesReceived(chunk);

    if (m_rawMode) return;          /* 波形流: 二进制, 不做行解析 */

    m_buf.append(chunk);

    for (;;) {
        int nl = m_buf.indexOf('\n');
        if (nl < 0) break;

        QByteArray one = m_buf.left(nl);
        m_buf.remove(0, nl + 1);

        while (one.endsWith('\r')) one.chop(1);
        emit lineReceived(QString::fromUtf8(one));
    }

    /* 防止没有换行的半行把缓冲撑大 */
    if (m_buf.size() > 8192) {
        emit lineReceived(QString::fromUtf8(m_buf));
        m_buf.clear();
    }
}
