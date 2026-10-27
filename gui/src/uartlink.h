#ifndef UARTLINK_H
#define UARTLINK_H

/* L103 UART1 控制台 (WCH-Link 串口桥, 默认 COM15 @115200 8N1)。
 * 命令以回车结束 —— 固件 Cmd_Poll() 只认 \r 或 \n 才执行整行。 */

#include <QByteArray>
#include <QObject>
#include <QString>
#include <QStringList>

class QSerialPort;

class UartLink : public QObject
{
    Q_OBJECT
public:
    explicit UartLink(QObject *parent = nullptr);
    ~UartLink() override;

    static QStringList availablePorts();          /* "COM15 | WCH-Link SERIAL" */

    bool    open(const QString &portName, int baud, QString *err = nullptr);
    void    close();
    bool    isOpen() const;
    QString portName() const { return m_portName; }
    int     baud() const { return m_baud; }

    /* 波形流模式: 打开后 onReadyRead 不再按行切分 (二进制里到处是假换行),
     * 只发 bytesReceived 给波形解析器。控制台文本回复就看不到日志了, 这是有意的。 */
    void setRawMode(bool on) { m_rawMode = on; m_buf.clear(); }
    bool rawMode() const { return m_rawMode; }

    void sendRaw(const QByteArray &bytes);
    void sendCommand(const QString &line);        /* 自动补 \r\n */

    quint64 rxBytes() const { return m_rxBytes; }
    quint64 txBytes() const { return m_txBytes; }

signals:
    void opened(const QString &portName);
    void closed();
    void bytesReceived(const QByteArray &raw);
    void lineReceived(const QString &line);
    void message(const QString &text);            /* 给日志窗口 */

private slots:
    void onReadyRead();

private:
    QSerialPort *m_port = nullptr;
    QByteArray   m_buf;
    QString      m_portName;
    int          m_baud = 115200;
    quint64      m_rxBytes = 0;
    quint64      m_txBytes = 0;
    bool         m_rawMode = false;
};

#endif /* UARTLINK_H */
