#ifndef WAVEFORM_H
#define WAVEFORM_H

/* L103 相电流波形流解析 —— 对应固件 User\wave.c
 *
 * 帧格式 (小端):
 *   off 0  0xAA 0x55            帧头
 *   off 2  0x01                 类型: 三相相电流
 *   off 3  iU int16   0.001 A/LSB
 *   off 5  iV int16
 *   off 7  iW int16
 *   共 9 字节
 *
 * 校验: 固件里 iU = -(iV+iW) 反推出来的, 所以帧内 iU+iV+iW 恒等于 0 (浮点取整误差级)。
 *       用它排除"数据里恰好出现 AA 55 01"的误同步 —— 误同步的帧几乎不可能满足这条。
 *
 * 为什么是 9 字节而不是更短: 帧头 2 B 是必需的 (串口会丢字节, 必须能重新同步);
 * 带上 iU 而不是让上位机自己算, 是因为上位机算的话每帧多一次判断, 不值。
 */

#include <QByteArray>
#include <QObject>

class WaveformParser : public QObject
{
    Q_OBJECT
public:
    static const int FrameBytes = 9;
    static const int SumTol     = 40;      /* 三相和容差, 0.001A/LSB -> 40 = 0.04 A */

    explicit WaveformParser(QObject *parent = nullptr);

    void feed(const QByteArray &chunk);
    void reset();

    quint64 frames() const     { return m_frames; }
    quint64 badFrames() const  { return m_bad; }     /* 校验没过, 判定为误同步 */
    quint64 strayBytes() const { return m_stray; }   /* 被当杂散跳过的字节 */

signals:
    void frame(double iU, double iV, double iW);

private:
    QByteArray m_buf;
    quint64 m_frames = 0;
    quint64 m_bad    = 0;
    quint64 m_stray  = 0;
};

#endif /* WAVEFORM_H */
