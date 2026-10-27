#include "waveform.h"

WaveformParser::WaveformParser(QObject *parent)
    : QObject(parent)
{
}

void WaveformParser::reset()
{
    m_buf.clear();
    m_frames = m_bad = m_stray = 0;
}

void WaveformParser::feed(const QByteArray &chunk)
{
    if (!chunk.isEmpty()) m_buf.append(chunk);

    const int n = m_buf.size();
    const quint8 *base = reinterpret_cast<const quint8 *>(m_buf.constData());
    int i = 0;

    while (i + FrameBytes <= n) {
        const quint8 *p = base + i;

        if (p[0] != 0xAA || p[1] != 0x55 || p[2] != 0x01) {
            ++i; ++m_stray;
            continue;
        }

        const qint16 iu = (qint16)(quint16)(p[3] | (p[4] << 8));
        const qint16 iv = (qint16)(quint16)(p[5] | (p[6] << 8));
        const qint16 iw = (qint16)(quint16)(p[7] | (p[8] << 8));

        if (qAbs((int)iu + (int)iv + (int)iw) > SumTol) {
            ++i; ++m_bad;
            continue;
        }

        emit frame(iu / 1000.0, iv / 1000.0, iw / 1000.0);
        ++m_frames;
        i += FrameBytes;
    }

    if (i > 0) m_buf.remove(0, i);

    /* 病态输入(比如一直没同步上)别把缓冲撑爆 */
    if (m_buf.size() > 4096) {
        m_stray += (quint64)m_buf.size();
        m_buf.clear();
    }
}
