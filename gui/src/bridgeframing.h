#ifndef BRIDGEFRAMING_H
#define BRIDGEFRAMING_H

/* H750 UART<->CAN 桥 · 帧模式 (v1) 的纯编解码, 不碰串口, 方便离线自测。
 *
 * 帧:
 *   AA 55 FLAGS ID(2 或 4 大端) DLC DATA... XOR
 *   XOR = FLAGS..DATA 全部字节异或 (不含 AA 55)
 *   FLAGS bit0 = 扩展帧, bit1 = 远程帧(无 DATA)
 *
 * 出处: H750 CAN<->UART 桥固件工程的桥接口协议 (v1)。
 */

#include <QByteArray>
#include <QVector>
#include <QtGlobal>

namespace BridgeFraming {

struct Frame {
    quint32  id       = 0;
    quint8   flags    = 0;
    QByteArray data;
    bool     extended = false;
    bool     remote   = false;
};

/* 编码一帧。data 最多 8 字节; 失败返回空 QByteArray。
 * 标准帧 id > 0x7FF 或扩展帧 id > 0x1FFFFFFF 视为失败。 */
inline QByteArray encode(quint32 id, const QByteArray &data,
                         bool extended = false, bool remote = false)
{
    if (data.size() > 8) return QByteArray();
    if (extended ? (id > 0x1FFFFFFFu) : (id > 0x7FFu)) return QByteArray();
    if (remote && !data.isEmpty()) return QByteArray();

    const int idLen = extended ? 4 : 2;
    QByteArray f;
    f.reserve(2 + 1 + idLen + 1 + data.size() + 1);
    f.append((char)0xAA);
    f.append((char)0x55);

    quint8 flags = 0;
    if (extended) flags |= 0x01;
    if (remote)   flags |= 0x02;
    f.append((char)flags);

    for (int i = idLen - 1; i >= 0; --i)
        f.append((char)(quint8)((id >> (8 * i)) & 0xFF));

    f.append((char)(quint8)data.size());
    f.append(data);

    quint8 x = 0;
    for (int i = 2; i < f.size(); ++i) x ^= (quint8)f[i];
    f.append((char)x);
    return f;
}

/* 字节流拆包器: 处理粘包/拆包 + XOR 校验 + 坏帧后重新同步 AA 55。
 * 用法: 每来一块数据调用一次 feed(), 返回本次凑齐的所有帧。 */
class Parser
{
public:
    QByteArray buffer;
    quint64    badXor      = 0;   /* XOR 不对被丢掉的候选帧 */
    quint64    badHeader   = 0;   /* 同步/DLC 不合法被丢掉的字节 */
    quint64    droppedByte = 0;   /* 找同步时跳过的字节 */

    void reset()
    {
        buffer.clear();
        badXor = 0;
        badHeader = 0;
        droppedByte = 0;
    }

    QVector<Frame> feed(const QByteArray &chunk)
    {
        QVector<Frame> out;
        if (!chunk.isEmpty()) buffer.append(chunk);

        for (;;) {
            const int sync = buffer.indexOf(QByteArray("\xAA\x55", 2));
            if (sync < 0) {
                /* 末尾单独一个 0xAA 可能是下一帧的前半, 留住它。 */
                if (!buffer.isEmpty() && (quint8)buffer.at(buffer.size() - 1) == 0xAA) {
                    if (buffer.size() > 1) {
                        droppedByte += (quint64)(buffer.size() - 1);
                        buffer = buffer.right(1);
                    }
                } else {
                    droppedByte += (quint64)buffer.size();
                    buffer.clear();
                }
                return out;
            }
            if (sync > 0) {
                droppedByte += (quint64)sync;
                buffer.remove(0, sync);
            }

            if (buffer.size() < 3) return out;
            const quint8 flags = (quint8)buffer.at(2);
            const bool extended = (flags & 0x01) != 0;
            const bool remote   = (flags & 0x02) != 0;
            const int  idLen    = extended ? 4 : 2;
            const int  fixed    = 2 + 1 + idLen + 1;   /* AA55 + FLAGS + ID + DLC */

            if (buffer.size() < fixed) return out;

            quint32 id = 0;
            for (int i = 0; i < idLen; ++i)
                id = (id << 8) | (quint8)buffer.at(3 + i);

            const int dlc = (quint8)buffer.at(3 + idLen);
            if (dlc > 8) {
                /* DLC 不合法: 只丢一个字节, 让后面的 AA 55 重新对齐。 */
                ++badHeader;
                buffer.remove(0, 1);
                continue;
            }

            const int payload = remote ? 0 : dlc;   /* 远程帧无 DATA */
            const int total   = fixed + payload + 1;
            if (buffer.size() < total) return out;

            quint8 x = 0;
            for (int i = 2; i < fixed + payload; ++i) x ^= (quint8)buffer.at(i);
            const quint8 rx = (quint8)buffer.at(fixed + payload);
            if (x != rx) {
                ++badXor;
                buffer.remove(0, 1);
                continue;
            }

            Frame f;
            f.id = id;
            f.flags = flags;
            f.extended = extended;
            f.remote = remote;
            if (payload > 0) f.data = buffer.mid(fixed, payload);
            buffer.remove(0, total);
            out.append(f);
        }
    }
};

} /* namespace BridgeFraming */

#endif /* BRIDGEFRAMING_H */
