#include "waveplot.h"

#include <QMouseEvent>
#include <QTimer>
#include <QPainter>
#include <QPainterPath>
#include <QtMath>

static const QColor kColU(255, 214, 102);   /* 琥珀 */
static const QColor kColV( 94, 232, 152);   /* 绿   */
static const QColor kColW( 97, 175, 255);   /* 蓝   */
static const QColor kColPsu(214, 122, 255); /* 紫 (预留) */

WavePlot::WavePlot(QWidget *parent)
    : QWidget(parent)
{
    setMinimumHeight(220);
    setAutoFillBackground(false);
    setWindowPoints(m_cap);

    /* 33ms = 30 帧/s。数据进来只置 dirty, 画面按这个节奏刷。 */
    m_repaintTimer = new QTimer(this);
    m_repaintTimer->setInterval(33);
    connect(m_repaintTimer, &QTimer::timeout, this, &WavePlot::flushRepaint);
    m_repaintTimer->start();
}

void WavePlot::flushRepaint()
{
    if (!m_dirty) return;
    m_dirty = false;
    update();
}

void WavePlot::setWindowPoints(int n)
{
    if (n < 50)    n = 50;
    if (n > 20000) n = 20000;

    m_cap = n;
    m_u.fill(0.0f, m_cap);
    m_v.fill(0.0f, m_cap);
    m_w.fill(0.0f, m_cap);
    m_vb.fill(0.0f, m_cap);
    m_ib.fill(0.0f, m_cap);
    m_head = 0;
    m_filled = 0;
    m_scale = 0.5f;
    resetPeaks();
    update();
}

void WavePlot::resetPeaks()
{
    m_pkU = m_pkV = m_pkW = 0.0f;
}

void WavePlot::clear()
{
    m_u.fill(0.0f, m_cap);
    m_v.fill(0.0f, m_cap);
    m_w.fill(0.0f, m_cap);
    m_vb.fill(0.0f, m_cap);
    m_ib.fill(0.0f, m_cap);
    m_head = 0;
    m_filled = 0;
    m_psuSeen = false;
    m_scale = 0.5f;
    resetPeaks();
    update();
}

void WavePlot::append(double iU, double iV, double iW)
{
    if (m_paused) return;

    m_u[m_head] = (float)iU;
    m_v[m_head] = (float)iV;
    m_w[m_head] = (float)iW;

    if (m_peakhold) {
        const float au = qAbs((float)iU), av = qAbs((float)iV), aw = qAbs((float)iW);
        if (au > m_pkU) m_pkU = au;
        if (av > m_pkV) m_pkV = av;
        if (aw > m_pkW) m_pkW = aw;
    }

    m_head = (m_head + 1) % m_cap;
    if (m_filled < m_cap) ++m_filled;

    /* 自动量程在这里更新, 不放在 paintEvent 里 ——
     * 放 paintEvent 就成了"按重绘次数收敛", 窗口被遮挡/离屏时重绘少, 量程会一直卡着不动。
     * 涨: 立刻跟上; 落: 每采样点收 0.5%, 约 200 点 (0.3 s @700 帧/s) 收敛。 */
    float a = 0.0f;
    if (m_showU) a = qMax(a, qAbs((float)iU));
    if (m_showV) a = qMax(a, qAbs((float)iV));
    if (m_showW) a = qMax(a, qAbs((float)iW));
    {
        const float want = qMax(0.5f, a * 1.15f);
        if (want > m_scale) m_scale = want;
        else                m_scale += (want - m_scale) * 0.005f;
    }

    m_dirty = true;      /* 不直接 update(): 交给 33ms 的节流定时器 */
}

void WavePlot::appendPsu(double vbusV, double ibusA)
{
    if (m_paused) return;

    m_vb[m_head] = (float)vbusV;
    m_ib[m_head] = (float)ibusA;
    m_psuSeen = true;
}

void WavePlot::mouseDoubleClickEvent(QMouseEvent *e)
{
    Q_UNUSED(e);
    resetPeaks();
    update();
}

void WavePlot::paintEvent(QPaintEvent *e)
{
    Q_UNUSED(e);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    const QRectF r = rect().adjusted(52, 10, -104, -22);

    p.fillRect(rect(), QColor(24, 26, 30));

    /* 量程已经在 append() 里跟采样点收敛过了, 这里只负责画 */
    const float sc = m_scale;
    const auto yOf = [&](float v) { return r.center().y() - (double)v / sc * (r.height() / 2.0); };

    /* ---- 网格 ---- */
    p.setPen(QPen(QColor(58, 62, 70), 1));
    for (int g = -3; g <= 3; ++g) {
        if (g == 0) continue;
        const double y = r.center().y() - (double)g / 3.0 * (r.height() / 2.0);
        p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
    }
    p.setPen(QPen(QColor(110, 118, 130), 1, Qt::DashLine));
    p.drawLine(QPointF(r.left(), r.center().y()), QPointF(r.right(), r.center().y()));
    p.setPen(QPen(QColor(70, 76, 86), 1));
    p.drawRect(r);

    /* 纵轴刻度 */
    p.setPen(QColor(160, 168, 180));
    QFont f = font(); f.setPointSizeF(qMax(7.0, f.pointSizeF() - 1.0)); p.setFont(f);
    for (int g = -1; g <= 1; ++g) {
        if (g == 0) continue;
        const double y = r.center().y() - (double)g * (r.height() / 2.0);
        p.drawText(QRectF(0, y - 8, r.left() - 6, 16),
                   Qt::AlignRight | Qt::AlignVCenter,
                   QStringLiteral("%1").arg(g * sc, 0, 'f', 1));
    }
    p.drawText(QRectF(0, r.center().y() - 8, r.left() - 6, 16),
               Qt::AlignRight | Qt::AlignVCenter, QStringLiteral("0"));
    p.drawText(QRectF(r.left() - 30, r.top() - 8, 60, 14), Qt::AlignHCenter, QStringLiteral("A"));

    /* ---- 曲线 ---- */
    if (m_filled >= 2) {
        struct Line { const QVector<float> *d; QColor c; bool on; };
        const Line lines[3] = {
            { &m_u, kColU, m_showU },
            { &m_v, kColV, m_showV },
            { &m_w, kColW, m_showW },
        };

        for (int li = 0; li < 3; ++li) {
            if (!lines[li].on) continue;

            const QVector<float> &d = *lines[li].d;
            QPainterPath path;

            const int start = (m_head - m_filled + m_cap * 2) % m_cap;
            for (int k = 0; k < m_filled; ++k) {
                const int idx = (start + k) % m_cap;
                const double x = r.left() + r.width() * (double)k / (double)(m_filled - 1);
                const double y = yOf(d[idx]);
                if (k == 0) path.moveTo(x, y);
                else        path.lineTo(x, y);
            }
            p.setPen(QPen(lines[li].c, 1.4));
            p.setBrush(Qt::NoBrush);
            p.drawPath(path);
        }
    } else {
        p.setPen(QColor(120, 128, 140));
        p.drawText(r, Qt::AlignCenter, QStringLiteral("还没有波形 —— 在下面点\"开始波形\""));
    }

    /* ---- 图例 / 读数 ---- */
    const QRectF lg(r.right() + 8, r.top(), width() - r.right() - 12, r.height());
    int ly = (int)lg.top() + 4;

    const auto legend = [&](const QColor &c, const QString &name, float pk, bool on) {
        p.setPen(on ? c : QColor(90, 94, 100));
        p.setBrush(on ? c : QColor(90, 94, 100));
        p.drawRect(QRectF(lg.left(), ly + 4, 10, 3));
        p.setPen(on ? QColor(220, 226, 236) : QColor(100, 104, 112));
        p.drawText(QRectF(lg.left() + 15, ly - 3, lg.width() - 15, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("%1 %2A").arg(name).arg((double)pk, 0, 'f', 2));
        ly += 18;
    };

    legend(kColU, QStringLiteral("iU"), m_pkU, m_showU);
    legend(kColV, QStringLiteral("iV"), m_pkV, m_showV);
    legend(kColW, QStringLiteral("iW"), m_pkW, m_showW);

    ly += 6;
    p.setPen(m_psuSeen ? kColPsu : QColor(100, 104, 112));
    p.drawText(QRectF(lg.left(), ly, lg.width(), 16), Qt::AlignLeft | Qt::AlignVCenter,
               m_psuSeen ? QStringLiteral("母线 已接") : QStringLiteral("母线 未接"));
    ly += 16;
    p.setPen(QColor(110, 116, 126));
    p.drawText(QRectF(lg.left(), ly, lg.width(), 30), Qt::TextWordWrap,
               QStringLiteral("(下一版 PCB)"));

    if (m_paused) {
        p.setPen(QColor(255, 120, 120));
        p.drawText(QRectF(r.left(), r.top() + 4, r.width(), 18), Qt::AlignHCenter,
                   QStringLiteral("已暂停 (双击曲线可清峰值)"));
    }
}
