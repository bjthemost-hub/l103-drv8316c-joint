#ifndef WAVEPLOT_H
#define WAVEPLOT_H

/* 相电流实时曲线 (自绘, 不依赖 QtCharts —— 本机 Qt 装的是 5.12.0 的精简版, 没有 charts 模块)
 *
 * 环形缓冲: 固定 m_cap 个点, 满了覆盖最老的; 画的时候只遍历窗口内的点, 不搬内存。
 * 一次 append = 三次写数组, 不做任何分配 —— 730 帧/s 下毫无压力。
 *
 * 重绘节流: 数据是 700+ 帧/s, 但画面只需要 30 帧/s。append() 只置 dirty,
 * 由一个 33ms 的定时器去 update() —— 否则事件循环会被逼着按采样率重绘,
 * 定时器都被拖晚 (实测离屏下拖到 230ms)。
 */

#include <QVector>
#include <QWidget>

class QTimer;

class WavePlot : public QWidget
{
    Q_OBJECT
public:
    explicit WavePlot(QWidget *parent = nullptr);

    void append(double iU, double iV, double iW);
    void clear();

    void setPaused(bool p) { m_paused = p; }
    bool isPaused() const  { return m_paused; }

    void setWindowPoints(int n);
    int  windowPoints() const { return m_cap; }

    void setShowU(bool b) { m_showU = b; update(); }
    void setShowV(bool b) { m_showV = b; update(); }
    void setShowW(bool b) { m_showW = b; update(); }
    void setPeakHold(bool b) { m_peakhold = b; if (!b) resetPeaks(); update(); }

    /* 预留: 下一版 PCB 有了母线分压/分流再接上。现在没人调, 图例里标"未接"。 */
    void appendPsu(double vbusV, double ibusA);
    void setShowPsu(bool b) { m_showPsu = b; update(); }
    bool psuSeen() const { return m_psuSeen; }

signals:
    void infoChanged(const QString &text);

protected:
    void paintEvent(QPaintEvent *e) override;
    void mouseDoubleClickEvent(QMouseEvent *e) override;

private:
    void resetPeaks();
    void flushRepaint();

    QVector<float> m_u, m_v, m_w;          /* 环形 */
    QVector<float> m_vb, m_ib;             /* 预留: 母线电压/电流 */
    int  m_cap    = 3000;
    int  m_head   = 0;                     /* 下一个写位置 */
    int  m_filled = 0;

    bool m_paused = false;
    bool m_showU = true, m_showV = true, m_showW = true, m_showPsu = false;
    bool m_peakhold = true;
    bool m_dirty    = false;
    QTimer *m_repaintTimer = nullptr;
    bool m_psuSeen  = false;

    float m_pkU = 0.0f, m_pkV = 0.0f, m_pkW = 0.0f;
    float m_scale = 0.5f;                  /* Y 轴半量程 (A), 只做显示 */
};

#endif /* WAVEPLOT_H */
