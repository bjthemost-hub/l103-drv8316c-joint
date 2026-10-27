#include "mainwindow.h"

#include <QScrollArea>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QRegularExpression>
#include <QSet>
#include <QSpinBox>
#include <QSplitter>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include "canlink.h"
#include "bridgelink.h"
#include "uartlink.h"
#include "waveform.h"
#include "waveplot.h"

using namespace L103;

/* 早先只有一台电机 (节点 5) 时写死的号。
 * 桥页现在跟着顶部「目标设备」走; 这个常量只留给「序列(桥)」页"节点列留空 = 5"的默认值。 */
static const int BRIDGE_MOTOR_NODE = 5;

/* 参数表的名字/单位/说明按 CAN 索引查, 不是按行位置。
 * 前 27 项与固件 motor_cfg.c 的 cfg_name[]/cfg_unit[] 一字不差;
 * 27..34 是 2026-10-23 固件新增的 8 项。表里到底有几行由板子 INFO 回报的
 * 参数个数决定 (见 resizeParamTable); 这里只是"已知名字"的字典,
 * 查不到就显示 "idx NN", 不会因为固件加项而错位。 */
static const char *const kParamName[PARAM_MAX] = {
    "pole_pairs", "phase_r_ohm", "ke_v_per_rad_s", "iq_limit_a",
    "iq_ramp_a_per_s", "vel_ref_accel", "vel_filter_alpha", "voltage_align",
    "voltage_limit", "velocity_limit", "velocity_limit_max", "stall_amp",
    "stall_win_ms", "stall_wins", "pid_vel_p", "pid_vel_i",
    "pid_ang_p", "pid_ang_d", "pv_decel", "pv_arrive",
    "pv_vlim_min", "ol_v_per_rad_s", "ol_v_boost", "ol_v_align",
    "ol_accel", "voltage_supply", "elec_angle_trim",
    /* 27..34: 2026-10-23 新增 */
    "pll_bw_hz", "cur_kp", "cur_ki", "ff_bias_a",
    "dith_amp_a", "dith_hz", "kick_max_a", "kick_rate_aps",
    /* 35..54: 2026-10-23 启动曲线 + 分段速度环 */
    "start_a_min", "start_a_k_rpm", "start_a_max", "start_accel",
    "band1_kp", "band1_ki", "band2_kp", "band2_ki",
    "band3_kp", "band3_ki", "band4_kp", "band4_ki",
    "band5_kp", "band5_ki", "band6_kp", "band6_ki",
    "band7_kp", "band7_ki", "band8_kp", "band8_ki"
};
static const char *const kParamUnit[PARAM_MAX] = {
    "", "ohm", "V/(rad/s)", "A",
    "A/s", "rad/s^2", "", "V",
    "V", "rad/s", "rad/s", "A",
    "ms", "", "A/(rad/s)", "A/rad",
    "(rad/s)/rad", "(rad/s)/(rad/s)", "rad/s^2", "rad",
    "rad/s", "V/(rad/s)", "V", "V",
    "rad/s^2", "V", "rad",
    "Hz", "V/A", "V/(A·s)", "A",
    "A", "Hz", "A", "A/s",
    "A", "A/rpm", "A", "rad/s^2",
    "A/(rad/s)", "A/rad", "A/(rad/s)", "A/rad",
    "A/(rad/s)", "A/rad", "A/(rad/s)", "A/rad",
    "A/(rad/s)", "A/rad", "A/(rad/s)", "A/rad",
    "A/(rad/s)", "A/rad", "A/(rad/s)", "A/rad"
};
/* 参数表第 5 列的中文说明: 只给人看, 不参与任何收发 */
static const char *const kParamDesc[PARAM_MAX] = {
    "极对数(14 对极就是 14)",
    "相电阻, 电流环按它把 iq 折成电压",
    "反电势常数, 由 Ke 推转矩常数",
    "速度环输出(iq 给定)的上限",
    "iq 给定斜坡, 越大加速越猛",
    "转速给定斜坡, 越大起步越快",
    "速度低通 0~0.999, 越大越平滑也越滞后",
    "对齐时吸住转子的电压",
    "相电压限幅, 放开才出得了大力矩",
    "角度环输出的速度钳位",
    "运行时允许设到的最高转速",
    "堵转/过流阈值(相电流幅值)",
    "堵转判定的滑动窗口长度",
    "连续几个窗口超限才算堵转",
    "速度环 P, 输出是 iq 给定",
    "速度环 I, 消稳态误差",
    "位置环 P, 输出是速度给定",
    "位置环 D, 抑制超调",
    "PV 轨迹的减速度",
    "到位判据(离目标多近算到)",
    "给定速度 0 时的兜底速度上限",
    "开环 V/f 斜率, 每 rad/s 给多少伏",
    "开环低速 IR 补偿电压",
    "开环起步前的对位电压",
    "开环爬频加速度",
    "母线电压标称值(没 VM 采样时用它归一化)",
    "电角度微调, 补偿编码器安装偏差",
    /* 27..34: 2026-10-23 新增 */
    "编码器 PLL 带宽, 越大跟得越快也越抖",
    "电流环 Kp (电流误差 -> 相电压)",
    "电流环 Ki (消稳态误差)",
    "摩擦前馈电流偏置",
    "dither 抖动幅值 (压静摩擦)",
    "dither 抖动频率",
    "stiction kick 电流上限 (0 = 关)",
    "stiction kick 电流上升速率",
    /* 35..54: 2026-10-23 */
    "启动电流地板 A (必须高于脱困门槛)",
    "启动电流随转速增量 A/rpm",
    "启动电流上限 A",
    "转子没动时的给定爬升 (越小越柔)",
    "段1 <5 rad/s 速度环 P", "段1 <5 rad/s 速度环 I",
    "段2 5~10 速度环 P", "段2 5~10 速度环 I",
    "段3 10~20 速度环 P", "段3 10~20 速度环 I",
    "段4 20~40 速度环 P", "段4 20~40 速度环 I",
    "段5 40~70 速度环 P", "段5 40~70 速度环 I",
    "段6 70~100 速度环 P", "段6 70~100 速度环 I",
    "段7 100~140 速度环 P", "段7 100~140 速度环 I",
    "段8 >=140 速度环 P", "段8 >=140 速度环 I"
};

/* ---- 板型 1: CH32L103 + DRV8316C, 18 项 ----
 * 索引与固件 can_bus.c param_get()/param_set() 一一对应。
 * 2026-10-23: 以前 L103 也套上面那张 V303 的 27 项表, 从索引 1 起全部错位 ——
 * 写着 vel_ref_accel 的其实是位置环 kp, 写着 velocity_limit 的其实是加速度。
 * 板型不同 = 参数表不同 = Flash 记录布局不同, 必须分开取名字。 */
static const char *const kParamNameL103[18] = {
    "pole_pairs", "iq_limit_a", "vbus_v", "spd_kp",
    "spd_ki", "pos_kp", "cur_kp", "cur_ki",
    "vel_limit", "accel", "v_align", "pll_bw_hz",
    "vel_src", "r_ohm", "l_mh", "node_id", "oc_trip_a",
    "oc_trip_ms"
};
static const char *const kParamUnitL103[18] = {
    "", "A", "V", "",
    "", "", "V/A", "V/(A*s)",
    "rad/s", "rad/s^2", "V", "Hz",
    "", "ohm", "mH", "", "A", "ms"
};
static const char *const kParamDescL103[18] = {
    "极对数(改完必须重跑对齐)",
    "相电流上限",
    "母线电压(存 Flash 时按当时实测值存)",
    "速度环 kp",
    "速度环 ki",
    "位置环 kp",
    "电流环 kp",
    "电流环 ki",
    "速度上限",
    "加速度上限",
    "对齐电压",
    "编码器 PLL 带宽",
    "测速源 0=自动 1=PLL 2=窗口(易跑飞)",
    "相电阻",
    "相电感",
    "节点号(只读, 用 % 或功能号 12 改)",
    "相电流立即跳闸阈值(超了直接关桥, 不动机构)",
    "过流持续多久才跳闸; 0=一拍就跳 (瞬时尖峰不误跳靠它)"
};

/* ---- 板型 2: CH32L103 + DRV8313 (V2), 18 项 ----
 * 索引顺序与板型 1 **完全相同** (固件 can_bus.c param_get):
 *   pp / ilim / vbus / spd_kp / spd_ki / pos_kp / cur_kp / cur_ki /
 *   vel_lim / accel / align_v / pll_bw / vel_src / r_ohm / l_mh / node / oc_trip_a / oc_trip_ms
 * 本板只有一路低边总电流, 没有三路相电流 —— 文案按 i_ls 写, 不要写成 ia/ib/ic。 */
static const char *const kParamNameL103V2[18] = {
    "pole_pairs", "iq_limit_a", "vbus_v", "spd_kp",
    "spd_ki", "pos_kp", "cur_kp", "cur_ki",
    "vel_limit", "accel", "v_align", "pll_bw_hz",
    "vel_src", "r_ohm", "l_mh", "node_id",
    "oc_trip_a", "oc_trip_ms"
};
static const char *const kParamUnitL103V2[18] = {
    "", "A", "V", "",
    "", "", "V/A", "V/(A*s)",
    "rad/s", "rad/s^2", "V", "Hz",
    "", "ohm", "mH", "",
    "A", "ms"
};
static const char *const kParamDescL103V2[18] = {
    "极对数(改完必须重跑对齐)",
    "低边总电流上限(本板只有一路 i_ls)",
    "母线电压(存 Flash 时按当时实测值存)",
    "速度环 kp",
    "速度环 ki",
    "位置环 kp",
    "电流环 kp",
    "电流环 ki",
    "速度上限",
    "加速度上限",
    "对齐电压",
    "编码器 PLL 带宽",
    "测速源 0=自动 1=PLL 2=窗口(易跑飞)",
    "相电阻",
    "相电感",
    "节点号(只读, 用 % 或功能号 12 改)",
    "低边总电流立即跳闸阈值(超了直接关桥, 不动机构)",
    "过流持续多久才跳闸; 0=一拍就跳 (瞬时尖峰不误跳靠它)"
};

/* 当前参数表按哪个板型取名字。0 = 还没读到板型, 退回 V303 老表。 */
static quint8 g_paramBoardId = 0;

/* 板型 1 = L103 + DRV8316C; 板型 2 = L103 + DRV8313 (V2)。
 * 两块板**索引相同 (都是固件 param_get 的 0..17)**, 只有名字/单位/描述的文案不同:
 * 板型 2 只有一路低边总电流 i_ls。拿错表 = 数字对、标签错。 */
static bool boardIsL103(quint8 id)      { return id == 1u || id == 2u; }
static bool boardHas3PhaseAmp(quint8 id){ return id != 2u; }   /* 板型 2 只有一路低边总电流 i_ls */
static bool paramIsL103()               { return boardIsL103(g_paramBoardId); }
static bool paramIsL103V2()             { return g_paramBoardId == 2u; }

/* L103 调参页: 逻辑项 -> 参数索引。0 = 板型未知时沿用板型 1 的老索引 (保持原行为)。 */
struct L103ParamIdx {
    int node, pp, ilim, vbus, spdKp, spdKi, posKp, curKp, curKi;
    int vlim, accel, alignV, pll, vs, r, l, oc, ocMs;
};
static L103ParamIdx l103ParamIdx(quint8 boardId)
{
    /* 板型 1 / 板型 2 的 CAN 参数索引**完全相同** (固件 can_bus.c param_get, 2026-10-24 核对):
     * 0 极对数 … 15 节点号 / 16 跳闸阈值A / 17 跳闸延时ms。
     * 板型 2 变的是名字表的文案 (只有一路低边 i_ls), 不是索引 —— 别把 node 挪到第 0 位。 */
    (void)boardId;
    return L103ParamIdx{15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17};
}

/* 已知名字有几项 (数组里第一个空位)。这只是没连板子时的初始行数;
 * 连上以后以板子 INFO 回报的个数为准。 */
static int paramKnownCount()
{
    int n = 0;
    while (n < PARAM_MAX && kParamName[n]) ++n;
    return n;
}
static QString paramNameText(int idx)
{
    const char *const *names = paramIsL103V2() ? kParamNameL103V2 : kParamNameL103;
    if (paramIsL103() && idx >= 0 && idx < 18 && names[idx])
        return QString::fromLatin1(names[idx]);
    if (idx >= 0 && idx < PARAM_MAX && kParamName[idx])
        return QString::fromLatin1(kParamName[idx]);
    return QStringLiteral("idx %1").arg(idx);
}
static QString paramUnitText(int idx)
{
    const char *const *units = paramIsL103V2() ? kParamUnitL103V2 : kParamUnitL103;
    if (paramIsL103() && idx >= 0 && idx < 18 && units[idx])
        return QString::fromLatin1(units[idx]);
    if (idx >= 0 && idx < PARAM_MAX && kParamUnit[idx])
        return QString::fromLatin1(kParamUnit[idx]);
    return QString();
}
static QString paramDescText(int idx)
{
    const char *const *desc = paramIsL103V2() ? kParamDescL103V2 : kParamDescL103;
    if (paramIsL103() && idx >= 0 && idx < 18 && desc[idx])
        return QString::fromUtf8(desc[idx]);
    if (idx >= 0 && idx < PARAM_MAX && kParamDesc[idx])
        return QString::fromUtf8(kParamDesc[idx]);
    return QString();
}

/* ============================== 构造 / 析构 ============================== */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    m_uart = new UartLink(this);
    m_can  = new CanLink(this);
    m_bridge = new BridgeLink(this);

    /* 必须在 buildUi() 之前建好 —— 状态页在构造的时候就会 start() 它 */
    m_timerStatus = new QTimer(this);
    m_timerStatus->setInterval(1000);
    connect(m_timerStatus, &QTimer::timeout, this, &MainWindow::refreshStatus);

    /* 波形: 解析器是纯逻辑对象, 曲线控件在 makeWaveTab() 里建 */
    m_waveParser = new WaveformParser(this);

    m_timerWaveStat = new QTimer(this);
    m_timerWaveStat->setInterval(1000);
    connect(m_timerWaveStat, &QTimer::timeout, this, &MainWindow::waveStatTick);

    buildUi();
    wireLinks();
    refreshPortLists();

    log(QStringLiteral("L103 关节电机调试上位机已启动。"));
    log(QStringLiteral("CAN 协议 (2026-10-16 起): 多节点, ID = (节点号 << 5) | 功能号; 节点 0 = 广播, 1..63 = 单台。"));
    log(QStringLiteral("打开 CAN 后点「扫描节点」认设备。老固件 (固定 ID 0x1xx) 与新固件不通用。"));
    log(QStringLiteral("命令语义出处: 04_工程\\CH32L103_DRV8316C_FOC_CAN\\命令表_CAN与UART_2026-10-16.md"));
    log(QStringLiteral("CAN DLL: %1").arg(CanLink::dllPathTried()));
    log(QStringLiteral("桥接口: H750 UART<->CAN 桥, 921600 8N1, AA 55 帧 (需桥固件 FRAMED=1)。"));
}

MainWindow::~MainWindow() = default;

static QLabel *mkLabel(const QString &t)
{
    QLabel *l = new QLabel(t);
    /* 带换行的都是成段提示。不打开自动换行的话, 一条长文本会把窗口的
     * 最小宽度顶到 1500+ px (实测: 日志页那条 ID 说明 → 1479 px), 窗口再也缩不小。 */
    if (t.contains(QLatin1Char(10))) l->setWordWrap(true);   /* 10 = '\n' */
    return l;
}

/* ============================== 设备表 (多节点, 2026-10-16) ==============================
 * 总线上的 ID = (节点号 << 5) | 功能号, 所以**一路 CAN 上能挂多台**, 每台一个号 (1..63)。
 * 每台一份状态存在 m_devs 里; 界面上的 m_st/m_cfg/... 只是「目标设备」选中那台的副本。
 * 出处: can_bus.h (CAN_ID_MAKE / CAN_FN_* / 功能号 12 设节点号)。
 */
static qint64 nowMs()
{
    return QDateTime::currentMSecsSinceEpoch();
}

/* 在线判定: 5 s 没收到它的任何帧就算离线。轮询开着时(每秒问一遍)这个阈值很宽松;
 * 把轮询关掉, 设备会统统变离线 —— 它表示"我多久没听到你", 不是"你坏了"。 */
static const qint64 DEV_ONLINE_MS = 5000;

int MainWindow::targetNode() const
{
    return m_spNode ? m_spNode->value() : NODE_MIN;
}

/* 「L103 调参(桥)」等桥专用控件用哪一台: 就是顶部「目标设备」。
 * 广播 (0) 时没有"某一台"可调, 退回 5 (原来那台)。 */
int MainWindow::bridgeNode() const
{
    const int n = targetNode();
    return (n >= NODE_MIN) ? n : BRIDGE_MOTOR_NODE;
}

bool MainWindow::targetIsBroadcast() const
{
    return targetNode() == NODE_BCAST;
}

/* 当前这台板到底是哪个板型。优先用目标节点回报过的那一帧 (功能22);
 * 还没回报时退回参数表记下的板型 (g_paramBoardId), 再没有就是 0 = 未知。 */
quint8 MainWindow::currentBoardId() const
{
    const int nd = targetNode();
    if (nd >= NODE_MIN) {
        const QMap<int, DeviceState>::const_iterator it = m_devs.constFind(nd);
        if (it != m_devs.constEnd() && it->info.valid)
            return it->info.boardId;
    }
    return g_paramBoardId;
}

/* 板型变了要跟着变的东西: 电流的叫法 (三相 vs 单路低边总电流)、波形页能不能用。
 * 板型 2 (L103 + DRV8313 V2) 固件已经删掉 SW 相电流波形流, 这里必须把入口关掉 ——
 * 否则点了"开始波形"会发 SWD/SW1, 板子不认, 还可能把串口留在二进制模式。 */
void MainWindow::updateBoardUi()
{
    const quint8 bid = currentBoardId();
    if (bid != g_paramBoardId) {
        /* 板型换了 = 名字表要换。resizeParamTable() 在"行数没变"时会提前返回,
         * 所以先把 m_paramCount 打成 -1 逼它重建 —— 否则 18 项的 8316C 表会顶着
         * DRV8313 的名字继续显示 (2026-10-24 自测用例 22 实测过)。 */
        const int keep = m_paramCount;
        g_paramBoardId = bid;
        if (m_tblParam && keep >= 0) {
            m_paramCount = -1;
            resizeParamTable(keep);              /* 名字表跟着板型换, 项数不变 */
        }
    }

    const bool v2   = (bid == 2u);
    const bool has3 = boardHas3PhaseAmp(bid);

    if (m_labAmpCaption) {
        m_labAmpCaption->setText(v2 ? QStringLiteral("低边总电流 i_ls")
                                    : QStringLiteral("相电流"));
        m_labAmpCaption->setToolTip(v2
            ? QStringLiteral("本板 (L103 + DRV8313 V2) 只有一路低边总电流 i_ls, 没有三相 ia/ib/ic。")
            : QString());
    }

    if (m_labWaveBoardNote) {
        m_labWaveBoardNote->setVisible(v2);
        if (v2)
            m_labWaveBoardNote->setText(QStringLiteral(
                "本板 (L103 + DRV8313 V2) 没有三相电流采样, 只有一路低边总电流 i_ls;\n"
                "固件已取消 SW 相电流波形流, 本页不可用。i_ls 见「状态」页和设备表。"));
    }

    if (m_btnWaveStart) m_btnWaveStart->setEnabled(!v2 && !m_waveStreaming);
    if (m_cbWaveDiv)    m_cbWaveDiv->setEnabled(!v2);
    if (m_chkShowU)     m_chkShowU->setEnabled(has3);
    if (m_chkShowV)     m_chkShowV->setEnabled(has3);
    if (m_chkShowW)     m_chkShowW->setEnabled(has3);

    if (m_spL103Oc) {
        m_spL103Oc->setToolTip(v2
            ? QStringLiteral("低边总电流超过这个数、并且连续顶住右边那个时间, 才关桥 (固件 foc.c)。\n"
                             "本板只有一路低边总电流, 不是三相各自的相电流。")
            : QStringLiteral("相电流超过这个数、并且连续顶住右边那个时间, 才关桥 (固件 foc.c)。范围 0.5..4.5 A ——\n"
                             "上限是采样链量程, 再高就没法保护了。这个阈值只管【跳闸】, 不是电流环给定上限 (那个是上面的【电流上限】)。"));
    }

    /* 已经开着波形流时板型才被认成 2: 本地停掉, **不发 SW0** (固件没这条命令)。 */
    if (v2 && m_waveStreaming) {
        m_waveStreaming = false;
        if (m_timerWaveStat) m_timerWaveStat->stop();
        if (m_uart)          m_uart->setRawMode(false);
        if (m_btnWaveStop)   m_btnWaveStop->setEnabled(false);
        if (m_wavePollPaused) {
            m_wavePollPaused = false;
            if (m_chkAutoStatus && m_chkAutoStatus->isChecked() && m_timerStatus)
                m_timerStatus->start();
        }
        log(QStringLiteral("[波形] 板型 2 没有 SW 波形流 —— 已在本地停止接收 (没有发 SW0)。"));
    }
}

QString MainWindow::nodeText(int node) const
{
    return node == NODE_BCAST ? QStringLiteral("广播(所有节点)")
                              : QStringLiteral("节点 %1").arg(node);
}

MainWindow::DeviceState &MainWindow::devEnsure(int node)
{
    DeviceState &d = m_devs[node];
    if (d.node == 0) d.node = node;
    return d;
}

void MainWindow::recomputeShaft(DeviceState &d)
{
    const double home = d.cfg.valid ? d.cfg.homeRad : 0.0;
    d.shaftRad   = d.st.mechRad + home;
    d.shaftTurns = d.shaftRad / TWO_PI;
}

void MainWindow::touchDevice(int node)
{
    if (node < NODE_MIN) return;
    DeviceState &d = devEnsure(node);
    d.frames++;
    d.lastMs = nowMs();
}

void MainWindow::syncViewFromDevices()
{
    const QMap<int, DeviceState>::const_iterator it = m_devs.constFind(m_curNode);
    if (it == m_devs.constEnd()) {
        /* 这台还没有数据 —— 视图清空, 别把上一台的读数留在屏幕上 */
        m_st  = L103::StatusFrame();
        m_cfg = L103::CfgFrame();
        m_ft  = L103::FaultFrame();
        m_mc  = L103::MotorCfgFrame();
        m_haveStatus = false;
        m_shaftRad   = 0.0;
        m_shaftTurns = 0.0;
        return;
    }
    m_st  = it->st;
    m_cfg = it->cfg;
    m_ft  = it->ft;
    m_mc  = it->mc;
    m_haveStatus = it->haveStatus;
    m_shaftRad   = it->shaftRad;
    m_shaftTurns = it->shaftTurns;
}

/* ---- 协议版本 (两套 CAN ID) ---- */
bool MainWindow::protoIsLegacy() const
{
    if (m_protoWant == ProtoLegacy) return true;
    if (m_protoWant == ProtoNew)    return false;
    return m_protoLocked == ProtoLegacy;
}

void MainWindow::lockProto(int mode, const QString &why)
{
    if (m_protoWant != ProtoAuto) return;          /* 手动选定了就不自动改 */
    if (m_protoLocked == mode)    return;
    m_protoLocked = mode;
    log(QStringLiteral("[协议] 认出来了: %1 —— %2")
        .arg(mode == ProtoLegacy
             ? QStringLiteral("旧协议 (2026-10-15 及以前固件, 固定 ID 0x110/0x113)")
             : QStringLiteral("新协议 (节点<<5 | 功能号)"))
        .arg(why));
    if (mode == ProtoLegacy)
        log(QStringLiteral("[协议] 旧固件没有 CAN 对齐 —— 点「对齐」会改走串口 'A' (要先打开 UART 控制台)"));
    refreshNodeInfo();
}

void MainWindow::onProtoChanged()
{
    if (!m_cbProto) return;
    m_protoWant   = m_cbProto->currentData().toInt();
    m_protoLocked = (m_protoWant == ProtoAuto) ? ProtoAuto : m_protoWant;
    log(QStringLiteral("[协议] 手动设为 %1")
        .arg(m_protoWant == ProtoLegacy ? QStringLiteral("旧协议 (固定 ID)")
             : m_protoWant == ProtoNew  ? QStringLiteral("新协议 (节点<<5)")
                                        : QStringLiteral("自动")));
    refreshNodeInfo();
}

/* 广播问一遍 —— 只发**只读**问询, 两套 ID 各一帧:
 *   新协议 0x05 (问状态/配置/故障/电机参数) + 0x0E (问板型/版本);
 *   旧协议 0x113 (同样的 b0=0xFF)。
 * 对面是哪一套, onCanFrame 收到哪一套的回音就锁哪一套。
 * 动作类命令不在这里双发 (原因见 sendCanFn 里的注释)。 */
void MainWindow::probeProtocol()
{
    if (!frameOpen()) return;
    sendCan(L103::canId(NODE_BCAST, (quint32)FN_QUERY), buildQuery(0xFF),
            QStringLiteral("探测: 新协议 0x05"));
    sendCan(L103::canId(NODE_BCAST, (quint32)FN_INFO), buildInfo(),
            QStringLiteral("探测: 新协议 0x0E (板型/版本)"));
    quint32 q = 0;
    if (L103::legacyIdOfFn((quint32)FN_QUERY, &q))
        sendCan(q, buildQuery(0xFF), QStringLiteral("探测: 旧协议 0x113"));

    /* 1.5 s 后还一帧没收到 -> 直接把话说清楚, 别让人对着"没反应"猜。 */
    const int base = m_canFrames;
    QTimer::singleShot(1500, this, [this, base]() {
        if (!frameOpen() || m_protoLocked != ProtoAuto) return;
        if (m_canFrames == base)
            log(QStringLiteral("[!] 1.5 s 内一帧回音都没有 —— 要么 CAN 线/收发器不通, 要么板上节点号不是 %1")
                .arg(targetNode()));
    });
}

bool MainWindow::sendCanFn(L103::CanFn fn, const QByteArray &data, const QString &note, int node)
{
    const int n = (node >= 0) ? node : targetNode();

    /* 旧协议: 换成固定 ID 发; 旧固件没有的命令明确报出来, 别静默发丢。 */
    if (protoIsLegacy()) {
        quint32 lid = 0;
        if (!L103::legacyIdOfFn((quint32)fn, &lid)) {
            log(QStringLiteral("[!] 旧协议固件没有「%1」这条 CAN 命令%2")
                .arg(canFnName((quint32)fn),
                     fn == FN_ALIGN ? QStringLiteral(" —— 对齐只能走串口 'A'") : QString()));
            return false;
        }
        return sendCan(lid, data, note);
    }

    /* ★ 动作类命令**绝不**两套 ID 双发: 旧 ID 0x100..0x1A0 在新协议里会被算成
     *   节点 8/9/12/13 的功能号 —— 总线上真有那个节点时, 会被误当成给它的命令。
     *   自动探测只靠 probeProtocol() 那几条只读问询, 谁回音就锁谁。 */
    return sendCan(L103::canId(n, (quint32)fn), data, note);
}

/* 广播不能做的操作: 会把总线上每一台都改掉 (设零点 / 写电机参数 / 改节点号)。
 * 家人 2026-10-16 要求去掉的是"确认框"; 这里留的是"这条根本发不出去"的拦路框。 */
bool MainWindow::needSingleNode(const QString &what)
{
    if (!targetIsBroadcast()) return true;
    QMessageBox::warning(this, QStringLiteral("广播不能做这个"),
        QStringLiteral("%1 是「按节点」的操作 —— 用广播会把总线上每一台都改掉。\n"
                       "请先把上面「目标设备」选成某一台节点。").arg(what));
    return false;
}

void MainWindow::uartNodeNote()
{
    if (targetIsBroadcast())
        log(QStringLiteral("[i] UART 不认节点号 —— 这条只会到串口上接线的那一台 (要按节点发请用 CAN)"));
}

void MainWindow::onNodeChanged()
{
    if (m_syncGuard) return;
    m_curNode = targetNode();
    syncViewFromDevices();
    redrawStatus();
    refreshDevTable();
    refreshNodeInfo();
    updateBoardUi();
    brL103FollowTarget();       /* 「L103 调参(桥)」页跟着切到这一台 */
}

void MainWindow::onDevRowChanged()
{
    if (m_syncGuard) return;
    const int r = m_tblDev->currentRow();
    if (r < 0) return;
    QTableWidgetItem *it = m_tblDev->item(r, 0);
    if (!it) return;
    const int node = it->data(Qt::UserRole).toInt();
    if (node < NODE_MIN) return;
    if (m_spNode->value() != node) m_spNode->setValue(node);   /* setValue 会触发 onNodeChanged */
    else                           onNodeChanged();
}

void MainWindow::scanNodes()
{
    if (!frameOpen()) {
        log(QStringLiteral("[!] 扫描节点要 CAN 或桥接口 —— UART 上没有节点号这回事(串口只会到接线那一台)"));
        return;
    }
    m_devs.clear();
    g_paramBoardId = 0;                  /* 设备表清了, 板型也回到"未知" —— 等新的 INFO */
    syncViewFromDevices();
    redrawStatus();
    refreshDevTable();
    refreshNodeInfo();
    updateBoardUi();
    log(QStringLiteral("[扫描] 清空设备表, 广播问一遍 (b0=0xFF 问状态/配置/故障/电机参数; 顺带问板型)"));
    probeProtocol();
}

void MainWindow::refreshDevTable()
{
    if (!m_tblDev) return;

    const QList<int> nodes = m_devs.keys();

    bool same = (m_tblDev->rowCount() == nodes.size());
    if (same) {
        for (int r = 0; r < nodes.size(); ++r) {
            QTableWidgetItem *it = m_tblDev->item(r, 0);
            if (!it || it->data(Qt::UserRole).toInt() != nodes[r]) { same = false; break; }
        }
    }

    m_syncGuard = true;
    if (!same) {
        m_tblDev->setRowCount(nodes.size());
        for (int r = 0; r < nodes.size(); ++r) {
            for (int c = 0; c < m_tblDev->columnCount(); ++c)
                if (!m_tblDev->item(r, c)) m_tblDev->setItem(r, c, new QTableWidgetItem);
            m_tblDev->item(r, 0)->setData(Qt::UserRole, nodes[r]);
        }
    }

    const qint64 t = nowMs();
    int online = 0;
    for (int r = 0; r < nodes.size(); ++r) {
        const DeviceState &d = m_devs[nodes[r]];
        const bool on = (d.lastMs > 0) && (t - d.lastMs < DEV_ONLINE_MS);
        if (on) online++;

        QStringList cells;
        cells << QString::number(d.node)
              << (on ? QStringLiteral("● 在线") : QStringLiteral("○ 离线"))
              << (d.haveStatus ? L103::statusText(d.st.status) : QStringLiteral("—"))
              << (d.haveStatus ? QString::number(d.st.mechTurns, 'f', 4) : QStringLiteral("—"))
              << (d.haveStatus ? QString::number(d.st.velRpm, 'f', 1)    : QStringLiteral("—"))
              << (d.haveStatus ? QString::number(d.st.ampA, 'f', 2)      : QStringLiteral("—"));
        for (int c = 0; c < cells.size() && c < m_tblDev->columnCount(); ++c)
            m_tblDev->item(r, c)->setText(cells[c]);

        const QString tip = QStringLiteral("节点 %1\n%2 · 收帧 %3\n%4\n%5")
            .arg(d.node)
            .arg(on ? QStringLiteral("在线") : QStringLiteral("离线"))
            .arg(d.frames)
            .arg(d.haveStatus
                 ? QStringLiteral("机械 %1 rad (%2 圈)\n原始 %3 rad\n活动故障: %4\n锁存故障: %5")
                   .arg(d.st.mechRad, 0, 'f', 3).arg(d.st.mechTurns, 0, 'f', 4)
                   .arg(d.shaftRad, 0, 'f', 3)
                   .arg(L103::faultText(d.ft.active))
                   .arg(L103::faultText(d.ft.latched))
                 : QStringLiteral("还没收到它的状态帧"))
            .arg(d.info.valid ? L103::infoText(d.info) : QStringLiteral("板型: 未知 (还没问过)"));
        for (int c = 0; c < m_tblDev->columnCount(); ++c)
            m_tblDev->item(r, c)->setToolTip(tip);

        if (nodes[r] == m_curNode) m_tblDev->selectRow(r);
    }
    m_syncGuard = false;

    m_labDevSum->setText(QStringLiteral("发现 %1 台节点, 在线 %2 台%3")
        .arg(nodes.size()).arg(online)
        .arg(nodes.isEmpty() ? QStringLiteral(" —— 打开 CAN 后点「扫描节点」") : QString()));
}

void MainWindow::refreshNodeInfo()
{
    if (!m_labNodeInfo) return;
    const int node = targetNode();
    const qint64 t = nowMs();

    if (node == NODE_BCAST) {
        int online = 0;
        for (QMap<int, DeviceState>::const_iterator it = m_devs.constBegin(); it != m_devs.constEnd(); ++it)
            if (it->lastMs > 0 && t - it->lastMs < DEV_ONLINE_MS) online++;
        m_labNodeInfo->setText(QStringLiteral("广播 —— 命令发给总线上所有节点 (发现 %1 台, 在线 %2 台)")
            .arg(m_devs.size()).arg(online));
        return;
    }

    const QMap<int, DeviceState>::const_iterator it = m_devs.constFind(node);
    if (it == m_devs.constEnd()) {
        m_labNodeInfo->setText(QStringLiteral("节点 %1 · 还没收到它的帧 —— 点「扫描节点」").arg(node));
        return;
    }
    const bool on = (it->lastMs > 0) && (t - it->lastMs < DEV_ONLINE_MS);
    QString protoTag;
    if (protoIsLegacy())                                        protoTag = QStringLiteral(" · 旧协议(≤10-15)");
    else if (m_protoWant == ProtoAuto && m_protoLocked == ProtoAuto)
                                                                protoTag = QStringLiteral(" · 协议待探测");
    m_labNodeInfo->setText(QStringLiteral("节点 %1 · %2 · 收帧 %3 · 最近一帧 %4 s 前%5%6%7")
        .arg(node)
        .arg(on ? QStringLiteral("在线") : QStringLiteral("离线"))
        .arg(it->frames)
        .arg((double)(t - it->lastMs) / 1000.0, 0, 'f', 1)
        .arg(it->haveStatus ? QStringLiteral(" · 机械 %1 圈").arg(it->st.mechTurns, 0, 'f', 4)
                            : QString())
        .arg(it->info.valid ? QStringLiteral(" · %1").arg(L103::infoText(it->info))
                            : (protoIsLegacy()
                               ? QStringLiteral(" · L103 + DRV8316C (旧协议固件, 不回报版本)")
                               : QStringLiteral(" · 板型未知(点「扫描节点」)")))
        .arg(protoTag));
}

/* ============================== UI ============================== */

/* 2026-10-24: spacing only. No behaviour, no values, no fonts, no tab structure touched.
 * Every layout used the Qt style defaults (~6 px spacing / ~9 px margin), which reads
 * cramped. This walks the finished widget tree once and gives the layouts explicit
 * margins + spacing. Group boxes get a bit more, so the frames breathe. */
static void tuneSpacing(QWidget *rootWidget)
{
    QList<QWidget *> widgets;
    widgets << rootWidget;
    widgets += rootWidget->findChildren<QWidget *>();
    for (QWidget *w : widgets) {
        QLayout *l = w->layout();
        if (!l)
            continue;
        if (qobject_cast<QGroupBox *>(w))
            l->setContentsMargins(14, 10, 14, 10);
        else
            l->setContentsMargins(12, 9, 12, 9);
    }

    const QList<QLayout *> lays = rootWidget->findChildren<QLayout *>();
    for (QLayout *l : lays) {
        bool holdsGroupBox = false;
        for (int i = 0; i < l->count(); ++i) {
            QLayoutItem *it = l->itemAt(i);
            if (it && qobject_cast<QGroupBox *>(it->widget())) {
                holdsGroupBox = true;
                break;
            }
        }
        if (QGridLayout *g = qobject_cast<QGridLayout *>(l)) {
            g->setHorizontalSpacing(16);
            g->setVerticalSpacing(9);
        } else if (QFormLayout *f = qobject_cast<QFormLayout *>(l)) {
            f->setHorizontalSpacing(16);
            f->setVerticalSpacing(9);
        } else if (QBoxLayout *b = qobject_cast<QBoxLayout *>(l)) {
            b->setSpacing(holdsGroupBox ? 11 : 8);
        }
    }
}

/* 每个页签套一层可滚动区: 屏上内容比可视区高时,
 * 靠滚轮把下面的按钮/输入框看到, 不再被窗口边缘切掉。 */
static QWidget *wrapScroll(QWidget *page)
{
    QScrollArea *sa = new QScrollArea;
    sa->setWidget(page);
    sa->setWidgetResizable(true);      /* 跟着窗口宽度走, 窄了折行 */
    sa->setFrameShape(QFrame::NoFrame);
    sa->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    sa->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    return sa;
}

void MainWindow::buildUi()
{
    setWindowTitle(QStringLiteral("L103 关节电机调试上位机  ·  多节点 CAN (ID = 节点号<<5 | 功能号)"));

    QWidget *central = new QWidget(this);
    QVBoxLayout *root = new QVBoxLayout(central);

    QGroupBox *gbConn = new QGroupBox(QStringLiteral("连接"));
    QGridLayout *g = new QGridLayout(gbConn);

    g->addWidget(mkLabel(QStringLiteral("UART 控制台")), 0, 0);
    m_cbUartPort = new QComboBox; m_cbUartPort->setMinimumWidth(230);
    g->addWidget(m_cbUartPort, 0, 1);

    m_cbUartBaud = new QComboBox;
    const int bauds[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    for (int i = 0; i < 8; ++i) m_cbUartBaud->addItem(QString::number(bauds[i]), bauds[i]);
    m_cbUartBaud->setCurrentText(QStringLiteral("115200"));
    g->addWidget(m_cbUartBaud, 0, 2);

    QPushButton *btnUartRefresh = new QPushButton(QStringLiteral("刷新"));
    g->addWidget(btnUartRefresh, 0, 3);

    m_btnUartOpen = new QPushButton(QStringLiteral("打开"));
    g->addWidget(m_btnUartOpen, 0, 4);

    m_labUartState = mkLabel(QStringLiteral("未连接"));
    m_labUartState->setMinimumWidth(160);
    g->addWidget(m_labUartState, 0, 5);
    g->addWidget(mkLabel(QStringLiteral("115200 8N1, 命令以回车结束")), 0, 6);

    /* 传输选择: 决定「动作 / 参数 / 序列」走哪条。默认「自动」= 老行为 (CAN 优先, 其次 COM)。 */
    g->addWidget(mkLabel(QStringLiteral("传输")), 1, 0);
    m_cbTransport = new QComboBox;
    m_cbTransport->addItem(QStringLiteral("自动 (原逻辑: CAN/桥 > COM)"), (int)TrAuto);
    m_cbTransport->addItem(QStringLiteral("COM (UART 控制台)"),             (int)TrCom);
    m_cbTransport->addItem(QStringLiteral("CAN (CH9433 / 达妙)"),            (int)TrCan);
    m_cbTransport->addItem(QStringLiteral("桥接口 (H750)"),                  (int)TrBridge);
    m_cbTransport->setToolTip(QStringLiteral(
        "自动 = 保持原行为: CAN 开着走 CAN, 否则走 COM。\n"
        "显式选 COM / CAN / 桥 = 只用那一条; 没打开就发不出去 (不会偷偷改道)。\n"
        "桥接口 = H750 UART<->CAN 桥, 串口上跑 AA 55 带 ID 的帧。"));
    g->addWidget(m_cbTransport, 1, 1);
    g->addWidget(mkLabel(QStringLiteral("COM 发文本命令; CAN / 桥发 (ID,data) 帧")), 1, 2, 1, 5);

    g->addWidget(mkLabel(QStringLiteral("CAN")), 2, 0);
    m_cbCanKind = new QComboBox;
    m_cbCanKind->addItem(QStringLiteral("关闭"), (int)CanLink::Kind_None);
    m_cbCanKind->addItem(QStringLiteral("沁恒 CH9433 EVB (裸帧)"), (int)CanLink::Kind_Ch9433);
    m_cbCanKind->addItem(QStringLiteral("达妙 DM-USB2CAN (30B 信封)"), (int)CanLink::Kind_DmUsb2Can);
    g->addWidget(m_cbCanKind, 2, 1);

    /* 设备名是一长串 \\?\usb#vid_... 路径 —— 不压一下它会把整行撑变形 */
    m_cbCanDev = new QComboBox;
    m_cbCanDev->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    m_cbCanDev->setMinimumContentsLength(26);
    m_cbCanDev->setMinimumWidth(230);
    g->addWidget(m_cbCanDev, 2, 2);

    m_cbCanBaud = new QComboBox;
    g->addWidget(m_cbCanBaud, 2, 3);

    m_btnCanRefresh = new QPushButton(QStringLiteral("刷新"));
    g->addWidget(m_btnCanRefresh, 2, 4);

    m_btnCanOpen = new QPushButton(QStringLiteral("打开"));
    g->addWidget(m_btnCanOpen, 2, 5);

    m_labCanState = mkLabel(QStringLiteral("未连接"));
    g->addWidget(m_labCanState, 2, 6);

    g->addWidget(mkLabel(QStringLiteral("桥接口")), 3, 0);
    m_cbBridgePort = new QComboBox;
    m_cbBridgePort->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    m_cbBridgePort->setMinimumContentsLength(26);
    m_cbBridgePort->setMinimumWidth(230);
    g->addWidget(m_cbBridgePort, 3, 1);

    m_cbBridgeBaud = new QComboBox;
    m_cbBridgeBaud->addItem(QStringLiteral("921600"), 921600);
    m_cbBridgeBaud->addItem(QStringLiteral("460800"), 460800);
    m_cbBridgeBaud->addItem(QStringLiteral("115200"), 115200);
    m_cbBridgeBaud->setCurrentText(QStringLiteral("921600"));
    g->addWidget(m_cbBridgeBaud, 3, 2);

    m_btnBridgeRefresh = new QPushButton(QStringLiteral("刷新"));
    g->addWidget(m_btnBridgeRefresh, 3, 3);

    m_btnBridgeOpen = new QPushButton(QStringLiteral("打开"));
    g->addWidget(m_btnBridgeOpen, 3, 4);

    m_labBridgeState = mkLabel(QStringLiteral("未连接"));
    g->addWidget(m_labBridgeState, 3, 5);
    g->addWidget(mkLabel(QStringLiteral("921600 8N1 · AA 55 帧")), 3, 6);

    root->addWidget(gbConn);

    /* ---- 目标设备: 多节点总线上"这条命令发给谁" ---- */
    QGroupBox *gbTgt = new QGroupBox(QStringLiteral("目标设备 (这一格决定「动作 / 参数」发给谁)"));
    QHBoxLayout *ht = new QHBoxLayout(gbTgt);
    m_spNode = new QSpinBox;
    m_spNode->setRange(NODE_BCAST, NODE_MAX);
    m_spNode->setSpecialValueText(QStringLiteral("0 = 广播 (所有节点)"));
    m_spNode->setValue(NODE_MIN);
    m_spNode->setMinimumWidth(175);
    m_spNode->setToolTip(QStringLiteral(
        "总线上的 ID = (节点号 << 5) | 功能号。\n"
        "选一台 = 命令只发给它; 选「广播」= 在场每一台都收到(使能/停/回零/询问 这类适合广播)。"));
    m_cbProto = new QComboBox;
    m_cbProto->addItem(QStringLiteral("自动"),        (int)ProtoAuto);
    m_cbProto->addItem(QStringLiteral("新 (节点<<5)"), (int)ProtoNew);
    m_cbProto->addItem(QStringLiteral("旧 (0x110/0x113)"), (int)ProtoLegacy);
    m_cbProto->setToolTip(QStringLiteral(
        "板子上固件是哪一套 CAN ID:\n"
        "  新 = 2026-10-16 起的固件, ID = (节点号<<5)|功能号, 对齐走 CAN 功能13;\n"
        "  旧 = 2026-10-15 及以前 (如 e3e0f5c), 固定 ID 0x110 使能 / 0x113 询问, 对齐只有串口 'A';\n"
        "  自动 = 两套都发一遍, 收到哪一套的回音就锁定哪一套 (推荐)。\n"
        "旧协议没有 CAN 对齐 —— 选「旧」时点对齐会自动改走串口 'A'。"));
    m_btnScanNodes = new QPushButton(QStringLiteral("扫描节点"));
    m_btnScanNodes->setToolTip(QStringLiteral(
        "广播问一遍 (ID 0x05, b0=0xFF) —— 总线上每台都用自己的节点号回答, 左边设备表就列出来了"));
    m_labNodeInfo = new QLabel(QStringLiteral("—"));
    m_labNodeInfo->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_labNodeInfo->setMinimumWidth(320);
    ht->addWidget(mkLabel(QStringLiteral("节点")));
    ht->addWidget(m_spNode);
    ht->addWidget(mkLabel(QStringLiteral("协议")));
    ht->addWidget(m_cbProto);
    ht->addWidget(m_btnScanNodes);
    ht->addSpacing(14);
    ht->addWidget(m_labNodeInfo, 1);
    root->addWidget(gbTgt);

    /* ---- 左边: 设备表; 右边: 各页 ---- */
    QSplitter *split = new QSplitter(Qt::Horizontal);

    QWidget *devPanel = new QWidget;
    QVBoxLayout *dv = new QVBoxLayout(devPanel);
    dv->addWidget(mkLabel(QStringLiteral("设备表 (点一行 = 把「目标设备」切到它)")));
    /* 6 列: 收帧数不再单独占一列 (它在每行的 tooltip 和上面的「节点信息」里都有),
     * 否则 7 列在左边这条轨道里总得横向滚。 */
    m_tblDev = new QTableWidget(0, 6);
    m_tblDev->setHorizontalHeaderLabels(QStringList()
        << QStringLiteral("节点") << QStringLiteral("在线") << QStringLiteral("状态")
        << QStringLiteral("位置(圈)") << QStringLiteral("速度(rpm)")
        << QStringLiteral("电流(A)"));
    m_tblDev->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_tblDev->setSelectionMode(QAbstractItemView::SingleSelection);
    m_tblDev->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_tblDev->verticalHeader()->setVisible(false);
    /* 列宽按内容收, 只有「状态」吃掉剩下的 —— 6 列才不会在左边这条轨道里挤出一条横向滚动条
     * (2026-10-16 实测: 列宽合计 558 px = 可视 558 px, 见 work\shots\shots_report.txt) */
    m_tblDev->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_tblDev->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    m_tblDev->horizontalHeader()->setMinimumSectionSize(46);
    m_tblDev->setMinimumWidth(560);
    dv->addWidget(m_tblDev, 1);
    m_labDevSum = new QLabel(QStringLiteral("还没发现设备 —— 打开 CAN 后点「扫描节点」"));
    m_labDevSum->setWordWrap(true);
    dv->addWidget(m_labDevSum);

    QTabWidget *tabs = new QTabWidget;
    tabs->addTab(wrapScroll(makeStatusTab()), QStringLiteral("状态"));
    tabs->addTab(wrapScroll(makeActionTab()), QStringLiteral("动作"));
    tabs->addTab(wrapScroll(makeSeqTab()),    QStringLiteral("序列"));
    tabs->addTab(wrapScroll(makeBridgeSeqTab()), QStringLiteral("序列(桥)"));
    tabs->addTab(wrapScroll(makeParamTab()),  QStringLiteral("参数"));
    tabs->addTab(wrapScroll(makeBridgeParamTab()), QStringLiteral("L103 调参(桥)"));
    tabs->addTab(wrapScroll(makeL103CfgTab()), QStringLiteral("L103 调参"));
    tabs->addTab(wrapScroll(makeLogTab()),    QStringLiteral("日志 / 原始"));
    tabs->addTab(wrapScroll(makeWaveTab()),   QStringLiteral("波形"));
    /* Tab labels: a little more air, same font. */
    tabs->setStyleSheet(QStringLiteral("QTabBar::tab { padding: 7px 16px; }"));

    split->addWidget(devPanel);
    split->addWidget(tabs);
    split->setStretchFactor(0, 0);
    split->setStretchFactor(1, 1);
    split->setSizes(QList<int>() << 570 << 1030);
    root->addWidget(split, 1);

    setCentralWidget(central);

    tuneSpacing(central);
    updateBoardUi();                     /* 板型还没探到 -> 保持板型 1 的默认文案 */
    resize(1660, 900);   /* 左边留出设备表那几列; 实测窗口最小 1577x804 (右边各页的最小宽度 942 px 顶着) */

    /* ---- 目标设备 / 设备表 接线 ---- */
    connect(m_spNode, QOverload<int>::of(&QSpinBox::valueChanged),
            this, [this](int) { onNodeChanged(); });
    connect(m_btnScanNodes, &QPushButton::clicked, this, &MainWindow::scanNodes);
    connect(m_cbProto, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onProtoChanged);
    connect(m_tblDev, &QTableWidget::itemSelectionChanged, this, &MainWindow::onDevRowChanged);

    /* 设备表 2 Hz 重画: 在线/离线、读数、收帧计数都在这里滚 (不靠收帧事件驱动, 免得刷爆界面) */
    m_timerDevUi = new QTimer(this);
    m_timerDevUi->setInterval(500);
    connect(m_timerDevUi, &QTimer::timeout, this, [this]() {
        refreshDevTable();
        refreshNodeInfo();
    });
    m_timerDevUi->start();

    connect(btnUartRefresh, &QPushButton::clicked, this, &MainWindow::refreshPortLists);
    connect(m_btnCanRefresh, &QPushButton::clicked, this, &MainWindow::refreshPortLists);
    connect(m_btnBridgeRefresh, &QPushButton::clicked, this, &MainWindow::refreshPortLists);
    connect(m_btnUartOpen, &QPushButton::clicked, this, &MainWindow::toggleUart);
    connect(m_btnCanOpen,  &QPushButton::clicked, this, &MainWindow::toggleCan);
    connect(m_btnBridgeOpen, &QPushButton::clicked, this, &MainWindow::toggleBridge);

    connect(m_cbTransport, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) {
        const int t = transportSel();
        const QString name = t == TrCom ? QStringLiteral("COM")
                           : t == TrCan ? QStringLiteral("CAN")
                           : t == TrBridge ? QStringLiteral("桥接口")
                                           : QStringLiteral("自动");
        log(QStringLiteral("[传输] 动作/参数通道 = %1").arg(name));
        if (bridgeActive()) QTimer::singleShot(300, this, [this]() {
            if (bridgeActive()) probeProtocol();
        });
    });

    connect(m_cbCanKind, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) {
        int k = m_cbCanKind->currentData().toInt();
        m_cbCanBaud->clear();
        if (k == (int)CanLink::Kind_Ch9433) {
            m_cbCanBaud->addItem(QStringLiteral("1 Mbps"),   1000000);
            m_cbCanBaud->addItem(QStringLiteral("500 kbps"),  500000);
            m_cbCanBaud->addItem(QStringLiteral("250 kbps"),  250000);
            m_cbCanBaud->addItem(QStringLiteral("125 kbps"),  125000);
        } else {
            m_cbCanBaud->addItem(QStringLiteral("921600"), 921600);
            m_cbCanBaud->addItem(QStringLiteral("460800"), 460800);
            m_cbCanBaud->addItem(QStringLiteral("115200"), 115200);
        }
        refreshPortLists();
    });
    m_cbCanKind->setCurrentIndex(1);
}
QWidget *MainWindow::makeStatusTab()
{
    QWidget *w = new QWidget;
    QVBoxLayout *v = new QVBoxLayout(w);

    QGroupBox *gb = new QGroupBox(QStringLiteral("板子实时状态"));
    QGridLayout *g = new QGridLayout(gb);

    /* 注意: 变量名不能叫 slots —— Qt 的 slots 是宏 */
    QLabel **labels[10] = { &m_labMech, &m_labShaft, &m_labVel, &m_labAmp, &m_labStatusBits,
                            &m_labFlags, &m_labFaultActive, &m_labFaultLatched,
                            &m_labMotorCfg, &m_labSeqProg };
    const char *names[10] = { "机械坐标", "原始坐标 shaft", "速度", "相电流", "状态位",
                              "配置", "活动故障", "锁存故障", "电机参数", "脚本进度" };

    for (int i = 0; i < 10; ++i) {
        int row = i / 2, col = i % 2;
        QLabel *cap = mkLabel(QString::fromUtf8(names[i]));
        if (i == 3) m_labAmpCaption = cap;      /* "相电流" / 板型 2 改叫 "低边总电流 i_ls" */
        g->addWidget(cap, row, col * 2);
        QLabel *l = new QLabel(QStringLiteral("—"));
        l->setMinimumWidth(320);
        l->setTextInteractionFlags(Qt::TextSelectableByMouse);
        *labels[i] = l;
        g->addWidget(l, row, col * 2 + 1);
    }

    v->addWidget(gb);

    QHBoxLayout *h = new QHBoxLayout;
    QPushButton *btn = new QPushButton(QStringLiteral("立即刷新"));
    m_chkAutoStatus = new QCheckBox(QStringLiteral("自动刷新 (1 Hz)"));
    m_chkAutoStatus->setChecked(true);
    m_chkPollOne = new QCheckBox(QStringLiteral("只轮询「目标设备」那一台"));
    m_chkPollOne->setToolTip(QStringLiteral(
        "不勾(默认): 每秒广播问一遍**所有**节点 —— 左边设备表里每台的读数才会动。\n"
        "勾上: 只问「目标设备」选的那一台, 总线上的帧少一些。"));
    h->addWidget(btn);
    h->addWidget(m_chkAutoStatus);
    h->addWidget(m_chkPollOne);
    h->addStretch();
    v->addLayout(h);

    /* ---- 只读诊断 (全在 UART) ---- */
    QGroupBox *gbD = new QGroupBox(QStringLiteral("诊断 (只有 UART 控制台有, 都不动机构)"));
    QGridLayout *gd = new QGridLayout(gbD);
    struct Diag { const char *label; const char *cmd; const char *tip; };
    const Diag diags[] = {
        { "运动日志 W",    "W",  "打一行运动/位置日志状态 (cmd.c:618)" },
        { "编码器 Y",      "Y",  "编码器诊断: raw/ang/AGC/MAG/err (cmd.c:585)" },
        { "引脚 Q",        "Q",  "DRV8313 / TIM3 引脚与复用诊断 (cmd.c:594)" },
        { "故障现场 I",    "I",  "打印故障现场 (cmd.c:657)" },
        { "总线恢复 R",    "R",  "强制 I2C 总线恢复 (cmd.c:589)" },
        { "CAN 状态 K",    "K",  "CAN 收发计数与状态 (cmd.c:607)" },
        { "CAN 清计数 KR", "KR", "清 CAN 计数器" },
        { "CAN 测试帧 J",  "J",  "让板子自己发一帧 CAN 测试帧 (cmd.c:614)" },
        { "轮询关 X0",     "X0", "关传感器轮询 (cmd.c:580)" },
        { "轮询开 X1",     "X1", "开传感器轮询 (打 5 ms)" },
        { "自动状态关 S0", "S0", "关 1Hz 周期状态打印" },
        { "自动状态开 S1", "S1", "开 1Hz 周期状态打印" },
        { "帮助 H",        "H",  "打印固件帮助 (cmd.c:236)" },
    };
    const int nDiag = (int)(sizeof(diags) / sizeof(diags[0]));
    for (int i = 0; i < nDiag; ++i) {
        QPushButton *b = new QPushButton(QString::fromUtf8(diags[i].label));
        b->setToolTip(QString::fromUtf8(diags[i].tip));
        const QString cmd = QString::fromUtf8(diags[i].cmd);
        connect(b, &QPushButton::clicked, this, [this, cmd]() {
            if (!m_uart->isOpen()) {
                log(QStringLiteral("[!] 这条只有 UART 控制台有 —— 先把 UART 打开"));
                return;
            }
            sendUart(cmd);
        });
        gd->addWidget(b, i / 5, i % 5);
    }
    v->addWidget(gbD);

    v->addWidget(mkLabel(QStringLiteral(
        "刷新方式: CAN 开着就广播「功能5 询问」(b0=0xFF) —— 每台都回 状态/配置/故障/电机参数 四帧,\n"
        "左边设备表会把每台的读数都列出来; 只有 UART 时发 S (串口只到接线那一台)。两者都是只读命令, 不会动机构。\n"
        "下面这些读数显示的是**「目标设备」选中的那一台** —— 想换台就改上面那格, 或者点左边设备表的一行。")));
    v->addStretch();

    connect(btn, &QPushButton::clicked, this, &MainWindow::refreshStatus);
    connect(m_chkAutoStatus, &QCheckBox::toggled, this, [this](bool on) {
        if (on) m_timerStatus->start(); else m_timerStatus->stop();
    });
    m_timerStatus->start();
    return w;
}

QWidget *MainWindow::makeActionTab()
{
    QWidget *w = new QWidget;
    QVBoxLayout *v = new QVBoxLayout(w);

    auto mkBtn = [](const QString &t, const QString &tip) {
        QPushButton *b = new QPushButton(t);
        b->setToolTip(tip);
        b->setMinimumHeight(32);
        return b;
    };

    QGroupBox *gb1 = new QGroupBox(QStringLiteral("使能 · 安全"));
    QGridLayout *g1 = new QGridLayout(gb1);

    QPushButton *bEn  = mkBtn(QStringLiteral("使能 (E)"),
        QStringLiteral("UART E / CAN 功能2 使能 b0=1 —— 合 EN 并进角度环抱住当前位置; 要求已对齐"));
    QPushButton *bDis = mkBtn(QStringLiteral("失能 / 松轴 (D)"),
        QStringLiteral("UART D / CAN 功能2 使能 b0=0 —— 松开, 轴可以手转"));
    QPushButton *bBH  = mkBtn(QStringLiteral("停 (B0)"),
        QStringLiteral("UART B0 / CAN 功能0 PV flags bit1=0 bit2=0 —— 2026-10-23 起固件一律「停 + 失能(桥臂高阻)」, 不再原地顶住"));
    QPushButton *bBR  = mkBtn(QStringLiteral("停 (B1)"),
        QStringLiteral("UART B1 / CAN 功能0 PV flags bit2=1 —— 和 B0 同效: 停 + 失能; 想手转轴就用它或「失能」"));
    QPushButton *bClr = mkBtn(QStringLiteral("清故障 (X)"),
        QStringLiteral("UART X / CAN 功能0 PV flags bit3=1 —— 只清故障: 不动机构、不重新使能。固件已没有 O 这条命令"));
    QPushButton *bFr  = mkBtn(QStringLiteral("设机械零点 (F)"),
        QStringLiteral("UART F / CAN 功能4 设零点 —— 把当前位置定为机械 0 点; 零点存 Flash: 设完立刻生效, 轴完全停稳 3 s 后板子自己写一次, 下次上电自动恢复"));
    QPushButton *bZer = mkBtn(QStringLiteral("重标电流零点 (U)"),
        QStringLiteral("UART U —— 必须在电机停转时做"));
    QPushButton *bAln = mkBtn(QStringLiteral("对齐 (A)"),
        QStringLiteral("CAN 功能13 (b0=0x5A) / UART A —— 会强行拽转子! 电机必须能自由转; 完事把此刻位置定义成机械 0 点, 停在失能"));
    bAln->setStyleSheet(QStringLiteral("font-weight:bold;"));

    g1->addWidget(bEn,  0, 0); g1->addWidget(bDis, 0, 1);
    g1->addWidget(bBH,  0, 2); g1->addWidget(bBR,  0, 3);
    g1->addWidget(bClr, 1, 0); g1->addWidget(bFr,  1, 1);
    g1->addWidget(bZer, 1, 2); g1->addWidget(bAln, 1, 3);
    v->addWidget(gb1);

    QGroupBox *gb2 = new QGroupBox(QStringLiteral("回机械原点"));
    QHBoxLayout *h2 = new QHBoxLayout(gb2);
    m_spHomeRpm = new QSpinBox;
    m_spHomeRpm->setRange(0, 1432);
    m_spHomeRpm->setValue(30);
    m_spHomeRpm->setSuffix(QStringLiteral(" rpm  (0 = 板子默认 30)"));
    m_spHomeRpm->setMinimumWidth(230);
    QPushButton *bHome = mkBtn(QStringLiteral("回机械 0 点 (N)"),
        QStringLiteral("UART N<rpm> / CAN 功能3 回零 —— 走回设过的机械零点; 没设过会被拒"));
    h2->addWidget(m_spHomeRpm);
    h2->addWidget(bHome);
    h2->addStretch();
    v->addWidget(gb2);

    QGroupBox *gb3 = new QGroupBox(QStringLiteral("位置运动 (PV)"));
    QGridLayout *g3 = new QGridLayout(gb3);

    m_spPvTarget = new QDoubleSpinBox;
    m_spPvTarget->setRange(-100000.0, 100000.0);
    m_spPvTarget->setDecimals(4);
    m_spPvTarget->setValue(0.0);
    m_spPvTarget->setMinimumWidth(180);

    m_cbPvUnit = new QComboBox;
    m_cbPvUnit->addItem(QStringLiteral("圈 turns"), 0);
    m_cbPvUnit->addItem(QStringLiteral("度 deg"),   1);
    m_cbPvUnit->addItem(QStringLiteral("弧度 rad"), 2);

    m_rbPvAbs = new QRadioButton(QStringLiteral("绝对目标"));
    m_rbPvRel = new QRadioButton(QStringLiteral("相对增量"));
    m_rbPvAbs->setChecked(true);

    m_chkPvMech = new QCheckBox(QStringLiteral("机械坐标"));
    m_chkPvMech->setToolTip(QStringLiteral("勾上后目标是机械坐标(相对机械零点); 与相对模式互斥。只有 CAN 有这一位, UART 的 G 没有。"));

    m_spPvRpm = new QSpinBox;
    m_spPvRpm->setRange(0, 1432);
    m_spPvRpm->setValue(60);
    m_spPvRpm->setSuffix(QStringLiteral(" rpm"));

    QPushButton *bFill = mkBtn(QStringLiteral("读当前位置 → 填目标"),
        QStringLiteral("把 spinbox 改成贴住当前轴位置的绝对目标(便于小幅试探)"));
    QPushButton *bGo = mkBtn(QStringLiteral("走位  ★"),
        QStringLiteral("UART G<turns>@<rpm> / CAN 功能0 PV —— 会真的让轴动起来; 发出前本程序先自动使能一遍"));
    bGo->setStyleSheet(QStringLiteral("font-weight:bold;"));
    bGo->setMinimumWidth(140);

    g3->addWidget(mkLabel(QStringLiteral("目标")), 0, 0);
    g3->addWidget(m_spPvTarget, 0, 1);
    g3->addWidget(m_cbPvUnit,   0, 2);
    g3->addWidget(m_rbPvAbs,    0, 3);
    g3->addWidget(m_rbPvRel,    0, 4);
    g3->addWidget(mkLabel(QStringLiteral("速度上限")), 1, 0);
    g3->addWidget(m_spPvRpm,    1, 1);
    g3->addWidget(m_chkPvMech,  1, 2, 1, 2);
    g3->addWidget(bFill,        2, 0, 1, 2);
    g3->addWidget(bGo,          2, 3, 1, 2);

    m_labPvDelta = mkLabel(QStringLiteral("—"));
    g3->addWidget(mkLabel(QStringLiteral("当前 → 目标")), 3, 0);
    g3->addWidget(m_labPvDelta, 3, 1, 1, 4);
    v->addWidget(gb3);

    QGroupBox *gb4 = new QGroupBox(QStringLiteral("速度运动 (连续转)"));
    QHBoxLayout *h4 = new QHBoxLayout(gb4);
    m_spVelRpm = new QDoubleSpinBox;
    m_spVelRpm->setRange(-99999.0, 99999.0);   /* 2026-10-23: 上限放到 99999, 板子自己钳 */
    m_spVelRpm->setDecimals(1);
    m_spVelRpm->setValue(100.0);
    m_spVelRpm->setSuffix(QStringLiteral(" rpm  (+ / − 都行)"));
    m_spVelRpm->setMinimumWidth(230);

    QPushButton *bVelGo = mkBtn(QStringLiteral("开始转"),
        QStringLiteral("UART: 先 M1 再 V<rpm> / CAN 功能1 速度 —— 会一直转下去, 直到按停"));
    QPushButton *bVelSt = mkBtn(QStringLiteral("停 (V0)"),
        QStringLiteral("UART V0 / CAN 功能1 速度 flags bit1=0"));
    h4->addWidget(m_spVelRpm);
    h4->addWidget(bVelGo);
    h4->addWidget(bVelSt);
    h4->addStretch();
    v->addWidget(gb4);

    /* 开环测试: 只给电压 + 电频率, 不读编码器 —— 新板 / 换固件 / 改相序后的第一步。
     * 同一条命令 UART 是 R<v>,<hz>, CAN 是功能号 24 (b0=0x5A 防呆)。 */
    QGroupBox *gb5 = new QGroupBox(QStringLiteral("开环测试 (不吃反馈 · 直接给电压 + 频率)"));
    QHBoxLayout *h5 = new QHBoxLayout(gb5);

    m_spOlVolt = new QDoubleSpinBox;
    m_spOlVolt->setRange(0.0, 24.0);
    m_spOlVolt->setDecimals(2);
    m_spOlVolt->setSingleStep(0.5);
    m_spOlVolt->setValue(3.0);
    m_spOlVolt->setSuffix(QStringLiteral(" V"));
    m_spOlVolt->setMinimumWidth(110);
    m_spOlVolt->setToolTip(QStringLiteral("加在电机上的电压幅值 —— 从小起 (3 V 级), 别一上来给满"));

    m_spOlHz = new QDoubleSpinBox;
    m_spOlHz->setRange(0.0, 200.0);
    m_spOlHz->setDecimals(1);
    m_spOlHz->setSingleStep(1.0);
    m_spOlHz->setValue(10.0);
    m_spOlHz->setSuffix(QStringLiteral(" Hz (电频率)"));
    m_spOlHz->setMinimumWidth(150);
    m_spOlHz->setToolTip(QStringLiteral("电角度旋转频率; 机械转速 ≈ 电频率 / 极对数 × 60"));

    QPushButton *bOlGo = mkBtn(QStringLiteral("开环转 (会真转)"),
        QStringLiteral("CAN 功能24 / UART R<v>,<hz> —— 不读编码器, 直接按电压和电频率转; 电机必须能自由转"));
    bOlGo->setStyleSheet(QStringLiteral("font-weight:bold; color:#b00000;"));
    QPushButton *bOlStop = mkBtn(QStringLiteral("停开环 (M0)"),
        QStringLiteral("CAN 功能24 (电压=0) / UART M0 —— 停, 并使能断开"));

    h5->addWidget(mkLabel(QStringLiteral("电压")));
    h5->addWidget(m_spOlVolt);
    h5->addWidget(mkLabel(QStringLiteral("频率")));
    h5->addWidget(m_spOlHz);
    h5->addWidget(bOlGo);
    h5->addWidget(bOlStop);
    h5->addStretch();
    v->addWidget(gb5);

    v->addWidget(mkLabel(QStringLiteral(
        "提示: 速度环只有 M1 下才有效 —— 角度环里直接发 V 会被 PV_HOLD 每拍覆盖掉。\n"
        "走位前建议先读一次当前位置。按动作键不弹确认框, 也不用先手动使能 ——\n"
        "本程序在每次动作前会无条件先发一遍使能(轴先被按住), 明细写进「日志 · 原始」页。\n"
        "多节点: 「目标设备」选谁, 动作就发给谁; 选「广播」= 总线上每一台都收到同一条(使能/停/清故障/\n"
        "回零/走位/速度 都允许广播; 设零点、写电机参数、改节点号 是「按节点」的, 广播会被拦下来)。")));
    v->addStretch();

    connect(bEn,   &QPushButton::clicked, this, &MainWindow::actEnable);
    connect(bDis,  &QPushButton::clicked, this, &MainWindow::actDisable);
    connect(bBH,   &QPushButton::clicked, this, &MainWindow::actStopHold);
    connect(bBR,   &QPushButton::clicked, this, &MainWindow::actStopRelease);
    connect(bClr,  &QPushButton::clicked, this, &MainWindow::actClearFault);
    connect(bAln,  &QPushButton::clicked, this, &MainWindow::actAlign);
    connect(bZer,  &QPushButton::clicked, this, &MainWindow::actCalCurrentZero);
    connect(bFr,   &QPushButton::clicked, this, &MainWindow::actSetMechZero);
    connect(bHome, &QPushButton::clicked, this, &MainWindow::actGotoMechZero);
    connect(bGo,   &QPushButton::clicked, this, &MainWindow::actMovePv);
    connect(bFill, &QPushButton::clicked, this, &MainWindow::actFillFromCurrent);
    connect(bVelGo,&QPushButton::clicked, this, &MainWindow::actVelocity);
    connect(bVelSt,&QPushButton::clicked, this, &MainWindow::actVelocityStop);
    connect(bOlGo,  &QPushButton::clicked, this, &MainWindow::actOpenloop);
    connect(bOlStop,&QPushButton::clicked, this, &MainWindow::actOpenloopStop);

    connect(m_rbPvRel, &QRadioButton::toggled, this, [this](bool on) {
        if (on) m_chkPvMech->setChecked(false);
    });
    connect(m_chkPvMech, &QCheckBox::toggled, this, [this](bool on) {
        if (on) m_rbPvAbs->setChecked(true);
    });
    connect(m_cbPvUnit, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { actFillFromCurrent(); });

    return w;
}
QWidget *MainWindow::makeSeqTab()
{
    QWidget *w = new QWidget;
    QVBoxLayout *v = new QVBoxLayout(w);

    v->addWidget(mkLabel(QStringLiteral(
        "板载动作脚本 —— 每段 = { 节点, 等待us, ±rpm, 角度deg, 停留ms }\n"
        "  · 节点 = 这一段发给哪一台(1..63); 留空 = 跟上面「目标设备」选的那一台\n"
        "  · 相对增量: 角度写正数, 方向看 rpm 的符号; 每段相对上一条命令的落点\n"
        "  · 绝对位置: 角度 = 机械坐标上的那个角度(度, 可正可负), rpm 只当速度上限\n"
        "    上位机按「当前位置 -> 目标」换算成相对增量再下发, 固件协议不变\n"
        "  · 绝对模式要先把这一台停稳(状态帧在手), 否则起点会算偏\n"
        "  · 多节点: 每台各装各的段(每台最多 16 段), 最后广播一帧让所有板在同一时刻起跑\n"
        "  · 各台的「等待」是相对**该台自己**上一段结束 —— 想让两台同时起跑, 两台第一段的等待都写 0\n"
        "  · 开始的前提: 已对齐 (使能由本程序在开始前广播发一遍)")));

    m_tblSeq = new QTableWidget(0, 5);
    m_tblSeq->setHorizontalHeaderLabels(QStringList()
        << QStringLiteral("节点") << QStringLiteral("等待 (us)")
        << QStringLiteral("速度 (rpm, ±=方向)")
        << QStringLiteral("角度 (deg, 正数)") << QStringLiteral("停留 (ms)"));
    m_tblSeq->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_tblSeq->setSelectionBehavior(QAbstractItemView::SelectRows);
    v->addWidget(m_tblSeq, 1);

    QHBoxLayout *h = new QHBoxLayout;
    QPushButton *bAdd = new QPushButton(QStringLiteral("加一段"));
    QPushButton *bDel = new QPushButton(QStringLiteral("删掉选中"));
    QPushButton *bUp  = new QPushButton(QStringLiteral("上移"));
    QPushButton *bDn  = new QPushButton(QStringLiteral("下移"));
    QPushButton *bClr = new QPushButton(QStringLiteral("清空表格"));
    QPushButton *bPv  = new QPushButton(QStringLiteral("预览要发的内容"));
    h->addWidget(bAdd); h->addWidget(bDel); h->addWidget(bUp); h->addWidget(bDn);
    h->addWidget(bClr); h->addSpacing(20); h->addWidget(bPv);
    h->addStretch();
    v->addLayout(h);

    QHBoxLayout *h2 = new QHBoxLayout;
    m_cbSeqChannel = new QComboBox;
    m_cbSeqChannel->addItem(QStringLiteral("自动 (谁开着用谁)"), 0);
    m_cbSeqChannel->addItem(QStringLiteral("只用 CAN / 桥"),     1);
    m_cbSeqChannel->addItem(QStringLiteral("只用 UART"),        2);

    m_cbSeqMode = new QComboBox;
    m_cbSeqMode->addItem(QStringLiteral("相对增量 (老方式)"), 0);
    m_cbSeqMode->addItem(QStringLiteral("绝对位置 (机械坐标)"), 1);
    auto syncSeqModeUi = [this]() {
        const bool a = seqAbsMode();
        QTableWidgetItem *hv = m_tblSeq->horizontalHeaderItem(2);
        QTableWidgetItem *ha = m_tblSeq->horizontalHeaderItem(3);
        if (hv) hv->setText(a ? QStringLiteral("速度 (rpm, 正数=上限)")
                              : QStringLiteral("速度 (rpm, ±=方向)"));
        if (ha) ha->setText(a ? QStringLiteral("角度 (deg, 绝对机械角)")
                              : QStringLiteral("角度 (deg, 正数)"));
    };
    connect(m_cbSeqMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [syncSeqModeUi](int) { syncSeqModeUi(); });

    m_spSeqLoops = new QSpinBox;
    m_spSeqLoops->setRange(1, 99);
    m_spSeqLoops->setValue(1);
    m_spSeqLoops->setSuffix(QStringLiteral(" 遍"));

    QPushButton *bSend = new QPushButton(QStringLiteral("发送到板子 ★"));
    bSend->setStyleSheet(QStringLiteral("font-weight:bold;"));
    QPushButton *bStop = new QPushButton(QStringLiteral("停止脚本 (B1)"));

    h2->addWidget(mkLabel(QStringLiteral("通道")));
    h2->addWidget(m_cbSeqChannel);
    h2->addWidget(mkLabel(QStringLiteral("角度模式")));
    h2->addWidget(m_cbSeqMode);
    h2->addWidget(mkLabel(QStringLiteral("遍数")));
    h2->addWidget(m_spSeqLoops);
    h2->addWidget(bSend);
    h2->addWidget(bStop);
    h2->addStretch();
    v->addLayout(h2);

    m_txtSeqPreview = new QPlainTextEdit;
    m_txtSeqPreview->setReadOnly(true);
    m_txtSeqPreview->setMaximumHeight(200);
    m_txtSeqPreview->setFont(QFont(QStringLiteral("Consolas"), 9));
    v->addWidget(m_txtSeqPreview);

    connect(bAdd, &QPushButton::clicked, this, &MainWindow::seqAddRow);
    connect(bDel, &QPushButton::clicked, this, &MainWindow::seqDelRow);
    connect(bUp,  &QPushButton::clicked, this, &MainWindow::seqMoveUp);
    connect(bDn,  &QPushButton::clicked, this, &MainWindow::seqMoveDown);
    connect(bClr, &QPushButton::clicked, this, &MainWindow::seqClear);
    connect(bPv,  &QPushButton::clicked, this, &MainWindow::seqPreview);
    connect(bSend,&QPushButton::clicked, this, &MainWindow::seqSend);
    connect(bStop,&QPushButton::clicked, this, &MainWindow::seqStop);

    /* 起手两行模板: 节点留空 = 用「目标设备」那一台; 第二行等 0.5 s 再反向转回来 */
    struct Tpl { int w; int rpm; int deg; int dw; };
    const Tpl tpl[2] = { {0, 100, 360, 0}, {500000, -100, 360, 0} };
    for (int i = 0; i < 2; ++i) {
        seqAddRow();
        m_tblSeq->item(i, 1)->setText(QString::number(tpl[i].w));
        m_tblSeq->item(i, 2)->setText(QString::number(tpl[i].rpm));
        m_tblSeq->item(i, 3)->setText(QString::number(tpl[i].deg));
        m_tblSeq->item(i, 4)->setText(QString::number(tpl[i].dw));
    }
    return w;
}

QWidget *MainWindow::makeParamTab()
{
    QWidget *w = new QWidget;
    QVBoxLayout *v = new QVBoxLayout(w);

    /* ---- 参数表: 行数由板子 INFO 回报的个数决定, 一帧一项 (CAN 功能15 -> 应答 23) ---- */
    m_gbpParam = new QGroupBox(QStringLiteral("参数表 (CH32V303 板 / CAN 功能15·23)"));
    QGroupBox *gbp = m_gbpParam;
    QVBoxLayout *vp = new QVBoxLayout(gbp);

    m_tblParam = new QTableWidget(0, 5);
    m_tblParam->setHorizontalHeaderLabels(QStringList()
        << QStringLiteral("#") << QStringLiteral("参数")
        << QStringLiteral("值") << QStringLiteral("单位")
        << QStringLiteral("说明"));
    m_tblParam->verticalHeader()->setVisible(false);
    m_tblParam->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_tblParam->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_tblParam->setColumnWidth(0, 40);
    m_tblParam->setColumnWidth(1, 220);
    m_tblParam->setColumnWidth(2, 130);
    m_tblParam->setColumnWidth(3, 100);
    m_tblParam->setColumnWidth(4, 340);

    /* 没连板子时先按"已知名字"的项数显示; 连上后 INFO 回报的个数会覆盖它 */
    resizeParamTable(paramKnownCount());
    m_tblParam->setMinimumHeight(340);
    vp->addWidget(m_tblParam);

    QHBoxLayout *hp = new QHBoxLayout;
    QPushButton *bPRead  = new QPushButton(QStringLiteral("从板子读取全部"));
    QPushButton *bPWrite = new QPushButton(QStringLiteral("写进板子 (RAM, 全部)"));
    QPushButton *bPRow   = new QPushButton(QStringLiteral("只写选中那一行"));
    QPushButton *bPSave  = new QPushButton(QStringLiteral("存 Flash"));
    QPushButton *bPDef   = new QPushButton(QStringLiteral("恢复出厂默认"));
    QPushButton *bPRel   = new QPushButton(QStringLiteral("从 Flash 重载"));
    hp->addWidget(bPRead); hp->addWidget(bPWrite); hp->addWidget(bPRow);
    hp->addSpacing(14);
    hp->addWidget(bPSave); hp->addWidget(bPDef); hp->addWidget(bPRel);
    hp->addStretch();
    vp->addLayout(hp);

    m_labParamState = mkLabel(QStringLiteral("还没读过"));
    vp->addWidget(m_labParamState);

    vp->addWidget(mkLabel(QStringLiteral(
        "这张表就是固件 motor_cfg 里的全部调参量 (行数以板子回报的个数为准): 板子上电先从 Flash 读它, 再按它跑电机 ——\n"
        "换一种电机只改这里 + 存 Flash, 不用改代码重编固件。\n"
        "  · 「写进板子」只改 RAM, 立刻生效; 断电就没了 —— 要留住必须「存 Flash」。\n"
        "  · 极对数在使能(抱轴)状态会被板子拒绝 —— 先失能再改, 改完必须重跑对齐。\n"
        "  · 存 Flash 时最好让电机停着: 擦一页要十几毫秒, 期间驱动中断会晚到。")));

    v->addWidget(gbp);

    connect(bPRead,  &QPushButton::clicked, this, &MainWindow::paramReadAll);
    connect(bPWrite, &QPushButton::clicked, this, &MainWindow::paramWriteAll);
    connect(bPRow,   &QPushButton::clicked, this, &MainWindow::paramWriteRow);
    connect(bPSave,  &QPushButton::clicked, this, &MainWindow::paramSaveFlash);
    connect(bPDef,   &QPushButton::clicked, this, &MainWindow::paramDefaults);
    connect(bPRel,   &QPushButton::clicked, this, &MainWindow::paramReloadFlash);

    QGroupBox *gb = new QGroupBox(QStringLiteral("L103 老格式 (极对数 / 堵转 / 母线, CAN 功能8)"));
    QFormLayout *f = new QFormLayout(gb);

    m_spPolePairs = new QSpinBox;
    m_spPolePairs->setRange(0, 64);
    m_spPolePairs->setSpecialValueText(QStringLiteral("0 = 不改"));
    m_spPolePairs->setValue(0);

    m_spStallA = new QDoubleSpinBox;
    m_spStallA->setRange(0.0, 3.0);
    m_spStallA->setDecimals(2);
    m_spStallA->setSingleStep(0.1);
    m_spStallA->setSpecialValueText(QStringLiteral("0 = 不改"));
    m_spStallA->setValue(0.0);

    m_spVbus = new QDoubleSpinBox;
    m_spVbus->setRange(0.0, 36.0);
    m_spVbus->setDecimals(2);
    m_spVbus->setSpecialValueText(QStringLiteral("0 = 不改"));
    m_spVbus->setValue(0.0);

    f->addRow(QStringLiteral("极对数 (1..64)"), m_spPolePairs);
    f->addRow(QStringLiteral("堵转阈值 A (0.1..3.0)"), m_spStallA);
    f->addRow(QStringLiteral("母线电压 V (8.0..36.0)"), m_spVbus);

    QHBoxLayout *h = new QHBoxLayout;
    QPushButton *bRead  = new QPushButton(QStringLiteral("回读当前参数"));
    QPushButton *bWrite = new QPushButton(QStringLiteral("写入 (UART # / CAN 功能8 电机参数)"));
    h->addWidget(bRead); h->addWidget(bWrite); h->addStretch();
    f->addRow(QString(), h);
    v->addWidget(gb);

    QGroupBox *gb2 = new QGroupBox(QStringLiteral("上限 / 上报"));
    QFormLayout *f2 = new QFormLayout(gb2);

    m_spVlimRadS = new QDoubleSpinBox;
    m_spVlimRadS->setRange(0.1, VEL_LIMIT_RAD_S);
    m_spVlimRadS->setDecimals(2);
    m_spVlimRadS->setValue(10.0);
    m_spVlimRadS->setSuffix(QStringLiteral(" rad/s"));
    QPushButton *bVlim = new QPushButton(QStringLiteral("设角度环速度上限 (CAN 功能7 速度上限)"));
    f2->addRow(QStringLiteral("角度环输出钳位"), m_spVlimRadS);
    f2->addRow(QString(), bVlim);

    m_chkFaultReport = new QCheckBox(QStringLiteral("故障主动上报开"));
    m_spReportPeriod = new QSpinBox;
    m_spReportPeriod->setRange(0, 255);
    m_spReportPeriod->setValue(10);
    m_spReportPeriod->setSuffix(QStringLiteral(" ×100ms"));
    QPushButton *bRep = new QPushButton(QStringLiteral("应用 (CAN 功能6 故障上报)"));
    QHBoxLayout *h3 = new QHBoxLayout;
    h3->addWidget(m_chkFaultReport); h3->addWidget(m_spReportPeriod); h3->addWidget(bRep);
    h3->addStretch();
    f2->addRow(QStringLiteral("故障上报"), h3);

    v->addWidget(gb2);

    QGroupBox *gb3 = new QGroupBox(QStringLiteral("节点地址 (多节点总线, 2026-10-16 起)"));
    QGridLayout *g3 = new QGridLayout(gb3);
    m_spNewNode = new QSpinBox;
    m_spNewNode->setRange(NODE_MIN, NODE_MAX);
    m_spNewNode->setValue(2);
    m_spNewNode->setToolTip(QStringLiteral("这台板在 CAN 上的名字, 1..63; 0 是广播, 不能当节点号"));
    QPushButton *bSetNode = new QPushButton(QStringLiteral("写入节点号 (存 Flash)"));
    g3->addWidget(mkLabel(QStringLiteral("把「目标设备」那台改成")), 0, 0);
    g3->addWidget(m_spNewNode, 0, 1);
    g3->addWidget(bSetNode, 0, 2);
    g3->setColumnStretch(3, 1);
    v->addWidget(gb3);

    v->addWidget(mkLabel(QStringLiteral(
        "节点号 = 这台板在 CAN 上的名字 (1..63; 0 是广播)。同一条总线上的板子必须**各不相同**。\n"
        "  · 这条发给「目标设备」选中的那一台; 用广播发它会把在场每一台都改成同一个号 —— 本程序直接拦掉。\n"
        "  · 改完立刻存 Flash, 板的答复用的是**改完之后**的新号。首次编号建议走串口: 在「日志 / 原始」页发 %2。\n"
        "  · 改完点上面的「扫描节点」重新认一遍设备表。")));

    v->addWidget(mkLabel(QStringLiteral(
        "注意: 改极对数会把「已对齐」作废 —— 改完必须重跑对齐 (UART 的 A / CAN 功能13)。\n"
        "堵转阈值 > 2.5 A 会提示: DRV8313 自身峰值限流就在 2.5 A 附近, 设更大也没用。")));
    v->addStretch();

    connect(bRead,  &QPushButton::clicked, this, &MainWindow::cfgRead);
    connect(bWrite, &QPushButton::clicked, this, &MainWindow::cfgWrite);
    connect(bVlim,  &QPushButton::clicked, this, &MainWindow::cfgVlimWrite);
    connect(bRep,   &QPushButton::clicked, this, &MainWindow::cfgFaultReportWrite);
    connect(bSetNode,&QPushButton::clicked, this, &MainWindow::actSetNodeNo);
    return w;
}

QWidget *MainWindow::makeL103CfgTab()
{
    QWidget *w = new QWidget;
    QVBoxLayout *v = new QVBoxLayout(w);

    QGroupBox *gb = new QGroupBox(QStringLiteral("L103 全部可调参数（UART 命令 / 掉电存 Flash）"));
    QGridLayout *g = new QGridLayout(gb);

    auto dspin = [](double lo, double hi, int dec, double step, const QString &suffix) {
        QDoubleSpinBox *s = new QDoubleSpinBox;
        s->setRange(lo, hi);
        s->setDecimals(dec);
        s->setSingleStep(step);
        s->setSuffix(suffix);
        return s;
    };

    m_spL103Pp = new QSpinBox;
    m_spL103Pp->setRange(1, 64);
    m_spL103Pp->setValue(11);

    m_spL103ILim = dspin(0.10, 3.00, 2, 0.05, QStringLiteral(" A"));
    m_spL103ILim->setValue(1.80);

    m_spL103Vbus = dspin(8.0, 36.0, 2, 0.1, QStringLiteral(" V"));
    m_spL103Vbus->setValue(24.0);

    m_spL103SpdKp = dspin(0.0, 2.0, 5, 0.005, QString());
    m_spL103SpdKp->setValue(0.06);
    m_spL103SpdKi = dspin(0.0, 100.0, 2, 0.1, QString());
    m_spL103SpdKi->setValue(5.00);

    m_spL103PosKp = dspin(0.0, 500.0, 2, 1.0, QString());
    m_spL103PosKp->setValue(25.0);

    m_spL103CurKp = dspin(0.0, 100.0, 4, 0.01, QStringLiteral(" V/A"));
    m_spL103CurKp->setValue(2.1991);
    m_spL103CurKi = dspin(0.0, 100000.0, 1, 10.0, QStringLiteral(" V/(A·s)"));
    m_spL103CurKi->setValue(8419.5);

    m_spL103R = dspin(0.0, 50.0, 3, 0.01, QStringLiteral(" Ω"));
    m_spL103R->setValue(6.700);
    m_spL103L = dspin(0.0, 100.0, 3, 0.01, QStringLiteral(" mH"));
    m_spL103L->setValue(1.750);

    m_spL103Vlim = dspin(0.1, 1000.0, 1, 5.0, QStringLiteral(" rad/s"));
    m_spL103Vlim->setValue(300.0);
    m_spL103Accel = dspin(1.0, 100000.0, 1, 100.0, QStringLiteral(" rad/s²"));
    m_spL103Accel->setValue(6600.0);

    m_spL103Align = dspin(0.5, 12.0, 2, 0.1, QStringLiteral(" V"));
    m_spL103Align->setValue(3.20);
    m_spL103Pll = dspin(10.0, 1000.0, 1, 5.0, QStringLiteral(" Hz"));
    m_spL103Pll->setValue(150.0);

    m_cbL103Vs = new QComboBox;
    m_cbL103Vs->addItem(QStringLiteral("0 自动"), 0);
    m_cbL103Vs->addItem(QStringLiteral("1 PLL"),  1);
    m_cbL103Vs->addItem(QStringLiteral("2 窗口 (易跑飞)"), 2);
    m_cbL103Vs->setCurrentIndex(1);

    int r = 0;
    g->addWidget(mkLabel(QStringLiteral("极对数")), r, 0);       g->addWidget(m_spL103Pp,   r, 1);
    g->addWidget(mkLabel(QStringLiteral("电流上限")), r, 2);      g->addWidget(m_spL103ILim, r, 3);
    g->addWidget(mkLabel(QStringLiteral("母线电压")), r, 4);      g->addWidget(m_spL103Vbus, r, 5);
    g->addWidget(mkLabel(QStringLiteral("测速源")), r, 6);        g->addWidget(m_cbL103Vs,   r, 7);
    ++r;
    g->addWidget(mkLabel(QStringLiteral("速度环 kp")), r, 0);      g->addWidget(m_spL103SpdKp, r, 1);
    g->addWidget(mkLabel(QStringLiteral("速度环 ki")), r, 2);      g->addWidget(m_spL103SpdKi, r, 3);
    g->addWidget(mkLabel(QStringLiteral("位置环 kp")), r, 4);      g->addWidget(m_spL103PosKp, r, 5);
    ++r;
    g->addWidget(mkLabel(QStringLiteral("电流环 kp")), r, 0);      g->addWidget(m_spL103CurKp, r, 1);
    g->addWidget(mkLabel(QStringLiteral("电流环 ki")), r, 2);      g->addWidget(m_spL103CurKi, r, 3);
    g->addWidget(mkLabel(QStringLiteral("相电阻 R")), r, 4);       g->addWidget(m_spL103R,     r, 5);
    g->addWidget(mkLabel(QStringLiteral("电感 L")), r, 6);         g->addWidget(m_spL103L,     r, 7);
    ++r;
    g->addWidget(mkLabel(QStringLiteral("速度上限")), r, 0);       g->addWidget(m_spL103Vlim,  r, 1);
    g->addWidget(mkLabel(QStringLiteral("加速度")), r, 2);         g->addWidget(m_spL103Accel, r, 3);
    g->addWidget(mkLabel(QStringLiteral("对齐电压")), r, 4);       g->addWidget(m_spL103Align, r, 5);
    g->addWidget(mkLabel(QStringLiteral("PLL 带宽")), r, 6);       g->addWidget(m_spL103Pll,   r, 7);

    /* 2026-10-23: 相电流立即跳闸阈值 (固件参数 16 / UART 'O<A>'), 放到界面上 */
    m_spL103Oc = dspin(0.50, 4.50, 2, 0.10, QStringLiteral(" A"));
    m_spL103Oc->setValue(2.00);
    m_spL103Oc->setToolTip(QStringLiteral(
        "相电流超过这个数、并且连续顶住右边那个时间, 才关桥 (固件 foc.c)。范围 0.5..4.5 A ——\n"
        "上限是采样链量程, 再高就没法保护了。这个阈值只管【跳闸】, 不是电流环给定上限 (那个是上面的【电流上限】)。"));

    /* 2026-10-23: 过流要"顶住 5 秒"才跳, 这个时间也要能调 (固件参数 17 / UART 'O<A>,<ms>')。 */
    m_spL103OcMs = new QSpinBox;
    m_spL103OcMs->setRange(0, 10000);
    m_spL103OcMs->setSingleStep(500);
    m_spL103OcMs->setValue(5000);
    m_spL103OcMs->setSuffix(QStringLiteral(" ms"));
    m_spL103OcMs->setMinimumWidth(120);
    m_spL103OcMs->setToolTip(QStringLiteral(
        "相电流超过阈值后, 要连续顶住这么久才跳闸 (中间只要有一拍回到阈值以下就重新计时)。\n"
        "0 = 一拍就跳 (老行为)。上限 10000 ms。注意: 这段时间里电流是真的在过。\n"
        "换相尖峰/单拍误采样靠它滤掉, 真堵转照样会到点跳。"));

    ++r;
    g->addWidget(mkLabel(QStringLiteral("过流跳闸")), r, 0);  g->addWidget(m_spL103Oc, r, 1);
    g->addWidget(mkLabel(QStringLiteral("持续")), r, 2);      g->addWidget(m_spL103OcMs, r, 3);
    g->addWidget(mkLabel(QStringLiteral("(超过阈值并连续顶住这么久才关桥; 与上面的『电流上限』不是一回事)")), r, 4, 1, 4);

    QHBoxLayout *h = new QHBoxLayout;
    m_cbL103Ch = new QComboBox;
    m_cbL103Ch->addItem(QStringLiteral("UART (串口控制台)"), 0);
    m_cbL103Ch->addItem(QStringLiteral("CAN / 桥 (功能号 15/23)"), 1);
    m_cbL103Ch->setCurrentIndex(0);
    QPushButton *bRead  = new QPushButton(QStringLiteral("从 Flash 读取 (CFG)"));
    QPushButton *bWrite = new QPushButton(QStringLiteral("写入 RAM"));
    QPushButton *bSave  = new QPushButton(QStringLiteral("存 Flash (SAVE)"));
    QPushButton *bErase = new QPushButton(QStringLiteral("擦除 Flash (DFLT)"));
    QPushButton *bClear = new QPushButton(QStringLiteral("清故障 (X)"));
    h->addWidget(mkLabel(QStringLiteral("通道"))); h->addWidget(m_cbL103Ch);
    h->addSpacing(10);
    h->addWidget(bRead); h->addWidget(bWrite); h->addWidget(bSave);
    h->addSpacing(12); h->addWidget(bErase); h->addStretch();
    g->addLayout(h, ++r, 0, 1, 8);

    /* 2026-10-23 家人: 使能 / 失能 / 对齐要能在这页直接点, 不用每次换页签。
     * 这三个按钮和「动作」页走的是同一套槽 (CAN 优先, 否则 UART), 行为完全一致。 */
    QPushButton *bEn  = new QPushButton(QStringLiteral("使能 (E)"));
    QPushButton *bDis = new QPushButton(QStringLiteral("失能 / 松轴 (D)"));
    QPushButton *bAl  = new QPushButton(QStringLiteral("对齐 (A)"));
    bEn->setToolTip(QStringLiteral("UART E / CAN 功能2 b0=1 —— 抱住当前位置; 要求已对齐"));
    bDis->setToolTip(QStringLiteral("UART D / CAN 功能2 b0=0 —— 松轴, 轴可以手转"));
    bAl->setToolTip(QStringLiteral("CAN 功能13 (b0=0x5A) / UART A —— 会硬拽转子, 轴必须能自由转"));
    bClear->setToolTip(QStringLiteral("只清故障: 不动机构、不重新使能 (CAN 功能0 flags bit3 / UART X)"));
    m_labL103Run = mkLabel(QStringLiteral("—"));

    QHBoxLayout *h2 = new QHBoxLayout;
    h2->addWidget(mkLabel(QStringLiteral("电机")));
    h2->addWidget(bEn); h2->addWidget(bDis); h2->addWidget(bAl);
    h2->addSpacing(12); h2->addWidget(bClear);
    h2->addSpacing(12);
    h2->addWidget(mkLabel(QStringLiteral("状态"))); h2->addWidget(m_labL103Run);
    h2->addStretch();
    g->addLayout(h2, ++r, 0, 1, 8);

    /* 2026-10-23: 速度模式的启动/停止也放到这一页, 不用为了一次连续转来回换页签。
     * 启动 = 先使能再进速度环 (CAN 功能1 / UART M1+V); 停止 = B1 (停 + 失能, 不顶力)。 */
    QPushButton *bVelGo = new QPushButton(QStringLiteral("启动速度 (M1/V)"));
    QPushButton *bVelSt = new QPushButton(QStringLiteral("停止 (B1)"));
    bVelGo->setToolTip(QStringLiteral("先使能, 再进速度环连续转 —— 会一直转, 直到按「停止 (B1)」"));
    bVelSt->setToolTip(QStringLiteral("UART B1 / CAN 功能0 flags bit2=1 —— 停 + 失能, 轴可以手转"));

    m_spL103VelRpm = new QDoubleSpinBox;
    m_spL103VelRpm->setRange(-99999.0, 99999.0);   /* 同上: 界面放开, 钳位交给固件 vel_limit */
    m_spL103VelRpm->setDecimals(1);
    m_spL103VelRpm->setValue(100.0);
    m_spL103VelRpm->setSuffix(QStringLiteral(" rpm"));
    m_spL103VelRpm->setMinimumWidth(140);

    QHBoxLayout *h3 = new QHBoxLayout;
    h3->addWidget(mkLabel(QStringLiteral("速度模式")));
    h3->addWidget(m_spL103VelRpm);
    h3->addWidget(bVelGo); h3->addWidget(bVelSt);
    h3->addSpacing(12);
    h3->addWidget(mkLabel(QStringLiteral("(启动会先使能; 停止 = 停 + 失能)")));
    h3->addStretch();
    g->addLayout(h3, ++r, 0, 1, 8);

    connect(bEn,  &QPushButton::clicked, this, &MainWindow::actEnable);
    connect(bDis, &QPushButton::clicked, this, &MainWindow::actDisable);
    connect(bAl,  &QPushButton::clicked, this, &MainWindow::actAlign);
    connect(bVelGo, &QPushButton::clicked, this, [this]() { actVelocityWith(m_spL103VelRpm->value()); });
    connect(bVelSt, &QPushButton::clicked, this, &MainWindow::actVelocityStop);

    m_labL103Cfg = mkLabel(QStringLiteral("还没读。先打开 UART 控制台（COM15, 115200）。"));
    g->addWidget(m_labL103Cfg, ++r, 0, 1, 8);

    v->addWidget(gb);
    v->addWidget(mkLabel(QStringLiteral(
        "写入前必须失能（轴不能受力）；改极对数会让“已对齐”作废，必须重新点“对齐”。\n"
        "“写入 RAM”立即生效但断电丢失；要留住必须再点“存 Flash (SAVE)”。\n"
        "SAVE 会擦一页 Flash（手册典型值 6.2~7.5 ms 擦除 + 2.0~2.5 ms 编程），电流环会丢几拍，禁止在运动中保存。\n"
        "“擦除 Flash (DFLT)”只擦掉保存记录，断电重上电才回默认；不会立刻改当前 RAM。")));
    v->addStretch();

    connect(bRead,  &QPushButton::clicked, this, &MainWindow::l103CfgRead);
    connect(bWrite, &QPushButton::clicked, this, &MainWindow::l103CfgWrite);
    connect(bSave,  &QPushButton::clicked, this, &MainWindow::l103CfgSave);
    connect(bErase, &QPushButton::clicked, this, &MainWindow::l103CfgErase);
    /* 2026-10-23 改: 以前写死发 UART X, 只开 CAN 时点它等于没反应。
     * 现在和上面使能/失能/对齐一样: CAN 开着走 CAN 功能0 的 bit3 (固件同日已改成「只清故障」),
     * 否则走 UART 的 X (cmd.c:387 Motor_ClearFault, 同样只清不使能)。 */
    connect(bClear, &QPushButton::clicked, this, [this]() {
        if (frameOpen()) {
            sendCanFn(FN_PV, buildPv(0, 0.0, PVF_CLEAR),
                      QStringLiteral("清故障 (不动机构) — %1").arg(nodeText(targetNode())));
            return;
        }
        if (uartFallbackOpen()) { uartNodeNote(); sendUart(QStringLiteral("X")); return; }
        log(QStringLiteral("[!] 两个口都没开"));
    });

    return w;
}

QWidget *MainWindow::makeLogTab()
{
    QWidget *w = new QWidget;
    QVBoxLayout *v = new QVBoxLayout(w);

    m_txtLog = new QPlainTextEdit;
    m_txtLog->setReadOnly(true);
    m_txtLog->setMaximumBlockCount(5000);
    m_txtLog->setFont(QFont(QStringLiteral("Consolas"), 9));
    v->addWidget(m_txtLog, 1);

    QHBoxLayout *h = new QHBoxLayout;
    h->addWidget(mkLabel(QStringLiteral("UART 原始命令")));
    m_editRawUart = new QLineEdit;
    m_editRawUart->setPlaceholderText(QStringLiteral("例如 S / G-20.002@300 / *? —— 回车即发"));
    QPushButton *bU = new QPushButton(QStringLiteral("发送"));
    h->addWidget(m_editRawUart, 1);
    h->addWidget(bU);
    v->addLayout(h);

    QHBoxLayout *h2 = new QHBoxLayout;
    h2->addWidget(mkLabel(QStringLiteral("CAN ID")));
    m_editRawCanId = new QLineEdit(QStringLiteral("0x25"));
    m_editRawCanId->setMaximumWidth(90);
    m_editRawCanId->setToolTip(QStringLiteral(
        "ID = (节点号 << 5) | 功能号\n"
        "  节点1 询问 = (1<<5)|5 = 0x25;  节点2 使能 = (2<<5)|2 = 0x42;  广播询问 = 0x05"));
    h2->addWidget(m_editRawCanId);
    h2->addWidget(mkLabel(QStringLiteral("数据(hex)")));
    m_editRawCanData = new QLineEdit(QStringLiteral("FF"));
    m_editRawCanData->setPlaceholderText(QStringLiteral("如 D0 07 64 00 98 08 C8 00"));
    QPushButton *bC = new QPushButton(QStringLiteral("发送"));
    h2->addWidget(m_editRawCanData, 1);
    h2->addWidget(bC);
    QPushButton *bClr = new QPushButton(QStringLiteral("清空日志"));
    h2->addWidget(bClr);
    v->addLayout(h2);

    v->addWidget(mkLabel(QStringLiteral(
        "ID = (节点号 << 5) | 功能号 —— 例: 节点1 询问 = 0x25, 广播询问 = 0x05,\n"
        "节点2 使能 = 0x42, 节点3 加一段脚本 = 0x69 (脚本大角度版 = 0x6B)")));

    connect(bU, &QPushButton::clicked, this, &MainWindow::rawUartSend);
    connect(m_editRawUart, &QLineEdit::returnPressed, this, &MainWindow::rawUartSend);
    connect(bC, &QPushButton::clicked, this, &MainWindow::rawCanSend);
    connect(bClr, &QPushButton::clicked, this, &MainWindow::clearLog);
    return w;
}

/* ============================== 波形页 ============================== */
QWidget *MainWindow::makeWaveTab()
{
    QWidget *w = new QWidget;
    QVBoxLayout *v = new QVBoxLayout(w);

    QGroupBox *gb = new QGroupBox(QStringLiteral("相电流波形流 (固件 SW 系列 · 只走 UART 控制台)"));
    QGridLayout *g = new QGridLayout(gb);

    m_btnWaveStart = new QPushButton(QStringLiteral("开始波形  ★"));
    m_btnWaveStop  = new QPushButton(QStringLiteral("停止波形"));
    m_btnWaveClear = new QPushButton(QStringLiteral("清曲线"));
    m_btnWaveStart->setMinimumHeight(32);
    m_btnWaveStop->setMinimumHeight(32);
    m_btnWaveStop->setEnabled(false);
    m_btnWaveStart->setStyleSheet(QStringLiteral("font-weight:bold;"));
    m_btnWaveStart->setToolTip(QStringLiteral("UART SWD<div> + SW1 —— 板子开始按 9 B/帧把三相相电流发出来"));
    m_btnWaveStop->setToolTip(QStringLiteral("UART SW0 —— 停下, 串口恢复成文本控制台"));

    m_cbWaveDiv = new QComboBox;
    m_cbWaveDiv->setToolTip(QStringLiteral(
        "SWD<div>: 每 div 个采样点发一帧; 板子采样周期 500 us, 标称帧率 = 2000/div。\n"
        "一帧 9 B, 115200 下最实用的是 SWD2; SWD1 会丢帧 (板子 SW 的 txdrop 会涨)。"));

    const int divs[6] = {2, 3, 4, 6, 10, 1};
    for (int i = 0; i < 6; ++i) {
        QString tail;
        if (divs[i] == 2)      tail = QStringLiteral("   ← 推荐");
        else if (divs[i] == 1) tail = QStringLiteral("   ← 会丢帧");
        m_cbWaveDiv->addItem(QStringLiteral("SWD%1   标称 %2 帧/s%3")
                             .arg(divs[i]).arg(2000 / divs[i]).arg(tail), divs[i]);
    }

    /* 这两个下拉框按最长条目撑宽 —— 一条 item 文本就把整个页签的最小宽度顶到 1300+ px,
     * 窗口再也缩不小。改成"按固定字数"算宽度。 */
    m_cbWaveDiv->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);
    m_cbWaveDiv->setMinimumContentsLength(10);
    m_cbWaveWindow = new QComboBox;
    m_cbWaveWindow->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);
    m_cbWaveWindow->setMinimumContentsLength(6);
    m_cbWaveWindow->setToolTip(QStringLiteral("曲线横轴画多少个点 —— 只影响画面, 不影响接收"));
    const int wins[6] = {500, 1000, 2000, 3000, 5000, 10000};
    for (int i = 0; i < 6; ++i) m_cbWaveWindow->addItem(QStringLiteral("%1 点").arg(wins[i]), wins[i]);
    m_cbWaveWindow->setCurrentIndex(3);

    m_chkWavePause = new QCheckBox(QStringLiteral("暂停画面"));

    g->addWidget(m_btnWaveStart, 0, 0);
    g->addWidget(m_btnWaveStop,  0, 1);
    g->addWidget(m_btnWaveClear, 0, 2);
    g->addWidget(mkLabel(QStringLiteral("分频")), 0, 3);
    g->addWidget(m_cbWaveDiv,    0, 4);
    g->addWidget(mkLabel(QStringLiteral("窗口")), 0, 5);
    g->addWidget(m_cbWaveWindow, 0, 6);
    g->addWidget(m_chkWavePause, 0, 7);
    g->setColumnStretch(8, 1);

    /* 勾上的说明挪进 tooltip: 三个长文本并排会把「波形」页的最小宽度顶到 1337 px,
     * 整个窗口就再也缩不小 (那几路电流的来历下面那行提示里也写了)。 */
    m_chkShowU = new QCheckBox(QStringLiteral("iU"));
    m_chkShowU->setToolTip(QStringLiteral("iU = 反推 -(iV+iW)"));
    m_chkShowV = new QCheckBox(QStringLiteral("iV (PA4)"));
    m_chkShowV->setToolTip(QStringLiteral("iV = PA4 / V 相 INA240 (U8)"));
    m_chkShowW = new QCheckBox(QStringLiteral("iW (PA1)"));
    m_chkShowW->setToolTip(QStringLiteral("iW = PA1 / W 相 INA240 (U10)"));
    m_chkShowU->setChecked(true);
    m_chkShowV->setChecked(true);
    m_chkShowW->setChecked(true);

    m_chkShowPsu = new QCheckBox(QStringLiteral("母线 I/V"));
    m_chkShowPsu->setEnabled(false);
    m_chkShowPsu->setToolTip(QStringLiteral(
        "本版硬件没有母线电流/电压采样通道 (VM_SENSE_ENABLE=0, CS_VM_Available()=0)。\n"
        "下一版 PCB 把分压/分流接上以后, 这里点亮。"));

    QHBoxLayout *hCh = new QHBoxLayout;
    hCh->addWidget(mkLabel(QStringLiteral("显示")));
    hCh->addWidget(m_chkShowU);
    hCh->addWidget(m_chkShowV);
    hCh->addWidget(m_chkShowW);
    hCh->addSpacing(16);
    hCh->addWidget(m_chkShowPsu);
    hCh->addStretch();
    g->addLayout(hCh, 1, 0, 1, 9);

    v->addWidget(gb);

    /* 板型 2 (L103 + DRV8313 V2) 专用提示: 本板没有三相电流, 固件也没有 SW 波形流。
     * 默认隐藏, updateBoardUi() 认到板型 2 才显示。 */
    m_labWaveBoardNote = new QLabel;
    m_labWaveBoardNote->setWordWrap(true);
    m_labWaveBoardNote->setStyleSheet(QStringLiteral("QLabel { color: #b03030; }"));
    m_labWaveBoardNote->setVisible(false);
    v->addWidget(m_labWaveBoardNote);

    m_plot = new WavePlot;
    v->addWidget(m_plot, 1);

    m_labWaveStat = new QLabel(QStringLiteral("未开始"));
    m_labWaveStat->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_labWaveStat->setFont(QFont(QStringLiteral("Consolas"), 9));
    v->addWidget(m_labWaveStat);

    v->addWidget(mkLabel(QStringLiteral(
        "· 波形流期间串口跑的是二进制, 日志页暂时看不到文本回复 —— 停止后自动恢复。\n"
        "· 板子那边不受影响: 每帧只是往 2 KB 发送环形缓冲塞 9 字节就返回, 缓冲满就丢帧 (板子 SW 里的 txdrop 计数), 不等、不中断 FOC。\n"
        "· 上面那行是「上位机实收」统计; 跟板子的 frames 对不上, 就是这段串口路上丢了字节。\n"
        "· 双击曲线清峰值保持 —— 峰值 = 画面上出现过的最大 |i|。")));

    QGroupBox *gbB = new QGroupBox(QStringLiteral("高级: 板子串口波特率 (SWB<baud>)"));
    QHBoxLayout *hB = new QHBoxLayout(gbB);
    m_cbWaveBaud = new QComboBox;
    const int bauds[4] = {115200, 230400, 460800, 921600};
    for (int i = 0; i < 4; ++i) m_cbWaveBaud->addItem(QString::number(bauds[i]), bauds[i]);
    m_cbWaveBaud->setToolTip(QStringLiteral(
        "本机实测 (2026-10-06): WCH-LinkE 的串口桥在 460800 上收不全 —— 想真提速得换 CH9433 的串口。\n"
        "固件里波特率不存 Flash, 板子复位就回到 115200。"));
    QPushButton *bSetB = new QPushButton(QStringLiteral("改 (会重开串口)"));
    hB->addWidget(mkLabel(QStringLiteral("改成")));
    hB->addWidget(m_cbWaveBaud);
    hB->addWidget(bSetB);
    hB->addStretch();
    v->addWidget(gbB);

    connect(m_btnWaveStart, &QPushButton::clicked, this, &MainWindow::waveStart);
    connect(m_btnWaveStop,  &QPushButton::clicked, this, &MainWindow::waveStop);
    connect(m_btnWaveClear, &QPushButton::clicked, this, &MainWindow::waveClear);
    connect(bSetB,          &QPushButton::clicked, this, &MainWindow::waveSetBaud);

    connect(m_chkWavePause, &QCheckBox::toggled, this, [this](bool on) {
        if (m_plot) m_plot->setPaused(on);
    });
    connect(m_chkShowU, &QCheckBox::toggled, this, [this](bool b) { if (m_plot) m_plot->setShowU(b); });
    connect(m_chkShowV, &QCheckBox::toggled, this, [this](bool b) { if (m_plot) m_plot->setShowV(b); });
    connect(m_chkShowW, &QCheckBox::toggled, this, [this](bool b) { if (m_plot) m_plot->setShowW(b); });
    connect(m_chkShowPsu, &QCheckBox::toggled, this, [this](bool b) { if (m_plot) m_plot->setShowPsu(b); });
    connect(m_cbWaveWindow, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) {
        if (m_plot) m_plot->setWindowPoints(m_cbWaveWindow->currentData().toInt());
    });

    return w;
}

/* ============================== 连接 / 日志 ============================== */
static QString portNameOf(const QString &item)
{
    const int bar = item.indexOf(QLatin1Char('|'));
    return (bar > 0 ? item.left(bar) : item).trimmed();
}

static QString hexOf(const QByteArray &d)
{
    QString s;
    for (int i = 0; i < d.size(); ++i) {
        if (i) s += QLatin1Char(' ');
        s += QStringLiteral("%1").arg((quint8)d[i], 2, 16, QLatin1Char('0')).toUpper();
    }
    return s.isEmpty() ? QStringLiteral("(空)") : s;
}

void MainWindow::wireLinks()
{
    connect(m_uart, &UartLink::opened,       this, &MainWindow::onUartState);
    connect(m_uart, &UartLink::closed,       this, &MainWindow::onUartState);
    connect(m_uart, &UartLink::message,      this, &MainWindow::log);
    connect(m_uart, &UartLink::lineReceived, this, &MainWindow::onUartLine);

    connect(m_can,  &CanLink::opened,        this, &MainWindow::onCanState);
    connect(m_can,  &CanLink::closed,        this, &MainWindow::onCanState);
    connect(m_can,  &CanLink::message,       this, &MainWindow::log);
    connect(m_can,  &CanLink::frameReceived, this, &MainWindow::onCanFrame);

    connect(m_bridge, &BridgeLink::opened,        this, &MainWindow::onBridgeState);
    connect(m_bridge, &BridgeLink::closed,        this, &MainWindow::onBridgeState);
    connect(m_bridge, &BridgeLink::message,       this, &MainWindow::log);
    connect(m_bridge, &BridgeLink::frameReceived, this, &MainWindow::onCanFrame);

    /* 波形流: 串口字节 -> 解析器 -> 曲线。raw 模式下走的也是这条 bytesReceived。 */
    connect(m_uart, &UartLink::bytesReceived, m_waveParser, &WaveformParser::feed);
    connect(m_waveParser, &WaveformParser::frame, this,
            [this](double iU, double iV, double iW) {
        if (m_plot) m_plot->append(iU, iV, iW);
    });

    /* 目标/单位/模式一变, 就把"当前 -> 目标"重算一遍 */
    connect(m_spPvTarget, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { redrawStatus(); });
    connect(m_cbPvUnit, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { redrawStatus(); });
    connect(m_rbPvAbs,  &QRadioButton::toggled, this, [this](bool) { redrawStatus(); });
    connect(m_rbPvRel,  &QRadioButton::toggled, this, [this](bool) { redrawStatus(); });
    connect(m_chkPvMech, &QCheckBox::toggled,   this, [this](bool) { redrawStatus(); });
}

void MainWindow::refreshPortLists()
{
    /* ---- UART ---- */
    const QString keepU = portNameOf(m_cbUartPort->currentText());
    m_cbUartPort->clear();
    m_cbUartPort->addItems(UartLink::availablePorts());
    {
        int idx = -1;
        for (int i = 0; i < m_cbUartPort->count(); ++i) {
            if (portNameOf(m_cbUartPort->itemText(i)) == keepU && !keepU.isEmpty()) { idx = i; break; }
        }
        if (idx < 0) {
            for (int i = 0; i < m_cbUartPort->count(); ++i) {
                if (portNameOf(m_cbUartPort->itemText(i)) == QStringLiteral("COM15")) { idx = i; break; }
            }
        }
        if (idx >= 0) m_cbUartPort->setCurrentIndex(idx);
    }

    /* ---- CAN ---- */
    const int kind = m_cbCanKind->currentData().toInt();
    const QString keepC = m_cbCanDev->currentText();
    m_cbCanDev->clear();

    if (kind == (int)CanLink::Kind_Ch9433) {
        const QStringList ds = CanLink::ch9433Devices();
        if (ds.isEmpty()) m_cbCanDev->addItem(QStringLiteral("(没找到设备 / USBCANx64.dll)"));
        else              m_cbCanDev->addItems(ds);
    } else if (kind == (int)CanLink::Kind_DmUsb2Can) {
        const QStringList ps = CanLink::dmPorts();
        if (ps.isEmpty()) m_cbCanDev->addItem(QStringLiteral("(没有串口)"));
        else              m_cbCanDev->addItems(ps);
    } else {
        m_cbCanDev->addItem(QStringLiteral("—"));
    }
    for (int i = 0; i < m_cbCanDev->count(); ++i) {
        if (m_cbCanDev->itemText(i) == keepC) { m_cbCanDev->setCurrentIndex(i); break; }
    }

    /* ---- 桥接口 (H750 串口) ---- */
    if (m_cbBridgePort) {
        const QString keepB = portNameOf(m_cbBridgePort->currentText());
        m_cbBridgePort->clear();
        m_cbBridgePort->addItems(BridgeLink::availablePorts());
        int idx = -1;
        for (int i = 0; i < m_cbBridgePort->count(); ++i) {
            if (!keepB.isEmpty() && portNameOf(m_cbBridgePort->itemText(i)) == keepB) { idx = i; break; }
        }
        if (idx < 0) {
            for (int i = 0; i < m_cbBridgePort->count(); ++i) {
                if (portNameOf(m_cbBridgePort->itemText(i)) == QStringLiteral("COM6")) { idx = i; break; }
            }
        }
        if (idx < 0 && m_cbBridgePort->count() > 0) idx = 0;
        if (idx >= 0) m_cbBridgePort->setCurrentIndex(idx);
    }
}

void MainWindow::toggleUart()
{
    if (m_uart->isOpen()) { m_uart->close(); return; }

    refreshPortLists();
    if (m_cbUartPort->count() == 0) {
        QMessageBox::warning(this, QStringLiteral("UART"),
            QStringLiteral("没有找到串口。检查 WCH-Link 插好没有; 若被 SSCOM 之类的工具占着, 先把那个工具关掉。"));
        return;
    }
    const QString port = portNameOf(m_cbUartPort->currentText());
    const int baud = m_cbUartBaud->currentData().toInt();
    if (m_bridge->isOpen() && m_bridge->portName() == port) {
        QMessageBox::warning(this, QStringLiteral("UART"),
            QStringLiteral("%1 正被「桥接口」用着。一个串口只能有一个写者, 先关桥接口。").arg(port));
        return;
    }
    QString err;
    if (!m_uart->open(port, baud, &err)) {
        QMessageBox::warning(this, QStringLiteral("UART 打开失败"),
            QStringLiteral("%1 @ %2\n%3").arg(port).arg(baud).arg(err));
    }
}

void MainWindow::toggleCan()
{
    if (m_can->isOpen()) { m_can->close(); return; }

    const int kind = m_cbCanKind->currentData().toInt();
    if (kind == (int)CanLink::Kind_None) {
        QMessageBox::information(this, QStringLiteral("CAN"),
            QStringLiteral("先在左边把 CAN 类型选上 (沁恒 CH9433 / 达妙 DM-USB2CAN)。"));
        return;
    }
    refreshPortLists();
    const QString dev  = m_cbCanDev->currentText();
    const int     baud = m_cbCanBaud->count() ? m_cbCanBaud->currentData().toInt() : 1000000;
    QString err;
    if (!m_can->open((CanLink::Kind)kind, dev, baud, &err)) {
        QMessageBox::warning(this, QStringLiteral("CAN 打开失败"),
            QStringLiteral("%1\n%2\n\n(CH9433 的 MI_00 会被 USBCanDemo.exe 独占, 先关掉它)").arg(dev, err));
    }
}

void MainWindow::toggleBridge()
{
    if (m_bridge->isOpen()) { m_bridge->close(); return; }

    refreshPortLists();
    if (!m_cbBridgePort || m_cbBridgePort->count() == 0) {
        QMessageBox::warning(this, QStringLiteral("桥接口"),
            QStringLiteral("没有找到串口。检查 H750 桥的 USB 串口插好没有; "
                           "若被别的软件占着 (COM6 做联调时), 先停掉那个软件。"));
        return;
    }
    const QString port = portNameOf(m_cbBridgePort->currentText());
    if (port.isEmpty() || port.startsWith(QStringLiteral("("))) {
        QMessageBox::warning(this, QStringLiteral("桥接口"), QStringLiteral("选一个有效的串口。"));
        return;
    }
    const int baud = m_cbBridgeBaud && m_cbBridgeBaud->count()
                   ? m_cbBridgeBaud->currentData().toInt() : 921600;
    if (m_uart->isOpen() && m_uart->portName() == port) {
        QMessageBox::warning(this, QStringLiteral("桥接口"),
            QStringLiteral("%1 正被「UART 控制台」用着。一个串口只能有一个写者, 先关 UART。").arg(port));
        return;
    }
    QString err;
    if (!m_bridge->open(port, baud, &err)) {
        QMessageBox::warning(this, QStringLiteral("桥接口打开失败"),
            QStringLiteral("%1 @ %2\n%3\n\n"
                           "(同一个串口只能被一个程序打开; COM6 正在联调时先别抢)")
                .arg(port).arg(baud).arg(err));
    }
}

void MainWindow::onUartState()
{
    const bool open = m_uart->isOpen();
    m_btnUartOpen->setText(open ? QStringLiteral("关闭") : QStringLiteral("打开"));
    m_labUartState->setText(open ? QStringLiteral("已连接 %1").arg(m_uart->portName())
                                 : QStringLiteral("未连接"));
    m_cbUartPort->setEnabled(!open);
    m_cbUartBaud->setEnabled(!open);

    /* 串口被关掉/掉了: 波形流必须跟着收摊, 别让界面停在"正在跑"上 */
    if (!open && m_waveStreaming) {
        m_waveStreaming = false;
        m_timerWaveStat->stop();
        if (m_btnWaveStart) m_btnWaveStart->setEnabled(true);
        if (m_btnWaveStop)  m_btnWaveStop->setEnabled(false);
        if (m_wavePollPaused) {
            m_wavePollPaused = false;
            if (m_chkAutoStatus->isChecked()) m_timerStatus->start();
        }
        log(QStringLiteral("[波形] UART 断了, 波形流收摊。"));
    }
}

void MainWindow::onCanState()
{
    const bool open = m_can->isOpen();
    m_btnCanOpen->setText(open ? QStringLiteral("关闭") : QStringLiteral("打开"));
    m_labCanState->setText(open ? m_can->describe() : QStringLiteral("未连接"));
    m_cbCanKind->setEnabled(!open);
    m_cbCanDev->setEnabled(!open);
    m_cbCanBaud->setEnabled(!open);

    /* 刚打开时链路还没站稳, 立刻发容易丢; 300ms 后广播问一遍 —— 顺便认协议版本:
     * 新固件回功能16..22, 旧固件回 0x180..0x184, 谁回就锁谁。 */
    if (open) QTimer::singleShot(300, this, [this]() {
        if (m_can->isOpen()) probeProtocol();
    });
}

void MainWindow::onBridgeState()
{
    const bool open = m_bridge->isOpen();
    if (m_btnBridgeOpen)
        m_btnBridgeOpen->setText(open ? QStringLiteral("关闭") : QStringLiteral("打开"));
    if (m_labBridgeState)
        m_labBridgeState->setText(open ? m_bridge->describe() : QStringLiteral("未连接"));
    if (m_cbBridgePort)  m_cbBridgePort->setEnabled(!open);
    if (m_cbBridgeBaud)  m_cbBridgeBaud->setEnabled(!open);

    if (open) {
        if (!bridgeActive())
            log(QStringLiteral("[桥] 已打开, 但当前传输=%1 —— 帧仍走另一条; 要改走桥请在上面「传输」里选「桥接口」")
                .arg(activeChannel()));
        QTimer::singleShot(300, this, [this]() {
            if (bridgeActive()) probeProtocol();
        });
    }
}

int MainWindow::transportSel() const
{
    return m_cbTransport ? m_cbTransport->currentData().toInt() : (int)TrAuto;
}

bool MainWindow::frameOpen() const
{
    switch (transportSel()) {
    case TrCom:    return false;
    case TrCan:    return m_can && m_can->isOpen();
    case TrBridge: return m_bridge && m_bridge->isOpen();
    default:       return (m_can && m_can->isOpen()) || (m_bridge && m_bridge->isOpen());
    }
}

bool MainWindow::uartFallbackOpen() const
{
    /* 显式选了 CAN / 桥, 就不许在它没打开时"偷偷"把动作发到 COM 上。
     * 自动 = 老行为 (帧通道优先, 其次 COM); COM = 明确只用串口。 */
    const int t = transportSel();
    return (t == TrAuto || t == TrCom) && m_uart && m_uart->isOpen();
}

bool MainWindow::bridgeActive() const
{
    if (!m_bridge || !m_bridge->isOpen()) return false;
    switch (transportSel()) {
    case TrBridge: return true;
    case TrCan:
    case TrCom:    return false;
    default:       return !(m_can && m_can->isOpen());
    }
}

QString MainWindow::activeChannel() const
{
    const int t = transportSel();
    if (t == TrCom)    return m_uart->isOpen()  ? QStringLiteral("UART") : QString();
    if (t == TrCan)    return m_can->isOpen()   ? QStringLiteral("CAN")  : QString();
    if (t == TrBridge) return m_bridge->isOpen()? QStringLiteral("桥")   : QString();
    if (bridgeActive()) return QStringLiteral("桥");
    if (m_can->isOpen())  return QStringLiteral("CAN");
    if (m_uart->isOpen()) return QStringLiteral("UART");
    return QString();
}

void MainWindow::log(const QString &s)
{
    if (m_txtLog) m_txtLog->appendPlainText(s);
}

void MainWindow::logTx(const QString &s) { log(QStringLiteral("» ") + s); }
void MainWindow::logRx(const QString &s) { log(QStringLiteral("« ") + s); }

void MainWindow::clearLog()
{
    if (m_txtLog) m_txtLog->clear();
}

void MainWindow::sendUart(const QString &cmd)
{
    if (!m_uart->isOpen()) {
        log(QStringLiteral("[!] UART 没打开, 发不出: %1").arg(cmd));
        return;
    }
    m_uart->sendCommand(cmd);          /* UartLink 自己会往日志里打 "[UART→] ..." */
}

bool MainWindow::sendCan(quint32 id, const QByteArray &data, const QString &note)
{
    const int t = transportSel();
    if ((t == TrBridge && !m_bridge->isOpen()) || (t == TrCan && !m_can->isOpen())) {
        log(QStringLiteral("[!] %1 没打开, 发不出 %2")
            .arg(t == TrBridge ? QStringLiteral("桥接口") : QStringLiteral("CAN"), canIdName(id)));
        return false;
    }

    QString err;
    if (bridgeActive()) {
        if (!m_bridge->sendFrame(id, data, &err)) {
            log(QStringLiteral("[!] 桥接口发送失败: %1").arg(err));
            return false;
        }
        logTx(QStringLiteral("桥 %1  [%2]%3").arg(canIdName(id), hexOf(data),
             note.isEmpty() ? QString() : QStringLiteral("   %1").arg(note)));
        return true;
    }

    if (!m_can->isOpen()) {
        log(QStringLiteral("[!] CAN 没打开, 发不出 %1").arg(canIdName(id)));
        return false;
    }
    if (!m_can->sendFrame(id, data, &err)) {
        log(QStringLiteral("[!] CAN 发送失败: %1").arg(err));
        return false;
    }
    logTx(QStringLiteral("CAN %1  [%2]%3").arg(canIdName(id), hexOf(data),
         note.isEmpty() ? QString() : QStringLiteral("   %1").arg(note)));
    return true;
}

void MainWindow::refreshStatus()
{
    if (frameOpen()) {
        /* 默认广播: 一发出去, 总线上每台都用自己的节点号回四帧 —— 设备表才有东西可显示 */
        const bool all = !m_chkPollOne || !m_chkPollOne->isChecked() || targetIsBroadcast();
        const int  node = all ? NODE_BCAST : targetNode();
        sendCanFn(FN_QUERY, buildQuery(0xFF),
                  all ? QStringLiteral("轮询(广播): 问所有节点的 状态/配置/故障/电机参数")
                      : QStringLiteral("轮询: 只问 %1").arg(nodeText(node)),
                  node);
        return;
    }
    if (uartFallbackOpen()) { sendUart(QStringLiteral("S")); return; }
}
/* ============================== 收帧 / 状态 ============================== */
static bool grabNum(const QString &line, const QString &key, double *out)
{
    const QRegularExpression re(QStringLiteral("(?:^|[\\s,])%1=([-+0-9.eE]+)")
                                .arg(QRegularExpression::escape(key)));
    const QRegularExpressionMatch m = re.match(line);
    if (!m.hasMatch()) return false;
    bool ok = false;
    const double v = m.captured(1).toDouble(&ok);
    if (ok && out) *out = v;
    return ok;
}

void MainWindow::onUartLine(const QString &line)
{
    if (line.trimmed().isEmpty()) return;
    logRx(line);

    l103CfgApplyLine(line);

    /* 只读状态行, UART 独有: "ST ... shaft=<rad> vel=<rad/s> en=0 aligned=1 stalled=0" */
    if (line.startsWith(QStringLiteral("ST "))) {
        double shaft = 0.0, vel = 0.0, v = 0.0;
        const bool okS = grabNum(line, QStringLiteral("shaft"), &shaft);
        const bool okV = grabNum(line, QStringLiteral("vel"),   &vel);
        if (okS) {
            /* ST 行的 shaft= 是**原始**坐标; 机械坐标 = 原始 - home (home 未知时暂时等同) */
            const double home = m_cfg.valid ? m_cfg.homeRad : 0.0;
            m_st.mechRad   = shaft - home;
            m_st.mechMrad  = (qint32)qRound(m_st.mechRad * 1000.0);
            m_st.mechTurns = m_st.mechRad / TWO_PI;
            syncRawFromMech();
        }
        if (okV) {
            m_st.velRadS = vel;
            m_st.velRpm  = vel * 60.0 / TWO_PI;
        }
        quint8 bits = 0;
        if (grabNum(line, QStringLiteral("en"),      &v) && v != 0.0) bits |= ST_ENABLED;
        if (grabNum(line, QStringLiteral("aligned"), &v) && v != 0.0) bits |= ST_ALIGNED;
        if (grabNum(line, QStringLiteral("stalled"), &v) && v != 0.0) bits |= ST_STALL;
        m_st.status = bits;
        if (okS || okV) { m_haveStatus = true; m_st.valid = true; }
        /* UART 这条路上没有节点号 —— 串口只会到接线那一台, 所以记在"当前选中那台"名下 */
        if (m_curNode >= NODE_MIN) {
            DeviceState &d = devEnsure(m_curNode);
            d.st = m_st;
            d.haveStatus = m_haveStatus;
            d.frames++;
            d.lastMs = nowMs();
            recomputeShaft(d);
        }
        redrawStatus();
    }
}

void MainWindow::onCanFrame(quint32 id, const QByteArray &d)
{
    m_canFrames++;

    /* 旧协议 (2026-10-15 及以前固件) 的应答是固定 ID (0x180..0x184 / 0x1A0)。
     * 先翻译成新协议的 "节点|功能号" 再交给同一套解析器 —— 两边载荷逐字节相同。 */
    const QString idHex = QString::number(id, 16).toUpper().rightJustified(3, QLatin1Char('0'));
    quint32 fid  = id;
    bool    oldp = false;
    quint32 lfn  = 0;
    /* 先判新协议: 它的应答功能号是 16..22; 旧协议的 0x180..0x184/0x1A0 在"节点|功能"
     * 口径下功能号是 0..4 —— 两边的**功能号段不重叠**, 所以不会误判 (节点 12 的板子
     * 发 0x180 也只可能被当成新协议, 不会被旧协议抢走)。 */
    const quint32 fnBits   = L103::canFn(id);
    const int     nodeBits = L103::canNode(id);
    const bool    looksNew = (nodeBits >= NODE_MIN &&
                              fnBits >= (quint32)FN_FB && fnBits <= (quint32)FN_FB_INFO);
    if (m_protoWant != ProtoNew && !looksNew && L103::legacyFnOfId(id, &lfn)) {
        oldp = true;
        const int ln = (targetNode() == NODE_BCAST) ? NODE_MIN : targetNode();
        fid = L103::canId(ln, lfn);
        lockProto(ProtoLegacy, QStringLiteral("收到旧协议帧 0x%1").arg(idHex));
    } else if (m_protoWant == ProtoAuto && m_protoLocked == ProtoAuto) {
        if (looksNew)
            lockProto(ProtoNew, QStringLiteral("收到新协议帧 0x%1").arg(idHex));
    }

    const int node = L103::canNode(fid);     /* 0 = 广播(只可能是我们自己发出去的回显) */
    const QString idTxt = oldp ? L103::legacyIdName(id) : canIdName(id);

    StatusFrame   st; CfgFrame cf; FaultFrame ft; MotorCfgFrame mc; FaultEvent fe; InfoFrame in;
    ParamRsp      pr;

    if (parseStatus(fid, d, &st)) {
        applyStatus(st, node);
    } else if (parseCfg(fid, d, &cf)) {
        applyCfg(cf, node);
    } else if (parseFault(fid, d, &ft)) {
        applyFault(ft, node);
    } else if (parseMotorCfg(fid, d, &mc)) {
        applyMotorCfg(mc, node);
    } else if (parseParamRsp(fid, d, &pr)) {
        applyParamRsp(pr, node);
        l103CfgApplyParam(pr);          /* L103 调参页签: 同一批帧也回填那 14 个框 */
    } else if (parseInfo(fid, d, &in)) {
        applyInfo(in, node);
        logRx(QStringLiteral("CAN<- %1  [板型/版本] %2")
              .arg(idTxt, L103::infoText(in)));
    } else if (parseFaultEvent(fid, d, &fe)) {
        if (node >= NODE_MIN) touchDevice(node);
        QString extra;
        switch (fe.kind) {
        case 1: extra = QStringLiteral("电流 %1 A").arg(fe.info * 0.01, 0, 'f', 2); break;
        case 2: extra = QStringLiteral("电压 %1 V").arg(fe.info * 0.1,  0, 'f', 1); break;
        case 3: extra = QStringLiteral("被拒命令(功能号 %1)").arg((quint16)fe.info); break;
        default: break;
        }
        logRx(QStringLiteral("CAN<- %1  [故障事件] %2  %3   t=%4ms   %5")
              .arg(idTxt)
              .arg(faultText(fe.bits))
              .arg(fe.cleared ? QStringLiteral("已清") : QStringLiteral("置位"))
              .arg(fe.tMs).arg(extra));
        return;
    } else {
        logRx(QStringLiteral("CAN<- %1  [%2]").arg(idTxt, hexOf(d)));
        if (node >= NODE_MIN) touchDevice(node);
        return;
    }

    logRx(QStringLiteral("CAN<- %1  [%2]").arg(idTxt, hexOf(d)));
}

void MainWindow::applyStatus(const StatusFrame &f, int node)
{
    if (node >= NODE_MIN) {
        DeviceState &d = devEnsure(node);
        d.st = f;             /* 功能17/16 的 b0..3 是**机械坐标** (固件 Motion_PosMechRad) */
        d.haveStatus = true;
        d.frames++;
        d.lastMs = nowMs();
        recomputeShaft(d);
    }
    /* 桥页那条「状态」: 看「目标设备」那一台 */
    if (m_brLabRun && node == bridgeNode()) {
        QString s = (f.status & ST_ENABLED) ? QStringLiteral("已使能") : QStringLiteral("未使能");
        s += (f.status & ST_ALIGNED) ? QStringLiteral(" · 已对齐") : QStringLiteral(" · 未对齐");
        if (f.status & ST_RUN)   s += QStringLiteral(" · 运行中");
        if (f.status & ST_STALL) s += QStringLiteral(" · 堵转");
        m_brLabRun->setText(s);
    }
    if (node != m_curNode) return;          /* 不是当前显示的那台: 只更新设备表, 不动界面读数 */
    m_st = f;
    m_haveStatus = true;
    syncRawFromMech();
    redrawStatus();
}

/* 坐标口径 (2026-10-06 理清, 依据固件源码):
 *   机械坐标 = 原始 shaft 角 - home    (固件 Motion_PosMechRad(), 功能17 状态帧/功能18 配置帧 报的就是它)
 *   原始坐标 = 固件 shaft_angle        (**功能0 PV 不带 bit5 的载荷、UART 'G' 用的是这个**)
 *   home     = 功能18 配置帧 b0..3 (int32 x0.001 rad), 等于串口 W 里的 home=
 * 两个都要留住, 而且比"要走多少圈"时必须同口径 —— 混一次就差一个 home (本台架 0.863 rad = 49.4 度)。 */
void MainWindow::syncRawFromMech()
{
    const double home = m_cfg.valid ? m_cfg.homeRad : 0.0;
    m_shaftRad   = m_st.mechRad + home;
    m_shaftTurns = m_shaftRad / TWO_PI;
    /* 回写设备表: 否则切到别的节点再切回来, 这一台的原始坐标就丢了 */
    const QMap<int, DeviceState>::iterator it = m_devs.find(m_curNode);
    if (it != m_devs.end()) { it->shaftRad = m_shaftRad; it->shaftTurns = m_shaftTurns; }
}

void MainWindow::applyCfg(const CfgFrame &f, int node)
{
    if (node >= NODE_MIN) {
        DeviceState &d = devEnsure(node);
        d.cfg = f;
        d.frames++;
        d.lastMs = nowMs();
        recomputeShaft(d);
    }

    /* 桥专用「参数(桥)」页: 看「目标设备」那一台。
     * 原有页签的回填逻辑原样保留在下面, 这里只多填一套独立控件。 */
    if (m_brSpVbus && node == bridgeNode()) {
        if (f.vmAvailable) m_brSpVbus->setValue(f.vbusV);
        if (m_brChkFaultReport) m_brChkFaultReport->setChecked(f.reportOn);
        if (m_brSpReportPeriod) m_brSpReportPeriod->setValue(f.reportPeriod100ms);
        if (m_brLabState) m_brLabState->setText(QStringLiteral("目标节点配置已回读: %1 V, 故障上报 %2 / %3 x100ms")
            .arg(f.vbusV, 0, 'f', 2)
            .arg(f.reportOn ? QStringLiteral("开") : QStringLiteral("关"))
            .arg(f.reportPeriod100ms));
    }

    if (node != m_curNode) return;
    m_cfg = f;
    syncRawFromMech();
    redrawStatus();
}

void MainWindow::applyFault(const FaultFrame &f, int node)
{
    if (node >= NODE_MIN) {
        DeviceState &d = devEnsure(node);
        d.ft = f;
        d.frames++;
        d.lastMs = nowMs();
    }
    if (node != m_curNode) return;
    m_ft = f;
    redrawStatus();
}

void MainWindow::applyMotorCfg(const MotorCfgFrame &f, int node)
{
    if (node >= NODE_MIN) {
        DeviceState &d = devEnsure(node);
        d.mc = f;
        d.frames++;
        d.lastMs = nowMs();
    }

    /* 桥专用「参数(桥)」页: 电机参数帧回填 (目标节点)。 */
    if (m_brSpPolePairs && node == bridgeNode()) {
        m_brSpPolePairs->setValue(f.polePairs);
        if (m_brSpStallA) m_brSpStallA->setValue(f.stallA);
        if (m_brSpVbus && f.vbusV > 0.0) m_brSpVbus->setValue(f.vbusV);
        if (m_brLabState) m_brLabState->setText(QStringLiteral("目标节点电机参数已回读: 极对数 %1, 堵转 %2 A, 母线 %3 V%4")
            .arg(f.polePairs).arg(f.stallA, 0, 'f', 2).arg(f.vbusV, 0, 'f', 2)
            .arg(f.fromFlash ? QStringLiteral(" (Flash)") : QStringLiteral(" (默认)")));
    }

    if (node != m_curNode) return;
    m_mc = f;
    redrawStatus();
}

/* 功能号 22: 板型 + 固件版本。只落在设备表 / 节点信息里 —— 它跟"这台现在转到哪"
 * 无关, 不该刷到状态页的读数上。 */
void MainWindow::applyInfo(const InfoFrame &f, int node)
{
    if (node >= NODE_MIN) {
        DeviceState &d = devEnsure(node);
        d.info = f;
        d.frames++;
        d.lastMs = nowMs();
    }
    if (node == targetNode()) {
        refreshNodeInfo();
        updateBoardUi();                 /* 板型 2: 电流叫法/波形页要跟着变 */
    }
}

void MainWindow::redrawStatus()
{
    if (m_haveStatus) {
        m_labMech->setText(QStringLiteral("%1 rad   (%2 圈)")
            .arg(m_st.mechRad, 0, 'f', 3).arg(m_st.mechTurns, 0, 'f', 4));
        m_labShaft->setText(QStringLiteral("%1 rad  =  %2 度   (%3 圈)")
            .arg(m_shaftRad, 0, 'f', 3).arg(m_shaftRad * RAD2DEG, 0, 'f', 2)
            .arg(m_shaftTurns, 0, 'f', 4));
        m_labVel->setText(QStringLiteral("%1 rad/s   (%2 rpm)")
            .arg(m_st.velRadS, 0, 'f', 3).arg(m_st.velRpm, 0, 'f', 1));
        m_labAmp->setText(QStringLiteral("%1 A").arg(m_st.ampA, 0, 'f', 2));
        m_labStatusBits->setText(statusText(m_st.status));

        QStringList fl;
        fl << ((m_st.status & ST_ENABLED) ? QStringLiteral("已使能") : QStringLiteral("未使能"));
        fl << ((m_st.status & ST_ALIGNED) ? QStringLiteral("已对齐") : QStringLiteral("未对齐"));
        if (m_st.status & ST_RUN)      fl << QStringLiteral("运行中");
        if (m_st.status & ST_ARRIVED)  fl << QStringLiteral("已到位");
        if (m_st.status & ST_STALL)    fl << QStringLiteral("堵转");
        if (m_st.status & ST_SENSOR)   fl << QStringLiteral("编码器异常");
        if (m_st.status & ST_RESTORED) fl << QStringLiteral("位置已恢复");
        if (m_st.status & ST_ZERO_SET) fl << QStringLiteral("已设机械零");
        m_labFlags->setText(fl.join(QStringLiteral("  |  ")));

        /* L103 调参页签上的电机状态 (2026-10-23) */
        if (m_labL103Run) {
            QString s = (m_st.status & ST_ENABLED) ? QStringLiteral("已使能")
                                                   : QStringLiteral("未使能");
            s += (m_st.status & ST_ALIGNED) ? QStringLiteral(" · 已对齐")
                                            : QStringLiteral(" · 未对齐");
            if (m_st.status & ST_RUN)   s += QStringLiteral(" · 运行中");
            if (m_st.status & ST_STALL) s += QStringLiteral(" · 堵转");
            m_labL103Run->setText(s);
        }
    }

    if (m_ft.valid) {
        m_labFaultActive ->setText(faultText(m_ft.active));
        m_labFaultLatched->setText(QStringLiteral("%1    (累计事件 %2)")
            .arg(faultText(m_ft.latched)).arg(m_ft.events));
    }

    if (m_cfg.valid) {
        m_labMotorCfg->setText(QStringLiteral("home %1  |  对齐 %2  |  使能 %3  |  母线 %4 V  |  上报 %5")
            .arg(m_cfg.homeSet ? QStringLiteral("%1 rad").arg(m_cfg.homeRad, 0, 'f', 3)
                               : QStringLiteral("(未设)"))
            .arg(m_cfg.aligned ? QStringLiteral("是") : QStringLiteral("否"))
            .arg(m_cfg.enabled ? QStringLiteral("是") : QStringLiteral("否"))
            .arg(m_cfg.vbusV, 0, 'f', 2)
            .arg(m_cfg.reportOn ? QStringLiteral("开 (%1 x100ms)").arg(m_cfg.reportPeriod100ms)
                                : QStringLiteral("关")));
    }

    if (m_mc.valid) {
        m_labSeqProg->setText(QStringLiteral("极对数 %1 | 堵转 %2 A | 母线 %3 V (%4) | 脚本剩 %5 段 / 第 %6 遍")
            .arg(m_mc.polePairs)
            .arg(m_mc.stallA, 0, 'f', 2)
            .arg(m_mc.vbusV,  0, 'f', 2)
            .arg(m_mc.fromFlash ? QStringLiteral("Flash") : QStringLiteral("默认"))
            .arg(m_mc.seqRemain).arg(m_mc.seqPass));
    }

    /* 当前 -> 目标 */
    if (m_labPvDelta && m_haveStatus && m_cbPvUnit && m_spPvTarget && m_rbPvRel && m_chkPvMech) {
        const int unit = m_cbPvUnit->currentData().toInt();
        double tgtTurns = m_spPvTarget->value();
        if      (unit == 1) tgtTurns /= 360.0;
        else if (unit == 2) tgtTurns /= TWO_PI;

        /* 2026-10-23: 设了机械零点以后这行"当前"不变 —— 因为它一直显示**原始坐标**。
         * 现在按勾选的坐标系显示: 勾「机械坐标」就显示机械坐标 (设完零点立刻变 0.0000),
         * 不勾显示原始坐标; 差值也按同一口径算。 */
        /* m_st.mechTurns 自己就是机械坐标 (状态帧每拍都带), 不必等配置帧 —— 等的话
         * 刚点完「设机械零点」那几秒会退回显示原始坐标。 */
        const bool   mechView = m_chkPvMech->isChecked();
        const double curTurns = mechView ? m_st.mechTurns : m_shaftTurns;
        const double curRad   = mechView ? m_st.mechRad   : m_shaftRad;

        double  delta = tgtTurns;
        QString what;
        if (m_rbPvRel->isChecked()) {
            what = QStringLiteral("相对 %1 圈").arg(tgtTurns, 0, 'f', 4);
        } else if (mechView) {
            delta = tgtTurns - m_st.mechTurns;
            what  = QStringLiteral("机械 %1 圈").arg(tgtTurns, 0, 'f', 4);
        } else {
            delta = tgtTurns - m_shaftTurns;
            what  = QStringLiteral("绝对 %1 圈").arg(tgtTurns, 0, 'f', 4);
        }
        m_labPvDelta->setText(QStringLiteral("当前 %1 圈 (%2 rad · %3)   ->   %4     差 %5 圈 (%6 度)")
            .arg(curTurns, 0, 'f', 4).arg(curRad, 0, 'f', 3)
            .arg(mechView ? QStringLiteral("机械坐标") : QStringLiteral("原始坐标"))
            .arg(what).arg(delta, 0, 'f', 4).arg(delta * 360.0, 0, 'f', 2));
    }
}
/* ============================== 动作 ============================== */
static double unitsToTurns(double v, int unit)
{
    if (unit == 1) return v / 360.0;
    if (unit == 2) return v / TWO_PI;
    return v;
}

/* 动作前无条件先使能 (家人 2026-10-16: "开始动作前先自动使能, 不管是不是在使能状态")。
 *
 * 通道跟着动作本身走, 不交叉:
 *   viaCan = true  -> CAN 功能2 使能 b0=1 (buildEnable(true))
 *   viaCan = false -> UART "E"
 * 固件里两者是同一个入口 (cmd.c 'E' / can_bus.c 功能2 使能 -> Motion_SetEnabled(1)):
 *   - 没对齐: 只回一句 MOT: ENABLE refused / ERR, 不合闸, 轴不会动;
 *   - 已经使能: 重新按"此刻轴位"抱一次 (motion_pv.c:955 先重读 shaft_angle, 再 tgt=ref=当前角),
 *              所以重复使能不会跳; 动作命令随后发, 目标从当前位置算起。
 */
void MainWindow::autoEnable(bool viaCan, int node)
{
    if (viaCan) {
        const int n = (node >= 0) ? node : targetNode();
        sendCanFn(FN_ENABLE, buildEnable(true),
                  QStringLiteral("动作前自动使能 (b0=1) — %1").arg(nodeText(n)), n);
    } else {
        uartNodeNote();
        sendUart(QStringLiteral("E"));
    }
}

void MainWindow::actEnable()
{
    /* CAN 优先: 只有 CAN 能指定节点 (多节点总线上 UART 只到接线那一台) */
    if (frameOpen())  { sendCanFn(FN_ENABLE, buildEnable(true),
                                      QStringLiteral("使能并保持当前位置 — %1").arg(nodeText(targetNode()))); return; }
    if (uartFallbackOpen()) { uartNodeNote(); sendUart(QStringLiteral("E")); return; }
    log(QStringLiteral("[!] 两个口都没开"));
}

void MainWindow::actDisable()
{
    if (frameOpen())  { sendCanFn(FN_ENABLE, buildEnable(false),
                                      QStringLiteral("失能 / 松轴 — %1").arg(nodeText(targetNode()))); return; }
    if (uartFallbackOpen()) { uartNodeNote(); sendUart(QStringLiteral("D")); return; }
    log(QStringLiteral("[!] 两个口都没开"));
}

void MainWindow::actStopHold()
{
    if (frameOpen())  { sendCanFn(FN_PV, buildPv(0, 0.0, 0x00),
                                      QStringLiteral("停 + 失能 (bit2=0) — %1").arg(nodeText(targetNode()))); return; }
    if (uartFallbackOpen()) { uartNodeNote(); sendUart(QStringLiteral("B")); return; }
    log(QStringLiteral("[!] 两个口都没开"));
}

void MainWindow::actStopRelease()
{
    if (frameOpen())  { sendCanFn(FN_PV, buildPv(0, 0.0, PVF_RELEASE),
                                      QStringLiteral("停 + 失能 (bit2=1) — %1").arg(nodeText(targetNode()))); return; }
    if (uartFallbackOpen()) { uartNodeNote(); sendUart(QStringLiteral("B1")); return; }
    log(QStringLiteral("[!] 两个口都没开"));
}

void MainWindow::actClearFault()
{
    if (frameOpen())  { sendCanFn(FN_PV, buildPv(0, 0.0, PVF_CLEAR),
                                      QStringLiteral("清故障 (bit3, 不动机构) — %1").arg(nodeText(targetNode()))); return; }
    if (uartFallbackOpen()) { uartNodeNote(); sendUart(QStringLiteral("X")); return; }
    log(QStringLiteral("[!] 两个口都没开"));
}

void MainWindow::actAlign()
{
    /* 2026-10-18: CAN 也有对齐入口了 (固件功能号 13, b0 必须 0x5A 防呆)。
     * 走 CAN 才能指定节点 —— 多节点总线上 UART 只到串线那一台。 */
    if (!frameOpen() && !uartFallbackOpen()) {
        QMessageBox::warning(this, QStringLiteral("需要连接"),
            QStringLiteral("对齐要走 CAN (功能号 13) 或 UART 控制台 (A) —— 先把其中一个口连上。"));
        return;
    }
    /* 这里原来弹确认框; 家人 2026-10-16 要求去掉, 改成写进日志备查。
     * 对齐**不**做"动作前自动使能" —— 固件 'A' 自己会先 Motor_DriveSuspend 去重对齐,
     * 对齐成功那次顺手把 EN 合上 (cmd.c:726 之后的 Motion_SyncAngleTarget); 先 E 只会让轴先被顶住。 */
    log(QStringLiteral("[动作] 对齐 —— 会硬拽转子: 轴要能自由转、没夹持、没拧劲; 完事把此刻位置定义成机械 0 点, 停在失能 (要动再点使能)"));

    /* 旧协议 (2026-10-15 及以前固件) 根本没有 CAN 对齐 —— 只能串口 'A'。 */
    if (protoIsLegacy()) {
        log(QStringLiteral("[动作] 旧协议固件: 对齐只有串口 'A' (它不认 CAN 功能13)"));
        if (m_uart->isOpen()) { uartNodeNote(); sendUart(QStringLiteral("A")); return; }
        QMessageBox::warning(this, QStringLiteral("需要 UART"),
            QStringLiteral("板子是旧协议固件 (2026-10-15 及以前): 它**没有** CAN 对齐命令，只能串口 'A'。\n\n"
                           "请先把「UART 控制台」连上 (COM15, 115200) 再点对齐；\n"
                           "或者把固件烧成 2026-10-18 之后的 V2 版 (CAN 功能号 13)。"));
        return;
    }
    if (m_protoWant == ProtoAuto && m_protoLocked == ProtoAuto)
        log(QStringLiteral("[i] 还没探到对面协议, 先按新协议 (功能13) 发; 若是 ≤10-15 旧固件它不认 —— 点「扫描节点」探一次, 或把「协议」选成旧"));

    if (frameOpen())  { sendCanFn(FN_ALIGN, buildAlign(),
                                      QStringLiteral("电角度对齐 (b0=0x5A) — %1").arg(nodeText(targetNode()))); return; }
    if (m_uart->isOpen()) { uartNodeNote(); sendUart(QStringLiteral("A")); return; }
}

void MainWindow::actCalCurrentZero()
{
    if (!m_uart->isOpen()) {
        QMessageBox::warning(this, QStringLiteral("需要 UART"),
            QStringLiteral("重标电流零点 (U) 只有 UART 有。"));
        return;
    }
    log(QStringLiteral("[动作] 重标电流零点 (U) —— 固件会先断驱动 30 ms 再合上, 必须是电机停着的时候做"));
    sendUart(QStringLiteral("U"));
}

void MainWindow::actSetMechZero()
{
    if (!needSingleNode(QStringLiteral("设机械零点"))) return;

    log(QStringLiteral("[动作] 设机械零点 — %1 —— 把当前位置定义为机械 0 点; 板子会在轴停稳 3 s 后写进 Flash, 断电重启自动恢复; 之后『回机械 0 点』/ 功能3 回零 都回到这里")
        .arg(nodeText(targetNode())));

    if (frameOpen())  { sendCanFn(FN_SETZERO, QByteArray(8, '\0'), QStringLiteral("设机械 0 点(存 Flash)")); refreshStatus(); return; }
    if (uartFallbackOpen()) { uartNodeNote(); sendUart(QStringLiteral("F")); refreshStatus(); return; }
    log(QStringLiteral("[!] 两个口都没开"));
}

void MainWindow::actGotoMechZero()
{
    const int    rpm  = m_spHomeRpm->value();
    const double vlim = (double)rpm * TWO_PI / 60.0;

    QString detail = QStringLiteral("速度上限 %1 rpm%2").arg(rpm)
                     .arg(rpm == 0 ? QStringLiteral("  (0 = 用板子默认 30 rpm)") : QString());
    bool big = true;
    if (m_haveStatus && m_cfg.valid) {
        /* 目标 = 机械 0 圈; 从当前机械坐标走到 0 = -mechTurns 圈 (原始坐标差同一个数) */
        const double d = (0.0 - m_st.mechRad) / TWO_PI;
        big = qAbs(d) > 1.0;
        detail = QStringLiteral("当前位置 机械 %1 圈 (%2 rad) / 原始 %3 rad\n机械零点 = 机械 0 圈 (原始 %4 圈)\n要走 %5 圈\n速度上限 %6 rpm")
                 .arg(m_st.mechTurns, 0, 'f', 4).arg(m_st.mechRad, 0, 'f', 3).arg(m_shaftRad, 0, 'f', 3)
                 .arg(m_cfg.homeRad / TWO_PI, 0, 'f', 4).arg(d, 0, 'f', 4).arg(rpm);
    }
    log(QStringLiteral("[动作] 回机械原点%1\n%2")
        .arg(big ? QStringLiteral("  \u2605要走 > 1 圈") : QString()).arg(detail));

    if (!frameOpen() && !uartFallbackOpen()) { log(QStringLiteral("[!] 两个口都没开")); return; }

    /* 提示而已, 不拦。固件 Motion_GotoHome() 对 al=0 是静默 return: 不动也不报错。 */
    if (m_cfg.valid && !m_cfg.aligned)
        log(QStringLiteral("[提示] 板子此刻 al=0 (本次上电还没对齐) —— 固件会静默不动, 先按「对齐 (A)」"));

    /* ---- 三条前置条件 (2026-10-19: 以前它们都表现为"按钮点了没反应") ----
     * 1) 没对齐: 固件 Motion_GotoHome() 不查 aligned, 而里面的 motion_runpv_internal()
     *    发现没对齐就**静默 return** —— 命令回"成功", 轴不动, 连故障位都不置。
     * 2) 从没设过机械零点: 2026-10-23 起零点随位置一起掉电保存 (posstore),
     *    只有重烧 / 擦过 Flash 才会没; 固件直接 return -1, 只发一个 1 s 的 CMD_REJECT 脉冲。
     * 3) 本来就在机械 0: 目标 = 当前, 不动是正常。
     * 这里把三种情况都在日志里说清楚, 别让人对着没反应的按钮猜。 */
    /* 2026-10-23: 上位机只是中间商 —— 按钮写什么就发什么。
     * 这里**不再**做这三件事: 判断"没对齐就不发" / 判断"已经在 0 点就不发" /
     * 没设零点时偷偷改走 PV 绝对坐标。固件拒了会把 ERR 回上来, 界面照原样显示。 */
    if (frameOpen()) {
        autoEnable(true);
        sendCanFn(FN_HOME, buildHome(vlim),
                  QStringLiteral("回机械原点 — %1").arg(nodeText(targetNode())));
        return;
    }
    if (uartFallbackOpen()) {
        autoEnable(false);
        sendUart(rpm > 0 ? QStringLiteral("N%1").arg(rpm) : QStringLiteral("N"));
        return;
    }
    log(QStringLiteral("[!] 两个口都没开"));
}

void MainWindow::actFillFromCurrent()
{
    if (!m_haveStatus) {
        log(QStringLiteral("[!] 还没读到位置 —— 先让状态页刷一次 (CAN 功能5 询问 / UART S)"));
        return;
    }
    const int  unit = m_cbPvUnit->currentData().toInt();
    const bool rel  = m_rbPvRel->isChecked();
    const bool mech = m_chkPvMech->isChecked() && m_cfg.valid;

    /* 填的数必须和当前模式同口径: 绝对 -> 原始坐标; 机械 -> 机械坐标; 相对 -> 0 */
    double turns = mech ? m_st.mechTurns : m_shaftTurns;
    if (rel) {
        turns = 0.0;
        log(QStringLiteral("[i] 相对模式: 目标框填 0 (= 原地不动), 要多大增量自己填"));
    }
    double v = turns;
    if      (unit == 1) v = turns * 360.0;
    else if (unit == 2) v = turns * TWO_PI;

    m_spPvTarget->setValue(v);
    redrawStatus();
}

void MainWindow::actMovePv()
{
    const int    unit  = m_cbPvUnit->currentData().toInt();
    const double val   = m_spPvTarget->value();
    const int    rpm   = m_spPvRpm->value();
    const bool   rel   = m_rbPvRel->isChecked();
    const bool   mech  = m_chkPvMech->isChecked();
    const double turns = unitsToTurns(val, unit);
    const double rad   = turns * TWO_PI;
    const double vlim  = (double)(rpm > 0 ? rpm : 30) * TWO_PI / 60.0;

    if (!frameOpen() && !uartFallbackOpen()) {
        log(QStringLiteral("[!] 两个口都没开"));
        return;
    }

    /* ---------- CAN: 功能0 PV ----------
     * 两个口都开着时优先走 CAN —— 功能0 PV 更能表达(可相对、可机械坐标), G 不能。
     *
     * 载荷语义以固件为准 (can_bus.c::handle_pv, 2026-10-06 逐行核过):
     *     相对 bit0: tgt = 载荷 + shaft_angle       <- 固件自己加"当前角"
     *     机械 bit5: tgt = 载荷 + Motion_HomeRad()  <- 固件自己加"零位"
     *     绝对     : tgt = 载荷                     (原始 shaft 坐标)
     * 所以载荷里**只放这一项本身**, 绝不能再把当前位置/零位加进去 —— 加了就是加两遍。
     * (2026-10-06 的 bug: 相对发的是 m_st.mechRad+rad, 固件又加一次 shaft_angle,
     *  实际目标 = 2*当前角 + 增量, 每按一次多跑"当前角"那么多 -> 越走越远;
     *  而且确认框比的是 载荷-当前角 = 增量(小), 1 圈告警恰好躲过去, 不拦。) */
    if (frameOpen()) {
        quint8 flags = PVF_RUN;
        if (rel)        flags |= PVF_REL;
        else if (mech)  flags |= PVF_MECH;

        const double payload = rad;                           /* 真正塞进 功能0 PV 的载荷 */
        const double effRaw  = rel  ? (m_shaftRad + rad)      /* 固件会 +当前**原始**角 */
                             : mech ? (m_cfg.homeRad + rad)   /* 固件会 +零位 */
                             : rad;                           /* 绝对: 载荷就是原始坐标目标 */

        QString detail;
        bool big = true;
        if (m_haveStatus) {
            const double dTurns = (effRaw - m_shaftRad) / TWO_PI;   /* 原始坐标差 = 实际要走多少圈 */
            big = qAbs(dTurns) > 1.0;
            detail = QStringLiteral("当前 原始 %1 圈 (%2 rad) / 机械 %3 圈\n目标(原始坐标) %4 圈 (%5 rad)\n要走 %6 圈\n速度上限 %7 rpm")
                     .arg(m_shaftTurns, 0, 'f', 4).arg(m_shaftRad, 0, 'f', 3).arg(m_st.mechTurns, 0, 'f', 4)
                     .arg(effRaw / TWO_PI, 0, 'f', 4).arg(effRaw, 0, 'f', 3)
                     .arg(dTurns, 0, 'f', 4).arg(rpm);
        } else {
            detail = QStringLiteral("还没读到当前位置。\n目标 %1 圈 (%2 rad)\n速度上限 %3 rpm")
                     .arg(effRaw / TWO_PI, 0, 'f', 4).arg(effRaw, 0, 'f', 3).arg(rpm);
        }
        /* 这里原来弹确认框 (只在"要走 > 1 圈"时); 家人 2026-10-16 要求去掉 —— 明细改写到日志页备查。 */
        log(QStringLiteral("[动作] 走位 (CAN 功能0 PV) —— 目标 %1%2\n%3%4")
            .arg(nodeText(targetNode()))
            .arg(targetIsBroadcast() ? QStringLiteral("  ★广播: 总线上每一台都会动") : QString())
            .arg(big ? QStringLiteral("  ★要走 > 1 圈\n") : QString())
            .arg(detail));
        autoEnable(true);                       /* 动作前无条件先使能, 不看当前状态 */

        sendCanFn(FN_PV, buildPv((qint32)qRound(payload * 1000.0), vlim, flags),
                QStringLiteral("载荷 %1 rad, 目标(原始) %2 圈, 上限 %3 rpm%4")
                .arg(payload, 0, 'f', 3).arg(effRaw / TWO_PI, 0, 'f', 4).arg(rpm)
                .arg(rel ? QStringLiteral(", 相对") : (mech ? QStringLiteral(", 机械坐标") : QString())));
        return;
    }

    /* ---------- UART: G 只有 "绝对 + 圈", 没有机械坐标 ---------- */
    if (rel) {
        QMessageBox::information(this, QStringLiteral("UART 没有相对走位"),
            QStringLiteral("G 是「绝对圈数」。\n相对走位请用 CAN (功能0 PV bit0),\n"
                           "或者先用「读当前位置 → 填目标」把目标改成贴住当前位置的绝对值。"));
        return;
    }
    if (mech) {
        QMessageBox::information(this, QStringLiteral("UART 没有机械坐标"),
            QStringLiteral("G 走的是原始坐标。要按机械坐标走, 请用 CAN (功能0 PV bit5), 或者自己减去 home 偏移。"));
        return;
    }

    QString detail = QStringLiteral("目标 %1 圈 (%2 rad)\n速度上限 %3 rpm")
                     .arg(turns, 0, 'f', 6).arg(rad, 0, 'f', 3).arg(rpm);
    bool big = true;
    if (m_haveStatus) {
        const double d = turns - m_shaftTurns;
        big = qAbs(d) > 1.0;
        detail = QStringLiteral("当前位置 %1 圈 (%2 rad)\n目标 %3 圈 (%4 rad)\n要走 %5 圈\n速度上限 %6 rpm")
                 .arg(m_shaftTurns, 0, 'f', 4).arg(m_shaftRad, 0, 'f', 3)
                 .arg(turns, 0, 'f', 6).arg(rad, 0, 'f', 3).arg(d, 0, 'f', 4).arg(rpm);
    }
    log(QStringLiteral("[动作] 走位 (UART G)%1\n%2")
        .arg(big ? QStringLiteral("  ★要走 > 1 圈") : QString()).arg(detail));
    autoEnable(false);                          /* 动作前无条件先使能, 不看当前状态 (顺带提醒 UART 不认节点号) */

    /* 目标离当前位置太远时固件没有任何行程校验, 会一路冲到过流 —— 这就是 2026-09-28 那次事故。 */
    sendUart(QStringLiteral("G%1@%2").arg(turns, 0, 'f', 6).arg(rpm));
}

void MainWindow::actVelocity()
{
    actVelocityWith(m_spVelRpm->value());
}

/* 2026-10-23: 抽成带参数的, 好让 L103 调参页签那一行「速度模式」复用同一套逻辑
 * (CAN 先无条件使能再发功能1; UART 必须先 E 再 M1 再 V —— E 会把 controller 拨回角度环)。 */
void MainWindow::actVelocityWith(double rpm)
{
    const double rad = rpm * TWO_PI / 60.0;
    /* 2026-10-23: 界面不再拦超限值 (速度模式上限放到 99999) —— 照发, 只记一条日志。
     * 钳位在固件里: 速度环把给定钳到 vel_limit (默认 330 rad/s), 再走加速度斜率限幅 (motor.c:625-633)。 */
    if (qAbs(rad) > VEL_LIMIT_RAD_S) {
        log(QStringLiteral("[!] 给定 %1 rpm 超过角度环上限 %2 rad/s (= 1432 rpm) —— 照发, 板子会钳到自己的 vel_limit")
            .arg(rpm, 0, 'f', 1).arg(VEL_LIMIT_RAD_S, 0, 'f', 1));
    }
    log(QStringLiteral("[动作] 连续转 %1 rpm —— 会一直转下去, 直到按「停」; 上电前先确认线束不会拧到头")
        .arg(rpm, 0, 'f', 1));

    if (frameOpen()) {
        log(QStringLiteral("[动作] 速度环 (功能1) —— 目标 %1%2")
            .arg(nodeText(targetNode()))
            .arg(targetIsBroadcast() ? QStringLiteral("  ★广播: 总线上每一台都会转") : QString()));
        autoEnable(true);                       /* 动作前无条件先使能 */
        sendCanFn(FN_VEL, buildVel(rad, PVF_RUN), QStringLiteral("速度 %1 rpm").arg(rpm, 0, 'f', 1));
        return;
    }
    if (uartFallbackOpen()) {
        autoEnable(false);                      /* 动作前无条件先使能 (E 会顺手把 controller 拨回角度环, 所以必须在 M1 之前) */
        sendUart(QStringLiteral("M1"));          /* 速度环: 角度环里发 V 会被 PV_HOLD 每拍覆盖 */
        sendUart(QStringLiteral("V%1").arg(rpm, 0, 'f', 1));
        return;
    }
    log(QStringLiteral("[!] 两个口都没开"));
}

void MainWindow::actVelocityStop()
{
    if (frameOpen())  { sendCanFn(FN_VEL, buildVel(0.0, 0x00),
                                      QStringLiteral("速度停 + 松轴 — %1").arg(nodeText(targetNode()))); return; }
    if (uartFallbackOpen()) { uartNodeNote(); sendUart(QStringLiteral("B1")); return; }
    log(QStringLiteral("[!] 两个口都没开"));
}

/* 开环: CAN 功能号 24 (b0=0x5A 防呆, b1..2 电压 mV, b3..4 频率 0.01Hz) /
 * UART R<v>,<hz>。新板 / 换固件 / 改相序后的第一步 —— 不吃反馈, 只看转不转。 */
void MainWindow::actOpenloop()
{
    if (!frameOpen() && !uartFallbackOpen()) {
        QMessageBox::warning(this, QStringLiteral("需要连接"),
            QStringLiteral("开环要走 CAN 控制台或 UART 控制台 —— 先把其中一个口连上。"));
        return;
    }

    const double v  = m_spOlVolt ? m_spOlVolt->value() : 0.0;
    const double hz = m_spOlHz   ? m_spOlHz->value()   : 0.0;

    if (v <= 0.0 || hz <= 0.0) {
        QMessageBox::warning(this, QStringLiteral("参数不对"),
            QStringLiteral("开环电压和频率都要大于 0 —— 想停请按「停开环 (M0)」。"));
        return;
    }

    log(QStringLiteral("[动作] 开环 %1 V / %2 Hz (电频率) — %3 —— 不读编码器, 电机真的会转; 三相 OUT 应该是 20 kHz PWM")
        .arg(v, 0, 'f', 2).arg(hz, 0, 'f', 1).arg(nodeText(targetNode())));

    if (protoIsLegacy())
        log(QStringLiteral("[i] 旧协议固件没有 CAN 功能24 —— 这条只有 2026-10-23 之后的固件认"));

    if (frameOpen()) {
        QByteArray p(8, '\0');
        const qint16 vm    = static_cast<qint16>(qRound(v * 1000.0));    /* mV */
        const qint16 hz100 = static_cast<qint16>(qRound(hz * 100.0));    /* 0.01 Hz */
        p[0] = static_cast<char>(0x5A);
        p[1] = static_cast<char>(vm & 0xFF);
        p[2] = static_cast<char>((vm >> 8) & 0xFF);
        p[3] = static_cast<char>(hz100 & 0xFF);
        p[4] = static_cast<char>((hz100 >> 8) & 0xFF);
        sendCanFn(FN_OL, p, QStringLiteral("开环 %1V %2Hz (b0=0x5A) — %3")
                  .arg(v, 0, 'f', 2).arg(hz, 0, 'f', 1).arg(nodeText(targetNode())));
        return;
    }
    uartNodeNote();
    sendUart(QStringLiteral("R%1,%2").arg(v, 0, 'f', 3).arg(hz, 0, 'f', 2));
}

void MainWindow::actOpenloopStop()
{
    log(QStringLiteral("[动作] 停开环 — %1").arg(nodeText(targetNode())));
    if (frameOpen()) {
        QByteArray p(8, '\0');
        p[0] = static_cast<char>(0x5A);   /* 电压/频率全 0 = 停 */
        sendCanFn(FN_OL, p, QStringLiteral("停开环 (电压=0) — %1").arg(nodeText(targetNode())));
        return;
    }
    if (uartFallbackOpen()) { uartNodeNote(); sendUart(QStringLiteral("M0")); return; }
    log(QStringLiteral("[!] 两个口都没开"));
}

/* ============================== 序列 ============================== */
static void swapSeqRows(QTableWidget *t, int a, int b)
{
    for (int c = 0; c < t->columnCount(); ++c) {
        QTableWidgetItem *ia = t->takeItem(a, c);
        QTableWidgetItem *ib = t->takeItem(b, c);
        t->setItem(a, c, ib);
        t->setItem(b, c, ia);
    }
}

void MainWindow::seqAddRow()
{
    const int r = m_tblSeq->rowCount();
    m_tblSeq->insertRow(r);
    /* 第 0 列(节点)留空 = 用「目标设备」选的那一台; 想指定就在格子里写 1..63 */
    const QString def[5] = { QString(), QStringLiteral("0"), QStringLiteral("100"),
                             QStringLiteral("360"), QStringLiteral("0") };
    for (int c = 0; c < 5; ++c)
        m_tblSeq->setItem(r, c, new QTableWidgetItem(def[c]));
}

void MainWindow::seqDelRow()
{
    int r = m_tblSeq->currentRow();
    if (r < 0) r = m_tblSeq->rowCount() - 1;
    if (r >= 0) m_tblSeq->removeRow(r);
}

void MainWindow::seqMoveUp()
{
    const int r = m_tblSeq->currentRow();
    if (r <= 0) return;
    swapSeqRows(m_tblSeq, r, r - 1);
    m_tblSeq->setCurrentCell(r - 1, 0);
}

void MainWindow::seqMoveDown()
{
    const int r = m_tblSeq->currentRow();
    if (r < 0 || r >= m_tblSeq->rowCount() - 1) return;
    swapSeqRows(m_tblSeq, r, r + 1);
    m_tblSeq->setCurrentCell(r + 1, 0);
}

void MainWindow::seqClear()
{
    m_tblSeq->setRowCount(0);
}

bool MainWindow::seqAbsMode() const
{
    return m_cbSeqMode && m_cbSeqMode->currentData().toInt() == 1;
}

/* 绝对模式: 表里的 deg 是**机械坐标**(度) —— 轴上的机械 0 点就是界面上「机械 0 圈」那个点。
 * 固件收到的每一段仍然是"相对上一条命令落点"的增量, 所以这里换算:
 *     delta_i = target_i - target_(i-1)      (target_0 用"发命令这一刻的机械位置")
 * 固件协议一个字不改, 老的 UART *{} / CAN 功能9,11 照旧能用。
 * loops>1: 固件每一遍是接着上一遍的落点继续走的; 绝对语义要求每遍都从第一个目标起步,
 * 所以按节点各补一段"从最后一个目标回到第一个目标"的增量。 */
bool MainWindow::absSeqToDeltas(const QVector<Seg> &absSegs, int loops,
                                QVector<Seg> *relOut, QStringList *notes, QString *err) const
{
    const double d2r = TWO_PI / 360.0;
    relOut->clear();

    QMap<int, double> cur, first, last, rpmFirst;
    for (int i = 0; i < absSegs.size(); ++i) {
        const Seg &s = absSegs.at(i);
        const QMap<int, DeviceState>::const_iterator it = m_devs.constFind(s.node);
        if (it == m_devs.constEnd() || !it->haveStatus || !it->st.valid) {
            *err = QStringLiteral("节点 %1 还没读到当前位置(状态帧/配置帧没到), 不能按绝对角度下发。\n"
                                  "先把这一台连上、等状态刷新再发。").arg(s.node);
            return false;
        }
        if (qAbs(it->st.velRadS) > 0.2) {
            *err = QStringLiteral("节点 %1 还在动 (vel = %2 rad/s)。\n"
                                  "绝对模式以「发命令这一刻的位置」为起点, 动的时候发会整体偏掉 —— 先停稳再发。")
                     .arg(s.node).arg(it->st.velRadS, 0, 'f', 3);
            return false;
        }
        if (!cur.contains(s.node)) {
            cur[s.node]   = it->st.mechRad;     /* 状态帧 0x181 报的就是机械坐标 */
            rpmFirst[s.node] = qAbs(s.rpm);
        }

        const double tgt = s.deg * d2r;
        const double d   = tgt - cur[s.node];
        if (!first.contains(s.node)) first[s.node] = tgt;
        last[s.node] = tgt;
        cur[s.node]  = tgt;

        if (qAbs(d) < 1e-9) {
            notes->append(QStringLiteral("节点 %1 第 %2 行的目标就是当前位置: 没有动作, 这一行的等待/停留不下发")
                          .arg(s.node).arg(i + 1));
            continue;
        }
        Seg r = s;
        r.abs = false;
        r.hr  = false;                 /* 交给 groupForCan 按实际增量重新挑 0x120/0x122 */
        r.deg = qAbs(d) / d2r;
        r.rpm = (d > 0 ? 1 : -1) * qAbs(s.rpm);
        relOut->append(r);
    }

    if (loops > 1) {
        for (QMap<int, double>::const_iterator it = first.constBegin(); it != first.constEnd(); ++it) {
            const int    node = it.key();
            const double d    = it.value() - last.value(node);
            if (qAbs(d) < 1e-9) continue;
            Seg c;
            c.node    = node;
            c.waitUs  = 0;
            c.dwellMs = 0;
            c.abs     = false;
            c.hr      = false;
            c.deg     = qAbs(d) / d2r;
            c.rpm     = (d > 0 ? 1 : -1) * (int)rpmFirst.value(node);
            relOut->append(c);
            notes->append(QStringLiteral("节点 %1 跑 %2 遍: 末尾自动补一段 %3 度 @ %4 rpm 回到第一个目标")
                          .arg(node).arg(loops).arg(c.deg, 0, 'f', 3).arg(qAbs(c.rpm)));
        }
    }
    return true;
}

bool MainWindow::readSegs(QVector<Seg> *out, QString *err) const
{
    out->clear();
    const int defNode = targetNode();       /* 节点列留空 = 用「目标设备」那一台 */

    for (int r = 0; r < m_tblSeq->rowCount(); ++r) {
        QString cell[5];
        for (int c = 0; c < 5; ++c) {
            QTableWidgetItem *it = m_tblSeq->item(r, c);
            cell[c] = it ? it->text().trimmed() : QString();
        }

        Seg s;
        if (cell[0].isEmpty()) {
            s.node = defNode;
        } else {
            bool okN = false;
            s.node = cell[0].toInt(&okN);
            if (!okN) { *err = QStringLiteral("第 %1 行节点号不是数字").arg(r + 1); return false; }
        }
        if (s.node < NODE_MIN || s.node > NODE_MAX) {
            *err = QStringLiteral("第 %1 行节点号 = %2 —— 要 1..63。\n"
                                  "(0 是广播, 脚本装不到广播上; 节点列留空 = 用上面「目标设备」选的那一台)")
                     .arg(r + 1).arg(s.node);
            return false;
        }

        bool ok[4] = { false, false, false, false };
        s.waitUs  = cell[1].toInt(&ok[0]);
        s.rpm     = cell[2].toInt(&ok[1]);
        s.deg     = cell[3].toDouble(&ok[2]);
        s.dwellMs = cell[4].toInt(&ok[3]);
        if (!ok[0] || !ok[1] || !ok[2] || !ok[3]) {
            *err = QStringLiteral("第 %1 行有空单元格或非数字").arg(r + 1); return false;
        }
        if (s.waitUs < 0 || s.waitUs > 10000000) {
            *err = QStringLiteral("第 %1 行「等待」超出 0..10000000 us").arg(r + 1); return false;
        }
        if (s.rpm == 0) {
            *err = QStringLiteral("第 %1 行速度为 0 —— 没有速度的段什么也不做").arg(r + 1); return false;
        }
        if (qAbs(s.rpm) > 1432) {
            *err = QStringLiteral("第 %1 行 |速度| 超过 1432 rpm").arg(r + 1); return false;
        }
        s.abs = seqAbsMode();
        if (s.abs) {
            if (qAbs(s.deg) > SEQ_DEG_MAX_UART) {
                *err = QStringLiteral("第 %1 行 |绝对角度| 超过 36000 度(100 圈)").arg(r + 1); return false;
            }
        } else {
            if (s.deg <= 0.0) {
                *err = QStringLiteral("第 %1 行角度必须写正数(方向看速度的符号)").arg(r + 1); return false;
            }
            if (s.deg > SEQ_DEG_MAX_UART) {
                *err = QStringLiteral("第 %1 行角度超过 36000 度(100 圈)").arg(r + 1); return false;
            }
        }
        if (s.dwellMs < 0 || s.dwellMs > 60000) {
            *err = QStringLiteral("第 %1 行「停留」超出 0..60000 ms").arg(r + 1); return false;
        }
        out->append(s);
    }
    if (out->isEmpty()) { *err = QStringLiteral("表格是空的"); return false; }
    /* 段数上限是"每台 16 段"(板载队列是每块板各一份), 所以这里不卡总数 ——
     * 拆段 + 按台统计在 groupForCan() 里做, 超了会在预览/发送时明确报出来。 */
    return true;
}

QStringList MainWindow::segsToUartLines(const QVector<Seg> &segs) const
{
    QStringList lines;
    QString cur;
    for (const Seg &s : segs) {
        const QString f = QStringLiteral("{%1,%2,%3,%4}")
            .arg(s.waitUs).arg(s.rpm).arg(QString::number(s.deg, 'g', 9)).arg(s.dwellMs);
        /* 固件一行只收得下 ~128 字符(LINE_MAX), 留点余量按 120 拆行, 分行是「追加」的语义 */
        if (!cur.isEmpty() && cur.size() + f.size() + 2 > 120) { lines << cur; cur.clear(); }
        cur += f;
    }
    if (!cur.isEmpty()) lines << cur;
    return lines;
}

/* 按节点分组 + 按角度上限拆段。
 *
 * 拆段口径 (依据 can_bus.h 功能 9 / 功能 11):
 *   · 角度 <= 3276.7 度: 一帧 功能9  (int16 x 0.1 度, 刻度细)
 *   · 角度 更大     : 一帧 功能11 (int16 x 1 度, 大角度版, 单帧到 32767 度 = 90.9 圈)
 *   · 再大才真拆成几帧 (每帧都占一个队列名额, 每台只有 16 个, 所以能用大角度版就用)
 * 每台的段序 = 表格里属于它的行, 按原顺序 —— 固件侧就是按这个顺序跑的。 */
QMap<int, QVector<MainWindow::Seg> > MainWindow::groupForCan(const QVector<Seg> &segs, QStringList *notes) const
{
    QMap<int, QVector<Seg> > byNode;

    for (const Seg &s : segs) {
        QVector<Seg> chunks;
        double remain = s.deg;
        bool first = true;
        int guard = 0;
        while (remain > 1e-9 && guard++ < 256) {
            const bool hr = (s.deg > SEQ_DEG_MAX_CAN + 1e-9);
            const double maxDeg = hr ? SEQ_DEG_MAX_CAN_HR : SEQ_DEG_MAX_CAN;
            const double chunk = qMin(remain, maxDeg);

            Seg c = s;
            c.hr      = hr;
            c.deg     = chunk;
            c.waitUs  = first ? s.waitUs : 0;                      /* 等待只放在第一小段 */
            c.dwellMs = (remain - chunk <= 1e-9) ? s.dwellMs : 0;   /* 停留只放在最后小段 */

            if (c.waitUs > 6553500) {                              /* 功能9/11 b0..1 uint16 x100us */
                c.waitUs = 6553500;
                notes->append(QStringLiteral("节点 %1: 等待超过 CAN 字段上限 6.5535 s, 已钳位 —— 想等更久请拆成多次发送").arg(s.node));
            }
            if (c.dwellMs > 655350) {                              /* 功能9/11 b6..7 uint16 x10ms */
                c.dwellMs = 655350;
                notes->append(QStringLiteral("节点 %1: 停留超过 CAN 字段上限 655.35 s, 已钳位").arg(s.node));
            }
            chunks.append(c);
            remain -= chunk;
            first = false;
        }
        byNode[s.node] += chunks;
    }

    for (QMap<int, QVector<Seg> >::const_iterator it = byNode.constBegin(); it != byNode.constEnd(); ++it) {
        if (it.value().size() > SEQ_MAX_SEG) {
            notes->append(QStringLiteral("节点 %1 拆完 %2 段, 超过每台队列上限 %3 段 —— 这一台发不完")
                          .arg(it.key()).arg(it.value().size()).arg(SEQ_MAX_SEG));
        }
    }
    return byNode;
}

void MainWindow::seqPreview()
{
    QVector<Seg> segs;
    QString err;
    if (!readSegs(&segs, &err)) {
        m_txtSeqPreview->setPlainText(QStringLiteral("表格有问题: ") + err);
        return;
    }
    const int loops = m_spSeqLoops->value();

    const bool absMode = seqAbsMode();
    QVector<Seg> sendSegs = segs;
    QStringList absNotes;
    if (absMode) {
        if (!absSeqToDeltas(segs, loops, &sendSegs, &absNotes, &err)) {
            m_txtSeqPreview->setPlainText(QStringLiteral("绝对模式: ") + err);
            return;
        }
        if (sendSegs.isEmpty()) {
            m_txtSeqPreview->setPlainText(QStringLiteral("绝对模式: 所有目标都等于当前位置, 没有可下发的段。"));
            return;
        }
    }

    QStringList notes = absNotes;
    const QMap<int, QVector<Seg> > byNode = groupForCan(sendSegs, &notes);

    int over = 0;
    for (QMap<int, QVector<Seg> >::const_iterator it = byNode.constBegin(); it != byNode.constEnd(); ++it)
        if (it.value().size() > SEQ_MAX_SEG) over++;

    QString out;
    out += QStringLiteral("共 %1 段, 跑 %2 遍, 分到 %3 台节点\n").arg(segs.size()).arg(loops).arg(byNode.size());
    if (absMode) {
        out += QStringLiteral("\n=== 绝对模式: 表里是机械坐标(度), 下发前换算成相对增量 ===\n");
        for (int i = 0; i < segs.size(); ++i)
            out += QStringLiteral("  第 %1 行: 目标 %2 度 (%3 圈)\n")
                     .arg(i + 1).arg(QString::number(segs.at(i).deg, 'f', 3))
                     .arg(segs.at(i).deg / 360.0, 0, 'f', 4);
        out += QStringLiteral("  实际下发 %1 段 (含多遍时每台的回程段)\n").arg(sendSegs.size());
    }
    out += QStringLiteral("多节点的打法: 广播使能 → 每台「清队列 + 装它自己的段」→ 最后广播一帧「开始」。\n"
                          "所有板在同一个 CAN 帧上起跑, 各自的时间轴都从那一刻算。\n"
                          "这是「共享时间表 + 各自相对等待」, 不是硬时间同步 —— 某台某段比标称耗时长,\n"
                          "那一台后面的段会整体顺延 (台与台之间有毫秒~几十毫秒级漂移)。\n\n");

    out += QStringLiteral("=== CAN: 每台各装各的 (ID = (节点号 << 5) | 功能号) ===\n");
    for (QMap<int, QVector<Seg> >::const_iterator it = byNode.constBegin(); it != byNode.constEnd(); ++it) {
        const int node = it.key();
        const QVector<Seg> &v = it.value();
        out += QStringLiteral("-- 节点 %1: %2 段%3\n")
                 .arg(node).arg(v.size())
                 .arg(v.size() > SEQ_MAX_SEG ? QStringLiteral("   ★超过每台 16 段上限, 发不完") : QString());
        out += QStringLiteral("   0x%1  [00 00]    清队列\n")
                 .arg(L103::canId(node, FN_SEGCTL), 3, 16, QLatin1Char('0')).toUpper();
        for (const Seg &s : v) {
            out += QStringLiteral("   0x%1  [%2]   %3 rpm, %4 度, 等 %5us, 停 %6ms%7\n")
                     .arg(L103::canId(node, s.hr ? FN_SEG_HR : FN_SEG), 3, 16, QLatin1Char('0')).toUpper()
                     .arg(hexOf(s.hr ? buildSegHr(s.waitUs, s.rpm, s.deg, s.dwellMs)
                                     : buildSeg  (s.waitUs, s.rpm, s.deg, s.dwellMs)))
                     .arg(s.rpm).arg(QString::number(s.deg, 'g', 9)).arg(s.waitUs).arg(s.dwellMs)
                     .arg(s.hr ? QStringLiteral("   [大角度版: 1 度刻度]") : QString());
        }
    }
    out += QStringLiteral("\n=== 起跑 (广播一帧: 所有板在同一时刻开始) ===\n");
    out += QStringLiteral("   0x%1  [%2]   功能10 脚本开始, %3 遍\n")
             .arg(L103::canId(NODE_BCAST, FN_SEGCTL), 3, 16, QLatin1Char('0')).toUpper()
             .arg(hexOf(buildSegCtl(1, loops))).arg(loops);

    out += QStringLiteral("\n=== UART 文本脚本 (只有 UART 时用 —— 没有节点号, 只到串口接线那一台) ===\n");
    out += QStringLiteral("*0                        <- 清队列 (运行中的脚本不会被它停掉!)\n");
    const QStringList lines = segsToUartLines(segs);
    for (const QString &l : lines)
        out += QStringLiteral("*%1\n").arg(l);
    out += QStringLiteral("*!%1\n").arg(loops);

    if (byNode.size() > 1)
        out += QStringLiteral("\n★ 这张表里有 %1 台节点 —— UART 文本脚本发不了多节点, 请把通道选成 CAN。\n").arg(byNode.size());
    if (over > 0)
        out += QStringLiteral("\n★ 有 %1 台节点的段数超过每台上限 %2 段, 按「发送」会被拦下来。\n")
                 .arg(over).arg(SEQ_MAX_SEG);
    for (const QString &n : notes)
        out += QStringLiteral("注意: %1\n").arg(n);

    out += QStringLiteral("\n停脚本: UART \"B1\" / CAN 广播 功能0 PV flags=0x04 (本程序「停止脚本」按钮)\n");
    out += QStringLiteral("跑完自己回机械 0: 板子做不到 —— 得由上位机再补一条 功能3 回零\n");

    m_txtSeqPreview->setPlainText(out);
}

void MainWindow::seqSend()
{
    QVector<Seg> segs;
    QString err;
    if (!readSegs(&segs, &err)) {
        QMessageBox::warning(this, QStringLiteral("序列有问题"), err);
        return;
    }

    const int  loops   = m_spSeqLoops->value();
    const bool absMode = seqAbsMode();
    QStringList absNotes;
    if (absMode) {
        QVector<Seg> rel;
        if (!absSeqToDeltas(segs, loops, &rel, &absNotes, &err)) {
            QMessageBox::warning(this, QStringLiteral("绝对序列"), err);
            return;
        }
        if (rel.isEmpty()) {
            QMessageBox::warning(this, QStringLiteral("绝对序列"),
                                 QStringLiteral("所有目标都等于当前位置, 没有可下发的段。"));
            return;
        }
        log(QStringLiteral("[序列] 绝对模式: %1 行目标 -> %2 段相对增量 (基准 = 发命令这一刻的机械位置)")
            .arg(segs.size()).arg(rel.size()));
        segs = rel;
    }
    const int  ch      = m_cbSeqChannel->currentData().toInt();
    const bool useCan  = (ch == 1) || (ch == 0 && frameOpen());
    const bool useUart = (ch == 2) || (ch == 0 && !frameOpen() && m_uart->isOpen());

    if (useCan && !frameOpen())  { QMessageBox::warning(this, QStringLiteral("序列"), QStringLiteral("选了 CAN / 桥, 但帧通道没打开。")); return; }
    if (useUart && !m_uart->isOpen()){ QMessageBox::warning(this, QStringLiteral("序列"), QStringLiteral("选了 UART, 但 UART 没打开。")); return; }
    if (!useCan && !useUart)         { QMessageBox::warning(this, QStringLiteral("序列"), QStringLiteral("先把 UART 或 CAN 打开, 或在下面显式选一个通道。")); return; }

    QStringList notes = absNotes;
    QMap<int, QVector<Seg> > byNode;

    if (useCan) {
        byNode = groupForCan(segs, &notes);
        QStringList over;
        for (QMap<int, QVector<Seg> >::const_iterator it = byNode.constBegin(); it != byNode.constEnd(); ++it)
            if (it.value().size() > SEQ_MAX_SEG)
                over << QStringLiteral("节点 %1: 拆完 %2 段").arg(it.key()).arg(it.value().size());
        if (!over.isEmpty()) {
            QMessageBox::warning(this, QStringLiteral("段数超了"),
                QStringLiteral("每台板载队列只有 %1 段, 下面这些超了:\n%2\n\n"
                               "办法: 减小角度, 或者把动作拆成几批(等第一批发完再发第二批),\n"
                               "或者把这一台的动作分成多条序列轮流发。")
                .arg(SEQ_MAX_SEG).arg(over.join(QStringLiteral("\n"))));
            return;
        }
    } else {
        /* UART 文本脚本**没有节点号** —— 只到串口接线那一台 */
        QSet<int> nodes;
        for (const Seg &s : segs) nodes.insert(s.node);
        if (nodes.size() > 1) {
            QMessageBox::warning(this, QStringLiteral("UART 发不了多节点"),
                QStringLiteral("这张表里有 %1 台节点, 而 UART 文本脚本没有节点号 —— 它只会到串口接线的那一台。\n"
                               "要多节点一起跑, 请把「通道」选成 CAN (固件支持「每台各装各的 + 广播起跑」)。")
                .arg(nodes.size()));
            return;
        }
        byNode[segs.first().node] = segs;
    }

    double totalDeg = 0.0;
    for (const Seg &s : segs) totalDeg += qAbs(s.deg);

    log(QStringLiteral("[动作] 发送动作脚本: %1 段 x %2 遍, 合计 %3 度, 分到 %4 台节点 —— 轴会自己动起来, 清场, 别再手抓")
        .arg(segs.size()).arg(loops).arg(totalDeg, 0, 'f', 1).arg(byNode.size()));
    for (QMap<int, QVector<Seg> >::const_iterator it = byNode.constBegin(); it != byNode.constEnd(); ++it)
        log(QStringLiteral("        节点 %1: %2 段").arg(it.key()).arg(it.value().size()));

    if (useCan) {
        /* 使能必须是第一步: 固件 Motion_Seq_Start 要求"已对齐 + 已使能", 没使能整个脚本会被拒;
         * 而且 Motion_SetEnabled(1) 一进去就 Motion_Seq_Abort() (motion_pv.c) ——
         * 装完段再发使能会把队列清空。广播发一条, 所有节点在同一个 CAN 帧上被合闸。 */
        autoEnable(true, NODE_BCAST);

        for (QMap<int, QVector<Seg> >::const_iterator it = byNode.constBegin(); it != byNode.constEnd(); ++it) {
            const int node = it.key();
            sendCanFn(FN_SEGCTL, buildSegCtl(0, 0), QStringLiteral("清这一台的脚本队列"), node);
            const QVector<Seg> &v = it.value();
            for (const Seg &s : v) {
                sendCanFn(s.hr ? FN_SEG_HR : FN_SEG,
                          s.hr ? buildSegHr(s.waitUs, s.rpm, s.deg, s.dwellMs)
                               : buildSeg  (s.waitUs, s.rpm, s.deg, s.dwellMs),
                          QStringLiteral("%1 rpm / %2 度 / 等 %3us / 停 %4ms%5")
                            .arg(s.rpm).arg(QString::number(s.deg, 'g', 9)).arg(s.waitUs).arg(s.dwellMs)
                            .arg(s.hr ? QStringLiteral("  (大角度版 1 度刻度)") : QString()),
                          node);
            }
        }
        /* 广播起跑: 这一帧所有板同时收到 —— 各台的时间轴起点对齐到同一时刻 */
        sendCanFn(FN_SEGCTL, buildSegCtl(1, loops),
                  QStringLiteral("广播起跑: 所有节点同一帧开始, %1 遍").arg(loops), NODE_BCAST);
        for (const QString &n : notes) log(QStringLiteral("[!] %1").arg(n));
        return;
    }

    uartNodeNote();
    autoEnable(false);                  /* 使能还是要发在装段之前 (E 会先 abort 队列) */
    sendUart(QStringLiteral("*0"));
    const QStringList lines = segsToUartLines(segs);
    for (const QString &l : lines) sendUart(QStringLiteral("*") + l);
    sendUart(QStringLiteral("*!%1").arg(loops));
}

void MainWindow::seqStop()
{
    /* *0 / 功能10 b0=0 只清队列, 不停正在跑的那一段 —— 实测要用 B1 / 功能0 PV bit2 (命令表 2.5 坑 1)。
     * 多节点时停脚本一律**广播**: 一条序列可能横跨好几台。 */
    if (frameOpen()) {
        sendCanFn(FN_PV, buildPv(0, 0.0, PVF_RELEASE),
                  QStringLiteral("广播停脚本 + 松轴 (总线上每一台)"), NODE_BCAST);
        return;
    }
    if (uartFallbackOpen()) { uartNodeNote(); sendUart(QStringLiteral("B1")); return; }
    log(QStringLiteral("[!] 两个口都没开"));
}
/* ============================== 参数 / 原始 ============================== */
void MainWindow::cfgRead()
{
    if (frameOpen()) {
        const int node = targetNode();
        sendCanFn(FN_QUERY, buildQuery(1), QStringLiteral("问配置帧 (功能18)"), node);
        sendCanFn(FN_QUERY, buildQuery(3), QStringLiteral("问电机参数帧 (功能20)"), node);
        return;
    }
    if (uartFallbackOpen()) { sendUart(QStringLiteral("#")); return; }
    log(QStringLiteral("[!] 两个口都没开"));
}

void MainWindow::cfgWrite()
{
    if (!needSingleNode(QStringLiteral("写电机参数 (会写 Flash)"))) return;

    const int    pp    = m_spPolePairs->value();
    const double stall = m_spStallA->value();
    const double vbus  = m_spVbus->value();

    if (pp == 0 && stall <= 0.0 && vbus <= 0.0) {
        QMessageBox::information(this, QStringLiteral("没有要改的"),
            QStringLiteral("三项都是「0 = 不改」。"));
        return;
    }
    if (stall > 0.0 && (stall < STALL_A_MIN || stall > STALL_A_MAX)) {
        QMessageBox::warning(this, QStringLiteral("堵转阈值越界"),
            QStringLiteral("板子只认 %1 .. %2 A。").arg(STALL_A_MIN, 0, 'f', 1).arg(STALL_A_MAX, 0, 'f', 1));
        return;
    }
    if (vbus > 0.0 && (vbus < VBUS_V_MIN || vbus > VBUS_V_MAX)) {
        QMessageBox::warning(this, QStringLiteral("母线电压越界"),
            QStringLiteral("板子只认 %1 .. %2 V。").arg(VBUS_V_MIN, 0, 'f', 1).arg(VBUS_V_MAX, 0, 'f', 1));
        return;
    }

    QString detail = QStringLiteral("极对数 %1 / 堵转 %2 A / 母线 %3 V")
        .arg(pp == 0 ? QStringLiteral("(不改)") : QString::number(pp))
        .arg(stall <= 0.0 ? QStringLiteral("(不改)") : QString::number(stall, 'f', 2))
        .arg(vbus  <= 0.0 ? QStringLiteral("(不改)") : QString::number(vbus,  'f', 2));
    detail += QStringLiteral("\n\n这几项会写进 Flash。改极对数会把「已对齐」作废, 之后必须重跑对齐 (UART 的 A / CAN 功能13)。");
    log(QStringLiteral("[动作] 写电机参数: ") + detail);

    if (frameOpen()) {
        sendCanFn(FN_MOTORCFG, buildMotorCfg(pp, stall, vbus),
                  QStringLiteral("电机参数(存 Flash) — %1").arg(nodeText(targetNode())));
        return;
    }
    if (uartFallbackOpen()) {
        QString cmd = QStringLiteral("#%1,%2").arg(pp).arg(stall, 0, 'f', 2);
        if (vbus > 0.0) cmd += QStringLiteral(",%1").arg(vbus, 0, 'f', 2);
        uartNodeNote();
        sendUart(cmd);
        return;
    }
    log(QStringLiteral("[!] 两个口都没开"));
}

void MainWindow::cfgVlimWrite()
{
    const double v = m_spVlimRadS->value();
    if (v <= 0.0 || v > VEL_LIMIT_RAD_S) {
        QMessageBox::warning(this, QStringLiteral("超上限"),
            QStringLiteral("范围 0 .. %1 rad/s (= 1432 rpm)。").arg(VEL_LIMIT_RAD_S, 0, 'f', 1));
        return;
    }
    if (frameOpen()) {
        sendCanFn(FN_VLIM, buildVlim(v),
                  QStringLiteral("角度环速度上限 %1 rad/s — %2").arg(v, 0, 'f', 2).arg(nodeText(targetNode())));
        return;
    }
    QMessageBox::information(this, QStringLiteral("需要 CAN / 桥"),
        QStringLiteral("这条只有 CAN / 桥的帧通道 (功能7 速度上限) 有。\nUART 的 L<volts> 是相电压上限, 不是速度上限。"));
}

void MainWindow::cfgFaultReportWrite()
{
    const bool on     = m_chkFaultReport->isChecked();
    const int  period = m_spReportPeriod->value();
    if (frameOpen()) {
        sendCanFn(FN_FAULTCFG, buildFaultCfg(on, (quint8)period),
                QStringLiteral("故障上报 %1, 周期 %2 x100ms — %3")
                  .arg(on ? QStringLiteral("开") : QStringLiteral("关")).arg(period).arg(nodeText(targetNode())));
        return;
    }
    QMessageBox::information(this, QStringLiteral("需要 CAN / 桥"),
        QStringLiteral("故障主动上报配置只有 CAN / 桥的帧通道 (功能6 故障上报) 有。\n(UART 的 K0/K1 开的是 CAN 反馈帧, 不是这个。)"));
}

/* 写节点号 (功能12)。防呆位 b1 = 0x5A 写在固件里; 广播发这条会把在场每一台都改成同一个号,
 * 所以本程序直接拦掉 —— 首次编号走串口 %<n> (家人 2026-10-16 的多节点方案)。 */
void MainWindow::actSetNodeNo()
{
    if (!needSingleNode(QStringLiteral("改节点号"))) return;

    const int oldNode = targetNode();
    const int newNode = m_spNewNode->value();
    if (newNode == oldNode) {
        QMessageBox::information(this, QStringLiteral("没必要改"),
            QStringLiteral("「目标设备」已经是节点 %1 了。").arg(oldNode));
        return;
    }

    if (frameOpen()) {
        QByteArray b(2, '\0');
        b[0] = (char)(quint8)newNode;
        b[1] = (char)(quint8)SETNODE_MAGIC;
        sendCanFn(FN_SETNODE, b,
                  QStringLiteral("把节点 %1 改成 %2 (存 Flash, 答复用新号)").arg(oldNode).arg(newNode),
                  oldNode);
        log(QStringLiteral("[动作] 节点号已发出 (功能12) —— 板子的答复会用新号 %1。注意: 老号 %2 会从设备表里掉出去,\n"
                           "        新号自己冒出来; 改完点一次「扫描节点」重新认一遍。")
            .arg(newNode).arg(oldNode));
        return;
    }
    if (uartFallbackOpen()) {
        /* 固件 %<n>: 直接设并存 Flash (串口只到接线那一台, 不用对号) */
        sendUart(QStringLiteral("%") + QString::number(newNode));
        log(QStringLiteral("[动作] 已用串口把这一台改成节点 %1 (固件 %%n 命令) —— 串口这条只到接线的那一台。")
            .arg(newNode));
        return;
    }
    log(QStringLiteral("[!] 两个口都没开"));
}

/* ============================== 参数表 (功能15/23) ============================== */

void MainWindow::resizeParamTable(int count)
{
    if (!m_tblParam) return;
    if (count < 0) count = 0;
    if (count > PARAM_MAX) count = PARAM_MAX;

    /* 2026-10-23: 名字表按"目标节点报上来的板型"选。L103 16 项 / V303 27+ 项,
     * 两张表不能混用 —— 混用的后果是"按名字改参数, 实际改到别的项上"。 */
    {
        const int nd = (targetNode() > 0) ? targetNode() : m_curNode;
        const quint8 bid = (nd > 0) ? devEnsure(nd).info.boardId : 0;
        if (bid != g_paramBoardId) {
            g_paramBoardId = bid;
            m_paramCount = -1;                 /* 逼着重建, 名字要换掉 */
        }
    }
    const QString bname = (g_paramBoardId == 0)
                              ? QStringLiteral("板型未知")
                              : L103::boardName(g_paramBoardId);

    if (count == m_paramCount && m_tblParam->rowCount() == count) {
        if (m_gbpParam)
            m_gbpParam->setTitle(QStringLiteral("参数表 · %1 项 · %2 / CAN 功能15·23").arg(count).arg(bname));
        return;
    }

    /* 项数变了也保留已经读到的值 (按行号对应, 索引没变) */
    QVector<double> old;
    old.reserve(m_paramSpin.size());
    for (int i = 0; i < m_paramSpin.size(); ++i)
        old.append(m_paramSpin[i]->value());

    m_paramSpin.clear();              /* 控件归 table 所有, setRowCount(0) 会删掉 */
    m_tblParam->setRowCount(0);
    m_tblParam->setRowCount(count);

    for (int i = 0; i < count; ++i) {
        m_tblParam->setItem(i, 0, new QTableWidgetItem(QString::number(i)));
        m_tblParam->setItem(i, 1, new QTableWidgetItem(paramNameText(i)));
        QDoubleSpinBox *sp = new QDoubleSpinBox;
        sp->setRange(-1.0e4, 1.0e4);
        sp->setDecimals(4);
        sp->setSingleStep(0.01);
        if (i < old.size()) sp->setValue(old[i]);
        m_tblParam->setCellWidget(i, 2, sp);
        m_paramSpin.append(sp);
        m_tblParam->setItem(i, 3, new QTableWidgetItem(paramUnitText(i)));
        m_tblParam->setItem(i, 4, new QTableWidgetItem(paramDescText(i)));
    }

    m_paramCount = count;
    if (m_gbpParam)
        m_gbpParam->setTitle(QStringLiteral("参数表 · %1 项 · %2 / CAN 功能15·23").arg(count).arg(bname));
}

void MainWindow::paramRedrawState()
{
    if (!m_labParamState) return;

    const int want = m_paramExpect > 0 ? m_paramExpect : m_paramCount;
    QString s = QStringLiteral("已读 %1/%2").arg(m_paramGot).arg(want);
    s += (m_paramFlags & 0x01) ? QStringLiteral("  ·  Flash 里有记录")
                               : QStringLiteral("  ·  Flash 空 (板子用出厂默认)");
    if (m_paramFlags & 0x02) s += QStringLiteral("  ·  有改动未存 Flash");
    if (m_paramFail)         s += QStringLiteral("  ·  上一帧被板子拒绝");
    m_labParamState->setText(s);
}

void MainWindow::paramReadAll()
{
    if (!frameOpen()) {
        QMessageBox::information(this, QStringLiteral("需要 CAN / 桥"),
            QStringLiteral("参数表走 CAN / 桥的帧通道 (功能 15, 应答功能 23)。串口没有这套。"));
        return;
    }

    /* 先问个数 (op INFO), 板子回 index=0xFF / value=个数; 收到后再按实际个数 READ。
     * 所以固件从 27 项加到 35 项, 界面不用改行数。 */
    m_paramExpect = 0;
    m_paramGot    = 0;
    m_paramFail   = false;
    m_labParamState->setText(QStringLiteral("正在问板子参数个数…"));

    sendCanFn(FN_PARAM, buildParamCmd(PARAM_INFO, 0, 0.0f),
              QStringLiteral("参数表: 问参数个数 — %1").arg(nodeText(targetNode())));
    log(QStringLiteral("[参数] 已问板子参数个数; 收到回帧后按回报的个数自动读全表。"));
}

void MainWindow::paramWriteAll()
{
    if (!needSingleNode(QStringLiteral("写参数表"))) return;
    if (!frameOpen()) {
        QMessageBox::information(this, QStringLiteral("需要 CAN / 桥"),
            QStringLiteral("参数表走 CAN / 桥的帧通道 (功能 15)。"));
        return;
    }

    const int n = qMin(m_paramCount, m_paramSpin.size());
    for (int i = 0; i < n; ++i) {
        const float v = (float)m_paramSpin[i]->value();
        sendCanFn(FN_PARAM, buildParamCmd(PARAM_WRITE, (quint8)i, v),
                  QStringLiteral("参数[%1] %2 = %3")
                      .arg(i).arg(paramNameText(i)).arg((double)v, 0, 'f', 4));
    }
    log(QStringLiteral("[参数] %1 项已发出 (只改 RAM, 立刻生效)。要留住请点「存 Flash」。").arg(n));
}

void MainWindow::paramWriteRow()
{
    if (!needSingleNode(QStringLiteral("写单个参数"))) return;
    if (!m_tblParam) return;
    const int row = m_tblParam->currentRow();
    if (row < 0 || row >= m_paramCount || row >= m_paramSpin.size()) {
        QMessageBox::information(this, QStringLiteral("先选一行"),
            QStringLiteral("在表里点一行, 再点这个按钮。"));
        return;
    }
    if (!frameOpen()) return;

    sendCanFn(FN_PARAM, buildParamCmd(PARAM_WRITE, (quint8)row, (float)m_paramSpin[row]->value()),
              QStringLiteral("参数[%1] %2").arg(row).arg(paramNameText(row)));
}

void MainWindow::paramSaveFlash()
{
    if (!needSingleNode(QStringLiteral("存参数到 Flash"))) return;
    if (!frameOpen()) {
        QMessageBox::information(this, QStringLiteral("需要 CAN / 桥"),
            QStringLiteral("参数表走 CAN / 桥的帧通道 (功能 15)。"));
        return;
    }
    log(QStringLiteral("[参数] 正在写 Flash (板子要擦一页, 十几毫秒) —— 电机最好停着。"));
    sendCanFn(FN_PARAM, buildParamCmd(PARAM_SAVE, 0, 0.0f), QStringLiteral("参数表: 存 Flash"));
}

void MainWindow::paramDefaults()
{
    if (!needSingleNode(QStringLiteral("恢复出厂默认"))) return;
    if (!frameOpen()) return;
    sendCanFn(FN_PARAM, buildParamCmd(PARAM_DEFAULTS, 0, 0.0f),
              QStringLiteral("参数表: 恢复出厂默认 (RAM)"));
}

void MainWindow::paramReloadFlash()
{
    if (!needSingleNode(QStringLiteral("从 Flash 重载"))) return;
    if (!frameOpen()) return;
    sendCanFn(FN_PARAM, buildParamCmd(PARAM_RELOAD, 0, 0.0f),
              QStringLiteral("参数表: 从 Flash 重载"));
}

/* ============================== L103 全参数页签 ============================== */

void MainWindow::l103CfgRead()
{
    if (l103UseCan()) {
        /* 2026-10-23: 参数表 18 项 (16 = 跳闸阈值 A, 17 = 跳闸延时 ms), 这里不能写死 */
        m_labL103Cfg->setText(QStringLiteral("正在经 CAN 读取 18 项参数…"));
        sendCanFn(FN_PARAM, buildParamCmd(PARAM_READ, 0, 0.0f, (quint8)18),
                  QStringLiteral("L103 参数: 经 CAN 读 0..17"));
        return;
    }
    if (!m_uart->isOpen()) {
        QMessageBox::warning(this, QStringLiteral("需要 UART"),
            QStringLiteral("这组参数走 L103 的串口命令 (CFG / SAVE / DFLT)。\n"
                           "先把 UART 控制台连上 (COM15, 115200)。"));
        return;
    }
    m_labL103Cfg->setText(QStringLiteral("正在读取 Flash 参数…"));
    sendUart(QStringLiteral("CFG"));
}

void MainWindow::l103CfgWrite()
{
    const bool useCan = l103UseCan();
    if (!useCan && !m_uart->isOpen()) {
        QMessageBox::warning(this, QStringLiteral("需要 UART"),
            QStringLiteral("这组参数走 L103 的串口命令。先把 UART 控制台连上。"));
        return;
    }
    const bool enabled = ((m_st.status & ST_ENABLED) != 0) || (m_cfg.valid && m_cfg.enabled);
    if (enabled) {
        QMessageBox::warning(this, QStringLiteral("先失能"),
            QStringLiteral("轴还在使能状态。改增益/电流/R/L 前先点「失能」，\n"
                           "确认轴不吃力、不再动，再写参数。"));
        return;
    }

    if (useCan) {
        m_l103CanWrites.clear();
        /* 参数索引按板型取 (板型 1 / 2 目前相同: node 在 15, oc 在 16/17)。
         * 写错索引 = 把"速度环 kp"写到别的参数上, 所以这里一个数字都不写死。 */
        const L103ParamIdx ix = l103ParamIdx(currentBoardId());
        /* 先 R/L (会按模型算一遍电流环), 再写用户填的电流环 kp/ki, 顺序与串口路径一致 */
        m_l103CanWrites << qMakePair(ix.oc,   m_spL103Oc->value());   /* 跳闸阈值先写, 免得后面调参时误跳 */
        m_l103CanWrites << qMakePair(ix.ocMs, (double)m_spL103OcMs->value());  /* 再写"顶多久才跳" */
        m_l103CanWrites << qMakePair(ix.r,    m_spL103R->value());
        m_l103CanWrites << qMakePair(ix.l,    m_spL103L->value());
        m_l103CanWrites << qMakePair(ix.curKp, m_spL103CurKp->value());
        m_l103CanWrites << qMakePair(ix.curKi, m_spL103CurKi->value());
        m_l103CanWrites << qMakePair(ix.pp,   (double)m_spL103Pp->value());
        m_l103CanWrites << qMakePair(ix.ilim, m_spL103ILim->value());
        m_l103CanWrites << qMakePair(ix.spdKp, m_spL103SpdKp->value());
        m_l103CanWrites << qMakePair(ix.spdKi, m_spL103SpdKi->value());
        m_l103CanWrites << qMakePair(ix.posKp, m_spL103PosKp->value());
        m_l103CanWrites << qMakePair(ix.vlim, m_spL103Vlim->value());
        m_l103CanWrites << qMakePair(ix.accel, m_spL103Accel->value());
        m_l103CanWrites << qMakePair(ix.alignV, m_spL103Align->value());
        m_l103CanWrites << qMakePair(ix.pll,  m_spL103Pll->value());
        m_l103CanWrites << qMakePair(ix.vs,   (double)m_cbL103Vs->currentData().toInt());
        m_l103CanPos = 0;
        m_labL103Cfg->setText(QStringLiteral("正在经 CAN 写入 %1 项 RAM 参数…")
                                  .arg(m_l103CanWrites.size()));
        l103CanWriteNext();
        return;
    }

    m_l103CfgQueue.clear();
    m_l103CfgQueue << QStringLiteral("PP%1").arg(m_spL103Pp->value());
    m_l103CfgQueue << QStringLiteral("L%1").arg(QString::number(m_spL103ILim->value(), 'f', 3));
    m_l103CfgQueue << QStringLiteral("S%1,%2")
                          .arg(QString::number(m_spL103SpdKp->value(), 'f', 5),
                               QString::number(m_spL103SpdKi->value(), 'f', 2));
    m_l103CfgQueue << QStringLiteral("KP%1").arg(QString::number(m_spL103PosKp->value(), 'f', 2));
    /* 先按 R/L 算一遍, 再用 C 写成用户明确填的电流环增益。 */
    m_l103CfgQueue << QStringLiteral("RL%1,%2")
                          .arg(QString::number(m_spL103R->value(), 'f', 3),
                               QString::number(m_spL103L->value(), 'f', 3));
    m_l103CfgQueue << QStringLiteral("C%1,%2")
                          .arg(QString::number(m_spL103CurKp->value(), 'f', 4),
                               QString::number(m_spL103CurKi->value(), 'f', 1));
    m_l103CfgQueue << QStringLiteral("W%1").arg(QString::number(m_spL103Vlim->value(), 'f', 1));
    m_l103CfgQueue << QStringLiteral("G%1").arg(QString::number(m_spL103Accel->value(), 'f', 1));
    m_l103CfgQueue << QStringLiteral("VA%1").arg(QString::number(m_spL103Align->value(), 'f', 2));
    m_l103CfgQueue << QStringLiteral("PB%1").arg(QString::number(m_spL103Pll->value(), 'f', 1));
    m_l103CfgQueue << QStringLiteral("VS%1").arg(m_cbL103Vs->currentData().toInt());
    m_l103CfgQueue << QStringLiteral("O%1,%2")
                          .arg(QString::number(m_spL103Oc->value(), 'f', 2))
                          .arg(m_spL103OcMs->value());

    m_labL103Cfg->setText(QStringLiteral("正在写入 %1 条 RAM 参数…").arg(m_l103CfgQueue.size()));
    l103CfgQueueNext();
}

void MainWindow::l103CfgQueueNext()
{
    if (m_l103CfgQueue.isEmpty()) {
        m_labL103Cfg->setText(QStringLiteral("RAM 参数已全部发出。要掉电保留，点「存 Flash (SAVE)」。"));
        return;
    }
    const QString cmd = m_l103CfgQueue.takeFirst();
    sendUart(cmd);
    QTimer::singleShot(80, this, &MainWindow::l103CfgQueueNext);
}

bool MainWindow::l103UseCan() const
{
    return m_cbL103Ch && m_cbL103Ch->currentData().toInt() == 1 && m_can && frameOpen();
}

void MainWindow::l103CanWriteNext()
{
    if (m_l103CanPos >= m_l103CanWrites.size()) {
        m_labL103Cfg->setText(QStringLiteral(
            "CAN 参数已全部发出（15 项；节点号不在这里改）。要掉电保留，点「存 Flash (SAVE)」。"));
        return;
    }
    const QPair<int, double> w = m_l103CanWrites.at(m_l103CanPos++);
    sendCanFn(FN_PARAM, buildParamCmd(PARAM_WRITE, (quint8)w.first, (float)w.second),
              QStringLiteral("L103 参数写[%1]=%2").arg(w.first).arg(w.second));
    QTimer::singleShot(30, this, &MainWindow::l103CanWriteNext);
}

void MainWindow::l103CfgSave()
{
    if (l103UseCan()) {
        const bool moving = (m_cfg.valid && m_cfg.enabled) || ((m_st.status & ST_ENABLED) != 0);
        if (moving) {
            QMessageBox::warning(this, QStringLiteral("先失能"),
                QStringLiteral("存 Flash 会擦一页，期间电流环会丢几拍。\n"
                               "请先失能、让电机停住，再点 SAVE。"));
            return;
        }
        sendCanFn(FN_PARAM, buildParamCmd(PARAM_SAVE, 0, 0.0f),
                  QStringLiteral("L103 参数: CAN SAVE"));
        m_labL103Cfg->setText(QStringLiteral("CAN SAVE 已发出（板子回 index=0xFD value=1 才算成功）。"));
        return;
    }
    if (!m_uart->isOpen()) {
        QMessageBox::warning(this, QStringLiteral("需要 UART"),
            QStringLiteral("SAVE 走串口。先把 UART 控制台连上。"));
        return;
    }
    const bool moving = (m_cfg.valid && m_cfg.enabled) || ((m_st.status & ST_ENABLED) != 0);
    if (moving) {
        QMessageBox::warning(this, QStringLiteral("先失能"),
            QStringLiteral("存 Flash 会擦一页，期间电流环会丢几拍。\n"
                           "请先失能、让电机停住，再点 SAVE。"));
        return;
    }
    sendUart(QStringLiteral("SAVE"));
    m_labL103Cfg->setText(QStringLiteral("SAVE 已发送（板子回 OK 才算真正写进去）。"));
}

void MainWindow::l103CfgErase()
{
    if (l103UseCan()) {
        if (QMessageBox::question(this, QStringLiteral("擦除 Flash 参数"),
                QStringLiteral("只擦掉 Flash 里的保存记录，不会立刻改当前 RAM；\n"
                               "断电重上电后才回默认。继续？"),
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
            return;
        sendCanFn(FN_PARAM, buildParamCmd(PARAM_DEFAULTS, 0, 0.0f),
                  QStringLiteral("L103 参数: CAN DFLT"));
        m_labL103Cfg->setText(QStringLiteral("CAN DFLT 已发出（回 index=0xFC value=1 才算成功）。"));
        return;
    }
    if (!m_uart->isOpen()) {
        QMessageBox::warning(this, QStringLiteral("需要 UART"),
            QStringLiteral("DFLT 走串口。先把 UART 控制台连上。"));
        return;
    }
    if (QMessageBox::question(this, QStringLiteral("擦除 Flash 参数"),
            QStringLiteral("只擦掉 Flash 里的保存记录，不会立刻改当前 RAM；\n"
                           "断电重上电后才回默认。继续？"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
        return;
    sendUart(QStringLiteral("DFLT"));
    m_labL103Cfg->setText(QStringLiteral("DFLT 已发送；断电重上电后回默认。"));
}

/* CAN 功能号 23 的应答: 一帧一项, b1=索引 b2..5=float b6=flags b7=0xA5。
 * 索引与固件 can_bus.c 的 param_get() 一一对应。 */
void MainWindow::l103CfgApplyParam(const L103::ParamRsp &f)
{
    if (!f.valid) return;

    if (f.flags & 0x04)
        m_labL103Cfg->setText(QStringLiteral("板子拒绝了这条（索引 %1）：越界 / 使能中改极对数 / 只读。")
                                  .arg((int)f.index));

    if (f.index == 0xFF) {
        m_labL103Cfg->setText(QStringLiteral("板子报 %1 项参数，正在读取…")
                                  .arg((int)(f.value + 0.5f)));
        return;
    }
    if (f.index == 0xFD) {
        m_labL103Cfg->setText(f.value > 0.5f
                                  ? QStringLiteral("CAN SAVE 成功：参数已进 Flash。")
                                  : QStringLiteral("CAN SAVE 失败（板子拒绝或 Flash 写失败）。"));
        return;
    }
    if (f.index == 0xFC) {
        m_labL103Cfg->setText(f.value > 0.5f
                                  ? QStringLiteral("已擦除 Flash 记录；断电重上电回默认。")
                                  : QStringLiteral("擦除失败（板子拒绝或 Flash 擦除失败）。"));
        return;
    }

    /* 参数索引按板型映射 (板型 1 / 2 目前相同, 见 l103ParamIdx), 不写死数字。
     * 节点号 (index 15) 界面上没有对应输入框 (用 % / 功能号 12 改), 这里忽略。 */
    const L103ParamIdx ix = l103ParamIdx(currentBoardId());
    const int v = (int)(f.value + 0.5f);
    if      (f.index == ix.pp)     m_spL103Pp->setValue(v);
    else if (f.index == ix.ilim)   m_spL103ILim->setValue(f.value);
    else if (f.index == ix.vbus)   m_spL103Vbus->setValue(f.value);
    else if (f.index == ix.spdKp)  m_spL103SpdKp->setValue(f.value);
    else if (f.index == ix.spdKi)  m_spL103SpdKi->setValue(f.value);
    else if (f.index == ix.posKp)  m_spL103PosKp->setValue(f.value);
    else if (f.index == ix.curKp)  m_spL103CurKp->setValue(f.value);
    else if (f.index == ix.curKi)  m_spL103CurKi->setValue(f.value);
    else if (f.index == ix.vlim)   m_spL103Vlim->setValue(f.value);
    else if (f.index == ix.accel)  m_spL103Accel->setValue(f.value);
    else if (f.index == ix.alignV) m_spL103Align->setValue(f.value);
    else if (f.index == ix.pll)    m_spL103Pll->setValue(f.value);
    else if (f.index == ix.vs) {
        const int i = m_cbL103Vs->findData(v);
        if (i >= 0) m_cbL103Vs->setCurrentIndex(i);
    }
    else if (f.index == ix.r)      m_spL103R->setValue(f.value);
    else if (f.index == ix.l)      m_spL103L->setValue(f.value);
    else if (f.index == ix.oc)     m_spL103Oc->setValue(f.value);
    else if (f.index == ix.ocMs)   m_spL103OcMs->setValue(v);
}

void MainWindow::l103CfgApplyLine(const QString &line)
{
    if (!line.startsWith(QStringLiteral("NVCFG:"))) return;

    static const QRegularExpression reHead(
        QStringLiteral("^NVCFG: node=(\\d+) pp=(\\d+) ilim=([0-9.]+) A vbus=([0-9.]+) V"));
    static const QRegularExpression reSpd(
        QStringLiteral("^NVCFG: spd ([0-9.]+)/([0-9.]+) pos ([0-9.]+)"));
    static const QRegularExpression reCur(
        QStringLiteral("^NVCFG: cur ([0-9.]+)/([0-9.]+)"));
    static const QRegularExpression reRL(
        QStringLiteral("^NVCFG: R=([0-9.]+) L=([0-9.]+)mH vlim=([0-9.]+) acc=([0-9.]+)"));
    static const QRegularExpression reAlign(
        QStringLiteral("^NVCFG: align=([0-9.]+)V pll=([0-9.]+)Hz vs=(\\d+)"));
    static const QRegularExpression reOc(
        QStringLiteral("^NVCFG: oc_trip=([0-9.]+) A"));
    static const QRegularExpression reOcMs(
        QStringLiteral("^NVCFG: oc_trip_ms=([0-9]+) ms"));

    QRegularExpressionMatch m;
    m = reHead.match(line);
    if (m.hasMatch()) {
        m_spL103Pp->setValue(m.captured(2).toInt());
        m_spL103ILim->setValue(m.captured(3).toDouble());
        m_spL103Vbus->setValue(m.captured(4).toDouble());
        m_labL103Cfg->setText(QStringLiteral("已读: node=%1 pp=%2 — 继续读…")
                                  .arg(m.captured(1), m.captured(2)));
        return;
    }
    m = reSpd.match(line);
    if (m.hasMatch()) {
        m_spL103SpdKp->setValue(m.captured(1).toDouble());
        m_spL103SpdKi->setValue(m.captured(2).toDouble());
        m_spL103PosKp->setValue(m.captured(3).toDouble());
        return;
    }
    m = reCur.match(line);
    if (m.hasMatch()) {
        m_spL103CurKp->setValue(m.captured(1).toDouble());
        m_spL103CurKi->setValue(m.captured(2).toDouble());
        return;
    }
    m = reRL.match(line);
    if (m.hasMatch()) {
        m_spL103R->setValue(m.captured(1).toDouble());
        m_spL103L->setValue(m.captured(2).toDouble());
        m_spL103Vlim->setValue(m.captured(3).toDouble());
        m_spL103Accel->setValue(m.captured(4).toDouble());
        return;
    }
    m = reAlign.match(line);
    if (m.hasMatch()) {
        m_spL103Align->setValue(m.captured(1).toDouble());
        m_spL103Pll->setValue(m.captured(2).toDouble());
        const int idx = m_cbL103Vs->findData(m.captured(3).toInt());
        if (idx >= 0) m_cbL103Vs->setCurrentIndex(idx);
        return;
    }
    m = reOc.match(line);
    if (m.hasMatch()) {
        m_spL103Oc->setValue(m.captured(1).toDouble());
        return;
    }
    m = reOcMs.match(line);
    if (m.hasMatch()) {
        m_spL103OcMs->setValue(m.captured(1).toInt());
        m_labL103Cfg->setText(QStringLiteral("已从 Flash 读取全部参数。"));
    }
}

void MainWindow::applyParamRsp(const ParamRsp &f, int node)
{
    if (node >= NODE_MIN) touchDevice(node);

    /* 桥专用「参数(桥)」页: 按「目标设备」那一台回填。 */
    applyBrParamRsp(f, node);

    if (node != m_curNode) return;

    m_paramFlags = f.flags;
    m_paramFail  = (f.flags & 0x04) != 0;

    switch (f.op) {
    case PARAM_INFO:
    {
        /* 板子回报的个数是唯一口径; 界面按它建表, 不写死 27/35。 */
        int cnt = (int)(f.value + 0.5f);
        if (cnt < 0) cnt = 0;
        if (cnt > PARAM_MAX) {
            log(QStringLiteral("[!] 板子报 %1 项, 界面最多支持 %2 项 —— 只显示前 %2 项。")
                .arg(cnt).arg(PARAM_MAX));
            cnt = PARAM_MAX;
        }
        resizeParamTable(cnt);
        m_paramExpect = cnt;
        m_paramGot    = 0;
        m_paramFail   = false;
        log(QStringLiteral("[参数] 板子回报 %1 项, 开始读取…").arg(cnt));
        if (cnt > 0) {
            sendCanFn(FN_PARAM, buildParamCmd(PARAM_READ, 0, 0.0f, (quint8)cnt),
                      QStringLiteral("参数表: 读 0..%1 (%2 项) — %3")
                          .arg(cnt - 1).arg(cnt).arg(nodeText(node)));
        }
        break;
    }
    case PARAM_READ:
        if (f.index < PARAM_MAX) {
            /* 板子回了个比当前表更靠后的索引 (理论上 INFO 已对齐) -> 长到能放下它 */
            if (f.index >= m_paramCount) resizeParamTable(f.index + 1);
            if (f.index < m_paramSpin.size()) {
                m_paramSpin[f.index]->setValue((double)f.value);
                m_paramGot++;
            }
        }
        break;
    case PARAM_WRITE:
        if (f.index < PARAM_MAX && f.index < m_paramSpin.size())
            m_paramSpin[f.index]->setValue((double)f.value);
        if (m_paramFail) {
            log(QStringLiteral("[!] 板子拒绝写参数 %1 (%2) —— 越界, 或使能状态下改极对数")
                .arg(f.index).arg(paramNameText(f.index)));
        }
        break;
    case PARAM_SAVE:
        log(f.value > 0.5f ? QStringLiteral("[参数] 存 Flash: OK")
                           : QStringLiteral("[!] 存 Flash: 失败 —— 看串口日志 (MCFG: 那几行)"));
        break;
    case PARAM_DEFAULTS:
        log(QStringLiteral("[参数] 板子已恢复出厂默认 (只改 RAM, 要留住请再存 Flash)"));
        paramReadAll();
        break;
    case PARAM_RELOAD:
        log(f.value > 0.5f ? QStringLiteral("[参数] 已从 Flash 重载")
                           : QStringLiteral("[!] Flash 里没有有效记录, 重载失败"));
        paramReadAll();
        break;
    default:
        break;
    }
    paramRedrawState();
}

void MainWindow::rawUartSend()
{
    const QString s = m_editRawUart->text().trimmed();
    if (s.isEmpty()) return;
    sendUart(s);
}

void MainWindow::rawCanSend()
{
    QString hexId = m_editRawCanId->text().trimmed();
    if (hexId.startsWith(QStringLiteral("0x"), Qt::CaseInsensitive)) hexId = hexId.mid(2);
    bool ok = false;
    const quint32 id = hexId.toUInt(&ok, 16);
    if (!ok || hexId.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("ID 不对"),
            QStringLiteral("CAN ID 写十六进制, ID = (节点号 << 5) | 功能号 ——\n"
                           "例如 节点1 询问 = 0x25, 广播询问 = 0x05, 节点2 使能 = 0x42。"));
        return;
    }

    QByteArray data;
    const QString ds = m_editRawCanData->text();
    QString tok;
    for (int i = 0; i <= ds.size(); ++i) {
        const QChar c = (i < ds.size()) ? ds.at(i) : QChar(QLatin1Char(' '));
        const bool sep = c.isSpace() || c == QLatin1Char(',')
                      || c == QLatin1Char('-') || c == QLatin1Char(':');
        if (!sep) { tok += c; continue; }
        if (tok.isEmpty()) continue;

        bool ok2 = false;
        const uint b = tok.toUInt(&ok2, 16);
        if (!ok2 || b > 0xFFu) {
            QMessageBox::warning(this, QStringLiteral("数据不对"),
                QStringLiteral("「%1」不是 00..FF 的十六进制字节。").arg(tok));
            return;
        }
        data.append((char)(quint8)b);
        tok.clear();
    }
    if (data.size() > 8) {
        QMessageBox::warning(this, QStringLiteral("太长了"),
            QStringLiteral("标准帧最多 8 字节, 现在 %1 字节。").arg(data.size()));
        return;
    }
    sendCan(id, data, QStringLiteral("原始发送"));
}

/* ============================== 波形槽 ============================== */
void MainWindow::waveStart()
{
    /* 板型 2 固件没有 SW 系列命令, 也没有三相采样 —— 一条 SWD/SW1 都不许发。 */
    if (currentBoardId() == 2u) {
        QMessageBox::information(this, QStringLiteral("本板没有三相电流"),
            QStringLiteral("L103 + DRV8313 V2 只有一路低边总电流 i_ls, 没有三相 ia/ib/ic;\n"
                           "这块板的固件已经取消 SW 相电流波形流, 所以这里不能开波形。\n\n"
                           "i_ls 在「状态」页和设备表的「电流(A)」列看。"));
        return;
    }
    if (!m_uart->isOpen()) {
        QMessageBox::warning(this, QStringLiteral("波形流要 UART"),
            QStringLiteral("相电流波形流只从 UART 控制台出 (CAN 没有这条通路)。\n\n"
                           "先把上面那行的 UART 打开 (COM15 @115200), 再点开始。"));
        return;
    }
    if (m_waveStreaming) return;

    const int div = m_cbWaveDiv->currentData().toInt();

    /* 先在文本模式把分频发过去, 让它的回显能被正常解析、写进日志 (350ms 只是等回显跑完) */
    sendUart(QStringLiteral("SWD%1").arg(div));

    QTimer::singleShot(350, this, [this, div]() {
        if (!m_uart->isOpen() || m_waveStreaming) return;

        /* 波形流期间串口是二进制, 1 Hz 那条 S 轮询的 ASCII 回复会混进来。
         * 只有"没开 CAN"时才需要停它 —— CAN 开着的话状态走 CAN, UART 上本来就没流量。 */
        if (!frameOpen() && m_chkAutoStatus->isChecked() && m_timerStatus->isActive()) {
            m_timerStatus->stop();
            m_wavePollPaused = true;
        }

        m_uart->setRawMode(true);       /* 二进制里到处是假换行, 不能再按行切 */
        m_plot->clear();
        m_waveStreaming = true;
        m_btnWaveStart->setEnabled(false);
        m_btnWaveStop->setEnabled(true);

        sendUart(QStringLiteral("SW1"));

        /* SW1 的 ASCII 回显、以及之前还在环形缓冲里的文本会先涌进来;
         * 400ms 之后再清零计数 —— 这样"误同步/杂散"才干净可用。 */
        QTimer::singleShot(400, this, [this]() {
            if (!m_waveStreaming) return;
            m_waveParser->reset();
            m_waveLastFrames = 0;
            m_timerWaveStat->start();
        });

        log(QStringLiteral("[波形] 开始 (SWD%1)。串口转二进制, 日志页暂时没有文本。").arg(div));
    });
}

void MainWindow::waveStop()
{
    /* 板型 2: 固件没有 SW0。只在本地收尾, 不往串口发任何东西。 */
    const bool noSw = (currentBoardId() == 2u);

    if (!m_waveStreaming) {
        /* 保险: 板子可能还在发 (比如手动敲过 SW1) —— 照样送一条 SW0 */
        if (!noSw && m_uart->isOpen()) sendUart(QStringLiteral("SW0"));
        return;
    }

    if (!noSw) sendUart(QStringLiteral("SW0"));
    m_timerWaveStat->stop();

    /* 等固件 TX 环形缓冲里剩下的二进制吐完, 再切回文本模式 ——
     * 切早了的话, 后面那些 ASCII 回复会按二进制喂进解析器。 */
    QTimer::singleShot(300, this, [this, noSw]() {
        if (!m_waveStreaming) return;

        m_waveStreaming = false;
        m_uart->setRawMode(false);
        m_btnWaveStart->setEnabled(!noSw);      /* 板型 2 不开波形 */
        m_btnWaveStop->setEnabled(false);

        if (m_wavePollPaused) {
            m_wavePollPaused = false;
            if (m_chkAutoStatus->isChecked()) m_timerStatus->start();
        }
        log(noSw ? QStringLiteral("[波形] 已本地停止 (板型 2 无 SW, 未发 SW0), 串口恢复文本。")
                 : QStringLiteral("[波形] 已停 (SW0), 串口恢复文本。"));
    });
}

void MainWindow::waveClear()
{
    if (m_plot)        m_plot->clear();
    if (m_waveParser)  m_waveParser->reset();
    m_waveLastFrames = 0;
    if (m_labWaveStat) m_labWaveStat->setText(QStringLiteral("已清空"));
}

void MainWindow::waveStatTick()
{
    if (!m_waveParser || !m_labWaveStat) return;

    const quint64 f  = m_waveParser->frames();
    const quint64 df = (f >= m_waveLastFrames) ? (f - m_waveLastFrames) : f;
    m_waveLastFrames = f;

    m_labWaveStat->setText(QStringLiteral(
        "实收 %1 帧  ·  %2 帧/s  ·  误同步 %3  ·  杂散 %4 字节")
        .arg(f).arg(df).arg(m_waveParser->badFrames()).arg(m_waveParser->strayBytes()));
}

void MainWindow::waveSetBaud()
{
    if (!m_uart->isOpen()) {
        QMessageBox::warning(this, QStringLiteral("先开 UART"),
            QStringLiteral("改板子波特率要先打开 UART。"));
        return;
    }

    const int baud = m_cbWaveBaud->currentData().toInt();
    if (baud == m_uart->baud()) {
        log(QStringLiteral("[波特率] 已经是 %1, 不用改。").arg(baud));
        return;
    }

    log(QStringLiteral("[波特率] 要把板子 UART1 改成 %1, 上位机跟着重开这个口。"
                       "本机实测 (2026-10-06): WCH-LinkE 的串口桥在 460800 上收不全 —— 想真提速得等 CH9433 修好。"
                       "固件里波特率不存 Flash, 板子复位就回到 115200。").arg(baud));

    if (m_waveStreaming) waveStop();

    const QString port = m_uart->portName();
    sendUart(QStringLiteral("SWB%1").arg(baud));
    log(QStringLiteral("[波特率] 已发 SWB%1 —— 等板子切过去再重开口…").arg(baud));

    QTimer::singleShot(600, this, [this, port, baud]() {
        QString err;
        m_uart->close();
        if (!m_uart->open(port, baud, &err)) {
            log(QStringLiteral("[!] 以 %1 重开 %2 失败: %3 —— 退回 115200 再试一次")
                .arg(baud).arg(port, err));
            if (!m_uart->open(port, 115200, &err))
                log(QStringLiteral("[!] 115200 也开不了: %1").arg(err));
            return;
        }
        if (m_cbUartBaud) m_cbUartBaud->setCurrentText(QString::number(baud));
        log(QStringLiteral("[波特率] %1 已按 %2 重开。").arg(port).arg(baud));
    });
}

/* ============================== 桥专用页签 ==============================
 * 这两页只服务 H750 桥:
 *   · 所有帧直接 m_bridge->sendFrame(), 不经过 sendCan()/sendCanFn() 的传输选择;
 *   · 控件全部独立 (一个 widget 只能有一个 parent), 不复用原「参数 / 序列」页;
 *   · 回帧仍走 onCanFrame() -> applyCfg()/applyMotorCfg(), 由那两个函数额外回填本页控件。
 * ========================================================================== */

bool MainWindow::bridgeEnsureOpen(const QString &what)
{
    if (m_bridge && m_bridge->isOpen()) return true;

    QMessageBox::warning(this, QStringLiteral("桥接口没打开"),
        QStringLiteral("%1 只能走桥接口, 但现在桥没连接。\n\n"
                       "请先在「连接」区把「桥接口」的串口选好 (默认 COM6)、点「打开」, 再回来操作。\n"
                       "本页不会改走 CAN / COM。").arg(what));
    return false;
}

bool MainWindow::bridgeSend(L103::CanFn fn, const QByteArray &data,
                            const QString &note, int node)
{
    if (!m_bridge || !m_bridge->isOpen()) return false;

    const quint32 id = L103::canId(node, (quint32)fn);
    QString err;
    if (!m_bridge->sendFrame(id, data, &err)) {
        log(QStringLiteral("[!] 桥发送失败 %1: %2").arg(canIdName(id), err));
        return false;
    }
    logTx(QStringLiteral("桥 %1  [%2]%3").arg(canIdName(id), hexOf(data),
         note.isEmpty() ? QString() : QStringLiteral("   %1").arg(note)));
    return true;
}

/* ------------------------- 参数(桥) ------------------------- */

void MainWindow::brMotorRead()
{
    if (!bridgeEnsureOpen(QStringLiteral("电机参数回读"))) return;
    bridgeSend(FN_QUERY, buildQuery(1), QStringLiteral("问配置帧 (b0=1) — 目标节点"), bridgeNode());
    bridgeSend(FN_QUERY, buildQuery(3), QStringLiteral("问电机参数帧 (b0=3) — 目标节点"), bridgeNode());
}

void MainWindow::brCfgRead()
{
    if (!bridgeEnsureOpen(QStringLiteral("配置回读"))) return;
    bridgeSend(FN_QUERY, buildQuery(1), QStringLiteral("问配置帧 (b0=1) — 目标节点"), bridgeNode());
}

void MainWindow::brMotorWrite()
{
    if (!bridgeEnsureOpen(QStringLiteral("写电机参数"))) return;

    const int    pp    = m_brSpPolePairs->value();
    const double stall = m_brSpStallA->value();
    const double vbus  = m_brSpVbus->value();

    if (pp == 0 && stall <= 0.0 && vbus <= 0.0) {
        QMessageBox::information(this, QStringLiteral("没有要改的"),
            QStringLiteral("三项都是「0 = 不改」。"));
        return;
    }
    if (stall > 0.0 && (stall < STALL_A_MIN || stall > STALL_A_MAX)) {
        QMessageBox::warning(this, QStringLiteral("堵转阈值越界"),
            QStringLiteral("板子只认 %1 .. %2 A。").arg(STALL_A_MIN, 0, 'f', 1).arg(STALL_A_MAX, 0, 'f', 1));
        return;
    }
    if (vbus > 0.0 && (vbus < VBUS_V_MIN || vbus > VBUS_V_MAX)) {
        QMessageBox::warning(this, QStringLiteral("母线电压越界"),
            QStringLiteral("板子只认 %1 .. %2 V。").arg(VBUS_V_MIN, 0, 'f', 1).arg(VBUS_V_MAX, 0, 'f', 1));
        return;
    }

    const QString detail = QStringLiteral("极对数 %1 / 堵转 %2 A / 母线 %3 V")
        .arg(pp == 0 ? QStringLiteral("(不改)") : QString::number(pp))
        .arg(stall <= 0.0 ? QStringLiteral("(不改)") : QString::number(stall, 'f', 2))
        .arg(vbus  <= 0.0 ? QStringLiteral("(不改)") : QString::number(vbus,  'f', 2));
    log(QStringLiteral("[参数(桥)] 写电机参数 (目标节点, 存 Flash): ") + detail);

    bridgeSend(FN_MOTORCFG, buildMotorCfg(pp, stall, vbus),
               QStringLiteral("电机参数(存 Flash) — 目标节点"), bridgeNode());
}

void MainWindow::brVlimWrite()
{
    if (!bridgeEnsureOpen(QStringLiteral("写角度环速度上限"))) return;

    const double v = m_brSpVlimRadS->value();
    if (v <= 0.0 || v > VEL_LIMIT_RAD_S) {
        QMessageBox::warning(this, QStringLiteral("超上限"),
            QStringLiteral("范围 0 .. %1 rad/s (= 1432 rpm)。").arg(VEL_LIMIT_RAD_S, 0, 'f', 1));
        return;
    }

    bridgeSend(FN_VLIM, buildVlim(v),
               QStringLiteral("角度环速度上限 %1 rad/s — 目标节点").arg(v, 0, 'f', 2),
               bridgeNode());
}

void MainWindow::brFaultReportWrite()
{
    if (!bridgeEnsureOpen(QStringLiteral("写故障主动上报配置"))) return;

    const bool on     = m_brChkFaultReport->isChecked();
    const int  period = m_brSpReportPeriod->value();
    bridgeSend(FN_FAULTCFG, buildFaultCfg(on, (quint8)period),
               QStringLiteral("故障上报 %1, 周期 %2 x100ms — 目标节点")
                 .arg(on ? QStringLiteral("开") : QStringLiteral("关")).arg(period),
               bridgeNode());
}

void MainWindow::brQueryStatus()
{
    if (!bridgeEnsureOpen(QStringLiteral("回读状态"))) return;
    bridgeSend(FN_QUERY, buildQuery(0), QStringLiteral("问状态帧 (b0=0) — 目标节点"), bridgeNode());
}

/* -------------------- 电机(桥): 使能 / 对齐 / 速度 (调参页要有这一坨) --------------------
 * 与「L103 调参」页上那两行功能一致, 差别只有三点:
 *   · 帧发「目标设备」那一台;
 *   · 一律 m_bridge->sendFrame(), 不看顶部「传输」;
 *   · 命令后跟着问一帧状态, 好让本页「状态」那一格立刻有更新。
 * 一个串口只有一个写者: 这里全是界面线程按顺序发, 没有并发写。 */
void MainWindow::brActEnable()
{
    if (!bridgeEnsureOpen(QStringLiteral("使能"))) return;
    bridgeSend(FN_ENABLE, buildEnable(true),
               QStringLiteral("使能并保持当前位置 — 目标节点"), bridgeNode());
    bridgeSend(FN_QUERY, buildQuery(0), QStringLiteral("并问一次状态 — 目标节点"), bridgeNode());
}

void MainWindow::brActDisable()
{
    if (!bridgeEnsureOpen(QStringLiteral("失能"))) return;
    bridgeSend(FN_ENABLE, buildEnable(false),
               QStringLiteral("失能 / 松轴 — 目标节点"), bridgeNode());
    bridgeSend(FN_QUERY, buildQuery(0), QStringLiteral("并问一次状态 — 目标节点"), bridgeNode());
}

/* 一键: 广播功能2 b0=0 —— 总线上每一台都失能松轴。
 * 只有这一条命令走广播 (节点 0); 其余动作都按「目标设备」单发。 */
void MainWindow::brActDisableAll()
{
    if (!bridgeEnsureOpen(QStringLiteral("全部失能"))) return;
    bridgeSend(FN_ENABLE, buildEnable(false),
               QStringLiteral("全部失能 / 松轴 (广播 0x02, 所有节点)"), NODE_BCAST);
}

void MainWindow::brActAlign()
{
    if (!bridgeEnsureOpen(QStringLiteral("对齐"))) return;
    log(QStringLiteral("[动作] 对齐(桥) —— 会硬拽转子: 轴要能自由转、没夹持、没拧劲; 完事把此刻位置定义成机械 0 点, 停在失能"));
    bridgeSend(FN_ALIGN, buildAlign(),
               QStringLiteral("电角度对齐 (b0=0x5A) — 目标节点"), bridgeNode());
}

void MainWindow::brActClearFault()
{
    if (!bridgeEnsureOpen(QStringLiteral("清故障"))) return;
    bridgeSend(FN_PV, buildPv(0, 0.0, PVF_CLEAR),
               QStringLiteral("清故障 (bit3, 不动机构) — 目标节点"), bridgeNode());
    bridgeSend(FN_QUERY, buildQuery(0), QStringLiteral("并问一次状态 — 目标节点"), bridgeNode());
}

void MainWindow::brActVelocityWith(double rpm)
{
    if (!bridgeEnsureOpen(QStringLiteral("速度模式"))) return;
    const double rad = rpm * TWO_PI / 60.0;
    if (qAbs(rad) > VEL_LIMIT_RAD_S)
        log(QStringLiteral("[!] 给定 %1 rpm 超过角度环上限 %2 rad/s — 照发, 板子会钳到自己的 vel_limit")
            .arg(rpm, 0, 'f', 1).arg(VEL_LIMIT_RAD_S, 0, 'f', 1));
    log(QStringLiteral("[动作] 桥 · 连续转 %1 rpm —— 会一直转下去, 直到按「停止 (B1)」; 先确认线束不会拧到头")
        .arg(rpm, 0, 'f', 1));
    bridgeSend(FN_ENABLE, buildEnable(true), QStringLiteral("速度前先使能 — 目标节点"), bridgeNode());
    bridgeSend(FN_VEL, buildVel(rad, PVF_RUN),
                QStringLiteral("速度 %1 rpm").arg(rpm, 0, 'f', 1), bridgeNode());
    bridgeSend(FN_QUERY, buildQuery(0), QStringLiteral("并问一次状态 — 目标节点"), bridgeNode());
}

void MainWindow::brActVelocityStop()
{
    if (!bridgeEnsureOpen(QStringLiteral("停止"))) return;
    bridgeSend(FN_VEL, buildVel(0.0, 0x00),
               QStringLiteral("速度停 + 松轴 — 目标节点"), bridgeNode());
    bridgeSend(FN_QUERY, buildQuery(0), QStringLiteral("并问一次状态 — 目标节点"), bridgeNode());
}

/* ------------------------- L103 参数(桥) ------------------------- */

static const int kBrL103ParamCount = 18;

double MainWindow::brL103ParamValue(int idx) const
{
    switch (idx) {
    case 0:  return m_brL103Pp    ? (double)m_brL103Pp->value()    : 0.0;
    case 1:  return m_brL103ILim  ? m_brL103ILim->value()          : 0.0;
    case 2:  return m_brL103Vbus  ? m_brL103Vbus->value()          : 0.0;
    case 3:  return m_brL103SpdKp ? m_brL103SpdKp->value()         : 0.0;
    case 4:  return m_brL103SpdKi ? m_brL103SpdKi->value()         : 0.0;
    case 5:  return m_brL103PosKp ? m_brL103PosKp->value()         : 0.0;
    case 6:  return m_brL103CurKp ? m_brL103CurKp->value()         : 0.0;
    case 7:  return m_brL103CurKi ? m_brL103CurKi->value()         : 0.0;
    case 8:  return m_brL103Vlim  ? m_brL103Vlim->value()          : 0.0;
    case 9:  return m_brL103Accel ? m_brL103Accel->value()         : 0.0;
    case 10: return m_brL103Align ? m_brL103Align->value()         : 0.0;
    case 11: return m_brL103Pll   ? m_brL103Pll->value()           : 0.0;
    case 12: return m_brL103Vs    ? (double)m_brL103Vs->currentData().toInt() : 0.0;
    case 13: return m_brL103R     ? m_brL103R->value()             : 0.0;
    case 14: return m_brL103L     ? m_brL103L->value()             : 0.0;
    case 16: return m_brL103Oc    ? m_brL103Oc->value()            : 0.0;
    case 17: return m_brL103OcMs  ? (double)m_brL103OcMs->value()  : 0.0;
    default: return 0.0;                    /* 15 = 节点号, 只读 */
    }
}

void MainWindow::brL103SetParam(int idx, double value)
{
    switch (idx) {
    case 0:  if (m_brL103Pp)    m_brL103Pp->setValue(qRound(value)); break;
    case 1:  if (m_brL103ILim)  m_brL103ILim->setValue(value); break;
    case 2:  if (m_brL103Vbus)  m_brL103Vbus->setValue(value); break;
    case 3:  if (m_brL103SpdKp) m_brL103SpdKp->setValue(value); break;
    case 4:  if (m_brL103SpdKi) m_brL103SpdKi->setValue(value); break;
    case 5:  if (m_brL103PosKp) m_brL103PosKp->setValue(value); break;
    case 6:  if (m_brL103CurKp) m_brL103CurKp->setValue(value); break;
    case 7:  if (m_brL103CurKi) m_brL103CurKi->setValue(value); break;
    case 8:  if (m_brL103Vlim)  m_brL103Vlim->setValue(value); break;
    case 9:  if (m_brL103Accel) m_brL103Accel->setValue(value); break;
    case 10: if (m_brL103Align) m_brL103Align->setValue(value); break;
    case 11: if (m_brL103Pll)   m_brL103Pll->setValue(value); break;
    case 12:
        if (m_brL103Vs) {
            const int i = m_brL103Vs->findData(qRound(value));
            if (i >= 0) m_brL103Vs->setCurrentIndex(i);
        }
        break;
    case 13: if (m_brL103R)     m_brL103R->setValue(value); break;
    case 14: if (m_brL103L)     m_brL103L->setValue(value); break;
    case 15:
        if (m_brL103Node)
            m_brL103Node->setText(QStringLiteral("%1 (只读; 改节点号走功能12)").arg(qRound(value)));
        break;
    case 16: if (m_brL103Oc)    m_brL103Oc->setValue(value); break;
    case 17: if (m_brL103OcMs)  m_brL103OcMs->setValue(qRound(value)); break;
    default: break;
    }
}

void MainWindow::brL103InitBaseline()
{
    m_brL103Baseline      = QVector<double>(kBrL103ParamCount, 0.0);
    m_brL103BaselineValid = QVector<bool>(kBrL103ParamCount, false);
    m_brL103Got           = QVector<bool>(kBrL103ParamCount, false);
    m_brL103GotCount      = 0;
    m_brL103WriteFail.clear();
    for (int i = 0; i < kBrL103ParamCount; ++i) {
        if (i == 15) continue;             /* 节点号只读, 不参与写 */
        m_brL103Baseline[i]      = brL103ParamValue(i);
        m_brL103BaselineValid[i] = true;
    }
    brL103UpdateState();
}

void MainWindow::brL103UpdateState(const QString &tail)
{
    if (!m_brL103ParamState) return;

    QString s = QStringLiteral("已回读 %1/18 项").arg(m_brL103GotCount);
    s += QStringLiteral("  ·  写失败项: ");
    if (m_brL103WriteFail.isEmpty()) {
        s += QStringLiteral("无");
    } else {
        QStringList ids;
        for (int i : m_brL103WriteFail) ids << QString::number(i);
        s += ids.join(QStringLiteral(", "));
    }
    if (!tail.isEmpty()) s += QStringLiteral("  ·  ") + tail;
    m_brL103ParamState->setText(s);
}

/* 切节点后「L103 调参(桥)」页要换成那一台: 上一台的 18 个值不能当成本台的。
 * 桥开着就直接回读; 桥没开就只把状态说清楚, 不动总线。 */
void MainWindow::brL103FollowTarget()
{
    if (!m_brL103ParamState) return;

    m_brL103Got      = QVector<bool>(kBrL103ParamCount, false);
    m_brL103GotCount = 0;
    m_brL103WriteFail.clear();

    if (!m_bridge || !m_bridge->isOpen()) {
        brL103UpdateState(QStringLiteral("目标节点 %1: 桥接口没打开, 这页显示的不是它")
                              .arg(bridgeNode()));
        return;
    }
    brL103UpdateState(QStringLiteral("目标节点 %1: 正在回读 0..17…").arg(bridgeNode()));
    brParamReadAll();
}

void MainWindow::brParamReadAll()
{
    if (!bridgeEnsureOpen(QStringLiteral("L103 参数回读"))) return;

    m_brL103Got = QVector<bool>(kBrL103ParamCount, false);
    m_brL103GotCount = 0;
    m_brL103WriteFail.clear();
    brL103UpdateState(QStringLiteral("正在回读 0..17…"));

    bridgeSend(FN_PARAM, buildParamCmd(PARAM_READ, 0, 0.0f, (quint8)kBrL103ParamCount),
               QStringLiteral("L103 参数: 读 0..17 (18 项) — 目标节点"), bridgeNode());
    log(QStringLiteral("[L103 参数(桥)] 已发功能15 READ b1=0 b2=18。"));
}

void MainWindow::brParamWriteAll()
{
    if (!bridgeEnsureOpen(QStringLiteral("写 L103 RAM 参数"))) return;

    static const int order[] = {16, 17, 13, 14, 6, 7, 0, 1, 3, 4, 5, 8, 9, 10, 11, 12};
    m_brL103WriteFail.clear();

    int sent = 0;
    for (int idx : order) {
        const double v = brL103ParamValue(idx);
        if (idx < m_brL103BaselineValid.size() && m_brL103BaselineValid[idx]
            && qAbs(v - m_brL103Baseline[idx]) <= 1e-9)
            continue;                       /* 值没变: 不发 */

        const float fv = (float)v;
        if (!bridgeSend(FN_PARAM, buildParamCmd(PARAM_WRITE, (quint8)idx, fv),
                        QStringLiteral("L103 参数 idx %1 = %2")
                            .arg(idx).arg((double)fv, 0, 'f', 5),
                        bridgeNode()))
            return;
        ++sent;
    }

    if (sent == 0) {
        brL103UpdateState(QStringLiteral("没有改动的项"));
        log(QStringLiteral("[L103 参数(桥)] 没有改动的项, 未发 WRITE。"));
        return;
    }

    brL103UpdateState(QStringLiteral("已发 %1 项, 等应答…").arg(sent));
    log(QStringLiteral("[L103 参数(桥)] 已发 %1 项 WRITE (只改 RAM, 立刻生效)。").arg(sent));
}

void MainWindow::brParamWriteRow()
{
    /* 新版 L103 参数页没有“选中行”概念, 保留槽只为兼容旧连接。 */
    brParamWriteAll();
}

void MainWindow::brParamSaveFlash()
{
    if (!bridgeEnsureOpen(QStringLiteral("L103 参数存 Flash"))) return;
    log(QStringLiteral("[L103 参数(桥)] 正在写 Flash (板子要擦一页, 十几毫秒) —— 电机最好停着。"));
    bridgeSend(FN_PARAM, buildParamCmd(PARAM_SAVE, 0, 0.0f),
               QStringLiteral("L103 参数: 存 Flash — 目标节点"), bridgeNode());
    brL103UpdateState(QStringLiteral("正在存 Flash…"));
}

void MainWindow::brParamDefaults()
{
    if (!bridgeEnsureOpen(QStringLiteral("L103 恢复出厂默认"))) return;
    bridgeSend(FN_PARAM, buildParamCmd(PARAM_DEFAULTS, 0, 0.0f),
               QStringLiteral("L103 参数: 恢复出厂默认 (RAM) — 目标节点"), bridgeNode());
    brL103UpdateState(QStringLiteral("正在恢复出厂默认…"));
}

void MainWindow::brParamReloadFlash()
{
    if (!bridgeEnsureOpen(QStringLiteral("L103 从 Flash 重载"))) return;
    bridgeSend(FN_PARAM, buildParamCmd(PARAM_RELOAD, 0, 0.0f),
               QStringLiteral("L103 参数: 从 Flash 重载 — 目标节点"), bridgeNode());
    brL103UpdateState(QStringLiteral("正在从 Flash 重载…"));
}

void MainWindow::applyBrParamRsp(const ParamRsp &f, int node)
{
    if (!m_brL103ParamState || node != bridgeNode()) return;
    if (m_brL103Baseline.size() != kBrL103ParamCount) brL103InitBaseline();

    const bool fail = (f.flags & 0x04) != 0;

    /* 特殊索引: 0xFF 项数 / 0xFD 存 Flash / 0xFC 默认 / 0xFE 重载 */
    if (f.index == 0xFF) {
        const int cnt = (int)(f.value + 0.5f);
        brL103UpdateState(QStringLiteral("板子报 %1 项").arg(cnt));
        return;
    }
    if (f.index == 0xFD) {
        const bool ok = !fail && f.value > 0.5f;
        log(ok ? QStringLiteral("[L103 参数(桥)] 存 Flash: OK")
               : QStringLiteral("[!] L103 参数(桥) 存 Flash: 失败"));
        brL103UpdateState(ok ? QStringLiteral("存 Flash 成功") : QStringLiteral("存 Flash 失败"));
        return;
    }
    if (f.index == 0xFC) {
        const bool ok = !fail && f.value > 0.5f;
        log(ok ? QStringLiteral("[L103 参数(桥)] 已恢复出厂默认 (RAM)")
               : QStringLiteral("[!] L103 参数(桥) 恢复出厂默认失败"));
        if (ok) {
            brParamReadAll();
        } else {
            brL103UpdateState(QStringLiteral("恢复出厂默认失败"));
        }
        return;
    }
    if (f.index == 0xFE) {
        const bool ok = !fail && f.value > 0.5f;
        log(ok ? QStringLiteral("[L103 参数(桥)] 已从 Flash 重载")
               : QStringLiteral("[!] L103 参数(桥) Flash 里没有有效记录, 重载失败"));
        if (ok) {
            brParamReadAll();
        } else {
            brL103UpdateState(QStringLiteral("从 Flash 重载失败"));
        }
        return;
    }

    if (f.index >= kBrL103ParamCount) return;

    if (f.op == PARAM_READ) {
        brL103SetParam(f.index, (double)f.value);
        m_brL103Baseline[f.index]      = (double)f.value;
        m_brL103BaselineValid[f.index] = true;
        if (!m_brL103Got[f.index]) {
            m_brL103Got[f.index] = true;
            ++m_brL103GotCount;
        }
        if (m_brL103GotCount == kBrL103ParamCount)
            log(QStringLiteral("[L103 参数(桥)] 回读完成: 18/18 项。"));
        brL103UpdateState();
        return;
    }

    if (f.op == PARAM_WRITE) {
        brL103SetParam(f.index, (double)f.value);
        if (fail) {
            if (!m_brL103WriteFail.contains(f.index))
                m_brL103WriteFail.append(f.index);
            log(QStringLiteral("[!] L103 参数(桥) 板子拒绝写 idx %1 —— 越界、只读或使能中改极对数")
                .arg(f.index));
        } else {
            m_brL103Baseline[f.index]      = (double)f.value;
            m_brL103BaselineValid[f.index] = true;
        }
        brL103UpdateState();
        return;
    }

    brL103UpdateState();
}

/* ------------------------- 序列(桥) ------------------------- */

bool MainWindow::brSeqAbsMode() const
{
    return m_brCbSeqMode && m_brCbSeqMode->currentData().toInt() == 1;
}

void MainWindow::brSeqAddRow()
{
    if (!m_brTblSeq) return;
    const int r = m_brTblSeq->rowCount();
    m_brTblSeq->insertRow(r);
    /* 节点列默认 5 (实测的电机节点号); 留空也按 5 处理。 */
    const QString def[5] = { QString::number(BRIDGE_MOTOR_NODE), QStringLiteral("0"),
                             QStringLiteral("100"), QStringLiteral("360"), QStringLiteral("0") };
    for (int c = 0; c < 5; ++c)
        m_brTblSeq->setItem(r, c, new QTableWidgetItem(def[c]));
}

void MainWindow::brSeqDelRow()
{
    if (!m_brTblSeq) return;
    int r = m_brTblSeq->currentRow();
    if (r < 0) r = m_brTblSeq->rowCount() - 1;
    if (r >= 0) m_brTblSeq->removeRow(r);
}

void MainWindow::brSeqMoveUp()
{
    if (!m_brTblSeq) return;
    const int r = m_brTblSeq->currentRow();
    if (r <= 0) return;
    swapSeqRows(m_brTblSeq, r, r - 1);
    m_brTblSeq->setCurrentCell(r - 1, 0);
}

void MainWindow::brSeqMoveDown()
{
    if (!m_brTblSeq) return;
    const int r = m_brTblSeq->currentRow();
    if (r < 0 || r >= m_brTblSeq->rowCount() - 1) return;
    swapSeqRows(m_brTblSeq, r, r + 1);
    m_brTblSeq->setCurrentCell(r + 1, 0);
}

void MainWindow::brSeqClear()
{
    if (m_brTblSeq) m_brTblSeq->setRowCount(0);
}

bool MainWindow::brReadSegs(QVector<Seg> *out, QString *err) const
{
    out->clear();
    if (!m_brTblSeq) { *err = QStringLiteral("序列(桥)表格还没建好"); return false; }

    for (int r = 0; r < m_brTblSeq->rowCount(); ++r) {
        QString cell[5];
        for (int c = 0; c < 5; ++c) {
            QTableWidgetItem *it = m_brTblSeq->item(r, c);
            cell[c] = it ? it->text().trimmed() : QString();
        }

        Seg s;
        if (cell[0].isEmpty()) {
            s.node = BRIDGE_MOTOR_NODE;
        } else {
            bool okN = false;
            s.node = cell[0].toInt(&okN);
            if (!okN) { *err = QStringLiteral("第 %1 行节点号不是数字").arg(r + 1); return false; }
        }
        if (s.node < NODE_MIN || s.node > NODE_MAX) {
            *err = QStringLiteral("第 %1 行节点号 = %2 —— 要 1..63。\n"
                                  "(0 是广播, 脚本装不到广播上; 节点列留空 = 节点 5)")
                     .arg(r + 1).arg(s.node);
            return false;
        }

        bool ok[4] = { false, false, false, false };
        s.waitUs  = cell[1].toInt(&ok[0]);
        s.rpm     = cell[2].toInt(&ok[1]);
        s.deg     = cell[3].toDouble(&ok[2]);
        s.dwellMs = cell[4].toInt(&ok[3]);
        if (!ok[0] || !ok[1] || !ok[2] || !ok[3]) {
            *err = QStringLiteral("第 %1 行有空单元格或非数字").arg(r + 1); return false;
        }
        if (s.waitUs < 0 || s.waitUs > 10000000) {
            *err = QStringLiteral("第 %1 行「等待」超出 0..10000000 us").arg(r + 1); return false;
        }
        if (s.rpm == 0) {
            *err = QStringLiteral("第 %1 行速度为 0 —— 没有速度的段什么也不做").arg(r + 1); return false;
        }
        if (qAbs(s.rpm) > 1432) {
            *err = QStringLiteral("第 %1 行 |速度| 超过 1432 rpm").arg(r + 1); return false;
        }

        s.abs = brSeqAbsMode();
        if (s.abs) {
            if (qAbs(s.deg) > SEQ_DEG_MAX_UART) {
                *err = QStringLiteral("第 %1 行 |绝对角度| 超过 36000 度(100 圈)").arg(r + 1); return false;
            }
        } else {
            if (s.deg <= 0.0) {
                *err = QStringLiteral("第 %1 行角度必须写正数(方向看速度的符号)").arg(r + 1); return false;
            }
            if (s.deg > SEQ_DEG_MAX_UART) {
                *err = QStringLiteral("第 %1 行角度超过 36000 度(100 圈)").arg(r + 1); return false;
            }
        }
        if (s.dwellMs < 0 || s.dwellMs > 60000) {
            *err = QStringLiteral("第 %1 行「停留」超出 0..60000 ms").arg(r + 1); return false;
        }
        out->append(s);
    }
    if (out->isEmpty()) { *err = QStringLiteral("表格是空的"); return false; }
    return true;
}

void MainWindow::brSeqPreview()
{
    if (!m_brTxtSeqPreview) return;

    QVector<Seg> segs;
    QString err;
    if (!brReadSegs(&segs, &err)) {
        m_brTxtSeqPreview->setPlainText(QStringLiteral("表格有问题: ") + err);
        return;
    }
    const int loops = m_brSpSeqLoops ? m_brSpSeqLoops->value() : 1;

    QVector<Seg> sendSegs = segs;
    QStringList notes;
    if (brSeqAbsMode()) {
        if (!absSeqToDeltas(segs, loops, &sendSegs, &notes, &err)) {
            m_brTxtSeqPreview->setPlainText(QStringLiteral("绝对模式: ") + err);
            return;
        }
        if (sendSegs.isEmpty()) {
            m_brTxtSeqPreview->setPlainText(QStringLiteral("绝对模式: 所有目标都等于当前位置, 没有可下发的段。"));
            return;
        }
    }

    const QMap<int, QVector<Seg> > byNode = groupForCan(sendSegs, &notes);
    QString out;
    out += QStringLiteral("桥专用序列: 共 %1 段, 跑 %2 遍, 分到 %3 台节点 (节点列留空 = 5)\n")
             .arg(segs.size()).arg(loops).arg(byNode.size());
    out += QStringLiteral("所有帧直接走 m_bridge, 不受顶部「传输」下拉影响。\n\n");

    for (QMap<int, QVector<Seg> >::const_iterator it = byNode.constBegin(); it != byNode.constEnd(); ++it) {
        const int node = it.key();
        const QVector<Seg> &v = it.value();
        out += QStringLiteral("-- 节点 %1: %2 段%3\n")
                 .arg(node).arg(v.size())
                 .arg(v.size() > SEQ_MAX_SEG ? QStringLiteral("   ★超过每台 16 段上限, 发不完") : QString());
        out += QStringLiteral("   %1  [00 00]    清队列\n")
                 .arg(canIdName(L103::canId(node, FN_SEGCTL)));
        for (const Seg &s : v) {
            out += QStringLiteral("   %1  [%2]   %3 rpm, %4 度, 等 %5us, 停 %6ms%7\n")
                     .arg(canIdName(L103::canId(node, s.hr ? FN_SEG_HR : FN_SEG)))
                     .arg(hexOf(s.hr ? buildSegHr(s.waitUs, s.rpm, s.deg, s.dwellMs)
                                     : buildSeg  (s.waitUs, s.rpm, s.deg, s.dwellMs)))
                     .arg(s.rpm).arg(QString::number(s.deg, 'g', 9)).arg(s.waitUs).arg(s.dwellMs)
                     .arg(s.hr ? QStringLiteral("   [大角度版: 1 度刻度]") : QString());
        }
    }
    out += QStringLiteral("\n起跑: %1  [%2]   功能10 脚本开始, %3 遍\n")
             .arg(canIdName(L103::canId(NODE_BCAST, FN_SEGCTL)))
             .arg(hexOf(buildSegCtl(1, loops))).arg(loops);
    out += QStringLiteral("停止: 桥广播 功能0 PV flags=0x04 (松轴)\n");
    for (const QString &n : notes)
        out += QStringLiteral("注意: %1\n").arg(n);

    m_brTxtSeqPreview->setPlainText(out);
}

void MainWindow::brSeqSend()
{
    if (!bridgeEnsureOpen(QStringLiteral("发送动作脚本"))) return;

    QVector<Seg> segs;
    QString err;
    if (!brReadSegs(&segs, &err)) {
        QMessageBox::warning(this, QStringLiteral("序列(桥)有问题"), err);
        return;
    }

    const int  loops = m_brSpSeqLoops ? m_brSpSeqLoops->value() : 1;
    QStringList notes;
    if (brSeqAbsMode()) {
        QVector<Seg> rel;
        if (!absSeqToDeltas(segs, loops, &rel, &notes, &err)) {
            QMessageBox::warning(this, QStringLiteral("序列(桥) 绝对模式"), err);
            return;
        }
        if (rel.isEmpty()) {
            QMessageBox::warning(this, QStringLiteral("序列(桥) 绝对模式"),
                                 QStringLiteral("所有目标都等于当前位置, 没有可下发的段。"));
            return;
        }
        log(QStringLiteral("[序列(桥)] 绝对模式: %1 行目标 -> %2 段相对增量 (基准 = 发命令这一刻的机械位置)")
            .arg(segs.size()).arg(rel.size()));
        segs = rel;
    }

    const QMap<int, QVector<Seg> > byNode = groupForCan(segs, &notes);
    QStringList over;
    for (QMap<int, QVector<Seg> >::const_iterator it = byNode.constBegin(); it != byNode.constEnd(); ++it)
        if (it.value().size() > SEQ_MAX_SEG)
            over << QStringLiteral("节点 %1: 拆完 %2 段").arg(it.key()).arg(it.value().size());
    if (!over.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("段数超了"),
            QStringLiteral("每台板载队列只有 %1 段, 下面这些超了:\n%2\n\n"
                           "办法: 减小角度, 或者把动作拆成几批(等第一批发完再发第二批)。")
            .arg(SEQ_MAX_SEG).arg(over.join(QStringLiteral("\n"))));
        return;
    }

    double totalDeg = 0.0;
    for (const Seg &s : segs) totalDeg += qAbs(s.deg);

    log(QStringLiteral("[动作/桥] 发送动作脚本: %1 段 x %2 遍, 合计 %3 度, 分到 %4 台节点 —— 轴会自己动起来, 清场, 别再手抓")
        .arg(segs.size()).arg(loops).arg(totalDeg, 0, 'f', 1).arg(byNode.size()));

    /* 使能必须先发: 固件 Motion_Seq_Start 要求已使能, 且 SetEnabled(1) 会 abort 队列。 */
    if (!bridgeSend(FN_ENABLE, buildEnable(true),
                    QStringLiteral("脚本前广播使能 (所有节点)"), NODE_BCAST))
        return;

    for (QMap<int, QVector<Seg> >::const_iterator it = byNode.constBegin(); it != byNode.constEnd(); ++it) {
        const int node = it.key();
        if (!bridgeSend(FN_SEGCTL, buildSegCtl(0, 0),
                        QStringLiteral("清节点 %1 的脚本队列").arg(node), node))
            return;
        const QVector<Seg> &v = it.value();
        for (const Seg &s : v) {
            if (!bridgeSend(s.hr ? FN_SEG_HR : FN_SEG,
                            s.hr ? buildSegHr(s.waitUs, s.rpm, s.deg, s.dwellMs)
                                 : buildSeg  (s.waitUs, s.rpm, s.deg, s.dwellMs),
                            QStringLiteral("%1 rpm / %2 度 / 等 %3us / 停 %4ms%5")
                              .arg(s.rpm).arg(QString::number(s.deg, 'g', 9)).arg(s.waitUs).arg(s.dwellMs)
                              .arg(s.hr ? QStringLiteral("  (大角度版 1 度刻度)") : QString()),
                            node))
                return;
        }
    }

    if (!bridgeSend(FN_SEGCTL, buildSegCtl(1, loops),
                    QStringLiteral("广播起跑: 所有节点同一帧开始, %1 遍").arg(loops), NODE_BCAST))
        return;

    for (const QString &n : notes) log(QStringLiteral("[!] %1").arg(n));
}

void MainWindow::brSeqStop()
{
    if (!bridgeEnsureOpen(QStringLiteral("停止脚本"))) return;
    bridgeSend(FN_PV, buildPv(0, 0.0, PVF_RELEASE),
               QStringLiteral("广播停脚本 + 松轴 (所有节点)"), NODE_BCAST);
}

/* ------------------------- 两个页签的界面 ------------------------- */

QWidget *MainWindow::makeBridgeParamTab()
{
    QWidget *w = new QWidget;
    QVBoxLayout *v = new QVBoxLayout(w);

    v->addWidget(mkLabel(QStringLiteral(
        "功能15/23 · 走桥接口 · 与 UART 那页同一批参数\n"
        "所有帧发「目标设备」那一台, 直接走 m_bridge, 完全不受顶部「传输」下拉影响。")));

    QGroupBox *gbp = new QGroupBox(QStringLiteral("L103 全部可调参数 · 桥接口 · 功能15 索引 0..17"));

    /* 2026-10-26: 把「L103 调参」页上那一坨 (使能/失能/对齐/清故障 + 状态 + 速度模式) 也带到本页,
     * 调参时不用来回换页签。帧发「目标设备」、固定走 m_bridge。 */
    QGroupBox *gbm = new QGroupBox(QStringLiteral("电机 · 速度 (走桥接口, 目标节点)"));
    QVBoxLayout *vm = new QVBoxLayout(gbm);

    QPushButton *bEn   = new QPushButton(QStringLiteral("使能 (E)"));
    QPushButton *bDis  = new QPushButton(QStringLiteral("失能 / 松轴 (D)"));
    QPushButton *bAl   = new QPushButton(QStringLiteral("对齐 (A)"));
    QPushButton *bDisAll = new QPushButton(QStringLiteral("全部失能 (广播)"));
    QPushButton *bClrB = new QPushButton(QStringLiteral("清故障 (X)"));
    QPushButton *bSt   = new QPushButton(QStringLiteral("读一次状态"));
    bEn  ->setToolTip(QStringLiteral("CAN 功能2 b0=1 (走桥) —— 抱住当前位置; 要求已对齐"));
    bDis ->setToolTip(QStringLiteral("CAN 功能2 b0=0 (走桥) —— 松轴, 轴可以手转"));
    bAl  ->setToolTip(QStringLiteral("CAN 功能13 b0=0x5A (走桥) —— 会硬拽转子, 轴必须能自由转"));
    bDisAll->setToolTip(QStringLiteral("广播功能2 b0=0 —— 总线上每一台都失能松轴, 三轴会掉"));
    bClrB->setToolTip(QStringLiteral("功能0 flags bit3 (走桥) —— 只清故障, 不动机构、不重新使能"));
    bSt  ->setToolTip(QStringLiteral("问一帧状态 (b0=0), 只刷新本页「状态」那一格"));

    m_brLabRun = mkLabel(QStringLiteral("—"));

    QHBoxLayout *hbm = new QHBoxLayout;
    hbm->addWidget(mkLabel(QStringLiteral("电机")));
    hbm->addWidget(bEn); hbm->addWidget(bDis); hbm->addWidget(bDisAll); hbm->addWidget(bAl);
    hbm->addSpacing(12); hbm->addWidget(bClrB);
    hbm->addSpacing(12); hbm->addWidget(mkLabel(QStringLiteral("状态")));
    hbm->addWidget(m_brLabRun);
    hbm->addSpacing(12); hbm->addWidget(bSt);
    hbm->addStretch();
    vm->addLayout(hbm);

    QPushButton *bVelGo = new QPushButton(QStringLiteral("启动速度 (M1/V)"));
    QPushButton *bVelSt = new QPushButton(QStringLiteral("停止 (B1)"));
    bVelGo->setToolTip(QStringLiteral("先使能, 再进速度环连续转 —— 会一直转, 直到按「停止 (B1)」"));
    bVelSt->setToolTip(QStringLiteral("功能1 速度 0 + flags 0 (走桥) —— 停 + 失能, 轴可以手转"));

    m_brVelRpm = new QDoubleSpinBox;
    m_brVelRpm->setRange(-99999.0, 99999.0);
    m_brVelRpm->setDecimals(1);
    m_brVelRpm->setValue(100.0);
    m_brVelRpm->setSuffix(QStringLiteral(" rpm"));
    m_brVelRpm->setMinimumWidth(140);

    QHBoxLayout *hbv = new QHBoxLayout;
    hbv->addWidget(mkLabel(QStringLiteral("速度模式")));
    hbv->addWidget(m_brVelRpm);
    hbv->addWidget(bVelGo); hbv->addWidget(bVelSt);
    hbv->addSpacing(12);
    hbv->addWidget(mkLabel(QStringLiteral("(启动会先使能; 停止 = 停 + 失能)")));
    hbv->addStretch();
    vm->addLayout(hbv);

    v->addWidget(gbm);
    QGridLayout *g = new QGridLayout(gbp);

    auto dspin = [](double lo, double hi, int dec, double step, const QString &suffix) {
        QDoubleSpinBox *s = new QDoubleSpinBox;
        s->setRange(lo, hi);
        s->setDecimals(dec);
        s->setSingleStep(step);
        s->setSuffix(suffix);
        return s;
    };
    auto label = [](const QString &name, int idx) {
        return mkLabel(QStringLiteral("%1 (idx %2)").arg(name).arg(idx));
    };

    m_brL103Pp = new QSpinBox;
    m_brL103Pp->setRange(1, 64);
    m_brL103Pp->setValue(11);

    m_brL103ILim = dspin(0.10, 3.00, 2, 0.05, QStringLiteral(" A"));
    m_brL103ILim->setValue(1.80);

    m_brL103Vbus = dspin(8.0, 36.0, 2, 0.1, QStringLiteral(" V"));
    m_brL103Vbus->setValue(24.0);

    m_brL103SpdKp = dspin(0.0, 2.0, 5, 0.005, QString());
    m_brL103SpdKp->setValue(0.06);
    m_brL103SpdKi = dspin(0.0, 100.0, 2, 0.1, QString());
    m_brL103SpdKi->setValue(5.00);

    m_brL103PosKp = dspin(0.0, 500.0, 2, 1.0, QString());
    m_brL103PosKp->setValue(25.0);

    m_brL103CurKp = dspin(0.0, 100.0, 4, 0.01, QStringLiteral(" V/A"));
    m_brL103CurKp->setValue(2.1991);
    m_brL103CurKi = dspin(0.0, 100000.0, 1, 10.0, QStringLiteral(" V/(A·s)"));
    m_brL103CurKi->setValue(8419.5);

    m_brL103R = dspin(0.0, 50.0, 3, 0.01, QStringLiteral(" Ω"));
    m_brL103R->setValue(6.700);
    m_brL103L = dspin(0.0, 100.0, 3, 0.01, QStringLiteral(" mH"));
    m_brL103L->setValue(1.750);

    m_brL103Vlim = dspin(0.1, 1000.0, 1, 5.0, QStringLiteral(" rad/s"));
    m_brL103Vlim->setValue(300.0);
    m_brL103Accel = dspin(1.0, 100000.0, 1, 100.0, QStringLiteral(" rad/s²"));
    m_brL103Accel->setValue(6600.0);

    m_brL103Align = dspin(0.5, 12.0, 2, 0.1, QStringLiteral(" V"));
    m_brL103Align->setValue(3.20);
    m_brL103Pll = dspin(10.0, 1000.0, 1, 5.0, QStringLiteral(" Hz"));
    m_brL103Pll->setValue(150.0);

    m_brL103Vs = new QComboBox;
    m_brL103Vs->addItem(QStringLiteral("0 自动"), 0);
    m_brL103Vs->addItem(QStringLiteral("1 PLL"),  1);
    m_brL103Vs->addItem(QStringLiteral("2 窗口 (易跑飞)"), 2);
    m_brL103Vs->setCurrentIndex(1);

    m_brL103Oc = dspin(0.50, 4.50, 2, 0.10, QStringLiteral(" A"));
    m_brL103Oc->setValue(2.00);
    m_brL103Oc->setToolTip(QStringLiteral(
        "相电流超过这个数、并且连续顶住右边那个时间, 才关桥。范围 0.5..4.5 A。\n"
        "这个阈值只管【跳闸】, 不是电流环给定上限 (那个是 idx 1)。"));

    m_brL103OcMs = new QSpinBox;
    m_brL103OcMs->setRange(0, 10000);
    m_brL103OcMs->setSingleStep(500);
    m_brL103OcMs->setValue(5000);
    m_brL103OcMs->setSuffix(QStringLiteral(" ms"));
    m_brL103OcMs->setMinimumWidth(120);

    int r = 0;
    g->addWidget(label(QStringLiteral("极对数"), 0), r, 0);       g->addWidget(m_brL103Pp, r, 1);
    g->addWidget(label(QStringLiteral("电流上限"), 1), r, 2);     g->addWidget(m_brL103ILim, r, 3);
    g->addWidget(label(QStringLiteral("母线电压"), 2), r, 4);     g->addWidget(m_brL103Vbus, r, 5);
    g->addWidget(label(QStringLiteral("测速源"), 12), r, 6);      g->addWidget(m_brL103Vs, r, 7);
    ++r;
    g->addWidget(label(QStringLiteral("速度环 kp"), 3), r, 0);    g->addWidget(m_brL103SpdKp, r, 1);
    g->addWidget(label(QStringLiteral("速度环 ki"), 4), r, 2);    g->addWidget(m_brL103SpdKi, r, 3);
    g->addWidget(label(QStringLiteral("位置环 kp"), 5), r, 4);    g->addWidget(m_brL103PosKp, r, 5);
    ++r;
    g->addWidget(label(QStringLiteral("电流环 kp"), 6), r, 0);    g->addWidget(m_brL103CurKp, r, 1);
    g->addWidget(label(QStringLiteral("电流环 ki"), 7), r, 2);    g->addWidget(m_brL103CurKi, r, 3);
    g->addWidget(label(QStringLiteral("相电阻 R"), 13), r, 4);    g->addWidget(m_brL103R, r, 5);
    g->addWidget(label(QStringLiteral("电感 L"), 14), r, 6);      g->addWidget(m_brL103L, r, 7);
    ++r;
    g->addWidget(label(QStringLiteral("速度上限"), 8), r, 0);     g->addWidget(m_brL103Vlim, r, 1);
    g->addWidget(label(QStringLiteral("加速度"), 9), r, 2);       g->addWidget(m_brL103Accel, r, 3);
    g->addWidget(label(QStringLiteral("对齐电压"), 10), r, 4);    g->addWidget(m_brL103Align, r, 5);
    g->addWidget(label(QStringLiteral("PLL 带宽"), 11), r, 6);    g->addWidget(m_brL103Pll, r, 7);
    ++r;
    g->addWidget(label(QStringLiteral("过流跳闸"), 16), r, 0);    g->addWidget(m_brL103Oc, r, 1);
    g->addWidget(label(QStringLiteral("持续"), 17), r, 2);        g->addWidget(m_brL103OcMs, r, 3);
    g->addWidget(mkLabel(QStringLiteral("(超过阈值并连续顶住这么久才关桥; 与 idx 1 的『电流上限』不是一回事)")), r, 4, 1, 4);
    ++r;
    g->addWidget(label(QStringLiteral("节点号"), 15), r, 0);
    m_brL103Node = mkLabel(QStringLiteral("未回读"));
    m_brL103Node->setToolTip(QStringLiteral("只读; 改节点号走功能12 (存 Flash), 本页不写这一项。"));
    g->addWidget(m_brL103Node, r, 1, 1, 3);
    g->addWidget(mkLabel(QStringLiteral("只读 · 改节点号走功能12")), r, 4, 1, 4);

    m_brL103ParamState = mkLabel(QStringLiteral("已回读 0/18 项  ·  写失败项: 无"));

    QHBoxLayout *hp = new QHBoxLayout;
    QPushButton *bPRead  = new QPushButton(QStringLiteral("回读全部"));
    QPushButton *bPWrite = new QPushButton(QStringLiteral("写入 RAM"));
    QPushButton *bPSave  = new QPushButton(QStringLiteral("存 Flash"));
    QPushButton *bPDef   = new QPushButton(QStringLiteral("恢复出厂默认"));
    QPushButton *bPRel   = new QPushButton(QStringLiteral("从 Flash 重载"));
    hp->addWidget(bPRead); hp->addWidget(bPWrite); hp->addWidget(bPSave);
    hp->addSpacing(14); hp->addWidget(bPDef); hp->addWidget(bPRel);
    hp->addStretch();
    g->addLayout(hp, ++r, 0, 1, 8);
    g->addWidget(m_brL103ParamState, ++r, 0, 1, 8);

    brL103InitBaseline();
    v->addWidget(gbp);
    v->addWidget(mkLabel(QStringLiteral(
        "写入前必须失能（轴不能受力）；改极对数会让“已对齐”作废，必须重新点“对齐”。\n"
        "“写入 RAM”只发有改动的项, 立即生效但断电丢失；要留住必须再点“存 Flash”。\n"
        "存 Flash 会擦一页, 禁止在运动中保存。节点号只读, 改节点号走功能12。")));

    QGroupBox *gb = new QGroupBox(QStringLiteral("电机参数 (CAN 功能8, 写 Flash)"));
    QFormLayout *f = new QFormLayout(gb);

    m_brSpPolePairs = new QSpinBox;
    m_brSpPolePairs->setRange(0, 64);
    m_brSpPolePairs->setSpecialValueText(QStringLiteral("0 = 不改"));
    m_brSpPolePairs->setValue(0);

    m_brSpStallA = new QDoubleSpinBox;
    m_brSpStallA->setRange(0.0, 3.0);
    m_brSpStallA->setDecimals(2);
    m_brSpStallA->setSingleStep(0.1);
    m_brSpStallA->setSpecialValueText(QStringLiteral("0 = 不改"));
    m_brSpStallA->setValue(0.0);

    m_brSpVbus = new QDoubleSpinBox;
    m_brSpVbus->setRange(0.0, 36.0);
    m_brSpVbus->setDecimals(2);
    m_brSpVbus->setSpecialValueText(QStringLiteral("0 = 不改"));
    m_brSpVbus->setValue(0.0);

    f->addRow(QStringLiteral("极对数 (1..64)"), m_brSpPolePairs);
    f->addRow(QStringLiteral("堵转阈值 A (0.1..3.0)"), m_brSpStallA);
    f->addRow(QStringLiteral("母线电压 V (8.0..36.0)"), m_brSpVbus);

    QHBoxLayout *h = new QHBoxLayout;
    QPushButton *bRead  = new QPushButton(QStringLiteral("回读 (b0=1 + b0=3)"));
    QPushButton *bWrite = new QPushButton(QStringLiteral("写入 (功能8 电机参数)"));
    h->addWidget(bRead); h->addWidget(bWrite); h->addStretch();
    f->addRow(QString(), h);
    v->addWidget(gb);

    QGroupBox *gb2 = new QGroupBox(QStringLiteral("角度环速度上限 (功能7)"));
    QFormLayout *f2 = new QFormLayout(gb2);
    m_brSpVlimRadS = new QDoubleSpinBox;
    m_brSpVlimRadS->setRange(0.1, VEL_LIMIT_RAD_S);
    m_brSpVlimRadS->setDecimals(2);
    m_brSpVlimRadS->setValue(10.0);
    m_brSpVlimRadS->setSuffix(QStringLiteral(" rad/s"));
    QHBoxLayout *h2 = new QHBoxLayout;
    QPushButton *bVlimRead  = new QPushButton(QStringLiteral("回读配置 (b0=1)"));
    QPushButton *bVlimWrite = new QPushButton(QStringLiteral("写入 (功能7 速度上限)"));
    h2->addWidget(bVlimRead); h2->addWidget(bVlimWrite); h2->addStretch();
    f2->addRow(QStringLiteral("角度环输出钳位"), m_brSpVlimRadS);
    f2->addRow(QString(), h2);
    f2->addRow(mkLabel(QStringLiteral("配置帧不含 vlim; 「回读配置」只刷新母线/上报等其它项, 不会回填这一格。")));
    v->addWidget(gb2);

    QGroupBox *gb3 = new QGroupBox(QStringLiteral("故障主动上报 (功能6)"));
    QFormLayout *f3 = new QFormLayout(gb3);
    m_brChkFaultReport = new QCheckBox(QStringLiteral("故障主动上报开"));
    m_brSpReportPeriod = new QSpinBox;
    m_brSpReportPeriod->setRange(0, 255);
    m_brSpReportPeriod->setValue(10);
    m_brSpReportPeriod->setSuffix(QStringLiteral(" ×100ms"));
    QHBoxLayout *h3 = new QHBoxLayout;
    QPushButton *bFaultRead  = new QPushButton(QStringLiteral("回读配置 (b0=1)"));
    QPushButton *bFaultWrite = new QPushButton(QStringLiteral("应用 (功能6 故障上报)"));
    h3->addWidget(m_brChkFaultReport); h3->addWidget(m_brSpReportPeriod);
    h3->addWidget(bFaultRead); h3->addWidget(bFaultWrite); h3->addStretch();
    f3->addRow(QStringLiteral("故障上报"), h3);
    v->addWidget(gb3);

    m_brLabState = mkLabel(QStringLiteral("桥专用页: 跟随「目标设备」; 还没回读过。"));
    v->addWidget(m_brLabState);
    v->addWidget(mkLabel(QStringLiteral(
        "参数表由图上的 INFO 实测回报 18 项 (CAN 功能15/23), 本页按回报行数自适应。\n"
        "写入电机参数会存 Flash; 改极对数会把「已对齐」作废, 之后必须重跑对齐。")));
    v->addStretch();

    connect(bRead,      &QPushButton::clicked, this, &MainWindow::brMotorRead);
    connect(bWrite,     &QPushButton::clicked, this, &MainWindow::brMotorWrite);
    connect(bVlimRead,  &QPushButton::clicked, this, &MainWindow::brCfgRead);
    connect(bVlimWrite, &QPushButton::clicked, this, &MainWindow::brVlimWrite);
    connect(bFaultRead, &QPushButton::clicked, this, &MainWindow::brCfgRead);
    connect(bFaultWrite,&QPushButton::clicked, this, &MainWindow::brFaultReportWrite);
    connect(bPRead,     &QPushButton::clicked, this, &MainWindow::brParamReadAll);
    connect(bPWrite,    &QPushButton::clicked, this, &MainWindow::brParamWriteAll);
    connect(bPSave,     &QPushButton::clicked, this, &MainWindow::brParamSaveFlash);
    connect(bPDef,      &QPushButton::clicked, this, &MainWindow::brParamDefaults);
    connect(bPRel,      &QPushButton::clicked, this, &MainWindow::brParamReloadFlash);
    connect(bEn,        &QPushButton::clicked, this, &MainWindow::brActEnable);
    connect(bDis,       &QPushButton::clicked, this, &MainWindow::brActDisable);
    connect(bDisAll,    &QPushButton::clicked, this, &MainWindow::brActDisableAll);
    connect(bAl,        &QPushButton::clicked, this, &MainWindow::brActAlign);
    connect(bClrB,      &QPushButton::clicked, this, &MainWindow::brActClearFault);
    connect(bSt,        &QPushButton::clicked, this, &MainWindow::brQueryStatus);
    connect(bVelGo,     &QPushButton::clicked, this, [this]() { brActVelocityWith(m_brVelRpm->value()); });
    connect(bVelSt,     &QPushButton::clicked, this, &MainWindow::brActVelocityStop);

    return w;
}

QWidget *MainWindow::makeBridgeSeqTab()
{
    QWidget *w = new QWidget;
    QVBoxLayout *v = new QVBoxLayout(w);

    v->addWidget(mkLabel(QStringLiteral(
        "桥专用动作脚本 —— 表格和「序列」页一样, 但所有 SEG / SEG_HR / SEGCTL 帧直接走 m_bridge。\n"
        "  · 节点列默认 5; 留空也按节点 5 处理\n"
        "  · 相对增量: 角度写正数, 方向看 rpm 的符号\n"
        "  · 绝对位置: 角度 = 机械坐标上的角度(度), 上位机按当前位置换算成相对增量再下发\n"
        "  · 绝对模式要先点「问一次状态」把节点 5 的机械位置读回来, 且轴要停稳")));

    m_brTblSeq = new QTableWidget(0, 5);
    m_brTblSeq->setHorizontalHeaderLabels(QStringList()
        << QStringLiteral("节点") << QStringLiteral("等待 (us)")
        << QStringLiteral("速度 (rpm, ±=方向)")
        << QStringLiteral("角度 (deg, 正数)") << QStringLiteral("停留 (ms)"));
    m_brTblSeq->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_brTblSeq->setSelectionBehavior(QAbstractItemView::SelectRows);
    v->addWidget(m_brTblSeq, 1);

    QHBoxLayout *h = new QHBoxLayout;
    QPushButton *bAdd = new QPushButton(QStringLiteral("加一段"));
    QPushButton *bDel = new QPushButton(QStringLiteral("删掉选中"));
    QPushButton *bUp  = new QPushButton(QStringLiteral("上移"));
    QPushButton *bDn  = new QPushButton(QStringLiteral("下移"));
    QPushButton *bClr = new QPushButton(QStringLiteral("清空表格"));
    QPushButton *bPv  = new QPushButton(QStringLiteral("打印要发的内容 (预览)"));
    h->addWidget(bAdd); h->addWidget(bDel); h->addWidget(bUp); h->addWidget(bDn);
    h->addWidget(bClr); h->addSpacing(20); h->addWidget(bPv);
    h->addStretch();
    v->addLayout(h);

    QHBoxLayout *h2 = new QHBoxLayout;
    m_brCbSeqMode = new QComboBox;
    m_brCbSeqMode->addItem(QStringLiteral("相对增量 (老方式)"), 0);
    m_brCbSeqMode->addItem(QStringLiteral("绝对位置 (机械坐标)"), 1);
    auto syncModeUi = [this]() {
        const bool a = brSeqAbsMode();
        QTableWidgetItem *hv = m_brTblSeq->horizontalHeaderItem(2);
        QTableWidgetItem *ha = m_brTblSeq->horizontalHeaderItem(3);
        if (hv) hv->setText(a ? QStringLiteral("速度 (rpm, 正数=上限)")
                              : QStringLiteral("速度 (rpm, ±=方向)"));
        if (ha) ha->setText(a ? QStringLiteral("角度 (deg, 绝对机械角)")
                              : QStringLiteral("角度 (deg, 正数)"));
    };
    connect(m_brCbSeqMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [syncModeUi](int) { syncModeUi(); });

    m_brSpSeqLoops = new QSpinBox;
    m_brSpSeqLoops->setRange(1, 99);
    m_brSpSeqLoops->setValue(1);
    m_brSpSeqLoops->setSuffix(QStringLiteral(" 遍"));

    QPushButton *bQuery = new QPushButton(QStringLiteral("问一次状态 (b0=0)"));
    QPushButton *bSend  = new QPushButton(QStringLiteral("发送到板子 ★"));
    bSend->setStyleSheet(QStringLiteral("font-weight:bold;"));
    QPushButton *bStop  = new QPushButton(QStringLiteral("停止脚本 (B1)"));

    h2->addWidget(mkLabel(QStringLiteral("角度模式")));
    h2->addWidget(m_brCbSeqMode);
    h2->addWidget(mkLabel(QStringLiteral("遍数")));
    h2->addWidget(m_brSpSeqLoops);
    h2->addWidget(bQuery);
    h2->addWidget(bSend);
    h2->addWidget(bStop);
    h2->addStretch();
    v->addLayout(h2);

    m_brTxtSeqPreview = new QPlainTextEdit;
    m_brTxtSeqPreview->setReadOnly(true);
    m_brTxtSeqPreview->setMaximumHeight(200);
    m_brTxtSeqPreview->setFont(QFont(QStringLiteral("Consolas"), 9));
    v->addWidget(m_brTxtSeqPreview);

    v->addWidget(mkLabel(QStringLiteral(
        "所有帧都直接 m_bridge->sendFrame(): 不受顶部「传输」影响; 桥没打开时按钮会拒绝发送。")));

    connect(bAdd,  &QPushButton::clicked, this, &MainWindow::brSeqAddRow);
    connect(bDel,  &QPushButton::clicked, this, &MainWindow::brSeqDelRow);
    connect(bUp,   &QPushButton::clicked, this, &MainWindow::brSeqMoveUp);
    connect(bDn,   &QPushButton::clicked, this, &MainWindow::brSeqMoveDown);
    connect(bClr,  &QPushButton::clicked, this, &MainWindow::brSeqClear);
    connect(bPv,   &QPushButton::clicked, this, &MainWindow::brSeqPreview);
    connect(bQuery,&QPushButton::clicked, this, &MainWindow::brQueryStatus);
    connect(bSend, &QPushButton::clicked, this, &MainWindow::brSeqSend);
    connect(bStop, &QPushButton::clicked, this, &MainWindow::brSeqStop);

    /* 起手两行模板: 节点 5; 第二行等 0.5 s 再反向转回来 */
    struct Tpl { int w; int rpm; int deg; int dw; };
    const Tpl tpl[2] = { {0, 100, 360, 0}, {500000, -100, 360, 0} };
    for (int i = 0; i < 2; ++i) {
        brSeqAddRow();
        m_brTblSeq->item(i, 1)->setText(QString::number(tpl[i].w));
        m_brTblSeq->item(i, 2)->setText(QString::number(tpl[i].rpm));
        m_brTblSeq->item(i, 3)->setText(QString::number(tpl[i].deg));
        m_brTblSeq->item(i, 4)->setText(QString::number(tpl[i].dw));
    }

    return w;
}
