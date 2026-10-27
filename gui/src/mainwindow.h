#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMap>
#include <QStringList>
#include <QVector>

#include "l103protocol.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QRadioButton;
class QSpinBox;
class QTableWidget;
class QTimer;

class UartLink;
class CanLink;
class BridgeLink;
class WaveformParser;
class WavePlot;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    /* 连接 */
    void refreshPortLists();
    void toggleUart();
    void toggleCan();
    void toggleBridge();         /* H750 UART<->CAN 桥 (AA 55 帧模式) */
    void onProtoChanged();       /* 「协议」那一格变了 (自动 / 新 / 旧) */

    /* 状态 */
    void refreshStatus();
    void onUartLine(const QString &line);
    void onCanFrame(quint32 id, const QByteArray &data);
    void onUartState();
    void onCanState();
    void onBridgeState();

    /* 多节点设备表 */
    void scanNodes();          /* 广播问一遍, 重建设备表 */
    void onNodeChanged();      /* 「目标设备」那一格变了 */
    void onDevRowChanged();    /* 设备表点了一行 */
    void refreshDevTable();
    void refreshNodeInfo();
    void actSetNodeNo();       /* 参数页: 写节点号 */

    /* 动作 */
    void actEnable();
    void actDisable();
    void actStopHold();
    void actStopRelease();
    void actClearFault();
    void actAlign();
    void actCalCurrentZero();
    void actSetMechZero();
    void actGotoMechZero();
    void actMovePv();
    void actFillFromCurrent();
    void actVelocity();
    void actVelocityWith(double rpm);   /* L103 页签用: 同一套逻辑, 转速由调用方给 */
    void actVelocityStop();
    void actOpenloop();          /* 开环: 只给电压/频率, 不读反馈 */
    void actOpenloopStop();

    /* 序列 */
    void seqAddRow();
    void seqDelRow();
    void seqMoveUp();
    void seqMoveDown();
    void seqClear();
    void seqPreview();
    void seqSend();
    void seqStop();

    /* 参数 */
    void cfgRead();
    void cfgWrite();
    void cfgVlimWrite();
    void cfgFaultReportWrite();

    /* 桥专用页签 (所有帧强制走 m_bridge, 不看顶部「传输」) */
    void brMotorRead();
    void brCfgRead();
    void brMotorWrite();
    void brVlimWrite();
    void brFaultReportWrite();
    void brQueryStatus();
    void brActEnable();
    void brActDisable();
    void brActDisableAll();                 /* 广播: 所有节点一起失能松轴 */
    void brActAlign();
    void brActClearFault();
    void brActVelocityWith(double rpm);
    void brActVelocityStop();
    void brSeqAddRow();
    void brSeqDelRow();
    void brSeqMoveUp();
    void brSeqMoveDown();
    void brSeqClear();
    void brSeqPreview();
    void brSeqSend();
    void brSeqStop();
    void brParamReadAll();
    void brL103FollowTarget();              /* 切节点: 「L103 调参(桥)」页跟着换一台并回读 */
    void brParamWriteAll();
    void brParamWriteRow();
    void brParamSaveFlash();
    void brParamDefaults();
    void brParamReloadFlash();

    /* 参数表 (CH32V303 板, CAN 功能15/23) */
    void paramReadAll();
    void paramWriteAll();
    void paramWriteRow();
    void paramSaveFlash();
    void paramReloadFlash();
    void paramDefaults();

    /* L103 全参数页签 (UART 命令 / Flash) */
    void l103CfgRead();
    void l103CfgWrite();
    void l103CfgSave();
    void l103CfgErase();
    void l103CfgQueueNext();
    bool l103UseCan() const;
    void l103CanWriteNext();
    void l103CfgApplyParam(const L103::ParamRsp &f);

    /* 原始 */
    void rawUartSend();
    void rawCanSend();
    void clearLog();

    /* 波形 (相电流流) */
    void waveStart();
    void waveStop();
    void waveClear();
    void waveStatTick();
    void waveSetBaud();

private:
    /* 多节点: 每台节点一份状态。真身在这里, 界面上的 m_st/m_cfg/... 只是"当前选中那台"的副本 */
    struct DeviceState {
        int    node       = 0;
        L103::StatusFrame   st;
        L103::CfgFrame      cfg;
        L103::FaultFrame    ft;
        L103::MotorCfgFrame mc;
        L103::InfoFrame     info;
        double  shaftRad   = 0.0;    /* 原始坐标 = 机械坐标 + home */
        double  shaftTurns = 0.0;
        bool    haveStatus = false;
        quint64 frames     = 0;      /* 收到的帧数 */
        qint64  lastMs     = 0;      /* 最后收到该节点帧的时刻 (epoch ms; 0 = 从没收到) */
    };

    void buildUi();
    void wireLinks();

    /* 各页构造 (buildUi 里调用) */
    QWidget *makeStatusTab();
    QWidget *makeActionTab();
    QWidget *makeSeqTab();
    QWidget *makeParamTab();
    QWidget *makeL103CfgTab();
    QWidget *makeLogTab();
    QWidget *makeWaveTab();
    QWidget *makeBridgeParamTab();
    QWidget *makeBridgeSeqTab();

    void log(const QString &s);
    void logTx(const QString &s);
    void logRx(const QString &s);

    void sendUart(const QString &cmd);
    bool sendCan(quint32 id, const QByteArray &data, const QString &note);
    bool bridgeEnsureOpen(const QString &what);
    bool bridgeSend(L103::CanFn fn, const QByteArray &data, const QString &note, int node = 5);
    bool brSeqAbsMode() const;
    void brL103InitBaseline();
    void brL103UpdateState(const QString &tail = QString());
    void brL103SetParam(int idx, double value);
    double brL103ParamValue(int idx) const;
    void applyBrParamRsp(const L103::ParamRsp &f, int node);

    void applyStatus(const L103::StatusFrame &f, int node);
    void applyCfg(const L103::CfgFrame &f, int node);
    void applyFault(const L103::FaultFrame &f, int node);
    void applyMotorCfg(const L103::MotorCfgFrame &f, int node);
    void applyParamRsp(const L103::ParamRsp &f, int node);
    void l103CfgApplyLine(const QString &line);
    void resizeParamTable(int count);   /* 按板子回报的项数重建表, 保留已读到的值 */
    void paramRedrawState();
    void applyInfo(const L103::InfoFrame &f, int node);
    void redrawStatus();
    void syncRawFromMech();   /* 机械坐标一变就按 home 反推原始坐标(口径见 .cpp 注释) */

    /* 动作前无条件先使能 —— 不看当前状态, 每次动作都先发一遍 (家人 2026-10-16 定)。
     * node 省略 = 跟「目标设备」走; 传 NODE_BCAST = 广播 (多节点脚本起跑前用)。 */
    void autoEnable(bool viaCan, int node = -1);
    QString activeChannel() const;          /* "UART" / "CAN" / "桥" / "" */

    /* 帧通道: CAN 与桥接口二选一 (或自动)。COM 走 UART 文本, 不是帧通道。 */
    enum TransportSel { TrAuto = 0, TrCom = 1, TrCan = 2, TrBridge = 3 };
    int  transportSel() const;
    bool frameOpen() const;                 /* 当前选择下有没有可用的帧通道 */
    bool uartFallbackOpen() const;          /* 允许退回 COM 吗 (只在 自动/COM 下允许) */
    bool bridgeActive() const;              /* 这一帧该从桥发/桥收 */

    /* ---- 多节点 ---- */
    int     targetNode() const;             /* 「目标设备」选的节点号, 0 = 广播 */
    int     bridgeNode() const;             /* 桥页用的节点 = 目标设备 (广播时退回 5) */
    bool    targetIsBroadcast() const;
    quint8  currentBoardId() const;         /* 目标节点回报的板型; 没回报就用参数表记下的 */
    void    updateBoardUi();                /* 按板型改文案 / 禁用本板没有的控件 */
    QString nodeText(int node) const;
    DeviceState &devEnsure(int node);
    bool    sendCanFn(L103::CanFn fn, const QByteArray &data, const QString &note, int node = -1);
    void    touchDevice(int node);
    void    recomputeShaft(DeviceState &d);
    void    syncViewFromDevices();
    bool    needSingleNode(const QString &what);   /* 广播不能做的操作 -> 拦路 */
    void    uartNodeNote();                        /* UART 发东西前提醒一句"串口不认节点号" */

    /* ---- 协议版本 (两套 CAN ID) ----
     * 新 = 2026-10-16 起: ID = (节点号<<5) | 功能号, 对齐走 CAN 功能13。
     * 旧 = 2026-10-15 及以前: 固定 ID 0x100/0x110/0x113…, 对齐只有串口 'A'。
     * 自动 = 先两套都发, 收到哪一套的回音就锁定哪一套。 */
    enum ProtoMode { ProtoAuto = 0, ProtoNew = 1, ProtoLegacy = 2 };
    bool    protoIsLegacy() const;
    void    lockProto(int mode, const QString &why);
    void    probeProtocol();      /* 广播问一遍 (新+旧各一条), 用来认出对面是哪一版 */

    /* --- 序列编解码 --- */
    struct Seg {
        int    node   = 1;      /* 这一段发给哪一台 */
        int    waitUs = 0;
        int    rpm    = 0;
        double deg    = 0.0;
        int    dwellMs = 0;
        bool   hr     = false;  /* true = 用功能11 (大角度版, 1 度刻度) */
        bool   abs    = false;  /* true = deg 是绝对机械角(度), 不是增量 */
    };
    bool readSegs(QVector<Seg> *out, QString *err) const;
    bool brReadSegs(QVector<Seg> *out, QString *err) const;
    bool seqAbsMode() const;
    /* 绝对(机械角) -> 相对增量: 固件协议不变, 由上位机按当前位置换算。
     * loops>1 时每台各自在末尾补一段"回第一个目标", 保证每遍起点相同。 */
    bool absSeqToDeltas(const QVector<Seg> &absSegs, int loops,
                        QVector<Seg> *relOut, QStringList *notes, QString *err) const;
    QStringList segsToUartLines(const QVector<Seg> &segs) const;
    /* 按节点分组 + 按角度上限拆段; 返回: 节点号 -> 该台要装的段 (已排好序) */
    QMap<int, QVector<Seg> > groupForCan(const QVector<Seg> &segs, QStringList *notes) const;

    /* ---- 连接 --- */
    UartLink *m_uart = nullptr;
    CanLink  *m_can  = nullptr;
    BridgeLink *m_bridge = nullptr;

    QComboBox *m_cbTransport = nullptr;     /* 自动 / COM / CAN / 桥 */

    QComboBox *m_cbUartPort = nullptr;
    QComboBox *m_cbUartBaud = nullptr;
    QPushButton *m_btnUartOpen = nullptr;
    QLabel    *m_labUartState = nullptr;

    QComboBox *m_cbCanKind = nullptr;
    QComboBox *m_cbCanDev  = nullptr;
    QComboBox *m_cbCanBaud = nullptr;
    QPushButton *m_btnCanOpen = nullptr;
    QPushButton *m_btnCanRefresh = nullptr;
    QLabel    *m_labCanState = nullptr;

    QComboBox *m_cbBridgePort = nullptr;
    QComboBox *m_cbBridgeBaud = nullptr;
    QPushButton *m_btnBridgeOpen = nullptr;
    QPushButton *m_btnBridgeRefresh = nullptr;
    QLabel    *m_labBridgeState = nullptr;

    QComboBox *m_cbProto = nullptr;
    int        m_protoWant   = ProtoAuto;   /* 下拉框选的 */
    int        m_protoLocked = ProtoAuto;   /* 自动探测认出来的: ProtoNew / ProtoLegacy */

    /* ---- 状态 ---- */
    QLabel *m_labMech = nullptr;
    QLabel *m_labShaft = nullptr;
    QLabel *m_labVel = nullptr;
    QLabel *m_labAmp = nullptr;
    QLabel *m_labAmpCaption = nullptr;      /* "相电流" / 板型 2 的"低边总电流 i_ls" */
    QLabel *m_labFlags = nullptr;
    QLabel *m_labStatusBits = nullptr;
    QLabel *m_labFaultActive = nullptr;
    QLabel *m_labFaultLatched = nullptr;
    QLabel *m_labMotorCfg = nullptr;
    QLabel *m_labSeqProg = nullptr;
    QCheckBox *m_chkAutoStatus = nullptr;
    QTimer *m_timerStatus = nullptr;

    /* ---- 动作 ---- */
    QDoubleSpinBox *m_spPvTarget = nullptr;
    QComboBox      *m_cbPvUnit = nullptr;
    QRadioButton   *m_rbPvAbs = nullptr;
    QRadioButton   *m_rbPvRel = nullptr;
    QSpinBox       *m_spPvRpm = nullptr;
    QLabel         *m_labPvDelta = nullptr;
    QCheckBox      *m_chkPvMech = nullptr;
    QSpinBox       *m_spHomeRpm = nullptr;
    QDoubleSpinBox *m_spVelRpm = nullptr;
    QDoubleSpinBox *m_spOlVolt = nullptr;   /* 开环电压 [V] */
    QDoubleSpinBox *m_spOlHz   = nullptr;   /* 开环电频率 [Hz] */

    /* ---- 序列 ---- */
    QTableWidget *m_tblSeq = nullptr;
    QSpinBox     *m_spSeqLoops = nullptr;
    QPlainTextEdit *m_txtSeqPreview = nullptr;
    QComboBox    *m_cbSeqChannel = nullptr;
    QComboBox    *m_cbSeqMode = nullptr;

    /* ---- 参数 ---- */
    QGroupBox               *m_gbpParam    = nullptr;   /* 参数表分组框 (标题带项数) */
    QTableWidget            *m_tblParam    = nullptr;   /* 参数表: 行数 = 板子 INFO 回报的个数 */
    QVector<QDoubleSpinBox *> m_paramSpin;
    QLabel                  *m_labParamState = nullptr;
    int                      m_paramGot     = 0;        /* 本轮读回来几项 */
    int                      m_paramCount   = 0;        /* 表里现在有几行 (板子 INFO 回报) */
    int                      m_paramExpect  = 0;
    quint8                   m_paramFlags   = 0;        /* 最近一帧的 flags */
    bool                     m_paramFail    = false;

    QSpinBox       *m_spPolePairs = nullptr;
    QDoubleSpinBox *m_spStallA = nullptr;
    QDoubleSpinBox *m_spVbus = nullptr;
    QDoubleSpinBox *m_spVlimRadS = nullptr;
    QCheckBox      *m_chkFaultReport = nullptr;
    QSpinBox       *m_spReportPeriod = nullptr;

    /* ---- 桥专用页签: 独立控件, 不复用原页签 ---- */
    QSpinBox       *m_brSpPolePairs = nullptr;
    QDoubleSpinBox *m_brSpStallA = nullptr;
    QDoubleSpinBox *m_brSpVbus = nullptr;
    QDoubleSpinBox *m_brSpVlimRadS = nullptr;
    QCheckBox      *m_brChkFaultReport = nullptr;
    QSpinBox       *m_brSpReportPeriod = nullptr;
    QLabel         *m_brLabState = nullptr;
    QLabel         *m_brLabRun   = nullptr;   /* 桥页「状态」那一格 (节点 5 的 使能/对齐/运行) */
    QDoubleSpinBox *m_brVelRpm   = nullptr;   /* 桥页「速度模式」转速 */
    QTableWidget   *m_brTblSeq = nullptr;
    QSpinBox       *m_brSpSeqLoops = nullptr;
    QComboBox      *m_brCbSeqMode = nullptr;
    QPlainTextEdit *m_brTxtSeqPreview = nullptr;
    QSpinBox       *m_brL103Pp    = nullptr;
    QDoubleSpinBox *m_brL103ILim  = nullptr;
    QDoubleSpinBox *m_brL103Vbus  = nullptr;
    QDoubleSpinBox *m_brL103SpdKp = nullptr;
    QDoubleSpinBox *m_brL103SpdKi = nullptr;
    QDoubleSpinBox *m_brL103PosKp = nullptr;
    QDoubleSpinBox *m_brL103CurKp = nullptr;
    QDoubleSpinBox *m_brL103CurKi = nullptr;
    QDoubleSpinBox *m_brL103R     = nullptr;
    QDoubleSpinBox *m_brL103L     = nullptr;
    QDoubleSpinBox *m_brL103Vlim  = nullptr;
    QDoubleSpinBox *m_brL103Accel = nullptr;
    QDoubleSpinBox *m_brL103Align = nullptr;
    QDoubleSpinBox *m_brL103Pll   = nullptr;
    QDoubleSpinBox *m_brL103Oc    = nullptr;
    QSpinBox       *m_brL103OcMs  = nullptr;
    QComboBox      *m_brL103Vs    = nullptr;
    QLabel         *m_brL103Node  = nullptr;
    QLabel         *m_brL103ParamState = nullptr;
    QVector<double> m_brL103Baseline;
    QVector<bool>   m_brL103BaselineValid;
    QVector<bool>   m_brL103Got;
    QVector<int>    m_brL103WriteFail;
    int             m_brL103GotCount = 0;

    /* ---- L103 全参数页签 ---- */
    QSpinBox       *m_spL103Pp       = nullptr;
    QDoubleSpinBox *m_spL103ILim     = nullptr;
    QDoubleSpinBox *m_spL103Vbus     = nullptr;
    QDoubleSpinBox *m_spL103SpdKp    = nullptr;
    QDoubleSpinBox *m_spL103SpdKi    = nullptr;
    QDoubleSpinBox *m_spL103PosKp    = nullptr;
    QDoubleSpinBox *m_spL103CurKp    = nullptr;
    QDoubleSpinBox *m_spL103CurKi    = nullptr;
    QDoubleSpinBox *m_spL103R        = nullptr;
    QDoubleSpinBox *m_spL103L        = nullptr;
    QDoubleSpinBox *m_spL103Vlim     = nullptr;
    QDoubleSpinBox *m_spL103Accel    = nullptr;
    QDoubleSpinBox *m_spL103Align    = nullptr;
    QDoubleSpinBox *m_spL103Pll      = nullptr;
    QDoubleSpinBox *m_spL103VelRpm   = nullptr;   /* L103 页签: 速度模式给定 (rpm) */
    QDoubleSpinBox *m_spL103Oc       = nullptr;   /* L103 页签: 相电流跳闸阈值 (A) */
QSpinBox       *m_spL103OcMs     = nullptr;   /* L103 页签: 连续超限多久才跳闸 (ms) */
    QComboBox      *m_cbL103Vs       = nullptr;
    QComboBox      *m_cbL103Ch       = nullptr;   /* 0=UART, 1=CAN */
    QLabel         *m_labL103Cfg     = nullptr;
    QLabel         *m_labL103Run     = nullptr;   /* L103 页签上的 en/al 状态 */
    QStringList     m_l103CfgQueue;
    QVector<QPair<int, double> > m_l103CanWrites; /* CAN 写参数: (索引, 值) */
    int             m_l103CanPos = 0;

    /* ---- 日志 ---- */
    QPlainTextEdit *m_txtLog = nullptr;
    QLineEdit      *m_editRawUart = nullptr;
    QLineEdit      *m_editRawCanId = nullptr;
    QLineEdit      *m_editRawCanData = nullptr;

    /* ---- 波形 (相电流流) ---- */
    WaveformParser *m_waveParser = nullptr;
    WavePlot       *m_plot       = nullptr;
    QPushButton    *m_btnWaveStart = nullptr;
    QPushButton    *m_btnWaveStop  = nullptr;
    QPushButton    *m_btnWaveClear = nullptr;
    QCheckBox      *m_chkWavePause = nullptr;
    QCheckBox      *m_chkShowU = nullptr;
    QCheckBox      *m_chkShowV = nullptr;
    QCheckBox      *m_chkShowW = nullptr;
    QCheckBox      *m_chkShowPsu = nullptr;
    QComboBox      *m_cbWaveDiv    = nullptr;
    QComboBox      *m_cbWaveWindow = nullptr;
    QComboBox      *m_cbWaveBaud   = nullptr;
    QLabel         *m_labWaveStat  = nullptr;
    QLabel         *m_labWaveBoardNote = nullptr;   /* 板型 2: "本板无三相电流" */
    QTimer         *m_timerWaveStat = nullptr;
    bool            m_waveStreaming  = false;
    bool            m_wavePollPaused = false;
    quint64         m_waveLastFrames = 0;

    /* ---- 目标设备 (多节点) ---- */
    QSpinBox     *m_spNode     = nullptr;   /* 0 = 广播("0 = 广播 (所有节点)") */
    QPushButton  *m_btnScanNodes = nullptr;
    QLabel       *m_labNodeInfo = nullptr;
    QTableWidget *m_tblDev     = nullptr;
    QLabel       *m_labDevSum  = nullptr;
    QTimer       *m_timerDevUi = nullptr;
    QCheckBox    *m_chkPollOne = nullptr;   /* 只轮询当前节点 */
    QSpinBox     *m_spNewNode  = nullptr;   /* 参数页: 把当前目标节点改成这个号 */

    QMap<int, DeviceState> m_devs;          /* key = 节点号 1..63 */
    int    m_curNode   = 1;                 /* 单节点页显示/操作的那一台 */
    bool   m_syncGuard = false;             /* 防"设备表 <-> 目标设备"互相触发 */

    /* ---- 板子状态缓存 = 选中那一台的副本 (真身在 m_devs) ---- */
    L103::StatusFrame   m_st;
    L103::CfgFrame      m_cfg;
    L103::FaultFrame    m_ft;
    L103::MotorCfgFrame m_mc;
    bool    m_haveStatus = false;
    /* m_shaft*=**原始** shaft 坐标 (功能0 PV 绝对 / UART G 的口径); m_st.mech*=**机械**坐标 (功能17 状态帧 报的) */
    double  m_shaftRad   = 0.0;
    double  m_shaftTurns = 0.0;
    int     m_uartFrames = 0;
    int     m_canFrames  = 0;
};

#endif /* MAINWINDOW_H */
