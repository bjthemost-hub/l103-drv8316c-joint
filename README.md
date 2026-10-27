# L103 + DRV8316C 关节电机驱动板（软硬件 + 两级行星减速机）

[中文](README.md) | [English](README.en.md)

一套可以直接拿去复现的关节电机方案：**CH32L103 主控 + DRV8316C 三相驱动 + MT6701 磁编码器**
的驱动板，跑 FOC 的固件、Qt 上位机，以及配套的**两级行星减速机（J 版，25:1）**。

MIT License。

---

## 目录

| 目录 | 内容 |
|---|---|
| `hardware/` | KiCad 工程（原理图 + PCB，KiCad 10），`gerber/` 里是出板文件 |
| `firmware/` | CH32L103 固件（FOC V2），Makefile + MounRiver Studio 工程两套都在 |
| `gui/` | Qt 上位机 `L103MotorControl`（串口 / CAN 两种通道） |
| `gearbox-J/` | 两级行星减速机 J 版（25:1）：STEP / STL、渲染、尺寸图、打印装配说明 |
| `docs/` | 调参手册、低速 FOC 调试方法与坑、编码器磁场核对、硬件说明 |

---

## 一、硬件

**设计值**（出处：`hardware/L103_DRV8316C.kicad_sch` 网表 + `docs/硬件说明_原README.md`）：

| 项 | 值 |
|---|---|
| 主控 | `CH32L103K8U6`（QFN-32，96 MHz） |
| 三相驱动 | `DRV8316CT`（VFQFN-40，内置三个半桥 / 电荷泵 / buck），**3x PWM 模式**（MODE 经 47 kΩ 到 AVDD） |
| 电流检测 | 8316C 内置三路低边 CSA，`SOx` → 330 Ω + 22 pF → `MOT_x_V` → `PA1/PA2/PA3` |
| CSA 增益 | GAIN 跳线默认 Hi-Z = **0.3 V/A**（R49/R50 DNP） |
| OCP | `OCP/SR` 经 R47 = 0R 到 AGND（触发值见 DRV8316C 手册，**不是电机工作电流**） |
| 编码器 | `MT6701` 磁编码器（SSI，14 位），径向充磁磁铁 6 mm × 2.5 mm、1 对极 |
| CAN | `MAX3051EKA+T`，1 Mbps |
| 母线 | 24 V 标称（固件默认 `MOTOR_VBUS_DEF = 24.0 V`，有采样时以实测为准） |
| 电机（参考） | `MY-4621F`，24N22P → **11 极对**（`firmware/User/board.h`） |

> 编码器气隙：MT6701 手册给的是 0.5 / 1.0 / 2.0 mm（最小 / 典型 / 最大），
> 我们实物目测约 1.5–2.0 mm（未用塞尺量），全周 `Mg` 状态位正常，详见 `docs/`。

## 二、固件

- 三环：电流环 20 kHz、速度环 1 kHz、位置环 200 Hz；相电流在 PWM 谷值同步采样。
- 速度反馈：MT6701 走 **SPI1 + DMA**，10 kHz 取帧，PLL 带宽默认 150 Hz。
- 通信：CAN 1 Mbps（节点号 1–63）或串口 115200（调试口，命令表见固件 `docs/命令表.md`）。
- 参数：16 项可调参数存在 Flash 配置页，上电读回；读不到就用编译默认值。

### 编译

```bash
# 需要 WCH RISC-V GCC（MounRiver Studio 自带）
make -B TOOLCHAIN_PATH="<MounRiver>/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC"
```

### 烧写

```bash
make flash OPENOCD='"<MounRiver>/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin/openocd.exe"' \
           OPENOCD_SCRIPTS='<MounRiver>/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin'
```

> **`make flash` 会整片擦掉配置页（0x0800FC00）** —— 烧完要重新 `SAVE` 一次参数。

也可以直接用 MounRiver Studio 2 打开 `firmware/L103_DRV8316C_FOC_V2.wvproj`，F7 编译、一键下载。

### 实测（2026-10-23，24 V 母线，空载）

- 对齐：`3.20 V` 给约 0.8 s，对齐电流峰值 0.85 A，结束 `aligned=1`。
- 速度环：给定 10.47 rad/s，实测 8.5–11.7 rad/s。
- 位置序列：18 趟 × 2 段、700 rpm，每段 298–321 ms，到位残差 ±0.02 rad（约 1.1°）。

## 三、上位机

`gui/` 是 Qt 工程（`L103MotorControl.pro`，Qt 5/6 均可编），功能：参数页签（读写 RAM / 存 Flash）、
对齐 / 使能 / 速度 / 走位 / 序列、状态与波形显示，通道可选串口或 CAN。

## 四、减速机（J 版，25:1）

两级行星减速机，针对打印件设计。零件、打印参数、装配顺序、强度与转速边界见
`gearbox-J/README.md` 与同目录的原始文档。

## 五、安全与边界

- 这是**开发板级**方案，没有做 UL/CE 之类的认证；上电前请自行确认母线与散热。
- 驱动板默认电流上限 1.80 A，改之前先确认电机额定值。
- 让机构动起来的第一件事永远是**开环**（不读反馈），确认转向、相电流、有没有某相不发力，再进闭环。
- 一旦发现不转、抖动、转速不跟随，**立刻停机**，不要靠抬电压硬顶。

## 六、关于 AI 辅助

本项目的硬件设计、固件、上位机、减速机模型与文档，在设计、编码、调试和写作过程中**大量使用了 AI 辅助**
（OpenAI Codex / GPT 系列模型）。AI 生成的代码与文本均经人工审阅，并在真实板卡与台架上逐项验证；
文中实测数据全部来自实机测试。AI 是工具，不是作者 —— 设计取舍、验证标准与最终结论由人负责。

## 七、许可

MIT，见 `LICENSE`。硬件设计文件与固件源码同样按 MIT 发布。
