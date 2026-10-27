# L103 + DRV8316C —— 三相驱动换代（DRV8313 → DRV8316C）+ 三相内置 CSA 电流检测

本仓库是这块**新板**的落盘位置：KiCad 工程（原理图 + PCB）、固件工程、文档、脚本、证据。

## 一句话

在 `L103_CANUSB_DRV8313` 台架板的基础上换两样东西：

- **三相驱动**：DRV8313 → **DRV8316CT**（VFQFN-40，内置三个半桥 + 电荷泵 + buck/LDO）；
- **电流检测**：外部 INA240A2 + 分流电阻 → **8316C 内置三路低边 CSA**
  （`SOA`/`SOB`/`SOC` → `MOT_A_V`/`MOT_B_V`/`MOT_C_V` → `PA1`/`PA2`/`PA3`）。

MCU 仍 `CH32L103K8U6`、编码器仍 MT6701（SSI），**固件的命令集对 8313 那版保持兼容**（只换驱动与采样）。

## 硬件要点（**设计值**，出处见 `docs/审查结论_2026-10-08_家人版原理图与PCB.md`）

| 项 | 值 | 出处 |
|---|---|---|
| MCU | `CH32L103K8U6`（QFN-32） | 网表 |
| 驱动 | `DRV8316CT`，**3x PWM 模式**（`MODE` = 47 kΩ → AVDD） | 网表 + 手册 Table 8-2 |
| PWM / EN | `PWM_A`=PB0、`PWM_B`=PA7、`PWM_C`=PA6；`EN_A`=PB5、`EN_B`=PB4、`EN_C`=PB3 | 网表 |
| `nSLEEP` / `nFAULT` | PB15 / PA8（R46 5.1 kΩ 上拉到 +3V3）；`DRVOFF` 悬空 | 网表 |
| 电流采样 | `SOx` → 330 Ω + 22 pF → `MOT_x_V` → PA1/PA2/PA3 | 网表 |
| CSA 增益 | GAIN 跳线**默认 Hi-Z = 0.3 V/A**（R49/R50 DNP） | 手册 GAIN 表（L966-969） |
| OCP 跳闸 | `OCP/SR` 经 R47 = 0R 到 AGND → 10 / 16 / 22 A（min/typ/max，**触发值，不是电机工作电流**） | 手册 L1059 |
| 母线采样 | `+24VSRC` → R8 47k → R13 3.9k → R18 1k → `ADC0`(PA0)；分压比 13.051:1（**算术值，未实测**） | 网表 |
| 编码器 | `MT6701CT-ACD`，SSI：`ENC_CLK`=PB6、`ENC_DO`=PB8、`ENC_CS`=PB9（=BOOT0，R1 4.7k 下拉） | 网表 + CH32 手册注6 |
| CAN | `MAX3051EKA+T`，CAN1 重映射 PD0(RX)/PD1(TX) | 网表 |
| UART1 | PA9/PA10 @115200 | 网表 |
| 指示灯 | `LED_OK`=PC14、`LED_ERR`=PC15（**灌电流**接法） | 网表 |

> 8316C 的电流是**低边 FET 上的 sense FET** 测的 —— `SOx` 只在**该相低边导通**时才代表相电流
> （手册 8.3.11 L2342-2356）。所以采样时刻要对齐 PWM 载波底部，不能随便在任务里采。

## 目录约定（与姊妹工程 `L103_DRV8313_MT6701` 一致）

| 目录 | 放什么 |
|---|---|
| **`docs/`** | **所有 .md 文档** —— 审查结论、工作报告、方案、交接、调研记录 |
| **`scripts/`** | **所有 .py 脚本** —— 网表/ERC/PCB 几何解析、比对、渲染、导 PDF |
| `pcb/` | KiCad 工程 `L103_DRV8316C`（`L103_DRV8316C.kicad_pro` + 根图 + `CH32L103.kicad_sch` + `DRV8316C.kicad_sch` + `.kicad_pcb`） |
| `firmware/` | 固件工程（MounRiver 7 件套 + Makefile），基线 = 同系列 8313 固件工程 |
| `work/` | **中间产物与证据**：网表、ERC 报告、自检图/PDF、对照材料、改前备份 |
| `outputs/` | **非 md** 的交付物（PDF / 图 / 网表） |
| `README.md` | 仓库首页 —— **唯一留在根上的 md**（GitHub 拿它渲染） |

> 规矩：加东西时 **md 一律进 `docs/`、py 一律进 `scripts/`**，不要再四处乱放。

## 起点（只读参考，我们不往上写）

- 基线固件：同系列 CH32L103 + DRV8313 台架工程（命令表与使用说明 UART+CAN）

- 姊妹工程（2026-10-03，DRV8313 + MT6701）

## 纪律

- **数字要有出处**（实测 / 手册+行号 / 明确标注的估算），**设计值与实测值分开写**。
- 汇报说**网名**（`MOT_A_V` / `ADC0` / `ENC_CLK` / `PWM_A` …），不说位号与 TP 号。
- 新文件一律写在本工程目录下；外部参考目录只读。
- git：`.gitattributes` = `* -text`（**禁行尾转换**，KiCad/C 源码保持原字节）。
