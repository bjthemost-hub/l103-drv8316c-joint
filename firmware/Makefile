# =============================================================================
# CH32L103K8U6 + DRV8316C  FOC V2 - GNU Make build
# Toolchain: WCH RISC-V Embedded GCC (MounRiver Studio 2 自带)
#   make           编 HEX/BIN/LST
#   make -B        全量重编 (改宏之后必须用这个: make 不跟踪编译宏)
#   make flash     用 MounRiver 的 OpenOCD 烧写
# 注意: **不许加 -flto** (2026-09-28 实测: LTO 会让 startup 的 .weak 中断桩
#       抢在真 ISR 之前链进去, 向量表全指向死循环, 烧完板子完全没动静)。
# =============================================================================

TOOLCHAIN_PATH ?= c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC
CC      := "$(TOOLCHAIN_PATH)/bin/riscv-none-embed-gcc"
OBJCOPY := "$(TOOLCHAIN_PATH)/bin/riscv-none-embed-objcopy"
OBJDUMP := "$(TOOLCHAIN_PATH)/bin/riscv-none-embed-objdump"
SIZE    := "$(TOOLCHAIN_PATH)/bin/riscv-none-embed-size"

OPENOCD        ?= "c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin/openocd.exe"
OPENOCD_SCRIPTS ?= c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin

TARGET    := L103_DRV8316C_FOC_V2
BUILD_DIR := build
SRC_DIR   := SRC
APP_DIR   := User

ARCH_FLAGS := -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore
CFLAGS  := $(ARCH_FLAGS) -Os -g -std=gnu99 \
           -ffunction-sections -fdata-sections -fno-common -fsigned-char \
           -fmessage-length=0 -Wall -fmax-errors=20 -MMD -MP \
           -I$(APP_DIR) -I$(SRC_DIR)/Debug -I$(SRC_DIR)/Core -I$(SRC_DIR)/Peripheral/inc
ASFLAGS := $(ARCH_FLAGS) -g -x assembler-with-cpp -I$(SRC_DIR)/Startup

# 调试提示总开关 (家规: 面向人的帮助/诊断文本默认关)
CMD_TIPS ?= 0
CFLAGS   += -DCMD_TIPS=$(CMD_TIPS)

# CAN 内部回环自检开关 (1 = 芯片自己发自己收, 只用于诊断; 正式固件必须 0)
CAN_LOOPBACK ?= 0
CFLAGS   += -DCAN_LOOPBACK=$(CAN_LOOPBACK)

LDFLAGS := $(ARCH_FLAGS) -nostartfiles -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
           -T$(SRC_DIR)/Ld/Link.ld --specs=nano.specs -lm \
           -Wl,--print-memory-usage

C_SOURCES := \
	$(APP_DIR)/main.c \
	$(APP_DIR)/cmd.c \
	$(APP_DIR)/isr.c \
	$(APP_DIR)/ch32l103_it.c \
	$(APP_DIR)/system_ch32l103.c \
	$(APP_DIR)/mini_fmt.c \
	$(APP_DIR)/fmt.c \
	$(APP_DIR)/pi.c \
	$(APP_DIR)/foc_math.c \
	$(APP_DIR)/foc.c \
	$(APP_DIR)/enc.c \
	$(APP_DIR)/csense.c \
	$(APP_DIR)/motor.c \
	$(APP_DIR)/drv8316c.c \
	$(APP_DIR)/motion.c \
	$(APP_DIR)/nvcfg.c \
	$(APP_DIR)/posstore.c \
	$(APP_DIR)/faults.c \
	$(APP_DIR)/can_bus.c \
	$(SRC_DIR)/Debug/debug.c \
	$(SRC_DIR)/Core/core_riscv.c \
	$(wildcard $(SRC_DIR)/Peripheral/src/*.c)

ASM_SOURCES := $(SRC_DIR)/Startup/startup_ch32l103.S

OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_SOURCES)) \
           $(patsubst %.S,$(BUILD_DIR)/%.o,$(ASM_SOURCES))
DEPS    := $(OBJECTS:.o=.d)

ELF := $(BUILD_DIR)/$(TARGET).elf
HEX := $(BUILD_DIR)/$(TARGET).hex
BIN := $(BUILD_DIR)/$(TARGET).bin
LST := $(BUILD_DIR)/$(TARGET).lst

.PHONY: all clean size flash rebuild

all: $(HEX) $(BIN) $(LST) size

$(ELF): $(OBJECTS) $(SRC_DIR)/Ld/Link.ld | $(BUILD_DIR)
	@echo "[LD]  $@"
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(HEX): $(ELF)
	@echo "[HEX] $@"
	$(OBJCOPY) -O ihex $< $@

$(BIN): $(ELF)
	@echo "[BIN] $@"
	$(OBJCOPY) -O binary $< $@

$(LST): $(ELF)
	@echo "[LST] $@"
	$(OBJDUMP) -h -S $< > $@

size: $(ELF)
	@echo "[SIZE]"
	$(SIZE) $<

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "[CC]  $<"
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo "[AS]  $<"
	$(CC) $(ASFLAGS) -c -o $@ $<

$(BUILD_DIR):
	@mkdir -p $@

flash: $(ELF)
	@echo "[FLASH] $<"
	$(OPENOCD) -f debug/openocd.cfg -s "$(OPENOCD_SCRIPTS)" \
		-c "init" -c "halt" -c "program $(ELF) verify reset" -c "exit"

clean:
	@rm -rf $(BUILD_DIR)

rebuild: clean all

-include $(DEPS)