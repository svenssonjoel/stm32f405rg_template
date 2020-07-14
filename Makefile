

BUILD_DIR = ./build

CMSIS_H = ./Libraries/CMSIS/Include
CMSIS_DEVICE_H = ./Libraries/CMSIS/Device/ST/STM32F4xx/Include
STM_H   = ./Libraries/STM32F4xx_StdPeriph_Driver/inc
STM_C   = ./Libraries/STM32F4xx_StdPeriph_Driver/src

DEVICE  = -DSTM32F40_41xxx


C_DEFS = $(DEVICE) -DUSE_FULL_ASSERT -DUSE_STDPERIPH_DRIVER 

# ASM sources
ASM_SOURCES = startup_stm32f405xx.s

PREFIX = arm-none-eabi-

CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size


HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES = -I$(CMSIS_H) -I$(STM_H) -I$(CMSIS_DEVICE_H) -I.

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -pedantic -fdata-sections -ffunction-sectionsa
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -pedantic -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# link script
LDSCRIPT = STM32F405RGTx.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


C_SOURCES = \
   $(STM_C)/misc.c\
   $(STM_C)/stm32f4xx_adc.c\
   $(STM_C)/stm32f4xx_can.c\
   $(STM_C)/stm32f4xx_cec.c\
   $(STM_C)/stm32f4xx_crc.c\
   $(STM_C)/stm32f4xx_cryp_aes.c\
   $(STM_C)/stm32f4xx_cryp.c\
   $(STM_C)/stm32f4xx_cryp_des.c\
   $(STM_C)/stm32f4xx_cryp_tdes.c\
   $(STM_C)/stm32f4xx_dac.c\
   $(STM_C)/stm32f4xx_dbgmcu.c\
   $(STM_C)/stm32f4xx_dcmi.c\
   $(STM_C)/stm32f4xx_dfsdm.c\
   $(STM_C)/stm32f4xx_dma2d.c\
   $(STM_C)/stm32f4xx_dma.c\
   $(STM_C)/stm32f4xx_dsi.c\
   $(STM_C)/stm32f4xx_exti.c\
   $(STM_C)/stm32f4xx_flash.c\
   $(STM_C)/stm32f4xx_flash_ramfunc.c\
   $(STM_C)/stm32f4xx_fmpi2c.c\
   $(STM_C)/stm32f4xx_fsmc.c\
   $(STM_C)/stm32f4xx_gpio.c\
   $(STM_C)/stm32f4xx_hash.c\
   $(STM_C)/stm32f4xx_hash_md5.c\
   $(STM_C)/stm32f4xx_hash_sha1.c\
   $(STM_C)/stm32f4xx_i2c.c\
   $(STM_C)/stm32f4xx_iwdg.c\
   $(STM_C)/stm32f4xx_lptim.c\
   $(STM_C)/stm32f4xx_ltdc.c\
   $(STM_C)/stm32f4xx_pwr.c\
   $(STM_C)/stm32f4xx_qspi.c\
   $(STM_C)/stm32f4xx_rcc.c\
   $(STM_C)/stm32f4xx_rng.c\
   $(STM_C)/stm32f4xx_rtc.c\
   $(STM_C)/stm32f4xx_sai.c\
   $(STM_C)/stm32f4xx_sdio.c\
   $(STM_C)/stm32f4xx_spdifrx.c\
   $(STM_C)/stm32f4xx_spi.c\
   $(STM_C)/stm32f4xx_syscfg.c\
   $(STM_C)/stm32f4xx_tim.c\
   $(STM_C)/stm32f4xx_usart.c\
   $(STM_C)/stm32f4xx_wwdg.c\
   ./system_stm32f4xx.c\
   $(wildcard ./src/*.c)

#   $(STM_C)/stm32f4xx_fmc.c 

# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	

$(BUILD_DIR):
	mkdir $@		

clean:
	-rm -fR $(BUILD_DIR)


