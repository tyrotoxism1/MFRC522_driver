# Project setup
PROJ_NAME = $(shell basename $(CURDIR))

# Adjust to root dir of STM32CubeF4 firmware pacakage
STM32F4_LIB_DIR = /home/tyrotoxism/dev/embedded_dev/STM32CubeF4

CMSIS_DIR = $(STM32F4_LIB_DIR)/Drivers/CMSIS
HAL_DIR = $(STM32F4_LIB_DIR)/Drivers/STM32F4xx_HAL_Driver
STARTUP_FILE = $(STM32F4_LIB_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f446xx.s 
SYS_INIT_FILE = $(STM32F4_LIB_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
LINKER_SCIRPT = $(STM32F4_LIB_DIR)/Projects/STM32446E-Nucleo/Templates/STM32CubeIDE/STM32F446RETX_FLASH.ld

# Toolchain 
CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
AS = arm-none-eabi-as
GDB = arm-none-eabi-gdb
OBJCOPY = arm-none-eabi-objcopy

# Debug
OPENOCD_CFG = -f interface/stlink.cfg -f target/stm32f4x.cfg
ELF = $(PROJ_NAME).elf

# Compiler options
CFLAGS = -O0 -g -DSTM32F446xx -DPRINTF_INCLUDE_CONFIG_H -mcpu=cortex-m4 -mthumb --specs=nosys.specs
LDFLAGS = -T$(LINKER_SCIRPT)

# Source files
SRC_FILES = src/main.c \
       src/MFRC522.c \
       lib/UART_driver/src/uart.c \
       lib/printf/printf.c \
	   lib/STM32F446_SPI_Driver/src/SPI.c \
       $(SYS_INIT_FILE) \
       $(STARTUP_FILE) \
	   $(HAL_DIR)/Src/stm32f4xx_hal.c \
	   $(HAL_DIR)/Src/stm32f4xx_hal_gpio.c \
	   $(HAL_DIR)/Src/stm32f4xx_hal_rcc.c \
	   $(HAL_DIR)/Src/stm32f4xx_hal_cortex.c \
	   $(HAL_DIR)/Src/stm32f4xx_hal_spi.c \
	   $(HAL_DIR)/Src/stm32f4xx_hal_dma.c \


# Include directories
INCLUDES = -Iinclude \
	   -Ilib/UART_driver/src \
	   -Ilib/printf \
	   -Ilib/STM32F446_SPI_Driver/include/ \
	   -I$(CMSIS_DIR)/Device/ST/STM32F4xx/Include \
	   -I$(CMSIS_DIR)/Core/Include \
	   -I$(HAL_DIR)/Inc \

CFLAGS += $(INCLUDES)

#------------- TARGETS -------------------------#
.PHONY: openocd gdb debug

all: $(PROJ_NAME).bin

clean:
	rm -f $(PROJ_NAME).bin $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRC_FILES)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(PROJ_NAME).bin: $(PROJ_NAME).elf
	$(OBJCOPY) -O binary $^ $@

flash: $(PROJ_NAME).bin 
	st-flash write $(PROJ_NAME).bin 0x8000000 

openocd: 
	openocd $(OPENOCD_CFG)

gdb:
	arm-none-eabi-gdb $(ELF) \
		-ex "target extended-remote localhost:3333" \
		-ex "monitor reset halt" \
		-ex "load"

debug:
	@openocd -f interface/stlink.cfg -f target/stm32f4x.cfg & \
	OCD_PID=$$!; \
	arm-none-eabi-gdb build/firmware.elf -ex "target remote localhost:3333" -ex "monitor reset halt"; \
	kill $$OCD_PID



