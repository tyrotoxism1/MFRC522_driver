# Project setup
PROJ_NAME = $(shell basename $(CURDIR))

# Adjust to root dir of STM32CubeF4 firmware pacakage
STM32F4_LIB_DIR = /home/tyrotoxism/dev/embedded_dev/STM32CubeF4

CMSIS_DIR = $(STM32F4_LIB_DIR)/Drivers/CMSIS
STARTUP_FILE = $(STM32F4_LIB_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f446xx.s 
SYS_INIT_FILE = $(STM32F4_LIB_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
LINKER_SCIRPT = $(STM32F4_LIB_DIR)/Projects/STM32446E-Nucleo/Templates/STM32CubeIDE/STM32F446RETX_FLASH.ld

# Toolchain 
CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
AS = arm-none-eabi-as
GDB = arm-none-eabi-gdb
OBJCOPY = arm-none-eabi-objcopy

# Compiler options
CFLAGS = -O0 -g -T$(LINKER_SCIRPT) -DSTM32F446xx -DPRINTF_INCLUDE_CONFIG_H -mcpu=cortex-m4 -mthumb --specs=nosys.specs

# Source files
SRC_FILES = src/main.c \
       src/MFRC522.c \
       lib/UART_driver/src/uart.c \
       lib/printf/printf.c \
	   lib/STM32F446_SPI_Driver/src/SPI.c \
       $(SYS_INIT_FILE) \
       $(STARTUP_FILE)


# Include directories
INCLUDES = -Iinclude \
	   -Ilib/UART_driver/src \
	   -Ilib/printf \
	   -Ilib/STM32F446_SPI_Driver/include/ \
	   -I$(STM32F4_LIB_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
	   -I$(STM32F4_LIB_DIR)/Drivers/CMSIS/Core/Include

CFLAGS += $(INCLUDES)

#------------- TARGETS -------------------------#
all: $(PROJ_NAME).bin

clean:
	rm -f $(PROJ_NAME).bin $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRC_FILES)
	$(CC) $(CFLAGS) -o $@ $^

$(PROJ_NAME).bin: $(PROJ_NAME).elf
	$(OBJCOPY) -O binary $^ $@

flash: $(PROJ_NAME).bin 
	st-flash write $(PROJ_NAME).bin 0x8000000 

