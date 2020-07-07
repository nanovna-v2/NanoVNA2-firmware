MCULIB ?= 		/persist/mculib
DEVICE          = stm32f103cc
OPENCM3_DIR     = /persist/libopencm3
BOARDNAME		= board_v2lite_0
OBJS			+= main.o $(BOARDNAME)/board.o common.o sin_rom.o stream_fifo.o command_parser.o
OBJS            += $(MCULIB)/fastwiring.o $(MCULIB)/dma_adc.o $(MCULIB)/dma_driver.o $(MCULIB)/usbserial.o

VPATH += ../

CFLAGS          += -O2 -g
CPPFLAGS		+= -O2 -g --std=c++0x -fno-exceptions -fno-rtti -I$(BOARDNAME) -I$(MCULIB)/include -DMCULIB_DEVICE_STM32F103 -D_XOPEN_SOURCE=600
LDFLAGS         += -static -nostartfiles
LDLIBS          += -Wl,--start-group -lgcc -lnosys -Wl,--end-group -lm


include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

LDSCRIPT = ./gd32f303cc.ld

.PHONY: clean all

all: binary.elf binary.hex

clean:
	$(Q)$(RM) -rf binary.* *.o

flash: binary.hex
	./st-flash --reset --format ihex write binary.hex


include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
