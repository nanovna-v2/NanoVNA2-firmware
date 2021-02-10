MCULIB         ?= ../mculib
DEVICE          = gd32f303cc
OPENCM3_DIR    ?= ../libopencm3
OBJS           += bootloader.o ../common.o ../command_parser.o ../stream_fifo.o
OBJS           += $(MCULIB)/fastwiring.o $(MCULIB)/usbserial.o


CFLAGS         += -Os -g
CPPFLAGS       += -Os -g --std=c++0x -fno-exceptions -fno-rtti -I$(MCULIB)/include -DMCULIB_DEVICE_STM32F103  -DSTM32F103 -DSTM32F1 -D_XOPEN_SOURCE=600
LDFLAGS        += -static -nostartfiles -nostdlib -Wl,--print-memory-usage
LDLIBS         += -Wl,--start-group -lgcc -lnosys -Wl,--end-group -lm


include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

LDSCRIPT = ./gd32f303cc.ld

.PHONY: clean all

all: binary.elf binary.hex binary.bin

clean:
	$(Q)$(RM) -rf binary.* *.o

flash: binary.hex
	./st-flash --reset --format ihex write binary.hex

factory_flash: binary.bin
	# set magic value to enter bootloader mode
	echo -n -e '\xbe\xba\x0b\xb0\xce\xfa\xef\xbe' | dd of=binary.bin bs=1024 seek=17
	./st-flash --reset write binary.bin 0x08000000

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
