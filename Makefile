MCULIB ?= /persist/mculib
DEVICE          = stm32f103cc
OPENCM3_DIR     = /persist/libopencm3
OBJS            += main2.o $(MCULIB)/si5351.o $(MCULIB)/dma_adc.o $(MCULIB)/dma_driver.o $(MCULIB)/usbserial.o

CFLAGS          += -Os -g
CPPFLAGS	+= -Os -g --std=c++0x -fno-exceptions -fno-rtti -I$(MCULIB)/include -DMCULIB_DEVICE_STM32F103
LDFLAGS         += -static -nostartfiles
LDLIBS          += -Wl,--start-group -lgcc -lnosys -Wl,--end-group

LDSCRIPT = ./gd32f303cc.ld

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all

all: binary.elf binary.hex

clean:
	$(Q)$(RM) -rf binary.* *.o

flash: binary.hex
	./st-flash --reset --format ihex write binary.hex


include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
