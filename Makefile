# paths to libraries
MCULIB         ?= /persist/mculib
OPENCM3_DIR    ?= /persist/libopencm3

# device config
BOARDNAME		= board_v2_2
DEVICE          = gd32f303cc_nofpu


OBJS			+= main2.o $(BOARDNAME)/board.o vna_measurement.o xpt2046.o uihw.o common.o synthesizers.o gitversion.hpp
OBJS			+= globals.o ui.o flash.o plot.o ili9341.o Font5x7.o Font7x13b.o numfont20x22.o fft.o command_parser.o stream_fifo.o
OBJS			+= sin_rom.o
OBJS            += $(MCULIB)/message_log.o $(MCULIB)/printf.o $(MCULIB)/fastwiring.o $(MCULIB)/si5351.o $(MCULIB)/dma_adc.o $(MCULIB)/dma_driver.o $(MCULIB)/usbserial.o

CFLAGS          += -O2 -g
CPPFLAGS		+= -O2 -g -ffast-math --std=c++17 -fno-exceptions -fno-rtti -fstack-protector-strong -I$(BOARDNAME) -I$(MCULIB)/include -DMCULIB_DEVICE_STM32F103 -DSTM32F103 -DSTM32F1 -D_XOPEN_SOURCE=600
#CPPFLAGS		+= -DDISPLAY_ST7796
CPPFLAGS		+=  -ffunction-sections -fdata-sections

# safe g++ flags
CPPFLAGS		+= -funsigned-char -fwrapv -fno-delete-null-pointer-checks -fno-strict-aliasing

LDFLAGS         += -static -nostartfiles -Wl,--exclude-libs,libssp -Wl,--print-memory-usage
LDFLAGS			+= -Wl,--gc-sections
LDLIBS          += -Wl,--start-group -lgcc -lnosys -Wl,--end-group -lm

GITVERSION		= "$(shell git log -n 1 --pretty=format:"git-%ad%h" --date=format:"%Y%m%d-")"

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

LDSCRIPT = ./gd32f303cc_with_bootloader.ld

.PHONY: clean all

all: binary.elf binary.hex binary.bin

gitversion.hpp: .git/HEAD .git/index
	echo "#define GITVERSION \"$(GITVERSION)\"" > $@

clean:
	$(Q)$(RM) -rf binary.* *.o

flash: binary.hex
	./st-flash --reset --format ihex write binary.hex


include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
