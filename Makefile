# paths to libraries
MCULIB         ?= mculib
OPENCM3_DIR    ?= libopencm3
DFU_PORT       ?= /dev/ttyACM0

# device config
BOARDNAME       ?= board_v2_2
EXTRA_CFLAGS	?= 

DEVICE          = gd32f303cc_nofpu

OBJS += $(BOARDNAME)/board.o \
    Font5x7.o \
    Font7x13b.o \
    command_parser.o \
    common.o \
    fft.o \
    flash.o \
    gain_cal.o \
    gitversion.hpp \
    globals.o \
    ili9341.o \
    main2.o \
    numfont20x22.o \
    plot.o \
    sin_rom.o \
    stream_fifo.o \
    synthesizers.o \
    ui.o \
    uihw.o \
    vna_measurement.o \
    xpt2046.o \
    $(NULL)

OBJS	+= \
	$(MCULIB)/dma_adc.o \
	$(MCULIB)/dma_driver.o \
	$(MCULIB)/fastwiring.o \
	$(MCULIB)/message_log.o \
	$(MCULIB)/printf.o \
	$(MCULIB)/si5351.o \
	$(MCULIB)/usbserial.o

CFLAGS         += -O2 -g
CPPFLAGS       += $(EXTRA_CFLAGS) -O2 -g -ffast-math -fstack-protector-strong -I$(BOARDNAME) -I$(MCULIB)/include -DMCULIB_DEVICE_STM32F103 -DSTM32F103 -DSTM32F1 -D_XOPEN_SOURCE=600
CPPFLAGS       += -Wall -Wno-unused-function
# CPPFLAGS      += -DDISPLAY_ST7796
CPPFLAGS       +=  -ffunction-sections -fdata-sections
# C++ only flags, CPP is used for both C++ and C files
CXXFLAGS       += --std=c++17 -fno-exceptions -fno-rtti

# safe g++ flags
CPPFLAGS       += -funsigned-char -fwrapv -fno-delete-null-pointer-checks -fno-strict-aliasing

LDFLAGS        += -static -nostartfiles -Wl,--exclude-libs,libssp -Wl,--print-memory-usage
LDFLAGS        += -Wl,--gc-sections
LDLIBS         += -Wl,--start-group -lgcc -lnosys -Wl,--end-group -lm

GITVERSION      = "$(shell git log -n 1 --pretty=format:"git-%ad%h" --date=format:"%Y%m%d-")"

# This is needed for the included genlink-config.mk to work properly
LIBNAME         = opencm3_$(genlink_family)
OPENCM3_LIB     = $(OPENCM3_DIR)/lib/lib$(LIBNAME).a

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

LDSCRIPT        = ./gd32f303cc_with_bootloader.ld

.PHONY: dist-clean clean all

all: $(OPENCM3_LIB) binary.elf binary.hex binary.bin

$(OPENCM3_LIB):
	$(MAKE) -C $(OPENCM3_DIR)

gitversion.hpp: .git/HEAD .git/index
	echo "#define GITVERSION \"$(GITVERSION)\"" > $@

clean:
	$(Q)$(RM) -rf binary.* *.o $(BOARDNAME)/*.o

dist-clean: clean
	make -C $(OPENCM3_DIR) clean

flash: binary.hex
	./st-flash --reset --format ihex write binary.hex

dfu: binary.bin
	python3 dfu.py --file $< --serial $(DFU_PORT)

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
