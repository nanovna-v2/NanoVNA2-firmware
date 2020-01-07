/*
 * This file is derived from libopencm3 example code.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#define PRNT(x)
#define PRNTLN(x)
#include <mculib/fastwiring.hpp>
#include <mculib/usbserial.hpp>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include "../stream_fifo.hpp"
#include "../command_parser.hpp"

using namespace mculib;
using namespace std;

// see https://lists.debian.org/debian-gcc/2003/07/msg00057.html
// this can be any value since we are not using shared libraries.
void* __dso_handle = (void*) &__dso_handle;

int cpu_mhz = 8;
Pad led = PA13;
Pad dfuKey = PB11;
Pad USB0_DP = PA12;
Pad USB0_DM = PA11;


constexpr uint32_t USER_CODE_FLASH = 0x8004000; // 16KB after start of flash
constexpr uint32_t USER_CODE_FLASH_END = 0x8000000 + 256*1024;
constexpr uint32_t FLASH_PAGESIZE = 2048;
constexpr uint32_t FLASH_PAGESIZE_MASK = FLASH_PAGESIZE - 1;
constexpr uint32_t BOOTLOADER_DFU_MAGIC = 0xdeadbabe;
volatile uint32_t& bootloaderDFUIndicator = *(uint32_t*)(0x20000000 + 48*1024 - 4);
volatile uint32_t& bootloaderFirstBootIndicator = *(uint32_t*)(USER_CODE_FLASH + 1024);

USBSerial serial;
CommandParser cmdParser;
StreamFIFO cmdInputFIFO;
uint8_t cmdInputBuffer[16384];
uint8_t registers[256];
uint32_t& reg_flashWriteStart = *(uint32_t*) &registers[0xe0];


// hardware specific functions

// same as rcc_set_usbpre, but with extended divider range:
// 0: divide by 1.5
// 1: divide by 1
// 2: divide by 2.5
// 3: divide by 2
void rcc_set_usbpre_gd32(uint32_t usbpre) {
	uint32_t RCC_CFGR_USBPRE_MASK = uint32_t(0b11) << 22;
	uint32_t old = (RCC_CFGR & ~RCC_CFGR_USBPRE_MASK);
	RCC_CFGR = old | ((usbpre & 0b11) << 22);
}

constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV2 = 0b0000;
constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV4 = 0b0001;
constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV6 = 0b0010;
constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV8 = 0b0011;
constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV12 = 0b0101;
constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV16 = 0b0111;
constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV5 = 0b1000;
constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV6 = 0b1001;
constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV10 = 0b1010;
constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV20 = 0b1011;

void rcc_set_adcpre_gd32(uint32_t adcpre) {
	uint32_t RCC_CFGR_ADCPRE_MASK = (0b11 << 14) | (1 << 28);
	uint32_t RCC_CFGR2_ADCPRE_MASK = 1 << 29;
	uint32_t old = (RCC_CFGR & ~RCC_CFGR_ADCPRE_MASK);
	uint32_t old2 = (RCC_CFGR2 & ~RCC_CFGR2_ADCPRE_MASK);
	RCC_CFGR = old | ((adcpre & 0b11) << 14) | ((adcpre & 0b100) << (28 - 2));
	RCC_CFGR2 = old2 | ((adcpre & 0b1000) << (29 - 3));
}
// mult:
// 4 => 24MHz in
// 5 => 19.2MHz in
void rcc_clock_setup_in_hse_out_96mhz(int mult) {
	 /* Enable internal high-speed oscillator. */
	 rcc_osc_on(RCC_HSI);
	 rcc_wait_for_osc_ready(RCC_HSI);

	 /* Select HSI as SYSCLK source. */
	 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	 /* Enable external high-speed oscillator. */
	 rcc_osc_on(RCC_HSE);
	 rcc_wait_for_osc_ready(RCC_HSE);
	 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

	 /*
	  * Set prescalers for AHB, ADC, ABP1, ABP2.
	  * Do this before touching the PLL (TODO: why?).
	  */
	 rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);					// Set. 96MHz Max. 96MHz
	 rcc_set_adcpre_gd32(GD32_RCC_CFGR_ADCPRE_PCLK2_DIV16);		// Set. 6MHz Max. 40MHz
	 rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);					// Set. 48MHz Max. 60MHz
	 rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);					// Set. 96MHz Max. 120MHz
	 rcc_set_usbpre_gd32(3);									// 96MHz / 2 = 48MHz

	 /*
	  * Sysclk runs with 96MHz -> 0 waitstates.
	  */
	 flash_set_ws(FLASH_ACR_LATENCY_0WS);

	 /*
	  * Set the PLL multiplication factor.
	  */
	 uint32_t multValues[] = {
		0,
		0,
		RCC_CFGR_PLLMUL_PLL_CLK_MUL2,
		RCC_CFGR_PLLMUL_PLL_CLK_MUL3,
		RCC_CFGR_PLLMUL_PLL_CLK_MUL4,
		RCC_CFGR_PLLMUL_PLL_CLK_MUL5};

	 rcc_set_pll_multiplication_factor(multValues[mult]);

	 /* Select HSE as PLL source. */
	 rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

	 /*
	  * External frequency undivided before entering PLL
	  * (only valid/needed for HSE).
	  */
	 rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

	 /* Enable PLL oscillator and wait for it to stabilize. */
	 rcc_osc_on(RCC_PLL);
	 rcc_wait_for_osc_ready(RCC_PLL);

	 /* Select PLL as SYSCLK source. */
	 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

	 /* Set the peripheral clock frequencies used */
	 rcc_ahb_frequency = 96000000;
	 rcc_apb1_frequency = 48000000;
	 rcc_apb2_frequency = 96000000;
}


void rcc_clock_setup_in_hse_out_120mhz(int mult) {
	 /* Enable internal high-speed oscillator. */
	 rcc_osc_on(RCC_HSI);
	 rcc_wait_for_osc_ready(RCC_HSI);

	 /* Select HSI as SYSCLK source. */
	 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	 /* Enable external high-speed oscillator. */
	 rcc_osc_on(RCC_HSE);
	 rcc_wait_for_osc_ready(RCC_HSE);
	 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

	 /*
	  * Set prescalers for AHB, ADC, ABP1, ABP2.
	  * Do this before touching the PLL (TODO: why?).
	  */
	 rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);					// Set. 120MHz Max. 120MHz
	 rcc_set_adcpre_gd32(GD32_RCC_CFGR_ADCPRE_HCLK_DIV20);		// Set. 6MHz Max. 40MHz
	 rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);					// Set. 60MHz Max. 60MHz
	 rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);					// Set. 120MHz Max. 120MHz
	 rcc_set_usbpre_gd32(2);									// 120MHz / 2.5 = 48MHz

	 /*
	  * Sysclk runs with 120MHz -> 0 waitstates.
	  */
	 flash_set_ws(FLASH_ACR_LATENCY_0WS);

	 /*
	  * Set the PLL multiplication factor.
	  */
	 uint32_t multValues[] = {
		0,
		0,
		RCC_CFGR_PLLMUL_PLL_CLK_MUL2,
		RCC_CFGR_PLLMUL_PLL_CLK_MUL3,
		RCC_CFGR_PLLMUL_PLL_CLK_MUL4,
		RCC_CFGR_PLLMUL_PLL_CLK_MUL5};

	 rcc_set_pll_multiplication_factor(multValues[mult]);

	 /* Select HSE as PLL source. */
	 rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

	 /*
	  * External frequency undivided before entering PLL
	  * (only valid/needed for HSE).
	  */
	 rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

	 /* Enable PLL oscillator and wait for it to stabilize. */
	 rcc_osc_on(RCC_PLL);
	 rcc_wait_for_osc_ready(RCC_PLL);

	 /* Select PLL as SYSCLK source. */
	 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

	 /* Set the peripheral clock frequencies used */
	 rcc_ahb_frequency = 120000000;
	 rcc_apb1_frequency = 60000000;
	 rcc_apb2_frequency = 120000000;
}


uint32_t detectHSEFreq() {
	cpu_mhz = 8;
	rcc_osc_on(RCC_HSE);
	rcc_osc_on(RCC_HSI);
	rtc_auto_awake(RCC_HSE, 1 << 19);
	rtc_exit_config_mode();
	delay(2);
	uint32_t tmp = rtc_get_prescale_div_val();
	delay(20);
	uint32_t tmp2 = rtc_get_prescale_div_val();
	// cycles of a fHSE/128 clock elapsed
	uint32_t cycles = (tmp - tmp2) & ((1 << 20) - 1);
	uint32_t freqHz = cycles * 128 * 50;
	return freqHz;
}

// initialize just enough peripherals for gpios to work,
// but do not do anything that can affect user program startup
void minimalHWInit() {
	// enable basic peripherals
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);
}

// initialize all peripherals used by dfu (usb etc).
// only called if we actually enter dfu mode.
void dfuHWInit() {
	uint32_t hseEstimateHz = detectHSEFreq();
	int mult = 2, xtalFreqHz = 0;
	if(hseEstimateHz < 21466200) {
		mult = 5; xtalFreqHz = 19200000;
	} else if(hseEstimateHz < 27712800) {
		mult = 4; xtalFreqHz = 24000000;
	} else if(hseEstimateHz < 35777000) {
		mult = 3; xtalFreqHz = 32000000;
	} else if(hseEstimateHz < 43817000) {
		mult = 3; xtalFreqHz = 40000000;
	} else {
		mult = 2; xtalFreqHz = 48000000;
	}
	if(xtalFreqHz == 40000000) {
		rcc_clock_setup_in_hse_out_120mhz(mult);
		cpu_mhz = 120;
	} else {
		rcc_clock_setup_in_hse_out_96mhz(mult);
		cpu_mhz = 96;
	}

	//rcc_clock_setup_in_hsi_out_48mhz();
	//cpu_mhz = 48;
}




/*
registers map:
-- e0: flashWriteStart[7..0]
-- e1: flashWriteStart[15..8]
-- e2: flashWriteStart[23..16]
-- e3: flashWriteStart[31..24]
-- e4: flash FIFO
-- ef: write 0x5e to reboot device
-- f0: device variant
-- f1: protocol version (01)
-- f2: hardware revision (always 0 in dfu mode)
-- f3: firmware major version (ff => dfu mode)
-- f4: firmware minor version (bootloader version)
*/

void setMspAndJump(uint32_t usrAddr) {
	// Dedicated function with no call to any function (appart the last call)
	// This way, there is no manipulation of the stack here, ensuring that GGC
	// didn't insert any pop from the SP after having set the MSP.

	// reset ptr in vector table
	typedef void (*funcPtr)(void);
	uint32_t jumpAddr = *(volatile uint32_t *)(usrAddr + 0x04);

	funcPtr usrMain = (funcPtr) jumpAddr;

	SCB_VTOR = (usrAddr);

	asm volatile("msr msp, %0"::"g"(*(volatile uint32_t *)usrAddr));

	usrMain();                                /* go! */
}

bool shouldEnterDFU() {
	// if the application left a magic value at the end of ram
	// before a soft reset, enter dfu mode.
	if(bootloaderDFUIndicator == BOOTLOADER_DFU_MAGIC)
		return true;

	// magic value in flash to indicate factory dfu
	if(bootloaderFirstBootIndicator == BOOTLOADER_DFU_MAGIC)
		return true;

	pinMode(dfuKey, INPUT_PULLUP);
	delayMicroseconds(10);
	if(digitalRead(dfuKey) == 0)
		return true;
	return false;
}

constexpr int wordSize = 4;
uint8_t wordBuffer[wordSize];
int wordBufferLength = 0;

inline void handleFlashWriteWord(uint32_t word) {
	// only erase/program if we are in the user flash region
	if(reg_flashWriteStart >= USER_CODE_FLASH && reg_flashWriteStart < USER_CODE_FLASH_END) {
		// if we are in a new page, erase the page
		if((reg_flashWriteStart & FLASH_PAGESIZE_MASK) == 0) {
			flash_erase_page(reg_flashWriteStart);
		}
		flash_program_word(reg_flashWriteStart, word);
	}
	reg_flashWriteStart += wordSize;
}
void handleFlashWrite(const uint8_t* data, int bytes) {
	const uint8_t* end = data + bytes;

	// if there are leftover bytes in the word buffer, fill it
	if(wordBufferLength != 0) {
		while(wordBufferLength < wordSize && data < end) {
			wordBuffer[wordBufferLength] = *data;
			data++;
			wordBufferLength++;
		}
		if(data >= end) return;
	}

	if(wordBufferLength == wordSize) {
		handleFlashWriteWord(*(uint32_t*) wordBuffer);
		wordBufferLength = 0;
	}

	while((data + wordSize) <= end) {
		handleFlashWriteWord(*(uint32_t*) data);
		data += wordSize;
	}

	// put leftover bytes in word buffer
	while(wordBufferLength < wordSize && data < end) {
		wordBuffer[wordBufferLength] = *data;
		data++;
		wordBufferLength++;
	}
}

void cmdInit() {
	cmdParser.handleReadFIFO = [](int address, int nValues) {
		
	};
	cmdParser.handleWriteFIFO = [](int address, int totalBytes, int nBytes, const uint8_t* data) {
		if(address == 0xe4)
			handleFlashWrite(data, nBytes);
	};
	cmdParser.handleWrite = [](int address) {
		if(address == 0xef && registers[address] == 0x5e) {
			// clear enter dfu indicator
			bootloaderDFUIndicator = 0;

			// reset device
			SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
			// the cpu may not be reset immediately and can go on to execute more code
			// before the reset actually happens. Wait a bit to ensure no more commands
			// are processed.
			delay(300);
			// if we get here the reset request wasn't honored (some platforms ignore it).
			// Continue processing commands.
		}
	};
	cmdParser.send = [](const uint8_t* s, int len) {
		return serial.print((char*) s, len);
	};
	cmdParser.registers = registers;
	cmdParser.registersSizeMask = sizeof(registers) - 1;

	cmdInputFIFO.buffer = cmdInputBuffer;
	cmdInputFIFO.bufferSize = sizeof(cmdInputBuffer);
	cmdInputFIFO.output = [](const uint8_t* s, int len) {
		cmdParser.handleInput(s, len);
	};
	serial.setReceiveCallback([](uint8_t* s, int len) {
		cmdInputFIFO.input(s, len);
	});
}

void dfuMain() {
	dfuHWInit();

	// set version registers (accessed through usb serial)
	registers[0xf0] = 2;	// device variant (NanoVNA V2)
	registers[0xf1] = 1;	// protocol version
	registers[0xf2] = 0;	// board revision (always 0 in dfu mode)
	registers[0xf3] = 0xff;	// firmware major version (0xff in dfu mode)
	registers[0xf4] = 0;	// firmware minor version

	// set up command interface
	cmdInit();
	flash_unlock();

	digitalWrite(USB0_DP, LOW);
	pinMode(USB0_DP, OUTPUT);
	delay(300);
	pinMode(USB0_DP, INPUT);
	// baud rate is ignored for usb serial
	serial.begin(115200);

	while(true) {
		cmdInputFIFO.drain();
	}
}

int main() {
	minimalHWInit();

	if(shouldEnterDFU()) {
		dfuMain();
	} else {
		setMspAndJump(USER_CODE_FLASH);
	}
}

extern "C" void abort() {
	while (1) {
		for(int i=0;i<3;i++) {
			digitalWrite(led, HIGH);
			delay(100);
			digitalWrite(led, LOW);
			delay(100);
		}
		delay(1000);
	}
}

extern "C" void *memcpy(void *dest, const void *src, size_t n) {
	for(int i=0;i<n;i++)
		((char*)dest)[i] = ((char*)src)[i];
	return dest;
}
extern "C" int atoi(const char* s) {
	// TODO: implement
	return 0;
}
extern "C" void __aeabi_atexit(void * arg , void (* func ) (void *)) {
	// Leave this function empty. Program never exits.
}
extern "C" size_t strlen(const char* s) {
	int i = 0;
	while(*s != 0) {
		i++;
		s++;
	}
	return i;
}

