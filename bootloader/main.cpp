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


USBSerial serial;
CommandParser cmdParser;
StreamFIFO cmdInputFIFO;
uint8_t cmdInputBuffer[8192];
uint8_t registers[256];
uint32_t& reg_flashWriteStart = *(uint32_t*) &registers[0xe0];

/*
registers map:
-- e0: flashWriteStart[7..0]
-- e1: flashWriteStart[15..8]
-- e2: flashWriteStart[23..16]
-- e3: flashWriteStart[31..24]
-- e4: flash FIFO
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
	rcc_clock_setup_in_hsi_out_48mhz();
	cpu_mhz = 48;

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
	// enable basic peripherals
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

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

