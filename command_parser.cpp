#include "command_parser.hpp"

void CommandParser::handleInput(const uint8_t* s, int len) {
	const uint8_t* end = s + len;
	while(s < end) {
		uint8_t c = *s;
		if(cmdPhase == 0) {
			cmdOpcode = c;
			if(cmdOpcode == 0)
				goto cont;
			if(cmdOpcode == 0x0d) {
				send((const uint8_t*) "2", 1);
				goto cont;
			}
			cmdPhase++;
			goto cont;
		}
		if(cmdPhase == 1) {
			cmdStartAddress = cmdAddress = c;
			if(cmdOpcode == 0x21)
				cmdEndAddress = cmdAddress + 2;
			if(cmdOpcode == 0x22)
				cmdEndAddress = cmdAddress + 4;
			if(cmdOpcode == 0x23)
				cmdEndAddress = cmdAddress + 8;
			cmdPhase++;
			goto cont;
		}
		switch(cmdOpcode) {
			case 0x10:
				send(registers + cmdAddress, 1);
				cmdPhase = 0;
				break;
			case 0x11:
				send(registers + cmdAddress, 2);
				cmdPhase = 0;
				break;
			case 0x12:
				send(registers + cmdAddress, 4);
				cmdPhase = 0;
				break;
			case 0x13:
				handleReadFIFO(cmdAddress, c);
				cmdPhase = 0;
				break;
			case 0x20:
				registers[cmdAddress & registersSizeMask] = c;
				cmdPhase = 0;
				handleWrite(cmdAddress);
				break;
			case 0x21:
			case 0x22:
			case 0x23:
				registers[cmdAddress & registersSizeMask] = c;
				cmdAddress++;
				if(cmdAddress == cmdEndAddress) {
					cmdPhase = 0;
					handleWrite(cmdStartAddress);
				}
				break;
			default:
				cmdPhase = 0;
				break;
		}
	cont:
		s++;
	}
}
