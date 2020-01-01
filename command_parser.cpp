#include "command_parser.hpp"

void CommandParser::handleInput(const uint8_t* s, int len) {
	const uint8_t* end = s + len;
	if(writeFIFOBytesLeft > 0) {
		int consume = writeFIFOBytesLeft;
		if(consume > len)
			consume = len;
		handleWriteFIFO(cmdAddress, 0, consume, s);
		s += consume;
		writeFIFOBytesLeft -= consume;
		if(writeFIFOBytesLeft > 0)
			return;
	}
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

			// 1-parameter commands
			if(cmdOpcode >= 0x10 && cmdOpcode <= 0x12) {
				uint8_t* rPtr = registers + (cmdAddress & registersSizeMask);
				switch(cmdOpcode) {
					case 0x10:
						send(rPtr, 1);
						break;
					case 0x11:
						send(rPtr, 2);
						break;
					case 0x12:
						send(rPtr, 4);
						break;
				}
				cmdPhase = 0;
				goto cont;
			} else {
				cmdPhase++;
				goto cont;
			}
		}
		// 2 or more parameter commands
		switch(cmdOpcode) {
			case 0x18:
			case 0x13: // FOR TEMPORARY BACKWARDS COMPATIBILITY ONLY. WILL BE REMOVED.
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
			case 0x28:
			{
				int totalBytes = (int) (uint8_t) c;
				s++; // move past the size byte
				cmdPhase = 0; // resume command processing once data is consumed

				int bufBytes = end - s;
				if(bufBytes >= totalBytes) {
					// all data is in the current buffer
					handleWriteFIFO(cmdAddress, totalBytes, totalBytes, s);
					s += totalBytes;
					continue;
				}
				// partial data is in the buffer
				handleWriteFIFO(cmdAddress, totalBytes, bufBytes, s);
				writeFIFOBytesLeft = totalBytes - bufBytes;
				return;
			}
			default:
				cmdPhase = 0;
				break;
		}
	cont:
		s++;
	}
}
