#include "stream_fifo.hpp"

void StreamFIFO::input(const uint8_t* s, int len) {
	uint32_t wrRPos = rpos;
	uint32_t wrWPos = wpos;
	int bufferSizeMask = bufferSize - 1;
	__sync_synchronize();
	uint32_t spaceLeft = (wrRPos - wrWPos - 1) & bufferSizeMask;
	if(len > int(spaceLeft)) len = spaceLeft;
	uint32_t target = (wrWPos + len) & bufferSizeMask;

	for(uint32_t i = wrWPos; i != target; i = ((i+1) & bufferSizeMask)) {
		buffer[i] = *s;
		s++;
	}
	__sync_synchronize();
	wpos = target;
}

bool StreamFIFO::drain() {
	uint32_t rdRPos = rpos;
	uint32_t rdWPos = wpos;
	__sync_synchronize();
	if(rdRPos == rdWPos)
		return false;
	if(rdWPos > rdRPos) {
		output(buffer + rdRPos, rdWPos - rdRPos);
	} else {
		output(buffer + rdRPos, bufferSize - rdRPos);
		if(rdWPos > 0)
			output(buffer, rdWPos);
	}
	rpos = rdWPos;
	return true;
}
