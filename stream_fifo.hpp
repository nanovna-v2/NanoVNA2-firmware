#pragma once
#include <mculib/small_function.hpp>
#include <stdint.h>

// simple byte stream FIFO for passing data between threads
class StreamFIFO {
public:
	// user provided buffer area; size must be a power of 2
	uint8_t* buffer = nullptr;
	int bufferSize = 0;

	// user provided handler at read side
	small_function<void(const uint8_t* s, int len)> output;

	// append stream data
	void input(const uint8_t* s, int len);

	// call this function on the consumer thread to empty
	// the FIFO by calling output() on buffered data.
	// returns true if data was processed, false otherwise.
	bool drain();

	// state variables
	volatile uint32_t wpos = 0;
	volatile uint32_t rpos = 0;
};
