#include <stdint.h>

// size must be a power of 2.
// actual capacity is size - 1 elements.
// thread safe in the case of one reader and multiple writers.
template<class T, int size>
class FIFO {
public:
	static constexpr uint32_t sizeMask = size - 1;

	T elements[size];

	// 1 if element is commited, 0 otherwise
	volatile uint8_t elementStatus[size] = {};

	FIFO() {}

	// status functions

	bool writable() const { return ((_wpos2+1) & sizeMask) != _rpos; }
	bool readable() const {
		if((_wpos2 & sizeMask) == _rpos)
			return false;

		__sync_synchronize();
		// the queue is readable only if the next item is committed
		return elementStatus[_rpos] == 0 ? false : true;
	}

	// access element at index i
	T& at(uint32_t i) { return elements[i & sizeMask]; }

	// peek at next value
	T& read() {
		__sync_synchronize();
		return elements[_rpos];
	}
	void dequeue() {
		if(!readable()) abort();
		elementStatus[_rpos] = 0;
		_rpos = (_rpos+1) & sizeMask;
	}

	void clear() {
		while(readable()) dequeue();
	}

	// returns (uint32_t)-1 if there is no space in the FIFO, or
	// an index into the elements array otherwise.
	uint32_t beginEnqueue() {
		uint32_t myWPos2, myRPos;
		bool success;
		
		// fetch a snapshot of write position, then read position.
	retry1:
		myWPos2 = _wpos2;
		__sync_synchronize();
		myRPos = _rpos;
		
		// check fifo full
		if(((myWPos2+1) & sizeMask) == myRPos)
			return (uint32_t) -1;
		
		// attempt to grab a spot in the write end of the queue.
		// if _wpos2 has changed in the meantime, repeat.
		// it is possible that in the meantime the FIFO has gone through a 
		// full cycle, _wpos2 has wrapped around to our value, and FIFO
		// is now full, which bypasses the fullness check above.
		// This is dealt with by having _wpos2 use the full range of a
		// 32 bit int, and only taking (mod size) when accessing elements.
		success = __sync_bool_compare_and_swap(&_wpos2, myWPos2, myWPos2 + 1);
		if(!success)
			goto retry1;
		return myWPos2 & sizeMask;
	}

	uint32_t endEnqueue(uint32_t index) {
		__sync_synchronize();
		elementStatus[index] = 1;
	}

	bool enqueue(const T& value) {
		uint32_t i = beginEnqueue();
		if(i == (uint32_t) -1)
			return false;
		at(i) = value;
		endEnqueue(i);
	}

protected:
	// _rpos points to the next to be consumed value.
	// (_wpos2 mod size) points to the next to be written value.
	volatile uint32_t _rpos = 0, _wpos2 = 0;
};

