#pragma once
#include <mculib/fastwiring.hpp>
#include <array>

using namespace mculib;
using namespace std;

enum class RFSWState {
	RF1 = 0,
	RF2 = 1,
	RF3 = 2,
	RF4 = 3
};
static inline void rfsw(Pad sw, int state) {
	digitalWrite(sw, state);
}
static inline void rfsw(array<Pad, 2> sw, RFSWState state) {
	int val = (int)state;
	digitalWrite(sw[0], (val&1) != 0);
	digitalWrite(sw[1], (val&2) != 0);
}
