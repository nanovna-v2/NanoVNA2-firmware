#pragma once
#include <mculib/fastwiring.hpp>

class Debouncer {
public:
	mculib::Pad pad;
	int transitionThreshold = 100;

	// true => high; false => low
	bool state = false;

	// how long the pin state has been different from .state
	int cnt = 0;
	void init() {
		state = mculib::digitalRead(pad) != 0;
		cnt = 0;
	}
	bool checkChanged() {
		bool newState = mculib::digitalRead(pad) != 0;
		if(newState == state) {
			cnt = 0;
		} else {
			cnt++;
			if(cnt > transitionThreshold) {
				state = newState;
				cnt = 0;
				return true;
			}
		}
		return false;
	}
};
