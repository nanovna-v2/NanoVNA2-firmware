#pragma once
#include "common.hpp"
#include "board.hpp"

namespace synthesizers {

	// initialize and configure si5351
	bool si5351_setup();

	// set si5351 frequency for tx and rx port.
	// returns 0 if one output divider changed;
	// returns 1 if two output dividers changed;
	// returns 2 if pll updated.
	int si5351_set(uint32_t rxFreqHz, uint32_t txFreqHz);


	// freqHz must be a multiple of freqStepHz
	template<class T>
	static void adf4350_set(T& adf4350, freqHz_t freqHz, uint32_t freqStepHz) {
		int O = 1;
		int R = 1; // adf4350 reference divide
		
		if(freqHz	> 2200000000)
			O = 1;
		else if(freqHz	> 1100000000)
			O = 2;
		else if(freqHz	> 550000000)
			O = 4;
		else if(freqHz	> 275000000)
			O = 8;
		else if(freqHz	> 137500000)
			O = 16;
		else if(freqHz	> 68750000)
			O = 32;
		else //if(freqHz	> 34375000)
			O = 64;

		uint64_t N = freqHz*O/freqStepHz;
		uint32_t modulus = board::xtalFreqHz/R/freqStepHz;

		if(board::xtalFreqHz > 32000000) {
			modulus /= 2;
			adf4350.refDiv2 = true;
		}
		adf4350.R = R;
		adf4350.O = O;
		adf4350.N = N / modulus;
		adf4350.numerator = N - (adf4350.N * modulus);
		adf4350.denominator = modulus;
		adf4350.sendConfig();
		adf4350.sendN();
	}
}
 
