#pragma once
#include "common.hpp"

namespace synthesizers {

	// initialize and configure si5351
	bool si5351_setup();

	// set si5351 frequency for tx and rx port.
	// returns 0 if one output divider changed;
	// returns 1 if two output dividers changed;
	// returns 2 if pll updated.
	int si5351_set(uint32_t rxFreqHz, uint32_t txFreqHz);


	// freq_khz must be a multiple of adf4350_freqStep
	template<class T>
	static void adf4350_set(T& adf4350, uint32_t freq_khz) {
		int O = 1;
		
		if(freq_khz	> 2200000)
			O = 1;
		else if(freq_khz	> 1100000)
			O = 2;
		else if(freq_khz	> 550000)
			O = 4;
		else if(freq_khz	> 275000)
			O = 8;
		else if(freq_khz	> 137500)
			O = 16;
		else if(freq_khz	> 68750)
			O = 32;
		else //if(freq_khz	> 34375)
			O = 64;

		uint32_t N = freq_khz*O/adf4350_freqStep;

		adf4350.R = adf4350_R;
		adf4350.O = O;
		adf4350.N = N / adf4350_modulus;
		adf4350.numerator = N - (adf4350.N * adf4350_modulus);
		adf4350.denominator = adf4350_modulus;
		adf4350.sendConfig();
		adf4350.sendN();
	}
}
 
