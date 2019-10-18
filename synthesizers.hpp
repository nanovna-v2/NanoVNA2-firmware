#pragma once
#include "common.hpp"
#include <mculib/si5351.hpp>

namespace synthesizers {
	using namespace mculib::Si5351;

	bool si5351_setup(Si5351Driver& dev);
	void si5351_set(Si5351Driver& dev, int i, int pll, uint32_t freq_khz);

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
		else //if(freq_khz	> 137500)
			O = 16;

		uint32_t N = freq_khz*O/adf4350_freqStep;

		adf4350.O = O;
		adf4350.N = N / adf4350_modulus;
		adf4350.numerator = N - (adf4350.N * adf4350_modulus);
		adf4350.denominator = adf4350_modulus;
		adf4350.sendConfig();
		adf4350.sendN();
	}
}
 
