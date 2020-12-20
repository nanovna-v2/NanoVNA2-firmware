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

	// Find better approximate values for n/d
	#define MAX_DENOMINATOR ((1 << 20) - 1)
	static void approximate_fraction(uint32_t *n, uint32_t *d)
	{
	  // cf. https://github.com/python/cpython/blob/master/Lib/fractions.py#L227
	  uint32_t denom = *d;
	  if (denom > MAX_DENOMINATOR) {
	    uint32_t num = *n;
	    uint32_t p0 = 0, q0 = 1, p1 = 1, q1 = 0;
	    while (denom != 0) {
	      uint32_t a = num / denom;
	      uint32_t b = num % denom;
	      uint32_t q2 = q0 + a*q1;
	      if (q2 > MAX_DENOMINATOR)
	        break;
	      uint32_t p2 = p0 + a*p1;
	      p0 = p1; q0 = q1; p1 = p2; q1 = q2;
	      num = denom; denom = b;
	    }
	    *n = p1;
	    *d = q1;
	  }
	}

	// freqHz must be a multiple of freqStepHz
	template<class T>
	static void adf4350_set(T& adf4350, freqHz_t freqHz, uint32_t freqStepHz) {
		uint32_t O = 1;
		uint32_t R = 1; // adf4350 reference divide
		
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

		adf4350.R = R;
		adf4350.O = O;
		adf4350.N = N / modulus;
		adf4350.numerator = N - (adf4350.N * modulus);
		adf4350.denominator = modulus;

		adf4350.sendConfig();
		adf4350.sendN();
	}
}
 
