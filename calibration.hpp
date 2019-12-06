#pragma once
#include "common.hpp"

// given the measured raw values for short, open, and load, compute the 3 calibration coefficients
inline array<complexf, 3> SOL_compute_coefficients(complexf sc, complexf oc, complexf load) {
	complexf a=load, b=oc, c=sc;
	complexf cal_X, cal_Y, cal_Z;
	cal_Z=(2.f*a-b-c)/(b-c);
	cal_X=a-c*(1.f-cal_Z);
	cal_Y=a/cal_X;

	return {cal_X, cal_Y, cal_Z};
}
// given the calibration coefficients and a raw value, compute the reflection coefficient
inline complexf SOL_compute_reflection(const array<complexf, 3>& coeffs, complexf raw) {
	auto cal_X = coeffs[0];
	auto cal_Y = coeffs[1];
	auto cal_Z = coeffs[2];

	return (cal_X*cal_Y-raw)/(raw*cal_Z-cal_X);
}

// given the measured raw values for S,O,L and a DUT raw value, compute the reflection coefficient
inline complexf SOL_compute_reflection(complexf sc, complexf oc, complexf load, complexf dut) {
	complexf a=load, b=oc, c=sc, d = dut;
	/*complexf cal_X, cal_Y, cal_Z;
	cal_Z=(2.f*a-b-c)/(b-c);
	cal_X=a-c*(1.f-cal_Z);
	cal_Y=a/cal_X;

	return (cal_X*cal_Y-dut)/(dut*cal_Z-cal_X);*/

	/* derived from the above formulas and simplified using sympy:
	from sympy import *
	a = Symbol('a')
	b = Symbol('b')
	c = Symbol('c')
	d = Symbol('d')
	z = (2*a - b - c) / (b-c)
	x = a - c*(1 - z)
	y = a/x
	result = (x*y-d)/(d*z-x)
	simplify(result)
	*/

	return -(a - d)*(b - c)/(a*(b - c) + 2.f*c*(a - b) + d*(-2.f*a + b + c));
}

