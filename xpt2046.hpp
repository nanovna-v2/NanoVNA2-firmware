#pragma once

/*
 * Copyright (c) 2015-2016  Spiros Papadimitriou
 *
 * This file is part of github.com/spapadim/XPT2046 and is released
 * under the MIT License: https://opensource.org/licenses/MIT
 *
 * This software is distributed on an "AS IS" basis,
 * WITHOUT WARRANTY OF ANY KIND, either express or implied.
 */

#include <mculib/fastwiring.hpp>
#include <mculib/small_function.hpp>


// On my display driver ICs i,j mapped to (width-y),x
//  Flipping can be handled by order of calibration points, but not swapping
#if !defined(XPT2046_SWAP_AXES)
#  define XPT2046_SWAP_AXES 1
#endif

class XPT2046 {
public:
	static const uint16_t CAL_MARGIN = 20;

	enum rotation_t : uint8_t { ROT0, ROT90, ROT180, ROT270 };
	enum adc_ref_t : uint8_t { MODE_SER, MODE_DFR };


	// hooks
	small_function<uint32_t(uint32_t data, int bits)> spiTransfer;


	XPT2046 (mculib::Pad cs_pin, mculib::Pad irq_pin);

	void begin(uint16_t width, uint16_t height);  // width and height with no rotation!
	void setRotation(rotation_t rot) { _rot = rot; }

	// Calibration needs to be done with no rotation, on both display and touch drivers
	void getCalibrationPoints(uint16_t &x1, uint16_t &y1, uint16_t &x2, uint16_t &y2);
	void setCalibration (uint16_t vi1, uint16_t vj1, uint16_t vi2, uint16_t vj2);

	bool isTouching() const { return (digitalRead(_irq_pin) == mculib::LOW); }

	void getRaw(uint16_t &vi, uint16_t &vj, adc_ref_t mode = MODE_DFR, uint8_t max_samples = 0xff) const;
	void getPosition(uint16_t &x, uint16_t &y, adc_ref_t mode = MODE_DFR, uint8_t max_samples = 0xff) const;

	void powerDown() const;

private:
	static const uint8_t CTRL_LO_DFR = 0b0011;
	static const uint8_t CTRL_LO_SER = 0b0100;
	static const uint8_t CTRL_HI_X = 0b1001  << 4;
	static const uint8_t CTRL_HI_Y = 0b1101  << 4;

	static const uint16_t ADC_MAX = 0x0fff;  // 12 bits

	uint16_t _width, _height;
	rotation_t _rot;
	mculib::Pad _cs_pin, _irq_pin;

	int32_t _cal_dx, _cal_dy, _cal_dvi, _cal_dvj;
	uint16_t _cal_vi1, _cal_vj1;

	uint16_t _readLoop(uint8_t ctrl, uint8_t max_samples) const;
};
