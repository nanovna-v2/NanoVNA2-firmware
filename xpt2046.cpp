/*
 * Copyright (c) 2015-2016  Spiros Papadimitriou
 *
 * This file is part of github.com/spapadim/XPT2046 and is released
 * under the MIT License: https://opensource.org/licenses/MIT
 *
 * This software is distributed on an "AS IS" basis,
 * WITHOUT WARRANTY OF ANY KIND, either express or implied.
 */

#include "xpt2046.hpp"

using namespace mculib;

inline static void swap(uint16_t &a, uint16_t &b) {
	uint16_t tmp = a;
	a = b;
	b = tmp;
}

XPT2046::XPT2046 (Pad cs_pin, Pad irq_pin) 
: _cs_pin(cs_pin), _irq_pin(irq_pin) {
}

void XPT2046::begin(uint16_t width, uint16_t height) {
	digitalWrite(_cs_pin, 1);
	pinMode(_cs_pin, OUTPUT);
	pinMode(_irq_pin, INPUT_PULLUP);

	delay(1);

	_width = width;
	_height = height;

	// Default calibration (map 0..ADC_MAX -> 0..width|height)
	setCalibration(
		/*vi1=*/((int32_t)CAL_MARGIN) * ADC_MAX / width,
		/*vj1=*/((int32_t)CAL_MARGIN) * ADC_MAX / height,
		/*vi2=*/((int32_t)width - CAL_MARGIN) * ADC_MAX / width,
		/*vj2=*/((int32_t)height - CAL_MARGIN) * ADC_MAX / height
	);
	// TODO(?) Use the following empirical calibration instead? -- Does it depend on VCC??
	// touch.setCalibration(209, 1759, 1775, 273);

	powerDown();  // Make sure PENIRQ is enabled
}

void XPT2046::getCalibrationPoints(uint16_t &x1, uint16_t &y1, uint16_t &x2, uint16_t &y2) {
	x1 = y1 = CAL_MARGIN;
	x2 = _width - CAL_MARGIN;
	y2 = _height - CAL_MARGIN;
}

void XPT2046::setCalibration (uint16_t vi1, uint16_t vj1, uint16_t vi2, uint16_t vj2) {
	_cal_dx = _width - 2*CAL_MARGIN;
	_cal_dy = _height - 2*CAL_MARGIN;

	_cal_vi1 = (int32_t)vi1;
	_cal_vj1 = (int32_t)vj1;
	_cal_dvi = (int32_t)vi2 - vi1;
	_cal_dvj = (int32_t)vj2 - vj1;
}

uint16_t XPT2046::_readLoop(uint8_t ctrl, uint8_t max_samples) const {
	uint16_t prev = 0xffff, cur = 0xffff;
	uint8_t i = 0;
	do {
		prev = cur;
		cur = spiTransfer(0, 8);
		cur = (cur << 4) | (spiTransfer(ctrl, 8) >> 4);  // 16 clocks -> 12-bits (zero-padded at end)
	} while ((prev != cur) && (++i < max_samples));
//Serial.print("RL i: "); Serial.println(i); Serial.flush();  // DEBUG
	return cur;
}

// TODO: Caveat - MODE_SER is completely untested!!
//   Need to measure current draw and see if it even makes sense to keep it as an option
void XPT2046::getRaw (uint16_t &vi, uint16_t &vj, adc_ref_t mode, uint8_t max_samples) const {
	// Implementation based on TI Technical Note http://www.ti.com/lit/an/sbaa036/sbaa036.pdf

	uint8_t ctrl_lo = ((mode == MODE_DFR) ? CTRL_LO_DFR : CTRL_LO_SER);
	
	digitalWrite(_cs_pin, LOW);
	spiTransfer(CTRL_HI_X | ctrl_lo, 8);  // Send first control byte
	vi = _readLoop(CTRL_HI_X | ctrl_lo, max_samples);
	vj = _readLoop(CTRL_HI_Y | ctrl_lo, max_samples);

	if (mode == MODE_DFR) {
		// Turn off ADC by issuing one more read (throwaway)
		// This needs to be done, because PD=0b11 (needed for MODE_DFR) will disable PENIRQ
		spiTransfer(0, 8);  // Maintain 16-clocks/conversion; _readLoop always ends after issuing a control byte
		spiTransfer(CTRL_HI_Y | CTRL_LO_SER, 8);
	}
	spiTransfer(0, 16);  // Flush last read, just to be sure
	
	digitalWrite(_cs_pin, HIGH);
}

void XPT2046::getPosition (uint16_t &x, uint16_t &y, adc_ref_t mode, uint8_t max_samples) const {
	if (!isTouching()) {
		x = y = 0xffff;
		return;
	}

	uint16_t vi, vj;
	getRaw(vi, vj, mode, max_samples);

	// Map to (un-rotated) display coordinates
#if defined(SWAP_AXES) && SWAP_AXES
	x = (uint16_t)(_cal_dx * (vj - _cal_vj1) / _cal_dvj + CAL_MARGIN);
	y = (uint16_t)(_cal_dy * (vi - _cal_vi1) / _cal_dvi + CAL_MARGIN);
#else
	x = (uint16_t)(_cal_dx * (vi - _cal_vi1) / _cal_dvi + CAL_MARGIN);
	y = (uint16_t)(_cal_dy * (vj - _cal_vj1) / _cal_dvj + CAL_MARGIN);
#endif

	// Transform based on current rotation setting
	// TODO: Is it worth to do this by tweaking _cal_* values instead?
	switch (_rot) {  // TODO double-check
	case ROT90:
		x = _width - x;
		swap(x, y);
		break;
	case ROT180:
		x = _width - x;
		y = _height - y; 
		break;
	case ROT270:
		y = _height - y; 
		swap(x, y);
		break;
	case ROT0:
	default:
		// Do nothing
		break;
	}
}

void XPT2046::powerDown() const {
	digitalWrite(_cs_pin, LOW);
	// Issue a throw-away read, with power-down enabled (PD{1,0} == 0b00)
	// Otherwise, ADC is disabled
	spiTransfer(CTRL_HI_Y | CTRL_LO_SER, 8);
	spiTransfer(0, 16);  // Flush, just to be sure
	digitalWrite(_cs_pin, HIGH);
}
