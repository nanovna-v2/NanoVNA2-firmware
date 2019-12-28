/*
 * This file is derived from libopencm3 example code.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#define PRNT(x)
#define PRNTLN(x)
#include <mculib/fastwiring.hpp>
#include <mculib/softi2c.hpp>
#include <mculib/si5351.hpp>
#include <mculib/dma_adc.hpp>
#include <mculib/usbserial.hpp>
#include <mculib/printf.hpp>
#include <mculib/printk.hpp>

#include <array>
#include <complex>

#include "main.hpp"
#include <board.hpp>
#include "ili9341.hpp"
#include "plot.hpp"
#include "uihw.hpp"
#include "ui.hpp"
#include "common.hpp"
#include "globals.hpp"
#include "synthesizers.hpp"
#include "vna_measurement.hpp"
#include "fifo.hpp"
#include "flash.hpp"
#include "calibration.hpp"
#include "fft.hpp"

#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>

using namespace mculib;
using namespace std;
using namespace board;

// see https://lists.debian.org/debian-gcc/2003/07/msg00057.html
// this can be any value since we are not using shared libraries.
void* __dso_handle = (void*) &__dso_handle;

bool outputRawSamples = false;
int cpu_mhz = 24;


USBSerial serial;

static const int adcBufSize=1024;	// must be power of 2
volatile uint16_t adcBuffer[adcBufSize];

VNAMeasurement vnaMeasurement;

uint8_t usbRxQueue[128];
constexpr int usbRxQueueMask = sizeof(usbRxQueue) - 1;
volatile int usbRxQueueWPos = 0;
volatile int usbRxQueueRPos = 0;


struct usbDataPoint {
	VNAObservation value;
	int freqIndex;
};
usbDataPoint usbTxQueue[64];
constexpr int usbTxQueueMask = 63;
volatile int usbTxQueueWPos = 0;
volatile int usbTxQueueRPos = 0;

// periods of a 1MHz clock; how often to call adc_process()
static constexpr int tim1Period = 25;	// 1MHz / 25 = 40kHz

// periods of a 1MHz clock; how often to call UIHW::checkButtons
static constexpr int tim2Period = 250;	// 1MHz / 250 = 4kHz


// value is in microseconds; increments at 40kHz by TIM1 interrupt
volatile uint32_t systemTimeCounter = 0;

FIFO<small_function<void()>, 8> eventQueue;

volatile bool usbDataMode = false;

volatile int collectMeasurementType = -1;
int collectMeasurementOffset = -1;
int collectMeasurementState = 0;
small_function<void()> collectMeasurementCB;

void adc_process();

template<unsigned int N>
static inline void pinMode(const array<Pad, N>& p, int mode) {
	for(int i=0; i<N; i++)
		pinMode(p[i], mode);
}

void errorBlink(int cnt) {
	digitalWrite(led, HIGH);
	while (1) {
		for(int i=0;i<cnt;i++) {
			digitalWrite(led, HIGH);
			delay(200);
			digitalWrite(led, LOW);
			delay(200);
		}
		delay(1000);
	}
}

// period is in units of us
void startTimer(uint32_t timerDevice, int period) {
	// set the timer to count one tick per us
	timer_set_mode(timerDevice, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(timerDevice, cpu_mhz-1);
	timer_set_repetition_counter(timerDevice, 0);
	timer_continuous_mode(timerDevice);
	
	// this doesn't really set the period, but the "autoreload value"; actual period is this plus 1.
	// this should be fixed in libopencm3.
	
	timer_set_period(timerDevice, period - 1);

	timer_enable_preload(timerDevice);
	timer_enable_preload_complementry_enable_bits(timerDevice);
	timer_enable_break_main_output(timerDevice);
	
	timer_enable_irq(timerDevice, TIM_DIER_UIE);
	
	TIM_EGR(timerDevice) = TIM_EGR_UG;
	timer_set_counter(timerDevice, 0);
	timer_enable_counter(timerDevice);
}
void ui_timer_setup() {
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_reset_pulse(RST_TIM2);
	nvic_set_priority(NVIC_TIM2_IRQ, 0x80);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	startTimer(TIM2, tim2Period);
}


void dsp_timer_setup() {
	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_reset_pulse(RST_TIM1);
	// set tim1 to highest priority
	nvic_set_priority(NVIC_TIM1_UP_IRQ, 0x00);
	nvic_enable_irq(NVIC_TIM1_UP_IRQ);
	startTimer(TIM1, tim1Period);
}

extern "C" void tim1_up_isr() {
	TIM1_SR = 0;
	systemTimeCounter += tim1Period;
	adc_process();
}
extern "C" void tim2_isr() {
	TIM2_SR = 0;
	UIHW::checkButtons();
}

int si5351_doUpdate(uint32_t freqHz) {
	// si5351 code seems to give high frequency errors when frequency
	// isn't a multiple of 10Hz. TODO: investigate
	freqHz = (freqHz/10) * 10;
	return synthesizers::si5351_set(freqHz+lo_freq, freqHz);
}

int si5351_update(uint32_t freqHz) {
	static uint32_t prevFreq = 0;
	int ret = si5351_doUpdate(freqHz);
	if(freqHz < prevFreq)
		si5351_doUpdate(freqHz);
	prevFreq = freqHz;
	return ret;
}



void adf4350_setup() {
	adf4350_rx.N = 120;
	adf4350_rx.rfPower = 0b00;
	adf4350_rx.sendConfig();
	adf4350_rx.sendN();

	adf4350_tx.N = 120;
	adf4350_tx.rfPower = 0b11;
	adf4350_tx.sendConfig();
	adf4350_tx.sendN();
}
void adf4350_update(uint32_t freq_khz) {
	freq_khz = uint32_t(freq_khz/adf4350_freqStep) * adf4350_freqStep;
	synthesizers::adf4350_set(adf4350_tx, freq_khz);
	synthesizers::adf4350_set(adf4350_rx, freq_khz + lo_freq/1000);
}


// set the measurement frequency including setting the tx and rx synthesizers
void setFrequency(freqHz_t freqHz) {
	if(freqHz > 2500000000)
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(2));
	else if(freqHz > 140000000)
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(1));
	else
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(0));

	// use adf4350 for f > 140MHz
	if(freqHz > 140000000) {
		adf4350_update(freqHz/1000);
		rfsw(RFSW_TXSYNTH, RFSW_TXSYNTH_HF);
		rfsw(RFSW_RXSYNTH, RFSW_RXSYNTH_HF);
		vnaMeasurement.nWaitSynth = 10;
	} else {
		int ret = si5351_update(freqHz);
		rfsw(RFSW_TXSYNTH, RFSW_TXSYNTH_LF);
		rfsw(RFSW_RXSYNTH, RFSW_RXSYNTH_LF);
		if(ret == 0)
			vnaMeasurement.nWaitSynth = 18;
		if(ret == 1)
			vnaMeasurement.nWaitSynth = 60;
		if(ret == 2)
			vnaMeasurement.nWaitSynth = 60;
	}
}

void adc_setup() {
	static uint8_t channel_array[16] = {adc_rxChannel};
	dmaADC.buffer = adcBuffer;
	dmaADC.bufferSizeBytes = sizeof(adcBuffer);
	dmaADC.init(channel_array, 1);

	adc_set_sample_time_on_all_channels(dmaADC.adcDevice, adc_ratecfg);
	dmaADC.start();
}

// read and consume data from the adc ring buffer
void adc_read(volatile uint16_t*& data, int& len) {
	static uint32_t lastIndex = 0;
	uint32_t cIndex = dmaADC.position();
	uint32_t bufWords = dmaADC.bufferSizeBytes / 2;
	cIndex &= (bufWords-1);
	
	data = ((volatile uint16_t*) dmaADC.buffer) + lastIndex;
	if(cIndex >= lastIndex) {
		len = cIndex - lastIndex;
	} else {
		len = bufWords - lastIndex;
	}
	lastIndex += len;
	if(lastIndex >= bufWords) lastIndex = 0;
}


void lcd_setup() {
	lcd_spi_init();

	ili9341_conf_cs = ili9341_cs;
	ili9341_conf_dc = ili9341_dc;
	ili9341_spi_transfer = [](uint32_t sdi, int bits) {
		return lcd_spi_transfer(sdi, bits);
	};
	ili9341_spi_transfer_bulk = [](uint32_t words) {
		int bytes = words*2;
		lcd_spi_transfer_bulk((uint8_t*)ili9341_spi_buffer, words*2);
	};
	ili9341_spi_wait_bulk = []() {
		lcd_spi_waitDMA();
	};

	xpt2046.spiTransfer = [](uint32_t sdi, int bits) {
		lcd_spi_waitDMA();
		digitalWrite(ili9341_cs, HIGH);
		
		lcd_spi_slow();
		delayMicroseconds(10);
		uint32_t ret = lcd_spi_transfer(sdi, bits);
		delayMicroseconds(10);
		lcd_spi_fast();
		return ret;
	};
	delay(10);

	xpt2046.begin(320, 240);
	
	ili9341_init();

	// show test pattern
	//ili9341_test(5);
	// clear screen
	ili9341_fill(0, 0, 320, 240, 0);

	plot_init();
	ui_init();
	plot_tick = []() {
		UIActions::application_doEvents();
	};
	redraw_request |= 0xff;
}

void enterUSBDataMode() {
	usbDataMode = true;
	vnaMeasurement.nPeriods = 4;
}


complexf ecalApplyReflection(complexf refl, int freqIndex) {
	#ifdef ECAL_PARTIAL
		return refl - measuredEcal[0][freqIndex];
	#else
		return SOL_compute_reflection(
					measuredEcal[1][freqIndex],
					1.f,
					measuredEcal[0][freqIndex],
					refl);
	#endif
}


complexf applyFixedCorrections(complexf refl, freqHz_t freq) {
	// These corrections do not affect calibrated measurements
	// and is only there to fix uglyness when uncalibrated and
	// without full ecal.

	// magnitude correction:
	// - Near DC the balun is ineffective and measured refl is
	//   0 for short circuit, 0.5 for load, and 1.0 for open circuit,
	//   requiring a correction of (refl*2 - 1.0).
	// - Above 5MHz no correction is needed.
	// - Between DC and 5MHz we apply something in between, with
	//   interpolation factor defined by a polynomial that is
	//   experimentally determined.

	if(freq < 5000000) {
		float x = float(freq) * 1e-6 * (3./5.);
		x = 1 - x*(0.7 - x*(0.141 - x*0.006));
		refl = refl * (1.f + x) - x;
	}

	// phase correction; experimentally determined polynomial
	// x: frequency in MHz
	// arg = -0.25 * x * (-1.39 + x*(0.35 - 0.022*x));

	if(freq < 7500000) {
		float x = float(freq) * 1e-6;
		float im = -0.8f * x*(0.45f + x*(-0.12f + x*0.008f));
		float re = 1.f;
		refl *= complexf(re, im);
	}
	return refl;
}


/*
-- command types:
-- bytes (hex):
--  0  1  2  3  4  5
-- 00                   : nop
-- 0d                   : return ascii '2' (0x32)
-- 10 AA                : read register (address in AA)
-- 11 AA                : read 2-byte register (address in AA)
-- 12 AA                : read 4-byte register (address in AA)
-- 14 AA NN             : read up to N values from FIFO (bytes per value is implementation defined)
-- 20 AA XX             : write register (address in AA, value in XX)
-- 21 AA XX XX          : write 2-byte register (address in AA, values in XX)
-- 22 AA XX XX XX XX    : write 4-byte register (address in AA, values in XX)
-- 23 AA XX XX XX XX    : write 8-byte register (address in AA, values in XX)

-- register map:
-- 00: sweepStartHz[7..0]
-- 01: sweepStartHz[15..8]
-- 02: sweepStartHz[23..16]
-- 03: sweepStartHz[31..24]
-- 04: sweepStartHz[39..32]
-- 05: sweepStartHz[47..40]
-- 06: sweepStartHz[55..48]
-- 07: sweepStartHz[63..56]
-- 10: sweepStepHz[7..0]
-- 11: sweepStepHz[15..8]
-- 12: sweepStepHz[23..16]
-- 13: sweepStepHz[31..24]
-- 14: sweepStepHz[39..32]
-- 15: sweepStepHz[47..40]
-- 16: sweepStepHz[55..48]
-- 17: sweepStepHz[63..56]
-- 20: sweepPoints[7..0]
-- 21: sweepPoints[15..8]
-- 22: valuesPerFrequency[7..0]
-- 23: valuesPerFrequency[15..8]
-- 30: valuesFIFO - returns data points; elements are 32-byte. See below for data format.
--                  command 0x14 reads FIFO data; writing any value clears FIFO.
-- f0: device variant (01)
-- f1: protocol version (01)
-- f2: hardware revision

-- register descriptions:
-- sweepStartHz - Sweep start frequency in Hz.
-- sweepStepHz - Sweep step frequency in Hz.
-- sweepPoints - Number of points in sweep.
-- valuesFIFO - Only command 0x13 supported; returns VNA data.

-- valuesFIFO element data format:
-- bytes:
-- 00: fwd0Re[7..0]
-- 01: fwd0Re[15..8]
-- 02: fwd0Re[23..16]
-- 03: fwd0Re[31..24]
-- 04: fwd0Im[7..0]
-- 05: fwd0Im[15..8]
-- 06: fwd0Im[23..16]
-- 07: fwd0Im[31..24]

-- 08: rev0Re[7..0]
-- 09: rev0Re[15..8]
-- 0a: rev0Re[23..16]
-- 0b: rev0Re[31..24]
-- 0c: rev0Im[7..0]
-- 0d: rev0Im[15..8]
-- 0e: rev0Im[23..16]
-- 0f: rev0Im[31..24]

-- 10: rev1Re[7..0]
-- 11: rev1Re[15..8]
-- 12: rev1Re[23..16]
-- 13: rev1Re[31..24]
-- 14: rev1Im[7..0]
-- 15: rev1Im[15..8]
-- 16: rev1Im[23..16]
-- 17: rev1Im[31..24]

-- 18: freqIndex[7..0]
-- 19: freqIndex[15..8]
-- 1a - 1f: reserved
*/

int cmdPhase = 0;
uint8_t cmdOpcode = 0;
uint8_t cmdAddress = 0xff;
uint8_t cmdEndAddress = 0xff;
uint8_t cmdStartAddress = 0;

void cmdReadFIFO(int address, int nValues);
void cmdRegisterWrite(int address);

void serialCharHandler(uint8_t* s, int len) {
	uint32_t wrRPos = usbRxQueueRPos;
	uint32_t wrWPos = usbRxQueueWPos;
	__sync_synchronize();
	uint32_t spaceLeft = (wrRPos - wrWPos - 1) & usbRxQueueMask;
	if(len > spaceLeft) len = spaceLeft;
	uint32_t target = (wrWPos + len) & usbRxQueueMask;

	for(uint32_t i = wrWPos; i != target; i = ((i+1) & usbRxQueueMask)) {
		usbRxQueue[i] = *s;
		s++;
	}
	__sync_synchronize();
	usbRxQueueWPos = target;
}
void cmdHandleInput(uint8_t* s, int len) {
	uint8_t* end = s + len;
	while(s < end) {
		uint8_t c = *s;
		if(cmdPhase == 0) {
			cmdOpcode = c;
			if(cmdOpcode == 0)
				goto cont;
			if(cmdOpcode == 0x0d) {
				serial.print("2", 1);
				goto cont;
			}
			cmdPhase++;
			goto cont;
		}
		if(cmdPhase == 1) {
			cmdStartAddress = cmdAddress = c;
			if(cmdOpcode == 0x21)
				cmdEndAddress = cmdAddress + 2;
			if(cmdOpcode == 0x22)
				cmdEndAddress = cmdAddress + 4;
			if(cmdOpcode == 0x23)
				cmdEndAddress = cmdAddress + 8;
			cmdPhase++;
			goto cont;
		}
		switch(cmdOpcode) {
			case 0x10:
				serial.print((char*) registers + cmdAddress, 1);
				cmdPhase = 0;
				break;
			case 0x11:
				serial.print((char*) registers + cmdAddress, 2);
				cmdPhase = 0;
				break;
			case 0x12:
				serial.print((char*) registers + cmdAddress, 4);
				cmdPhase = 0;
				break;
			case 0x13:
				cmdReadFIFO(cmdAddress, c);
				cmdPhase = 0;
				break;
			case 0x20:
				registers[cmdAddress & registersSizeMask] = c;
				cmdPhase = 0;
				cmdRegisterWrite(cmdAddress);
				break;
			case 0x21:
			case 0x22:
			case 0x23:
				registers[cmdAddress & registersSizeMask] = c;
				cmdAddress++;
				if(cmdAddress == cmdEndAddress) {
					cmdPhase = 0;
					cmdRegisterWrite(cmdStartAddress);
				}
				break;
			default:
				cmdPhase = 0;
				break;
		}
	cont:
		s++;
	}
}
//1425tX^^^^^^^^^^^^^^XXXXXXXXXXXXXXXXXXXXXXMMMMMM%Vc222$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$44443 \uuuuuuuuuuuuiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiyhz<ggggggggggggggggggggggggggggggggggg


bool cmdProcessInput() {
	uint32_t rdRPos = usbRxQueueRPos;
	uint32_t rdWPos = usbRxQueueWPos;
	__sync_synchronize();
	if(rdRPos == rdWPos)
		return false;
	if(rdWPos > rdRPos) {
		cmdHandleInput(usbRxQueue + rdRPos, rdWPos - rdRPos);
	} else {
		cmdHandleInput(usbRxQueue + rdRPos, sizeof(usbRxQueue) - rdRPos);
		cmdHandleInput(usbRxQueue, rdWPos);
	}
	usbRxQueueRPos = rdWPos;
	return true;
}
void cmdReadFIFO(int address, int nValues) {
	if(address != 0x30) return;
	if(!usbDataMode)
		enterUSBDataMode();

	for(int i=0; i<nValues;) {
		int rdRPos = usbTxQueueRPos;
		int rdWPos = usbTxQueueWPos;
		__sync_synchronize();

		if(rdRPos == rdWPos) { // queue empty
			continue;
		}

		usbDataPoint& usbDP = usbTxQueue[rdRPos];
		VNAObservation& value = usbDP.value;
		if(usbDP.freqIndex < 0 || usbDP.freqIndex > USB_POINTS_MAX)
			continue;

		value[0] = ecalApplyReflection(value[0] / value[1], usbDP.freqIndex) * value[1];

		int32_t fwdRe = value[1].real();
		int32_t fwdIm = value[1].imag();
		int32_t reflRe = value[0].real();
		int32_t reflIm = value[0].imag();
		int32_t thruRe = value[2].real();
		int32_t thruIm = value[2].imag();

		uint8_t txbuf[32];
		txbuf[0] = uint8_t(fwdRe >> 0);
		txbuf[1] = uint8_t(fwdRe >> 8);
		txbuf[2] = uint8_t(fwdRe >> 16);
		txbuf[3] = uint8_t(fwdRe >> 24);

		txbuf[4] = uint8_t(fwdIm >> 0);
		txbuf[5] = uint8_t(fwdIm >> 8);
		txbuf[6] = uint8_t(fwdIm >> 16);
		txbuf[7] = uint8_t(fwdIm >> 24);

		txbuf[8] = uint8_t(reflRe >> 0);
		txbuf[9] = uint8_t(reflRe >> 8);
		txbuf[10] = uint8_t(reflRe >> 16);
		txbuf[11] = uint8_t(reflRe >> 24);

		txbuf[12] = uint8_t(reflIm >> 0);
		txbuf[13] = uint8_t(reflIm >> 8);
		txbuf[14] = uint8_t(reflIm >> 16);
		txbuf[15] = uint8_t(reflIm >> 24);

		txbuf[16] = uint8_t(thruRe >> 0);
		txbuf[17] = uint8_t(thruRe >> 8);
		txbuf[18] = uint8_t(thruRe >> 16);
		txbuf[19] = uint8_t(thruRe >> 24);

		txbuf[20] = uint8_t(thruIm >> 0);
		txbuf[21] = uint8_t(thruIm >> 8);
		txbuf[22] = uint8_t(thruIm >> 16);
		txbuf[23] = uint8_t(thruIm >> 24);

		txbuf[24] = uint8_t(usbDP.freqIndex >> 0);
		txbuf[25] = uint8_t(usbDP.freqIndex >> 8);

		txbuf[26] = 0;
		txbuf[27] = 0;
		txbuf[28] = 0;
		txbuf[29] = 0;
		txbuf[30] = 0;
		txbuf[31] = 0;

		uint8_t checksum=0b01000110;
		for(int i=0; i<31; i++)
			checksum = (checksum xor ((checksum<<1) | 1)) xor txbuf[i];
		txbuf[31] = checksum;

		serial.print((char*)txbuf, sizeof(txbuf));
		__sync_synchronize();
		usbTxQueueRPos = (rdRPos + 1) & usbTxQueueMask;
		i++;
	}
}

void cmdRegisterWrite(int address) {
	if(!usbDataMode)
		enterUSBDataMode();
	if(address == 0x00) {
		vnaMeasurement.sweepStartHz = (freqHz_t)*(uint64_t*)(registers + 0x00);
		vnaMeasurement.resetSweep();
	}
	if(address == 0x10) {
		vnaMeasurement.sweepStepHz = (freqHz_t)*(uint64_t*)(registers + 0x10);
		vnaMeasurement.resetSweep();
	}
	if(address == 0x20) {
		int points = *(uint16_t*)(registers + 0x20);
		if(points > USB_POINTS_MAX)
			points = USB_POINTS_MAX;
		vnaMeasurement.sweepPoints = points;
		vnaMeasurement.resetSweep();
	}
	if(address == 0x22) {
		int values = *(uint16_t*)(registers + 0x22);
		vnaMeasurement.sweepDataPointsPerFreq = values;
		vnaMeasurement.resetSweep();
	}
	if(address == 0x00 || address == 0x10 || address == 0x20) {
		ecalState = ECAL_STATE_MEASURING;
		vnaMeasurement.ecalIntervalPoints = 1;
		vnaMeasurement.nPeriods = MEASUREMENT_NPERIODS_CALIBRATING;
	}
	if(address == 0x30) {
		usbTxQueueRPos = usbTxQueueWPos;
	}
}



// callback called by VNAMeasurement to change rf switch positions.
void measurementPhaseChanged(VNAMeasurementPhases ph) {
	switch(ph) {
		case VNAMeasurementPhases::REFERENCE:
			rfsw(RFSW_REFL, RFSW_REFL_ON);
			rfsw(RFSW_RECV, RFSW_RECV_REFL);
			rfsw(RFSW_ECAL, RFSW_ECAL_OPEN);
			break;
		case VNAMeasurementPhases::REFL:
			rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);
			break;
		case VNAMeasurementPhases::THRU:
			rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);
			rfsw(RFSW_REFL, RFSW_REFL_OFF);
			rfsw(RFSW_RECV, RFSW_RECV_PORT2);
			break;
		case VNAMeasurementPhases::ECALTHRU:
			rfsw(RFSW_ECAL, RFSW_ECAL_LOAD);
			rfsw(RFSW_RECV, RFSW_RECV_REFL);
			break;
		case VNAMeasurementPhases::ECALLOAD:
			rfsw(RFSW_REFL, RFSW_REFL_ON);
			rfsw(RFSW_RECV, RFSW_RECV_REFL);
			rfsw(RFSW_ECAL, RFSW_ECAL_LOAD);
			break;
		case VNAMeasurementPhases::ECALSHORT:
			rfsw(RFSW_ECAL, RFSW_ECAL_SHORT);
			break;
	}
}

// callback called by VNAMeasurement when an observation is available.
static void measurementEmitDataPoint(int freqIndex, freqHz_t freqHz, VNAObservation v, const complexf* ecal) {
	digitalWrite(led, vnaMeasurement.clipFlag?1:0);
	// reference channel is weaker than thru channel, so apply a correction to get
	// an average of 0dB thru magnitude.
	v[2] *= 0.3f;
	v[0] = applyFixedCorrections(v[0]/v[1], freqHz) * v[1];

	if(ecal != nullptr) {
		complexf scale = complexf(1., 0.)/v[1];
		auto ecal0 = applyFixedCorrections(ecal[0] * scale, freqHz);

		if(collectMeasurementType >= 0) {
			// we are collecting a measurement for calibration
			measuredEcal[0][freqIndex] = ecal[0] * scale;
#ifndef ECAL_PARTIAL
			measuredEcal[1][freqIndex] = ecal[1] * scale;
			measuredEcal[2][freqIndex] = ecal[2] * scale;
#endif

			current_props._cal_data[collectMeasurementType][freqIndex] = ecalApplyReflection(v[0]/v[1], freqIndex);

			auto tmp = v[2]/v[1];
			if(collectMeasurementType == CAL_OPEN)
				current_props._cal_data[CAL_ISOLN_OPEN][freqIndex] = tmp;
			else if(collectMeasurementType == CAL_SHORT)
				current_props._cal_data[CAL_ISOLN_SHORT][freqIndex] = tmp;
			else if(collectMeasurementType == CAL_THRU)
				current_props._cal_data[CAL_THRU][freqIndex] = tmp;

			if(collectMeasurementState == 0) {
				collectMeasurementState = 1;
				collectMeasurementOffset = freqIndex;
			} else if(collectMeasurementState == 1 && collectMeasurementOffset == freqIndex) {
				collectMeasurementState = 2;
				collectMeasurementOffset += 2;
				if(collectMeasurementOffset >= vnaMeasurement.sweepPoints)
					collectMeasurementOffset -= vnaMeasurement.sweepPoints;
			} else if(collectMeasurementState == 2 && collectMeasurementOffset == freqIndex) {
				collectMeasurementState = 0;
				collectMeasurementType = -1;
				eventQueue.enqueue(collectMeasurementCB);
			}
		} else {
			if(ecalState == ECAL_STATE_DONE) {
				scale *= 0.2f;
				measuredEcal[0][freqIndex] = measuredEcal[0][freqIndex] * 0.8f + ecal0 * 0.2f;
				#ifndef ECAL_PARTIAL
					measuredEcal[1][freqIndex] = measuredEcal[1][freqIndex] * 0.8f + ecal[1] * scale;
					measuredEcal[2][freqIndex] = measuredEcal[2][freqIndex] * 0.8f + ecal[2] * scale;
				#endif
			} else {
				measuredEcal[0][freqIndex] = ecal0;
				#ifndef ECAL_PARTIAL
					measuredEcal[1][freqIndex] = ecal[1] * scale;
					measuredEcal[2][freqIndex] = ecal[2] * scale;
				#endif
			}
			if(ecalState == ECAL_STATE_MEASURING
					&& freqIndex == vnaMeasurement.sweepPoints - 1) {
				ecalState = ECAL_STATE_2NDSWEEP;
			} else if(ecalState == ECAL_STATE_2NDSWEEP) {
				ecalState = ECAL_STATE_DONE;
				vnaMeasurement.ecalIntervalPoints = MEASUREMENT_ECAL_INTERVAL;
				vnaMeasurement.nPeriods = MEASUREMENT_NPERIODS_NORMAL;
			}
		}
	}
	// enqueue new data point
	int wrRPos = usbTxQueueRPos;
	int wrWPos = usbTxQueueWPos;
	__sync_synchronize();
	if(((wrWPos + 1) & usbTxQueueMask) == wrRPos) {
		// overflow
	} else {
		usbTxQueue[wrWPos].freqIndex = freqIndex;
		usbTxQueue[wrWPos].value = v;
		__sync_synchronize();
		usbTxQueueWPos = (wrWPos + 1) & usbTxQueueMask;
	}
}


void updateSweepParams() {
	freqHz_t start = current_props._frequency0;
	freqHz_t step = (current_props._frequency1 - current_props._frequency0) / (current_props._sweep_points - 1);
	ecalState = ECAL_STATE_MEASURING;
	vnaMeasurement.ecalIntervalPoints = 1;
	vnaMeasurement.nPeriods = MEASUREMENT_NPERIODS_CALIBRATING;
	vnaMeasurement.setSweep(start, step, current_props._sweep_points, 1);
	ecalState = ECAL_STATE_MEASURING;
}

void measurement_setup() {
	vnaMeasurement.phaseChanged = [](VNAMeasurementPhases ph) {
		measurementPhaseChanged(ph);
	};
	vnaMeasurement.emitDataPoint = [](int freqIndex, freqHz_t freqHz, const VNAObservation& v, const complexf* ecal) {
		measurementEmitDataPoint(freqIndex, freqHz, v, ecal);
	};
	vnaMeasurement.frequencyChanged = [](freqHz_t freqHz) {
		setFrequency(freqHz);
	};
	vnaMeasurement.nPeriods = MEASUREMENT_NPERIODS_NORMAL;
	vnaMeasurement.init();
	updateSweepParams();
}

void adc_process() {
	if(!outputRawSamples) {
		volatile uint16_t* buf;
		int len;
		for(int i=0; i<2; i++) {
			adc_read(buf, len);
			vnaMeasurement.processSamples((uint16_t*)buf, len);
		}
	}
}

static int cnt = 0;
void usb_transmit_rawSamples() {
	volatile uint16_t* buf;
	int len;
	adc_read(buf, len);
	int8_t tmpBuf[adcBufSize];
	for(int i=0; i<len; i++)
		tmpBuf[i] = int8_t(buf[i] >> 4) - 128;
	serial.print((char*)tmpBuf, len);
	
	cnt += len;

	rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);
	rfsw(RFSW_RECV, ((cnt / 500) % 2) ? RFSW_RECV_REFL : RFSW_RECV_PORT2);
	//rfsw(RFSW_REFL, ((cnt / 500) % 2) ? RFSW_REFL_ON : RFSW_REFL_OFF);
	//rfsw(RFSW_RECV, RFSW_RECV_REFL);
	rfsw(RFSW_REFL, RFSW_REFL_ON);
}

float bessel0(float x) {
	const float eps = 0.0001;

	float ret = 0;
	float term = 1;
	float m = 0;

	while (term  > eps * ret) {
		ret += term;
		++m;
		term *= (x*x) / (4*m*m);
	}

	return ret;
}

float kaiser_window(float k, float n, float beta) {
	if (beta == 0.0) return 1.0;
	float r = (2 * k) / (n - 1) - 1;
	return bessel0(beta * sqrt(1 - r * r)) / bessel0(beta);
}

void transform_domain() {
	if ((domain_mode & DOMAIN_MODE) != DOMAIN_TIME) return; // nothing to do for freq domain
	// use spi_buffer as temporary buffer
	// and calculate ifft for time domain
	float* tmp = (float*)ili9341_spi_buffer;

	int points = current_props._sweep_points;
	uint8_t window_size = current_props._sweep_points, offset = 0;
	bool is_lowpass = false;
	switch (domain_mode & TD_FUNC) {
		case TD_FUNC_BANDPASS:
			offset = 0;
			window_size = points;
			break;
		case TD_FUNC_LOWPASS_IMPULSE:
		case TD_FUNC_LOWPASS_STEP:
			is_lowpass = true;
			offset = points;
			window_size = points * 2;
			break;
	}

	float beta = 0.0;
	switch (domain_mode & TD_WINDOW) {
		case TD_WINDOW_MINIMUM:
			beta = 0.0; // this is rectangular
			break;
		case TD_WINDOW_NORMAL:
			beta = 6.0;
			break;
		case TD_WINDOW_MAXIMUM:
			beta = 13;
			break;
	}


	for (int ch = 0; ch < 2; ch++) {
		memcpy(tmp, measuredFreqDomain[ch], sizeof(measuredFreqDomain[0]));
		for (int i = 0; i < points; i++) {
			float w = kaiser_window(i+offset, window_size, beta);
			tmp[i*2+0] *= w;
			tmp[i*2+1] *= w;
		}
		for (int i = points; i < FFT_SIZE; i++) {
			tmp[i*2+0] = 0.0;
			tmp[i*2+1] = 0.0;
		}
		if (is_lowpass) {
			for (int i = 1; i < points; i++) {
				tmp[(FFT_SIZE-i)*2+0] =  tmp[i*2+0];
				tmp[(FFT_SIZE-i)*2+1] = -tmp[i*2+1];
			}
		}

		fft256_inverse((float(*)[2])tmp);
		memcpy(measured[ch], tmp, sizeof(measured[0]));
		for (int i = 0; i < points; i++) {
			measured[ch][i] /= (float)FFT_SIZE;
			if (is_lowpass) {
				measured[ch][i] = {measured[ch][i].real(), 0.f};
			}
		}
		if ( (domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_STEP ) {
			for (int i = 1; i < points; i++) {
				measured[ch][i] += measured[ch][i-1];
			}
		}
	}
}

// consume all items in the values fifo and update the "measured" array.
bool processDataPoint() {
	int ret = -1;
	int rdRPos = usbTxQueueRPos;
	int rdWPos = usbTxQueueWPos;
	__sync_synchronize();

	while(rdRPos != rdWPos) {
		usbDataPoint& usbDP = usbTxQueue[rdRPos];
		VNAObservation& value = usbDP.value;
		int freqIndex = usbDP.freqIndex;
		auto refl = value[0]/value[1];
		auto thru = value[2]/value[1];// - measuredEcal[2][freqIndex]*0.8f;
		
		refl = ecalApplyReflection(refl, freqIndex);
		if(current_props._cal_status & CALSTAT_APPLY) {
			// apply thru leakage correction
			auto x1 = current_props._cal_data[CAL_SHORT][freqIndex],
				y1 = current_props._cal_data[CAL_ISOLN_SHORT][freqIndex],
				x2 = current_props._cal_data[CAL_OPEN][freqIndex],
				y2 = current_props._cal_data[CAL_ISOLN_OPEN][freqIndex];
			auto cal_thru_leak_r = (y1-y2)/(x1-x2);
			auto cal_thru_leak = y2-cal_thru_leak_r*x2;
			thru = thru - (cal_thru_leak + refl*cal_thru_leak_r);

			// apply thru response correction
			if(current_props._cal_status & CALSTAT_THRU) {
				auto refThru = current_props._cal_data[CAL_THRU][freqIndex];
				//refThru = refThru - (cal_thru_leak + refl*cal_thru_leak_r);
				// TODO: we can't do proper leakage correction on the reference thru measurement
				// because we didn't store the S11 of the thru calibration. In V2 hardware
				// this causes ~0.05dB of S21 error but it will be a problem if attempting
				// to use this code with low-spec hardware with lots of leakage.
				refThru = refThru - (cal_thru_leak + refl*cal_thru_leak_r);
				thru = thru / refThru;
			}

			// apply reflection correction
			refl = SOL_compute_reflection(
						current_props._cal_data[CAL_SHORT][freqIndex],
						current_props._cal_data[CAL_OPEN][freqIndex],
						current_props._cal_data[CAL_LOAD][freqIndex],
						refl);
		}
		measuredFreqDomain[0][usbDP.freqIndex] = refl;
		measuredFreqDomain[1][usbDP.freqIndex] = thru;
		if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
			measured[0][usbDP.freqIndex] = refl;
			measured[1][usbDP.freqIndex] = thru;
		}

		rdRPos = (rdRPos + 1) & usbTxQueueMask;
		usbTxQueueRPos = rdRPos;

		if(freqIndex == vnaMeasurement.sweepPoints - 1) {
			transform_domain();
			return true;
		}
	}
	return false;
}




int main(void) {
	uint32_t fpuEnable = 0b1111 << 20;
	bool shouldShowDmesg = false;

	if((SCB_CPACR & fpuEnable) != fpuEnable) {
		SCB_CPACR |= fpuEnable;
		if((SCB_CPACR & fpuEnable) != fpuEnable) {
			// printk1() does not invoke printf() and does not use fpu

			// if you encounter this error, see:
			// https://www.amobbs.com/thread-5719892-1-1.html
			printk1("FPU NOT DETECTED!\nCHECK GD32F303 BATCH OR REBUILD WITHOUT FPU\n");
		} else {
			printk1("LIBOPENCM3 DID NOT ENABLE FPU!\n CHECK lib/dispatch/vector_chipset.c\n");
		}
#ifndef GD32F3_NOFPU
		shouldShowDmesg = true;
#endif
	}
	int i;
	boardInit();

	// set version registers (accessed through usb serial)
	registers[0xf0 & registersSizeMask] = 1;	// device variant
	registers[0xf1 & registersSizeMask] = 1;	// protocol version
	registers[0xf2 & registersSizeMask] = (uint8_t) BOARD_REVISION;

	// we want all higher priority irqs to preempt lower priority ones
	scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);

	pinMode(led, OUTPUT);
	pinMode(led2, OUTPUT);
	pinMode(RFSW_ECAL, OUTPUT);
	pinMode(RFSW_BBGAIN, OUTPUT);
	pinMode(RFSW_TXSYNTH, OUTPUT);
	pinMode(RFSW_RXSYNTH, OUTPUT);
	pinMode(RFSW_REFL, OUTPUT);
	pinMode(RFSW_RECV, OUTPUT);
	pinMode(USB0_DP, OUTPUT);

	digitalWrite(USB0_DP, LOW);

	digitalWrite(led, HIGH);
	
	rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(0));
	rfsw(RFSW_RXSYNTH, RFSW_RXSYNTH_LF);
	rfsw(RFSW_TXSYNTH, RFSW_TXSYNTH_LF);
	rfsw(RFSW_REFL, RFSW_REFL_ON);
	rfsw(RFSW_RECV, RFSW_RECV_REFL);
	rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);

	delay(500);

	serial.setReceiveCallback([](uint8_t* s, int len) {
		serialCharHandler(s, len);
	});
	// baud rate is ignored for usbserial
	serial.begin(115200);
	pinMode(USB0_DP, INPUT);

	nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 0xf0);


	lcd_setup();
	UIHW::init(tim2Period);
	ui_timer_setup();

	// work around spurious ui events at startup
	delay(50);
	while(eventQueue.readable())
		eventQueue.dequeue();

	// show dmesg and wait for user input if there is an important error
	if(shouldShowDmesg) {
		printk1("Touch anywhere to continue...\n");
		show_dmesg();
	}

	flash_config_recall();
	UIActions::printTouchCal();

	si5351_i2c.init();
	if(!synthesizers::si5351_setup()) {
		printk1("ERROR: si5351 init failed\n");
		printk1("Touch anywhere to continue...\n");
		current_props._frequency0 = 200000000;
		show_dmesg();
	}


	setFrequency(56000000);

	measurement_setup();
	adc_setup();
	dsp_timer_setup();

	adf4350_setup();

	redraw_frame();

	bool testSG = false;
	
	if(testSG) {
		while(1) {
			uint16_t tmp = 1;
			vnaMeasurement.processSamples(&tmp, 1);
		}
		return 0;
	}

	
	bool lastUSBDataMode = false;
	while(true) {
		cmdProcessInput();
		if(usbDataMode) {
			if(outputRawSamples)
				usb_transmit_rawSamples();
			// display "usb mode" screen
			if(!lastUSBDataMode) {
				show_usb_data_mode();
			}
			lastUSBDataMode = usbDataMode;
			continue;
		}
		lastUSBDataMode = usbDataMode;
		if(sweep_enabled) {
			if(processDataPoint()) {
				// a full sweep has completed
				if ((domain_mode & DOMAIN_MODE) == DOMAIN_TIME) {
					plot_into_index(measured);
					draw_all(true);
					continue;
				}
			}
		}

		// if we have no pending events, use idle cycles to refresh the graph
		if(!eventQueue.readable()) {
			if(sweep_enabled) {
				if((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ)
					plot_into_index(measured);
			}
			draw_all(true);
			continue;
		}
		auto callback = eventQueue.read();
		eventQueue.dequeue();
		if(!callback)
			abort();
		callback();
	}
}

extern "C" void abort() {
	while (1) {
		for(int i=0;i<3;i++) {
			digitalWrite(led, HIGH);
			delay(100);
			digitalWrite(led, LOW);
			delay(100);
		}
		delay(1000);
	}
}
/*
extern "C" void *memcpy(void *dest, const void *src, size_t n) {
	for(size_t i=0;i<n;i++)
		((char*)dest)[i] = ((char*)src)[i];
	return dest;
}
extern "C" void *memset(void *s, int c, size_t n) {
	for(size_t i=0;i<n;i++)
		((char*)s)[i] = c;
}
extern "C" size_t strlen(const char* s) {
	int i = 0;
	while(*s != 0) {
		i++;
		s++;
	}
	return i;
}
extern "C" int atoi(const char* s) {
	// TODO: implement
	return 0;
}
extern "C" void __aeabi_atexit(void * arg , void (* func ) (void *)) {
	// Leave this function empty. Program never exits.
}*/

extern "C" uintptr_t __stack_chk_guard = 0xdeadbeef;
extern "C" void __cxa_pure_virtual() {
	errorBlink(4);
	while(1);
}
extern "C" void __stack_chk_fail() {
	errorBlink(5);
	while(1);
}


// nanovna UI callbacks
namespace UIActions {

	void cal_collect(int type) {
		collectMeasurementCB = [type]() {
			vnaMeasurement.ecalIntervalPoints = MEASUREMENT_ECAL_INTERVAL;
			vnaMeasurement.nPeriods = MEASUREMENT_NPERIODS_NORMAL;
			current_props._cal_status |= (1 << type);
			ui_cal_collected();
		};
		__sync_synchronize();
		vnaMeasurement.ecalIntervalPoints = 1;
		vnaMeasurement.nPeriods = MEASUREMENT_NPERIODS_CALIBRATING;
		collectMeasurementType = type;
	}
	void cal_done(void) {
		current_props._cal_status |= CALSTAT_APPLY;
	}

	static inline void clampFrequency(freqHz_t& f) {
		if(f < FREQUENCY_MIN)
			f = FREQUENCY_MIN;
		if(f > FREQUENCY_MAX)
			f = FREQUENCY_MAX;
	}

	void set_sweep_frequency(SweepParameter type, freqHz_t frequency) {
		clampFrequency(frequency);
		switch(type) {
			case ST_START:
				current_props._frequency0 = frequency;
				if(current_props._frequency1 < current_props._frequency0) {
					current_props._frequency1 = current_props._frequency0;
				}
				break;
			case ST_STOP:
				current_props._frequency1 = frequency;
				if(current_props._frequency1 < current_props._frequency0) {
					current_props._frequency0 = current_props._frequency1;
				}
				break;
			case ST_CENTER:
			{
				freqHz_t span = current_props._frequency1 - current_props._frequency0;
				current_props._frequency0 = frequency - span/2;
				current_props._frequency1 = frequency + span/2;
				break;
			}
			case ST_SPAN:
			{
				freqHz_t center = (frequency0 + frequency1)/2;
				current_props._frequency0 = center - frequency/2;
				current_props._frequency1 = center + frequency/2;
				break;
			}
			case ST_CW:
			{
				current_props._frequency0 = frequency;
				current_props._frequency1 = frequency;
				break;
			}
			default: return;
		}
		clampFrequency(current_props._frequency0);
		clampFrequency(current_props._frequency1);
		if(current_props._frequency1 < current_props._frequency0) {
			current_props._frequency1 = current_props._frequency0;
		}
		updateSweepParams();
		current_props._cal_status = 0;
		draw_cal_status();
	}
	void set_sweep_points(int points) {
		if(points < SWEEP_POINTS_MIN)
			points = SWEEP_POINTS_MIN;
		if(points > SWEEP_POINTS_MAX)
			points = SWEEP_POINTS_MAX;
		current_props._sweep_points = points;
		updateSweepParams();
		current_props._cal_status = 0;
		draw_cal_status();
	}
	freqHz_t get_sweep_frequency(int type) {
		switch (type) {
		case ST_START: return frequency0;
		case ST_STOP: return frequency1;
		case ST_CENTER: return (frequency0 + frequency1)/2;
		case ST_SPAN: return frequency1 - frequency0;
		case ST_CW: return (frequency0 + frequency1)/2;
		}
	}
	freqHz_t frequencyAt(int index) {
		return vnaMeasurement.sweepStartHz + vnaMeasurement.sweepStepHz * index;
	}

	void toggle_sweep(void) {
		sweep_enabled = !sweep_enabled;
	}
	void enable_refresh(bool enable) {
		sweep_enabled = enable;
	}



	void set_trace_type(int t, int type) {
		int polar = (type == TRC_SMITH || type == TRC_POLAR);
		int enabled = type != TRC_OFF;
		bool force = false;

		if (trace[t].polar != polar) {
			trace[t].polar = polar;
			force = true;
		}
		if (trace[t].enabled != enabled) {
			trace[t].enabled = enabled;
			force = true;
		}
		if (trace[t].type != type) {
			trace[t].type = type;
			trace[t].refpos = trace_info[type].refpos;
			if (polar)
				force = true;
		}
		if (force) {
			plot_into_index(measured);
			force_set_markmap();
		}
	}
	void set_trace_channel(int t, int channel)
	{
		if (trace[t].channel != channel) {
			trace[t].channel = channel;
			force_set_markmap();
		}
	}

	void set_trace_scale(int t, float scale)
	{
		scale /= trace_info[trace[t].type].scale_unit;
		if (trace[t].scale != scale) {
			trace[t].scale = scale;
			force_set_markmap();
		}
	}


	void set_trace_refpos(int t, float refpos)
	{
		if (trace[t].refpos != refpos) {
			trace[t].refpos = refpos;
			force_set_markmap();
		}
	}

	void set_electrical_delay(float picoseconds)
	{
		if (electrical_delay != picoseconds) {
			electrical_delay = picoseconds;
			force_set_markmap();
		}
	}

	float get_electrical_delay(void)
	{
		return electrical_delay;
	}

	void apply_edelay_at(int i) {
		float w = 2 * M_PI * electrical_delay * frequencyAt(i) * 1E-12;
		float s = sin(w);
		float c = cos(w);
		float real = measured[0][i].real();
		float imag = measured[0][i].imag();
		measured[0][i] = {real * c - imag * s,
							imag * c + real * s};
		real = measured[1][i].real();
		imag = measured[1][i].imag();
		measured[1][i] = {real * c - imag * s,
							imag * c + real * s};
	}
	
	int caldata_save(int id) {
		int ret = flash_caldata_save(id);
		return ret;
	}
	int caldata_recall(int id) {
		int ret = flash_caldata_recall(id);
		if(ret == 0)
			updateSweepParams();
		return ret;
	}

	int config_save() {
		return flash_config_save();
	}
	int config_recall() {
		return flash_config_recall();
	}

	void printTouchCal() {
		printk1("touch cal:\n");
		printk("    %d, %d\n    %d, %d\n",
				(int)config.touch_cal[0], (int)config.touch_cal[1],
				(int)config.touch_cal[2], (int)config.touch_cal[3]);
	}


	void application_doEvents() {
		while(eventQueue.readable()) {
			auto callback = eventQueue.read();
			eventQueue.dequeue();
			if(!callback)
				abort();
			callback();
		}
	}

	void application_doSingleEvent() {
		if(eventQueue.readable()) {
			auto callback = eventQueue.read();
			eventQueue.dequeue();
			if(!callback)
				abort();
			callback();
		}
	}
	void enqueueEvent(const small_function<void()>& cb) {
		eventQueue.enqueue(cb);
	}
}
