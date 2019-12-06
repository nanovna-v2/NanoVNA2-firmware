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

#include <libopencm3/stm32/timer.h>

using namespace mculib;
using namespace std;
using namespace board;

// see https://lists.debian.org/debian-gcc/2003/07/msg00057.html
// this can be any value since we are not using shared libraries.
void* __dso_handle = (void*) &__dso_handle;

bool outputRawSamples = false;
int cpu_mhz = 24;


USBSerial serial;

static const int adcBufSize=4096;	// must be power of 2
volatile uint16_t adcBuffer[adcBufSize];

VNAMeasurement vnaMeasurement;


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
volatile bool refreshEnabled = true;

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
	//while (1) {
		for(int i=0;i<cnt;i++) {
			digitalWrite(led, HIGH);
			delay(200);
			digitalWrite(led, LOW);
			delay(200);
		}
		delay(1000);
	//}
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
void timers_setup() {
	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_reset_pulse(RST_TIM1);
	rcc_periph_reset_pulse(RST_TIM2);
	
	// set tim1 to highest priority
	nvic_set_priority(NVIC_TIM1_UP_IRQ, 1 * 16);
	nvic_set_priority(NVIC_TIM2_IRQ, 3 * 16);

	nvic_enable_irq(NVIC_TIM1_UP_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	startTimer(TIM1, tim1Period);
	startTimer(TIM2, tim2Period);
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
	ili9341_test(5);
	// clear screen
	ili9341_fill(0, 0, 320, 240, 0);

	plot_init();
	ui_init();
	plot_tick = []() {
		UIActions::application_doEvents();
	};
	plot_into_index(measured);
	redraw_request |= 0xff;
	draw_all(true);
}

void enterUSBDataMode() {
	usbDataMode = true;
	vnaMeasurement.nPeriods = 4;
}


/*
-- register map:
-- 00: pll_frequency [23..16]
-- 01: pll_frequency [15..8]
-- 02: pll_frequency [7..0]
-- 03: update_trigger; write 1 to update all plls
-- 04: attenuation, in 0.5dB increments
-- 05: 
--   [1..0]: signal generator output power
--   [3..2]: LO output power
--   [7..4]: output data mode:
--				0: normal
--				1: adc data
-- 06: auto-sweep params
--   06: 
-- note: the output signal frequency is pll_frequency * 10kHz*/

void serialCharHandler(uint8_t* s, int len) {
	static uint8_t writingRegister = 255;
	for(int i=0;i<len;i++) {
		uint8_t ch=s[i];
		if(writingRegister == 255) // set write address
			writingRegister = (ch == 0 ? 255 : (ch - 1));
		else { // write reg
			if(writingRegister < sizeof(registers)) {
				registers[writingRegister] = ch;
			}
			if(writingRegister == 3) {
				// update pll
				uint32_t freq = (uint32_t(registers[0]) << 16)
						| (uint32_t(registers[1]) << 8)
						| (uint32_t(registers[2]) << 0);
				
				//setFrequency(freq*10);
				vnaMeasurement.setSweep(freq*10000, 0, 1);
				if(!usbDataMode)
					enterUSBDataMode();
			}
			if(writingRegister == 5) {
				uint8_t outpMode = ch >> 4;
				if(outpMode > 0)
					outputRawSamples = true;
				else
					outputRawSamples = false;
			}
			writingRegister = 255;
		}
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
			//rfsw(RFSW_ECAL, RFSW_ECAL_LOAD);
			break;
		case VNAMeasurementPhases::ECALSHORT:
			rfsw(RFSW_ECAL, RFSW_ECAL_SHORT);
			break;
	}
}

// callback called by VNAMeasurement when an observation is available.
static void measurementEmitDataPoint(int freqIndex, uint64_t freqHz, const VNAObservation& v, const complexf* ecal) {
	if(ecal != nullptr) {
		complexf scale = complexf(1., 0.)/v[1];

		if(collectMeasurementType >= 0) {
			// we are collecting a measurement for calibration
			measuredEcal[0][freqIndex] = ecal[0] * scale;
			measuredEcal[1][freqIndex] = ecal[1] * scale;
			measuredEcal[2][freqIndex] = ecal[2] * scale;
			current_props._cal_data[collectMeasurementType][freqIndex] = v[0]/v[1] - measuredEcal[0][freqIndex];

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
				measuredEcal[0][freqIndex] = measuredEcal[0][freqIndex] * 0.8f + ecal[0] * scale;
				measuredEcal[1][freqIndex] = measuredEcal[1][freqIndex] * 0.8f + ecal[1] * scale;
				measuredEcal[2][freqIndex] = measuredEcal[2][freqIndex] * 0.8f + ecal[2] * scale;
			} else {
				measuredEcal[0][freqIndex] = ecal[0] * scale;
				measuredEcal[1][freqIndex] = ecal[1] * scale;
				measuredEcal[2][freqIndex] = ecal[2] * scale;
			}
			if(ecalState == ECAL_STATE_MEASURING
					&& freqIndex == vnaMeasurement.sweepPoints - 1) {
				ecalState = ECAL_STATE_2NDSWEEP;
			} else if(ecalState == ECAL_STATE_2NDSWEEP) {
				ecalState = ECAL_STATE_DONE;
				vnaMeasurement.ecalIntervalPoints = MEASUREMENT_ECAL_INTERVAL;
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
	uint64_t start = current_props._frequency0;
	uint64_t step = (current_props._frequency1 - current_props._frequency0) / (current_props._sweep_points - 1);
	ecalState = ECAL_STATE_MEASURING;
	vnaMeasurement.ecalIntervalPoints = 1;
	vnaMeasurement.setSweep(start, step, current_props._sweep_points, 1);
	ecalState = ECAL_STATE_MEASURING;
}

void measurement_setup() {
	vnaMeasurement.phaseChanged = [](VNAMeasurementPhases ph) {
		measurementPhaseChanged(ph);
	};
	vnaMeasurement.emitDataPoint = [](int freqIndex, uint64_t freqHz, const VNAObservation& v, const complexf* ecal) {
		measurementEmitDataPoint(freqIndex, freqHz, v, ecal);
	};
	vnaMeasurement.frequencyChanged = [](uint64_t freqHz) {
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
			vnaMeasurement.sampleProcessor.clipFlag = false;
			vnaMeasurement.processSamples((uint16_t*)buf, len);
			digitalWrite(led, vnaMeasurement.sampleProcessor.clipFlag?1:0);
		}
	}
}

// transmit any outstanding data in the usbTxQueue
void usb_transmit() {
	int rdRPos = usbTxQueueRPos;
	int rdWPos = usbTxQueueWPos;
	__sync_synchronize();

	if(rdRPos == rdWPos) // queue empty
		return;
	
	usbDataPoint& usbDP = usbTxQueue[rdRPos];
	VNAObservation& value = usbDP.value;
	int32_t fwdRe = value[1].real();
	int32_t fwdIm = value[1].imag();
	int32_t reflRe = value[0].real();
	int32_t reflIm = value[0].imag();
	int32_t thruRe = value[2].real();
	int32_t thruIm = value[2].imag();
	uint8_t txbuf[31];
	txbuf[0] = fwdRe & 0x7F;
	txbuf[1] = (fwdRe >> 7) | 0x80;
	txbuf[2] = (fwdRe >> 14) | 0x80;
	txbuf[3] = (fwdRe >> 21) | 0x80;
	txbuf[4] = (fwdRe >> 28) | 0x80;
	
	txbuf[5] = (fwdIm >> 0) | 0x80;
	txbuf[6] = (fwdIm >> 7) | 0x80;
	txbuf[7] = (fwdIm >> 14) | 0x80;
	txbuf[8] = (fwdIm >> 21) | 0x80;
	txbuf[9] = (fwdIm >> 28) | 0x80;
	
	txbuf[10] = (reflRe >> 0) | 0x80;
	txbuf[11] = (reflRe >> 7) | 0x80;
	txbuf[12] = (reflRe >> 14) | 0x80;
	txbuf[13] = (reflRe >> 21) | 0x80;
	txbuf[14] = (reflRe >> 28) | 0x80;
	
	txbuf[15] = (reflIm >> 0) | 0x80;
	txbuf[16] = (reflIm >> 7) | 0x80;
	txbuf[17] = (reflIm >> 14) | 0x80;
	txbuf[18] = (reflIm >> 21) | 0x80;
	txbuf[19] = (reflIm >> 28) | 0x80;
	
	txbuf[20] = (thruRe >> 0) | 0x80;
	txbuf[21] = (thruRe >> 7) | 0x80;
	txbuf[22] = (thruRe >> 14) | 0x80;
	txbuf[23] = (thruRe >> 21) | 0x80;
	txbuf[24] = (thruRe >> 28) | 0x80;
	
	txbuf[25] = (thruIm >> 0) | 0x80;
	txbuf[26] = (thruIm >> 7) | 0x80;
	txbuf[27] = (thruIm >> 14) | 0x80;
	txbuf[28] = (thruIm >> 21) | 0x80;
	txbuf[29] = (thruIm >> 28) | 0x80;
	
	uint8_t checksum=0b01000110;
	for(int i=0; i<30; i++)
		checksum = (checksum xor ((checksum<<1) | 1)) xor txbuf[i];
	txbuf[30] = checksum | (1<<7);
	
	serial.print((char*)txbuf, sizeof(txbuf));
	__sync_synchronize();
	usbTxQueueRPos = (rdRPos + 1) & usbTxQueueMask;
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

void processDataPoint() {
	int rdRPos = usbTxQueueRPos;
	int rdWPos = usbTxQueueWPos;
	__sync_synchronize();

	while(rdRPos != rdWPos) {
		usbDataPoint& usbDP = usbTxQueue[rdRPos];
		VNAObservation& value = usbDP.value;
		int freqIndex = usbDP.freqIndex;
		auto refl = value[0]/value[1] - measuredEcal[0][freqIndex];
		auto thru = value[2]/value[1] - measuredEcal[2][freqIndex]*0.8f;
		if(current_props._cal_status & CALSTAT_APPLY) {
			//refl -= current_props._cal_data[CAL_LOAD][usbDP.freqIndex];
			refl = SOL_compute_reflection(
						current_props._cal_data[CAL_SHORT][freqIndex],
						current_props._cal_data[CAL_OPEN][freqIndex],
						current_props._cal_data[CAL_LOAD][freqIndex],
						refl);
		}
		measured[0][usbDP.freqIndex] = refl;
		measured[1][usbDP.freqIndex] = thru;
		
		rdRPos = (rdRPos + 1) & usbTxQueueMask;
	}
	usbTxQueueRPos = rdRPos;
}

int main(void) {
	int i;
	boardInit();

	pinMode(led, OUTPUT);
	pinMode(led2, OUTPUT);
	pinMode(RFSW_ECAL, OUTPUT);
	pinMode(RFSW_BBGAIN, OUTPUT);
	pinMode(RFSW_TXSYNTH, OUTPUT);
	pinMode(RFSW_RXSYNTH, OUTPUT);
	pinMode(RFSW_REFL, OUTPUT);
	pinMode(RFSW_RECV, OUTPUT);

	digitalWrite(led, HIGH);
	
	rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(0));
	rfsw(RFSW_RXSYNTH, RFSW_RXSYNTH_LF);
	rfsw(RFSW_TXSYNTH, RFSW_TXSYNTH_LF);
	rfsw(RFSW_REFL, RFSW_REFL_ON);
	rfsw(RFSW_RECV, RFSW_RECV_REFL);
	rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);

	delay(200);

	serial.setReceiveCallback([](uint8_t* s, int len) {
		serialCharHandler(s, len);
	});
	// baud rate is ignored for usbserial
	serial.begin(115200);

	flash_config_recall();

	lcd_setup();
	UIHW::init(tim2Period);

	si5351_i2c.init();
	if(!synthesizers::si5351_setup())
		errorBlink(2);

	setFrequency(56000000);

	measurement_setup();
	adc_setup();
	timers_setup();

	adf4350_setup();


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
		if(usbDataMode) {
			if(outputRawSamples)
				usb_transmit_rawSamples();
			else
				usb_transmit();
			// display "usb mode" screen
			if(!lastUSBDataMode) {
				show_usb_data_mode();
				
			}
			lastUSBDataMode = usbDataMode;
			continue;
		}
		lastUSBDataMode = usbDataMode;
		processDataPoint();

		// if we have no pending events, use idle cycles to refresh the graph
		if(!eventQueue.readable()) {
			if(refreshEnabled) {
				plot_into_index(measured);
				draw_all(true);
			}
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
extern "C" void *memcpy(char *dest, const char *src, uint32_t n) {
	for(int i=0;i<n;i++) dest[i] = src[i];
	return dest;
}*/



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


	void set_sweep_frequency(SweepParameter type, freqHz_t frequency) {
		switch(type) {
			case ST_START:
				current_props._frequency0 = frequency;
				break;
			case ST_STOP:
				current_props._frequency1 = frequency;
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
		refreshEnabled = !refreshEnabled;
	}
	void enable_refresh(bool enable) {
		refreshEnabled = enable;
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
