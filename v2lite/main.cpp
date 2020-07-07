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

#include <array>
#include <complex>

#include <board.hpp>
#include "../common.hpp"
#include "../vna_measurement_noswitch.hpp"
#include "../synthesizers.hpp"
#include "../fifo.hpp"
#include "../sin_rom.hpp"

#include <libopencm3/stm32/timer.h>

using namespace mculib;
using namespace std;
using namespace board;

// see https://lists.debian.org/debian-gcc/2003/07/msg00057.html
// this can be any value since we are not using shared libraries.
void* __dso_handle = (void*) &__dso_handle;

bool outputRawSamples = false;
int cpu_mhz = 24;
uint32_t adf4350_freqStep = 10000;
uint32_t lo_freq = 250000;

USBSerial serial;

static const int adcBufSize=4096;	// must be power of 2
volatile uint16_t adcBuffer[adcBufSize];

constexpr int HWCHANNELS = 2;
VNAMeasurementNoSwitch<HWCHANNELS> vnaMeasurement;

uint8_t registers[32];

struct usbDataPoint {
	VNAObservation value;
	int freqIndex;
};
usbDataPoint usbTxQueue[64];
constexpr int usbTxQueueMask = 63;
volatile int usbTxQueueWPos = 0;
volatile int usbTxQueueRPos = 0;

// periods of a 1MHz clock; how often to call adc_process()
static constexpr int tim1Period = 250;	// 1MHz / 250 = 4kHz


// value is in microseconds; increments at 40kHz by TIM1 interrupt
volatile uint32_t systemTimeCounter = 0;


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
	rcc_periph_reset_pulse(RST_TIM1);
	
	// set tim1 to highest priority
	nvic_set_priority(NVIC_TIM1_UP_IRQ, 1 * 16);

	nvic_enable_irq(NVIC_TIM1_UP_IRQ);
	startTimer(TIM1, tim1Period);
}
extern "C" void tim1_up_isr() {
	TIM1_SR = 0;
	systemTimeCounter += tim1Period;
	adc_process();
	if(TIM1_SR & 0b1)
		digitalWrite(led2, 1);
	else
		digitalWrite(led2, 0);
	//digitalWrite(led2, !digitalRead(led2));
}




void adf4350_setup() {
	adf4350_rx.N = 120;
	adf4350_rx.rfPower = 0b10;
	adf4350_rx.sendConfig();
	adf4350_rx.sendN();

	adf4350_tx.N = 120;
	adf4350_tx.rfPower = 0b01;
	adf4350_tx.sendConfig();
	adf4350_tx.sendN();
}
void adf4350_update(freqHz_t freq) {
	freq = uint32_t(freq/adf4350_freqStep) * adf4350_freqStep;
	synthesizers::adf4350_set(adf4350_tx, freq, adf4350_freqStep);
	synthesizers::adf4350_set(adf4350_rx, freq + lo_freq, adf4350_freqStep);
}


// set the measurement frequency including setting the tx and rx synthesizers
void setFrequency(freqHz_t freq) {
	adf4350_update(freq);
}

void adc_setup() {
	static uint8_t channel_array0[16] = {6};
	static uint8_t channel_array1[16] = {7};
	dmaADC.buffer = adcBuffer;
	dmaADC.bufferSizeBytes = sizeof(adcBuffer);
	dmaADC.init(channel_array0, channel_array1, 1);

	adc_set_sample_time_on_all_channels(ADC1, adc_ratecfg);
	adc_set_sample_time_on_all_channels(ADC2, adc_ratecfg);
	
	dmaADC.start();
}

// read and consume data from the adc ring buffer.
// the number of words consumed is always an integer multiple of modulus.
void adc_read(volatile uint16_t*& data, int& len, int modulus=1) {
	static uint32_t lastIndex = 0;
	uint32_t cIndex = dmaADC.position()*2;
	uint32_t bufWords = dmaADC.bufferSizeBytes / 2;
	cIndex &= (bufWords-1);
	cIndex = (cIndex / modulus) * modulus;
	
	data = ((volatile uint16_t*) dmaADC.buffer) + lastIndex;
	if(cIndex >= lastIndex) {
		len = cIndex - lastIndex;
	} else {
		len = bufWords - lastIndex;
	}
	len = (len/modulus) * modulus;
	lastIndex += len;
	if(lastIndex >= bufWords) lastIndex = 0;
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


// callback called by VNAMeasurement when an observation is available.
static void measurementEmitDataPoint(int freqIndex, uint64_t freqHz, VNAObservation v) {
	v[0] *= 0.3;
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


/*
optimized sample processor for the use case:
* 2 interleaved channels (reference and reflection)
* IF frequency of fSample/4
*/
struct MySampleProcessor {
	VNAMeasurementNoSwitch<2>* outputValuesTo = nullptr;
	int samplesPerValue = 8;
	int phase = 0;
	
	
	int32_t v0re = 0, v0im = 0, v1re = 0, v1im = 0;

	// len must be a multiple of 8!!!
	void process(uint16_t* samples, int len) {
		uint16_t* ptr = samples;
		uint16_t* end = samples + len;
		while(ptr < end) {
			// there is a DC bias of +2048 in these values, but they cancel out
			// after the downconversion.
			int16_t sample0 = int16_t(ptr[0]);
			int16_t sample1 = int16_t(ptr[1]);
			int16_t sample2 = int16_t(ptr[2]);
			int16_t sample3 = int16_t(ptr[3]);
			int16_t sample4 = int16_t(ptr[4]);
			int16_t sample5 = int16_t(ptr[5]);
			int16_t sample6 = int16_t(ptr[6]);
			int16_t sample7 = int16_t(ptr[7]);

			v0re += sample0;
			v1re += sample1;
			v0im += sample2;
			v1im += sample3;
			v0re -= sample4;
			v1re -= sample5;
			v0im -= sample6;
			v1im -= sample7;

			phase += 4;
			ptr += 8;
			if(phase >= samplesPerValue) {
				int32_t accumRe[2] = {v0re, v1re};
				int32_t accumIm[2] = {v0im, v1im};
				outputValuesTo->processValue(accumRe, accumIm);
				phase = 0;
				v0re = v0im = v1re = v1im = 0;
			}
		}
	}
};

MySampleProcessor mySampleProcessor;

void measurement_setup() {
	vnaMeasurement.emitDataPoint = [](int freqIndex, uint64_t freqHz, const VNAObservation& v) {
		measurementEmitDataPoint(freqIndex, freqHz, v);
	};
	vnaMeasurement.frequencyChanged = [](uint64_t freqHz) {
		setFrequency(freqHz);
	};
	vnaMeasurement.init();
	mySampleProcessor.outputValuesTo = &vnaMeasurement;
}


void adc_process() {
	if(!outputRawSamples) {
		volatile uint16_t* buf;
		int len;
		adc_read(buf, len, 8);
		bool clipFlag = false;

		mySampleProcessor.process((uint16_t*)buf, len);
		digitalWrite(led, clipFlag?1:0);
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
	int len, aggregates;
	adc_read(buf, len, HWCHANNELS);
	aggregates = len/HWCHANNELS;

	int8_t tmpBuf[adcBufSize/HWCHANNELS];
	for(int i=0; i<aggregates; i++)
		tmpBuf[i] = int8_t(buf[i*HWCHANNELS + 1] >> 4) - 128;
	serial.print((char*)tmpBuf, aggregates);
	
	cnt += aggregates;

	rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);
}


int main(void) {
	int i;
	boardInit();

	pinMode(led, OUTPUT);
	pinMode(led2, OUTPUT);
	pinMode(RFSW_ECAL, OUTPUT);

	digitalWrite(led, HIGH);
	rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);

	pinMode(USB0_DP, OUTPUT);
	digitalWrite(USB0_DP, LOW);

	delay(300);

	serial.setReceiveCallback([](uint8_t* s, int len) {
		serialCharHandler(s, len);
	});
	// baud rate is ignored for usbserial
	serial.begin(115200);

	pinMode(USB0_DP, INPUT);
	//setFrequency(56000);

	measurement_setup();
	adc_setup();
	timers_setup();

	adf4350_setup();
	
	vnaMeasurement.setSweep(500000000, 0, 1);
	setFrequency(500000000);

	while(true) {
		if(outputRawSamples)
			usb_transmit_rawSamples();
		else
			usb_transmit();
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

