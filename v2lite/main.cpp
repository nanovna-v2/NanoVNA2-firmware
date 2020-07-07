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
#include "../command_parser.hpp"
#include "../stream_fifo.hpp"

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
uint32_t lo_freq = 150000;

USBSerial serial;

static const int adcBufSize=4096;	// must be power of 2
volatile uint16_t adcBuffer[adcBufSize];

constexpr int HWCHANNELS = 2;
VNAMeasurementNoSwitch<HWCHANNELS> vnaMeasurement;
static CommandParser cmdParser;
static StreamFIFO cmdInputFIFO;
static uint8_t cmdInputBuffer[128];

uint8_t registers[64];
constexpr int registerSize = sizeof(registers);
constexpr int registersSizeMask = registerSize - 1;

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



// callback called by VNAMeasurement when an observation is available.
static void measurementEmitDataPoint(int freqIndex, uint64_t freqHz, VNAObservation v) {
	//v[0] *= 0.3;
	// enqueue new data point
	int wrRPos = usbTxQueueRPos;
	int wrWPos = usbTxQueueWPos;
	__sync_synchronize();
	//digitalWrite(led, !digitalRead(led));
	if(((wrWPos + 1) & usbTxQueueMask) == wrRPos) {
		// overflow
		digitalWrite(led, 1);
	} else {
		usbTxQueue[wrWPos].freqIndex = freqIndex;
		usbTxQueue[wrWPos].value = v;
		__sync_synchronize();
		usbTxQueueWPos = (wrWPos + 1) & usbTxQueueMask;
		digitalWrite(led, 0);
	}
}


/*
optimized sample processor for the use case:
* 2 interleaved channels (reference and reflection)
* IF frequency of fSample/4
*/
struct MySampleProcessor {
	VNAMeasurementNoSwitch<2>* outputValuesTo = nullptr;
	int samplesPerValue = 96;	// 100us
	int phase = 0;
	
	
	int32_t v0re = 0, v0im = 0, v1re = 0, v1im = 0;
	int32_t tmpRe = 0, tmpIm = 0;

	// len must be a multiple of 8!!!
	void process(uint16_t* samples, int len) {
		uint16_t* ptr = samples;
		uint16_t* end = samples + len;
		while(ptr < end) {
			// there is a DC bias of +2048 in these values, but they cancel out
			// after the downconversion.

#define CONVERTSAMPLE(x) int32_t(x)
			auto sample0 = CONVERTSAMPLE(ptr[0]);
			auto sample1 = CONVERTSAMPLE(ptr[1]);
			auto sample2 = CONVERTSAMPLE(ptr[2]);
			auto sample3 = CONVERTSAMPLE(ptr[3]);
			auto sample4 = CONVERTSAMPLE(ptr[4]);
			auto sample5 = CONVERTSAMPLE(ptr[5]);
			auto sample6 = CONVERTSAMPLE(ptr[6]);
			auto sample7 = CONVERTSAMPLE(ptr[7]);
#undef CONVERTSAMPLE

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
			if(phase == samplesPerValue/2) {
				tmpRe = v0re;
				tmpIm = v0im;
				//v0re = v0im = v1re = v1im = 0;
			}
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



/*
optimized sample processor for the use case:
* 2 interleaved channels (reference and reflection)
* IF frequency of fSample/8
*/
struct MySampleProcessor2 {
	VNAMeasurementNoSwitch<2>* outputValuesTo = nullptr;
	int samplesPerValue = 96;	// 100us
	int phase = 0;
	
	
	int32_t a0re = 0, a0im = 0, a1re = 0, a1im = 0;
	int32_t tmpRe = 0, tmpIm = 0;

	// len must be a multiple of 16!!!
	void process(uint16_t* samples, int len) {
		uint16_t* ptr = samples;
		uint16_t* end = samples + len;
		while(ptr < end) {
			// there is a DC bias of +2048 in these values, but they cancel out
			// after the downconversion.
#define CONVERTSAMPLE(x) int32_t(x)
			auto sample0a = CONVERTSAMPLE(ptr[0]);
			auto sample0b = CONVERTSAMPLE(ptr[1]);
			auto sample1a = CONVERTSAMPLE(ptr[2]);
			auto sample1b = CONVERTSAMPLE(ptr[3]);
			auto sample2a = CONVERTSAMPLE(ptr[4]);
			auto sample2b = CONVERTSAMPLE(ptr[5]);
			auto sample3a = CONVERTSAMPLE(ptr[6]);
			auto sample3b = CONVERTSAMPLE(ptr[7]);
			auto sample4a = CONVERTSAMPLE(ptr[8]);
			auto sample4b = CONVERTSAMPLE(ptr[9]);
			auto sample5a = CONVERTSAMPLE(ptr[10]);
			auto sample5b = CONVERTSAMPLE(ptr[11]);
			auto sample6a = CONVERTSAMPLE(ptr[12]);
			auto sample6b = CONVERTSAMPLE(ptr[13]);
			auto sample7a = CONVERTSAMPLE(ptr[14]);
			auto sample7b = CONVERTSAMPLE(ptr[15]);
#undef CONVERTSAMPLE

			int32_t v0re = 0, v0im = 0, v1re = 0, v1im = 0;
			// 0 deg
			v0re += sample0a*32768;
			v1re += sample0b*32768;
			// 45 deg
			v0re += sample1a*23170;
			v0im += sample1a*23170;
			v1re += sample1b*23170;
			v1im += sample1b*23170;
			// 90 deg
			v0im += sample2a*32768;
			v1im += sample2b*32768;
			// 135 deg
			v0re -= sample3a*23170;
			v0im += sample3a*23170;
			v1re -= sample3b*23170;
			v1im += sample3b*23170;
			// 180 deg
			v0re -= sample4a*32768;
			v1re -= sample4b*32768;
			// 225 deg
			v0re -= sample5a*23170;
			v0im -= sample5a*23170;
			v1re -= sample5b*23170;
			v1im -= sample5b*23170;
			// 270 deg
			v0im -= sample6a*32768;
			v1im -= sample6b*32768;
			// 315 deg
			v0re += sample7a*23170;
			v0im -= sample7a*23170;
			v1re += sample7b*23170;
			v1im -= sample7b*23170;

			a0re += v0re/128;
			a0im += v0im/128;
			a1re += v1re/128;
			a1im += v1im/128;
			phase += 8;
			ptr += 16;
			if(phase == samplesPerValue/2) {
				tmpRe = v1re;
				tmpIm = v1im;
				//a0re = a0im = a1re = a1im = 0;
			}
			if(phase >= samplesPerValue) {
				int32_t accumRe[2] = {a0re/256, a1re/256};
				int32_t accumIm[2] = {a0im/256, a1im/256};
				outputValuesTo->processValue(accumRe, accumIm);
				phase = 0;
				a0re = a0im = a1re = a1im = 0;
			}
		}
	}
};


MySampleProcessor2 mySampleProcessor;

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
		adc_read(buf, len, 16);

		mySampleProcessor.process((uint16_t*)buf, len);
	}
}







bool serialSendTimeout(const char* s, int len, int timeoutMillis) {
	for(int i = 0; i < timeoutMillis; i++) {
		if(serial.trySend(s, len))
			return true;
		delay(1);
	}
	return false;
}

/*
For a description of the command interface see command_parser.hpp
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
-- 26: dataMode: 0 => VNA data, 1 => raw data, 2 => exit usb data mode
-- 30: valuesFIFO - returns data points; elements are 32-byte. See below for data format.
--                  command 0x14 reads FIFO data; writing any value clears FIFO.
-- f0: device variant (01)
-- f1: protocol version (01)
-- f2: hardware revision
-- f3: firmware major version

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


static void cmdRegisterWrite(int address);

//1425tX^^^^^^^^^^^^^^XXXXXXXXXXXXXXXXXXXXXXMMMMMM%Vc222$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$44443 \uuuuuuuuuuuuiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiyhz<ggggggggggggggggggggggggggggggggggg


static void cmdReadFIFO(int address, int nValues) {
	if(address != 0x30) return;

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

		if(!serialSendTimeout((char*)txbuf, sizeof(txbuf), 1500)) {
			return;
		}

		__sync_synchronize();
		usbTxQueueRPos = (rdRPos + 1) & usbTxQueueMask;
		i++;
	}
}

// apply usb-configured sweep parameters
static void setVNASweepToUSB() {
	int points = *(uint16_t*)(registers + 0x20);
	int values = *(uint16_t*)(registers + 0x22);

	if(points > USB_POINTS_MAX)
		points = USB_POINTS_MAX;

	vnaMeasurement.sweepStartHz = (freqHz_t)*(uint64_t*)(registers + 0x00);
	vnaMeasurement.sweepStepHz = (freqHz_t)*(uint64_t*)(registers + 0x10);
	vnaMeasurement.sweepDataPointsPerFreq = values;
	vnaMeasurement.sweepPoints = points;
	vnaMeasurement.resetSweep();
}
static void cmdRegisterWrite(int address) {
	if(address == 0x00 || address == 0x10 || address == 0x20 || address == 0x22) {
		setVNASweepToUSB();
	}
	if(address == 0x26) {
		auto val = registers[0x26];
		if(val == 0) {
			outputRawSamples = false;
		} else if(val == 1) {
			outputRawSamples = true;
		} else if(val == 2) {
			outputRawSamples = false;
		}
	}
	if(address == 0x00 || address == 0x10 || address == 0x20) {
		// TODO: reset ecal
	}
	if(address == 0x30) {
		usbTxQueueRPos = usbTxQueueWPos;
	}
}


static void cmdInit() {
	cmdParser.handleReadFIFO = [](int address, int nValues) {
		return cmdReadFIFO(address, nValues);
	};
	cmdParser.handleWriteFIFO = [](int address, int totalBytes, int nBytes, const uint8_t* data) {};
	cmdParser.handleWrite = [](int address) {
		return cmdRegisterWrite(address);
	};
	cmdParser.send = [](const uint8_t* s, int len) {
		serialSendTimeout((char*) s, len, 1500);
	};
	cmdParser.registers = registers;
	cmdParser.registersSizeMask = registersSizeMask;

	cmdInputFIFO.buffer = cmdInputBuffer;
	cmdInputFIFO.bufferSize = sizeof(cmdInputBuffer);
	cmdInputFIFO.output = [](const uint8_t* s, int len) {
		cmdParser.handleInput(s, len);
	};
}



static int cnt = 0;
void usb_transmit_rawSamples() {
	volatile uint16_t* buf;
	int len, aggregates;
	adc_read(buf, len, HWCHANNELS);
	aggregates = len/HWCHANNELS;

	int8_t tmpBuf[adcBufSize/HWCHANNELS];
	for(int i=0; i<aggregates; i++)
		tmpBuf[i] = int8_t(buf[i*HWCHANNELS + 0] >> 4) - 128;
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

	cmdInit();
	serial.setReceiveCallback([](uint8_t* s, int len) {
		cmdInputFIFO.input(s, len);
	});
	// baud rate is ignored for usbserial
	serial.begin(115200);

	pinMode(USB0_DP, INPUT);
	//setFrequency(56000);

	measurement_setup();
	adc_setup();
	timers_setup();

	adf4350_setup();
	
	vnaMeasurement.sweepStartHz = 200000000;
	vnaMeasurement.sweepStepHz = 0;
	vnaMeasurement.sweepPoints = 101;
	vnaMeasurement.sweepDataPointsPerFreq = 2;
	vnaMeasurement.resetSweep();

	setFrequency(vnaMeasurement.sweepStartHz);

	while(true) {
		if(outputRawSamples)
			usb_transmit_rawSamples();
		else {
			// process any outstanding commands from usb
			cmdInputFIFO.drain();
		}
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

