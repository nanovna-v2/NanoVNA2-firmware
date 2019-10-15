/*
 * This file is part of the libopencm3 project.
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
//#define SERIAL_USE_USBSERIAL

#define PRNT(x)
// Serial.print(x)
#define PRNTLN(x)
// Serial.println(x)
#include <mculib/fastwiring.hpp>
#include <mculib/softi2c.hpp>
#include <mculib/si5351.hpp>
#include <mculib/dma_adc.hpp>
#include <mculib/usbserial.hpp>

#include <array>
#include <complex>

#include "board_v2_0.hpp"
#include "ili9341.hpp"

#include <libopencm3/stm32/timer.h>
#include "sample_processor.H"

using namespace mculib;
using namespace std;

// see https://lists.debian.org/debian-gcc/2003/07/msg00057.html
// this can be any value since we are not using shared libraries.
void* __dso_handle = (void*) &__dso_handle;

bool outputRawSamples = false;
int lo_freq = 12000; // IF frequency, Hz
int xtal_freq = 24000; //19200; // si5351 input frequency, kHz
int cpu_mhz = 24;
int adf4350_freqStep = 6; // adf4350 resolution, kHz
int adf4350_modulus = xtal_freq/adf4350_freqStep;

USBSerial serial;

static const int adcBufSize=4096;	// must be power of 2
volatile uint16_t adcBuffer[adcBufSize];


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

void sampleProcessor_emitValue(int32_t valRe, int32_t valIm);
auto emitValue = [](int32_t valRe, int32_t valIm) {
	sampleProcessor_emitValue(valRe, valIm);
};
SampleProcessor<decltype(emitValue)> sampleProcessor(emitValue);


void timer_setup() {
	// set the timer to count one tick per us
	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_reset_pulse(RST_TIM1);
	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM1, cpu_mhz-1);
	timer_set_repetition_counter(TIM1, 0);
	timer_continuous_mode(TIM1);
	
	// this doesn't really set the period, but the "autoreload value"; actual period is this plus 1.
	// this should be fixed in libopencm3.
	// 1MHz / 25 = 40kHz
	timer_set_period(TIM1, 24);

	timer_enable_preload(TIM1);
	timer_enable_preload_complementry_enable_bits(TIM1);
	timer_enable_break_main_output(TIM1);
	
	timer_enable_irq(TIM1, TIM_DIER_UIE);
	nvic_enable_irq(NVIC_TIM1_UP_IRQ);
	
	TIM1_EGR = TIM_EGR_UG;
	timer_set_counter(TIM1, 0);
	timer_enable_counter(TIM1);
}
extern "C" void tim1_up_isr() {
	TIM1_SR = 0;
	adc_process();
}


void si5351_setup() {
	using namespace Si5351;

	si5351_i2c.init();

	//delay(300);

	//if(!si5351_i2c.probe((0xC0) | 1))
	//	errorBlink(1);
	//return;

	si5351.SetFieldsToDefault();	//initialize the structure with default "safe" values
	
	// hook up i2c
	uint8_t devAddr = 0xC0;
	si5351.ReadRegister = [devAddr](uint8_t addr) -> uint8_t {
		return si5351_i2c.read_si5351(devAddr, addr);
	};
	si5351.WriteRegister = [devAddr](uint8_t addr, uint8_t data) -> int {
		return si5351_i2c.write(devAddr, addr, data);
	};
	si5351.OSC.OSC_XTAL_Load = XTAL_Load_4_pF;	//use 4 pF load for crystal


	si5351.PLL[0].PLL_Clock_Source = PLL_Clock_Source_XTAL;	//select xrystal as clock input for the PLL
	si5351.PLL[0].PLL_Multiplier_Integer = 32*128;				//multiply the clock frequency by 32, this gets us 800 MHz clock
	si5351.PLL[0].PLL_Multiplier_Numerator = 1*8;
	si5351.PLL[0].PLL_Multiplier_Denominator = xtal_freq;
	
	si5351.PLL[1].PLL_Clock_Source = PLL_Clock_Source_XTAL;
	si5351.PLL[1].PLL_Multiplier_Integer = 32*128;
	si5351.PLL[1].PLL_Multiplier_Numerator = 1*8 + lo_freq/1000*8;
	si5351.PLL[1].PLL_Multiplier_Denominator = xtal_freq;

	si5351.MS[0].MS_Clock_Source = MS_Clock_Source_PLLA;
	si5351.MS[0].MS_Divider_Integer = 8; // divide pll frequency by 8

	si5351.MS[2].MS_Clock_Source = MS_Clock_Source_PLLB;
	si5351.MS[2].MS_Divider_Integer = 8; // divide pll frequency by 8
	

	//si5351.CLK[0].CLK_R_Div = CLK_R_Div64;	//divide the MultiSynth output by 64, this gets us 50 kHz
	si5351.CLK[0].CLK_R_Div = CLK_R_Div1; // divide by 1; 100MHz
	si5351.CLK[0].CLK_Enable = ON;	//turn on the output
	si5351.CLK[0].CLK_I_Drv = CLK_I_Drv_8mA;

	si5351.CLK[1].CLK_Clock_Source = CLK_Clock_Source_XTAL;
	si5351.CLK[1].CLK_R_Div = CLK_R_Div1; // divide by 1; 24MHz
	si5351.CLK[1].CLK_Enable = ON;	//turn on the output
	si5351.CLK[1].CLK_I_Drv = CLK_I_Drv_8mA;

	si5351.CLK[2].CLK_R_Div = CLK_R_Div1; // divide by 1; 100MHz
	si5351.CLK[2].CLK_Enable = ON;	//turn on the output
	si5351.CLK[2].CLK_I_Drv = CLK_I_Drv_8mA;
	
	
	if(si5351.Init() != 0)
		errorBlink(2);
}

void si5351_set(int i, int pll, uint32_t freq_khz) {
	using namespace Si5351;

	CLKRDiv rDiv = CLK_R_Div1;

	// PLL should be configured between 600 and 900 MHz
	// round up
	uint32_t msDiv = 900000/freq_khz;
	uint32_t totalDiv = msDiv;
	
	// FIXME: output divider value of 5 is broken for some reason
	if(msDiv == 5) msDiv = 4;
	if(msDiv < 4) msDiv = 4;
	if(msDiv > 1024) {
		msDiv /= 64;
		totalDiv = msDiv*64;
		rDiv = CLK_R_Div64;
	}
	
	uint32_t vco = freq_khz * totalDiv;
	uint32_t mult = vco*128;
	uint32_t N = mult/xtal_freq;
	uint32_t frac = mult - N*xtal_freq;

	si5351.PLL[pll].PLL_Multiplier_Integer = N;
	si5351.PLL[pll].PLL_Multiplier_Numerator = frac;
	//Si5351_ConfigStruct.PLL[i].PLL_Multiplier_Denominator = xtal_freq;
	
	si5351.PLLConfig((PLLChannel) pll);
	
	if(si5351.MS[i].MS_Divider_Integer != msDiv) {
		si5351.MS[i].MS_Divider_Integer = msDiv;
		si5351.MSConfig((MSChannel) i);
	}
	if(si5351.CLK[i].CLK_R_Div != rDiv) {
		si5351.CLK[i].CLK_R_Div = rDiv;
		si5351.CLKConfig((CLKChannel) i);
	}
}
void si5351_doUpdate(uint32_t freq_khz) {
	using namespace Si5351;
	
	si5351_set(0, 0, freq_khz+lo_freq/1000);
	si5351_set(2, 1, freq_khz);
	si5351.PLLReset2();
}

void si5351_update(uint32_t freq_khz) {
	static uint32_t prevFreq = 0;
	si5351_doUpdate(freq_khz);
	if(freq_khz < prevFreq)
		si5351_doUpdate(freq_khz);
	prevFreq = freq_khz;
}

// freq_khz must be a multiple of adf4350_freqStep
template<class T>
void adf4350_set(T& adf4350, uint32_t freq_khz) {
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

	/*if(freq_khz > 2000000) {
		// take feedback from divided output
		N = freq_khz/adf4350_freqStep;
		adf4350.feedbackFromDivided = true;
	} else adf4350.feedbackFromDivided = false;*/

	adf4350.O = O;
	adf4350.N = N / adf4350_modulus;
	adf4350.numerator = N - (adf4350.N * adf4350_modulus);
	adf4350.denominator = adf4350_modulus;
	adf4350.sendConfig();
	adf4350.sendN();
}

void adf4350_setup() {
	adf4350_rx.N = 120;
	adf4350_rx.rfPower = 0b01;
	adf4350_rx.sendConfig();
	adf4350_rx.sendN();

	adf4350_tx.N = 120;
	adf4350_tx.rfPower = 0b10;
	adf4350_tx.sendConfig();
	adf4350_tx.sendN();
}
void adf4350_update(uint32_t freq_khz) {
	freq_khz = uint32_t(freq_khz/adf4350_freqStep) * adf4350_freqStep;
	adf4350_set(adf4350_tx, freq_khz);
	adf4350_set(adf4350_rx, freq_khz + lo_freq/1000);
}

void setFrequency(uint32_t freq_khz) {
	/*if(freq_khz > 2000000) {
		lo_freq = 36000;
	} else {
		lo_freq = 12000;
	}*/

	if(freq_khz > 2700000)
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(3));
	else if(freq_khz > 1500000)
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(2));
	else
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(1));

	// use adf4350 for f > 140MHz
	if(freq_khz > 140000) {
		adf4350_update(freq_khz);
		rfsw(RFSW_TXSYNTH, RFSW_TXSYNTH_HF);
		rfsw(RFSW_RXSYNTH, RFSW_RXSYNTH_HF);
	} else {
		si5351_update(freq_khz);
		rfsw(RFSW_TXSYNTH, RFSW_TXSYNTH_LF);
		rfsw(RFSW_RXSYNTH, RFSW_RXSYNTH_LF);
	}
	uint64_t tmp = uint64_t(lo_freq * adc_period_cycles) << 32;
	tmp = tmp / (adc_clk);
	sampleProcessor.sgRate = uint32_t(tmp);
}

void adc_setup() {
	static uint8_t channel_array[16] = {1};
	dmaADC.buffer = adcBuffer;
	dmaADC.bufferSizeBytes = sizeof(adcBuffer);
	dmaADC.init(channel_array, 1);

	adc_set_sample_time_on_all_channels(dmaADC.adcDevice, adc_ratecfg);
	dmaADC.start();
}

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
	ili9341_spi_transfer = [](uint32_t sdi, int bits) {
		return ili9341_spi.doTransfer(sdi, bits);
	};
	ili9341_spi_transfer_bulk = [](uint32_t words) {
		ili9341_spi.doTransfer_bulk_send(spi_buffer, words);
	};
	ili9341_init();
	ili9341_test(5);
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
--				1: adc0 data (filtered and decimated to 19.2MSPS)
--				2: adc1 data (filtered and decimated to 19.2MSPS)
--				3: adc0 data unfiltered (downsampled to 19.2MSPS)
--				4: adc1 data unfiltered (downsampled to 19.2MSPS)
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
				
				setFrequency(freq*10);
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


struct usbDataPoint {
	complex<int32_t> port0Out, port0In, port1In;
};
usbDataPoint usbTxQueue[32];
constexpr int usbTxQueueMask = 31;
volatile int usbTxQueueWPos = 0;
volatile int usbTxQueueRPos = 0;



enum class MeasurementPhases {
	REFERENCE,
	REFL1,
	REFL2,
	THRU
};

MeasurementPhases measurementPhase = MeasurementPhases::REFERENCE;
constexpr uint32_t nWait = 1, nValues = 6;
int32_t dpRe = 0, dpIm = 0;
uint32_t valueCounter = 0;

void setMeasurementPhase(MeasurementPhases ph) {
	switch(ph) {
		case MeasurementPhases::REFERENCE:
			rfsw(RFSW_REFL, RFSW_REFL_ON);
			rfsw(RFSW_RECV, RFSW_RECV_REFL);
			rfsw(RFSW_ECAL, RFSW_ECAL_SHORT);
			break;
		case MeasurementPhases::REFL1:
			rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);
			break;
		case MeasurementPhases::REFL2:
			rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);
			break;
		case MeasurementPhases::THRU:
			rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);
			rfsw(RFSW_REFL, RFSW_REFL_OFF);
			rfsw(RFSW_RECV, RFSW_RECV_PORT2);
			break;
	}
	measurementPhase = ph;
	valueCounter = 0;
	dpRe = dpIm = 0;
}
void sampleProcessor_emitValue(int32_t valRe, int32_t valIm) {
	if(valueCounter >= nWait) {
		dpRe += valRe;
		dpIm += valIm;
	}
	valueCounter++;

	// state variables
	static int32_t fwdRe = 0, fwdIm = 0;
	static int32_t reflRe = 0, reflIm = 0;
	static int32_t thruRe = 0, thruIm = 0;
	
	if(measurementPhase == MeasurementPhases::REFERENCE
		&& valueCounter == (nWait + nValues*2)) {
		fwdRe = dpRe;
		fwdIm = dpIm;
		setMeasurementPhase(MeasurementPhases::REFL1);
	} else if(measurementPhase == MeasurementPhases::REFL1
		&& valueCounter == (nWait + nValues)) {
		reflRe = dpRe;
		reflIm = dpIm;
		setMeasurementPhase(MeasurementPhases::REFL2);
	} else if(measurementPhase == MeasurementPhases::REFL2
		&& valueCounter == (nWait + nValues)) {
		//reflRe = (reflRe - dpRe) * 2;
		//reflIm = (reflIm - dpIm) * 2;
		reflRe += dpRe;
		reflIm += dpIm;
		setMeasurementPhase(MeasurementPhases::THRU);
	} else if(measurementPhase == MeasurementPhases::THRU
		&& valueCounter == (nWait + nValues*2)) {
		thruRe = dpRe;
		thruIm = dpIm;
		setMeasurementPhase(MeasurementPhases::REFERENCE);
		
		// enqueue new data point
		int wrRPos = usbTxQueueRPos;
		int wrWPos = usbTxQueueWPos;
		__sync_synchronize();
		if((wrRPos + 1) & usbTxQueueMask == usbTxQueueWPos) {
			// overflow
		} else {
			usbTxQueue[wrWPos].port0Out = {fwdRe, fwdIm};
			usbTxQueue[wrWPos].port0In = {reflRe, reflIm};
			usbTxQueue[wrWPos].port1In = {thruRe, thruIm};
			__sync_synchronize();
			usbTxQueueWPos = (wrRPos + 1) & usbTxQueueMask;
		}
	}
}




void adc_process() {
	if(!outputRawSamples) {
		volatile uint16_t* buf;
		int len;
		for(int i=0; i<2; i++) {
			adc_read(buf, len);
			sampleProcessor.clipFlag = false;
			sampleProcessor.process((uint16_t*)buf, len);
			digitalWrite(led, sampleProcessor.clipFlag?1:0);
		}
	}
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

	lcd_setup();

	si5351_setup();

	/*bool b = false;
	while(1) {
		b = !b;
		digitalWrite(led, b?1:0);
		delay(300);
		serial.println("aaaaa");
	}*/

	setFrequency(56000);

	adc_setup();
	timer_setup();

	/*while(1) {
		volatile uint16_t* buf;
		int len;
		adc_read(buf, len);
		
		int8_t tmpBuf[adcBufSize];
		for(int i=0; i<len; i++)
			tmpBuf[i] = int8_t(buf[i] >> 4) - 128;
		serial.print((char*)tmpBuf, len);
		//serial.println(len);
		//delay(100);
	}*/
	
	adf4350_setup();


	// sgRate = lo_freq / (adc_clk/adc_period_cycles) * 2 * 2^32
	// sgRate = lo_freq * adc_period_cycles/adc_clk * 2 * 2^32
	uint64_t tmp = uint64_t(lo_freq * adc_period_cycles) << 32;
	tmp = tmp / (adc_clk);
	sampleProcessor.sgRate = uint32_t(tmp);
	//sampleProcessor.sgRate = (((lo_freq * adc_period_cycles) << 15) / (adc_clk/1000000)) * (8590);
	sampleProcessor.init();


	bool testSG = false;
	
	if(testSG) {
		//sampleProcessor.sgRate /= 100;
		while(1) {
			uint16_t tmp = 1;
			sampleProcessor.process(&tmp, 1);
		}
	} else {
		timer_setup();
		uint32_t cnt = 0;
		while(1) {
			if(outputRawSamples) {
				volatile uint16_t* buf;
				int len;
				adc_read(buf, len);
				cnt += len;
				int8_t tmpBuf[adcBufSize];
				for(int i=0; i<len; i++)
					tmpBuf[i] = int8_t(buf[i] >> 4) - 128;
				serial.print((char*)tmpBuf, len);
				rfsw(RFSW_ECAL, RFSW_ECAL_NORMAL);
				//rfsw(RFSW_RECV, ((cnt / 500) % 2) ? RFSW_RECV_REFL : RFSW_RECV_PORT2);
				//rfsw(RFSW_REFL, ((cnt / 500) % 2) ? RFSW_REFL_ON : RFSW_REFL_OFF);
				rfsw(RFSW_RECV, RFSW_RECV_REFL);
				rfsw(RFSW_REFL, RFSW_REFL_ON);
			} else {
				int rdRPos = usbTxQueueRPos;
				int rdWPos = usbTxQueueWPos;
				__sync_synchronize();

				if(rdRPos == rdWPos) // queue empty
					continue;
				
				usbDataPoint& value = usbTxQueue[rdRPos];
				int32_t fwdRe = value.port0Out.real();
				int32_t fwdIm = value.port0Out.imag();
				int32_t reflRe = value.port0In.real();
				int32_t reflIm = value.port0In.imag();
				int32_t thruRe = value.port1In.real();
				int32_t thruIm = value.port1In.imag();
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
