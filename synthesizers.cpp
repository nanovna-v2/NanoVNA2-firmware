#include "synthesizers.hpp"
#include <board.hpp>
#include <mculib/si5351.hpp>
#include <mculib/printk.hpp>

using namespace board;

namespace synthesizers {
	bool si5351_setup() {
		using namespace Si5351;

		si5351.SetFieldsToDefault();	//initialize the structure with default "safe" values
		
		// hook up i2c
		uint8_t devAddr = 0xC0;
		si5351.ReadRegister = [devAddr](uint8_t addr) -> uint8_t {
			return si5351_i2c.read_si5351(devAddr, addr);
		};
		si5351.WriteRegister = [devAddr](uint8_t addr, uint8_t data) -> int {
			return si5351_i2c.write(devAddr, addr, data);
		};
		si5351.WriteRegisters = [devAddr](uint8_t* data, int len) -> int {
			return si5351_i2c.write(devAddr, data, len);
		};
		si5351.OSC.OSC_XTAL_Load = XTAL_Load_4_pF;	//use 4 pF load for crystal

		auto rPLL = si5351_rxPLL;
		auto tPLL = si5351_txPLL;
		auto rPort = si5351_rxPort;
		auto tPort = si5351_txPort;
		auto pPort = si5351_passthruPort;

		si5351.PLL[rPLL].PLL_Clock_Source = PLL_Clock_Source_XTAL;	//select xrystal as clock input for the PLL
		si5351.PLL[rPLL].PLL_Multiplier_Integer = 32*128;				//multiply the clock frequency by 32, this gets us 800 MHz clock
		si5351.PLL[rPLL].PLL_Multiplier_Numerator = 1*8;
		si5351.PLL[rPLL].PLL_Multiplier_Denominator = xtal_freq;
		
		si5351.PLL[tPLL].PLL_Clock_Source = PLL_Clock_Source_XTAL;
		si5351.PLL[tPLL].PLL_Multiplier_Integer = 32*128;
		si5351.PLL[tPLL].PLL_Multiplier_Numerator = 1*8 + lo_freq/1000*8;
		si5351.PLL[tPLL].PLL_Multiplier_Denominator = xtal_freq;

		si5351.MS[rPort].MS_Clock_Source = MS_Clock_Source_PLLA;
		si5351.MS[rPort].MS_Divider_Integer = 8; // divide pll frequency by 8

		si5351.MS[tPort].MS_Clock_Source = MS_Clock_Source_PLLA;
		si5351.MS[tPort].MS_Divider_Integer = 8; // divide pll frequency by 8

		si5351.CLK[rPort].CLK_R_Div = CLK_R_Div1; // divide by 1; 100MHz
		si5351.CLK[rPort].CLK_Enable = ON;	//turn on the output
		si5351.CLK[rPort].CLK_I_Drv = CLK_I_Drv_8mA;

		if(pPort >= 0) {
			si5351.CLK[pPort].CLK_Clock_Source = CLK_Clock_Source_XTAL;
			si5351.CLK[pPort].CLK_R_Div = CLK_R_Div1; // divide by 1; 24MHz
			si5351.CLK[pPort].CLK_Enable = ON;	//turn on the output
			si5351.CLK[pPort].CLK_I_Drv = CLK_I_Drv_2mA;
		}

		si5351.CLK[tPort].CLK_R_Div = CLK_R_Div1; // divide by 1; 100MHz
		si5351.CLK[tPort].CLK_Enable = ON;	//turn on the output
		si5351.CLK[tPort].CLK_I_Drv = CLK_I_Drv_2mA;
		
		return si5351.Init() == 0;
	}
	static uint32_t gcd(uint32_t x, uint32_t y) {
		uint32_t z;
		while (y != 0) {
			z = x % y;
			x = y;
			y = z;
		}
		return x;
	}
	int si5351_set(uint32_t rxFreqHz, uint32_t txFreqHz) {
		using namespace Si5351;
		int ret = 0;
		CLKRDiv rDiv = CLK_R_Div1;

		// choose the same pll frequency and rdiv settings for both ports.
		// PLL should be configured between 600 and 900 MHz
		int pllFreqHz = 888000000;
		int divInputFreqHz = pllFreqHz;
		
		uint32_t mult = pllFreqHz/1000*128;
		uint32_t N = mult/xtal_freq;
		
		if(rxFreqHz < 500000) {
			rDiv = CLK_R_Div128;
			divInputFreqHz /= 128;
		} else if(rxFreqHz < 1000000) {
			rDiv = CLK_R_Div4;
			divInputFreqHz /= 4;
		} else if(rxFreqHz >= 100000000) {
			// div by 6 mode
			for(int i=0; i<2; i++) {
				uint32_t freqHz = (i == 0) ? rxFreqHz : txFreqHz;
				int pll = (i == 0) ? si5351_rxPLL : si5351_txPLL;
				int port = (i == 0) ? si5351_rxPort : si5351_txPort;
				
				// set msdiv to 6
				if(si5351.MS[port].MS_Divider_Integer != 6
						|| si5351.MS[port].MS_Divider_Numerator != 0) {
					si5351.MS[port].MS_Divider_Integer = 6;
					si5351.MS[port].MS_Divider_Numerator = 0;
					si5351.MS[port].MS_Divider_Denominator = 1;
					si5351.MS[port].MS_Clock_Source = (i == 1) ? MS_Clock_Source_PLLB : MS_Clock_Source_PLLA;
					si5351.MSConfig((MSChannel) port);
				}
				
				if(si5351.CLK[port].CLK_R_Div != rDiv) {
					si5351.CLK[port].CLK_R_Div = rDiv;
					si5351.CLKConfig((CLKChannel) port);
				}
				
				// calculate pll settings
				uint32_t mult = (freqHz/1000)*6*128;
				uint32_t N = mult/xtal_freq;
				uint32_t frac = mult - N*xtal_freq;

				si5351.PLL[pll].PLL_Multiplier_Integer = N;
				si5351.PLL[pll].PLL_Multiplier_Numerator = frac;
				si5351.PLL[pll].PLL_Multiplier_Denominator = xtal_freq;
				
				si5351.PLLConfig2((PLLChannel) pll);
				
				if(freqHz == 100000000) {
					printk("freq %d, N %d, frac %d\n", freqHz, N, frac);
				}
			}
			si5351.PLLReset2();
			
			return 2;
		}


		if(si5351.PLL[si5351_rxPLL].PLL_Multiplier_Integer != N
				|| si5351.PLL[si5351_rxPLL].PLL_Multiplier_Numerator != 0) {
			si5351.PLL[si5351_rxPLL].PLL_Multiplier_Integer = N;
			si5351.PLL[si5351_rxPLL].PLL_Multiplier_Numerator = 0;
			si5351.PLL[si5351_rxPLL].PLL_Multiplier_Denominator = 1;
			si5351.PLLConfig((PLLChannel) si5351_rxPLL);
			si5351.PLLReset((PLLChannel) si5351_rxPLL);
			ret = 2;
		}

		for(int i=0; i<2; i++) {
			uint32_t freqHz = (i == 0) ? rxFreqHz : txFreqHz;
			int port = (i == 0) ? si5351_rxPort : si5351_txPort;
			
			int32_t div = divInputFreqHz / freqHz; // range: 8 ~ 1800
			int32_t num = divInputFreqHz % freqHz;
			int32_t denom = freqHz;
			//int32_t k = freq / (1<<20) + 1;
			int32_t k = gcd(num, denom);
			num /= k;
			denom /= k;
			while (denom >= (1<<20)) {
				num >>= 1;
				denom >>= 1;
			}

			
			si5351.MS[port].MS_Divider_Integer = div;
			si5351.MS[port].MS_Divider_Numerator = num;
			si5351.MS[port].MS_Divider_Denominator = denom;
			si5351.MS[port].MS_Clock_Source = (si5351_rxPLL == 1) ? MS_Clock_Source_PLLB : MS_Clock_Source_PLLA;
			si5351.MSConfig((MSChannel) port);
			//printk("freq %d, div %d, num %d, denom %d\n", freq_khz, div, num, denom);

			if(si5351.CLK[port].CLK_R_Div != rDiv) {
				si5351.CLK[port].CLK_R_Div = rDiv;
				si5351.CLKConfig((CLKChannel) port);
				if(ret < 1) ret = 1;
			}
		}
		return ret;
	}
}
