// this file must only be included by toplevel module

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <mculib/fastwiring.hpp>
#include <mculib/softi2c.hpp>
#include <mculib/softspi.hpp>
#include <mculib/si5351.hpp>
#include <mculib/adf4350.hpp>
#include <mculib/dma_adc.hpp>

#include <array>
#include <stdint.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/adc.h>

#include "board.hpp"
#include "../rfsw.hpp"

using namespace mculib;
using namespace std;


namespace board {

	// set by board_init()
	uint32_t adc_ratecfg = 0;
	uint32_t adc_srate = 0; // Hz
	uint32_t adc_period_cycles, adc_clk;
	uint8_t registers[32];


	DMADriver dma(DMA1);
	DMAChannel dmaChannel(dma, 1);
	DMAADC dmaADC(dmaChannel, ADC1);


	i2cDelay_t i2cDelay;
	spiDelay_t spiDelay;
	spiDelay_fast_t spiDelay_fast;

	// TODO(gabu-chan): get rid of template argument after switch to c++17
	SoftI2C<i2cDelay_t> si5351_i2c(i2cDelay);
	Si5351::Si5351Driver si5351;

	SoftSPI<spiDelay_t> adf4350_tx_spi(spiDelay);
	SoftSPI<spiDelay_t> adf4350_rx_spi(spiDelay);


	ADF4350::ADF4350Driver<adf4350_sendWord_t> adf4350_tx(adf4350_sendWord_t {adf4350_tx_spi});
	ADF4350::ADF4350Driver<adf4350_sendWord_t> adf4350_rx(adf4350_sendWord_t {adf4350_rx_spi});

	SoftSPI<spiDelay_fast_t> ili9341_spi(spiDelay_fast);


	// same as rcc_set_usbpre, but with extended divider range:
	// 0: divide by 1.5
	// 1: divide by 1
	// 2: divide by 2.5
	// 3: divide by 2
	void rcc_set_usbpre_gd32(uint32_t usbpre) {
		uint32_t RCC_CFGR_USBPRE_MASK = uint32_t(0b11) << 22;
		uint32_t old = (RCC_CFGR & ~RCC_CFGR_USBPRE_MASK);
		RCC_CFGR = old | ((usbpre & 0b11) << 22);
	}

	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV2 = 0b0000;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV4 = 0b0001;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV6 = 0b0010;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV8 = 0b0011;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV12 = 0b0101;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV16 = 0b0111;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV5 = 0b1000;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV6 = 0b1001;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV10 = 0b1010;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV20 = 0b1011;

	void rcc_set_adcpre_gd32(uint32_t adcpre) {
		uint32_t RCC_CFGR_ADCPRE_MASK = (0b11 << 14) | (1 << 28);
		uint32_t RCC_CFGR2_ADCPRE_MASK = 1 << 29;
		uint32_t old = (RCC_CFGR & ~RCC_CFGR_ADCPRE_MASK);
		uint32_t old2 = (RCC_CFGR2 & ~RCC_CFGR2_ADCPRE_MASK);
		RCC_CFGR = old | ((adcpre & 0b11) << 14) | ((adcpre & 0b100) << (28 - 2));
		RCC_CFGR2 = old2 | ((adcpre & 0b1000) << (29 - 3));
	}

	void rcc_clock_setup_in_hse_24mhz_out_96mhz(void)
	{
		 /* Enable internal high-speed oscillator. */
		 rcc_osc_on(RCC_HSI);
		 rcc_wait_for_osc_ready(RCC_HSI);

		 /* Select HSI as SYSCLK source. */
		 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

		 /* Enable external high-speed oscillator 24MHz. */
		 rcc_osc_on(RCC_HSE);
		 rcc_wait_for_osc_ready(RCC_HSE);
		 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

		 /*
		  * Set prescalers for AHB, ADC, ABP1, ABP2.
		  * Do this before touching the PLL (TODO: why?).
		  */
		 rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);					// Set. 96MHz Max. 96MHz
		 rcc_set_adcpre_gd32(GD32_RCC_CFGR_ADCPRE_PCLK2_DIV16);		// Set. 6MHz Max. 40MHz
		 rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);					// Set. 48MHz Max. 60MHz
		 rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);					// Set. 96MHz Max. 120MHz
		 rcc_set_usbpre_gd32(3);									// 96MHz / 2 = 48MHz

		 /*
		  * Sysclk runs with 96MHz -> 0 waitstates.
		  */
		 flash_set_ws(FLASH_ACR_LATENCY_0WS);

		 /*
		  * Set the PLL multiplication factor to 4.
		  * 24MHz (external) * 4 (multiplier) = 96MHz
		  */
		 rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL4);

		 /* Select HSE as PLL source. */
		 rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

		 /*
		  * External frequency undivided before entering PLL
		  * (only valid/needed for HSE).
		  */
		 rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

		 /* Enable PLL oscillator and wait for it to stabilize. */
		 rcc_osc_on(RCC_PLL);
		 rcc_wait_for_osc_ready(RCC_PLL);

		 /* Select PLL as SYSCLK source. */
		 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

		 /* Set the peripheral clock frequencies used */
		 rcc_ahb_frequency = 96000000;
		 rcc_apb1_frequency = 48000000;
		 rcc_apb2_frequency = 96000000;
	}

	void boardInit() {
		rcc_clock_setup_in_hse_24mhz_out_96mhz();
		cpu_mhz = 96;
		
		// enable basic peripherals
		rcc_periph_clock_enable(RCC_GPIOA);
		rcc_periph_clock_enable(RCC_GPIOB);
		rcc_periph_clock_enable(RCC_GPIOC);

		rcc_periph_clock_enable(RCC_AFIO);
		// jtag pins should be used as GPIOs (SWD is used for debugging)
		gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON,0);
		
		si5351_i2c.clk = PB10;
		si5351_i2c.sda = PB11;

		adf4350_tx_spi.sel = PA2;
		adf4350_tx_spi.clk = PA7;
		adf4350_tx_spi.mosi = PA6;
		adf4350_tx_spi.miso = PA3;

		adf4350_rx_spi.sel = PA5;
		adf4350_rx_spi.clk = PA7;
		adf4350_rx_spi.mosi = PA6;
		adf4350_rx_spi.miso = PA3;

		ili9341_spi.sel = PA15;
		ili9341_spi.clk = PB3;
		ili9341_spi.mosi = PB5;
		ili9341_spi.miso = PB4;

		adf4350_tx_spi.init();
		adf4350_rx_spi.init();
		ili9341_spi.init();
		/*digitalWrite(ili9341_spi.sel, LOW);
		digitalWrite(ili9341_spi.clk, LOW);
		digitalWrite(ili9341_spi.mosi, LOW);
		digitalWrite(ili9341_dc, LOW);*/
		pinMode(ili9341_dc, OUTPUT);
		

		adc_ratecfg = ADC_SMPR_SMP_7DOT5CYC;
		adc_srate = 6000000/(7.5+12.5);
		adc_period_cycles = (7.5+12.5);
		adc_clk = 6000000;
	}


	void ledPulse() {
		digitalWrite(led2, HIGH);
		delayMicroseconds(1);
		digitalWrite(led2, LOW);
	}


	bool si5351_setup() {
		using namespace Si5351;
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
		
		return si5351.Init() == 0;
	}
	void si5351_set(bool isRX, uint32_t freq_khz) {
		using namespace Si5351;

		int i = isRX ? 0 : 2;
		int pll = isRX ? 0 : 1;

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

}
