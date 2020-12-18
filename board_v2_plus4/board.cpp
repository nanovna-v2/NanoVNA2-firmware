// this file must only be included by toplevel module

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
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
	uint32_t hseEstimateHz = 0;
	uint32_t adc_ratecfg = 0;
	uint32_t adc_srate = 0; // Hz
	uint32_t adc_period_cycles, adc_clk;

	DMADriver dma(DMA1);
	DMAChannel dmaChannelADC(dma, 1);
	DMAChannel dmaChannelSPI(dma, 3);
	DMAADC dmaADC(dmaChannelADC, ADC1);


	i2cDelay_t i2cDelay;
	spiDelay_t spiDelay;

	// TODO(gabu-chan): get rid of template argument after switch to c++17
	SoftI2C<i2cDelay_t> si5351_i2c(i2cDelay);
	Si5351::Si5351Driver si5351;

	SoftSPI<spiDelay_t> adf4350_tx_spi(spiDelay);
	SoftSPI<spiDelay_t> adf4350_rx_spi(spiDelay);

	ADF4350::ADF4350Driver<adf4350_sendWord_t> adf4350_tx(adf4350_sendWord_t {adf4350_tx_spi});
	ADF4350::ADF4350Driver<adf4350_sendWord_t> adf4350_rx(adf4350_sendWord_t {adf4350_rx_spi});

	XPT2046 xpt2046(xpt2046_irq);

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

	static void rcc_set_adcpre_gd32(uint32_t adcpre) {
		uint32_t RCC_CFGR_ADCPRE_MASK = (0b11 << 14) | (1 << 28);
		uint32_t RCC_CFGR2_ADCPRE_MASK = 1 << 29;
		uint32_t old = (RCC_CFGR & ~RCC_CFGR_ADCPRE_MASK);
		uint32_t old2 = (RCC_CFGR2 & ~RCC_CFGR2_ADCPRE_MASK);
		RCC_CFGR = old | ((adcpre & 0b11) << 14) | ((adcpre & 0b100) << (28 - 2));
		RCC_CFGR2 = old2 | ((adcpre & 0b1000) << (29 - 3));
	}

	/* TODO this code is a copy of code in libopencm3. 
	 * Should this move to libopencm3? */
	void rcc_clock_setup_in_hse_24mhz_out_96mhz(void) {
		 /* Enable internal high-speed oscillator. */
		 rcc_osc_on(RCC_HSI);
		 rcc_wait_for_osc_ready(RCC_HSI);

		 /* Select HSI as SYSCLK source. */
		 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

		 /* Enable external high-speed oscillator. */
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
		  * Set the PLL multiplication factor. (x4) 24 * 4 = 96Mhz
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

		 cpu_mhz = 96;
	}

	void rcc_clock_setup_in_hse_24mhz_out_120mhz(void) {
		 cpu_mhz = 120;
		 // only run the initialization once
		 if(RCC_CR & RCC_CR_HSEON) return;

		 /* Enable internal high-speed oscillator. */
		 rcc_osc_on(RCC_HSI);
		 rcc_wait_for_osc_ready(RCC_HSI);
		 
		 /* Enable external high-speed oscillator. */
		 rcc_osc_on(RCC_HSE);
		 rcc_wait_for_osc_ready(RCC_HSE);

		 /* Select HSE as SYSCLK source. */
		 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);
		 
		 rcc_osc_off(RCC_PLL);

		 /*
		  * Set prescalers for AHB, ADC, ABP1, ABP2.
		  * Do this before touching the PLL (TODO: why?).
		  */
		 rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);					// Set. 120MHz Max. 120MHz
		 rcc_set_adcpre_gd32(GD32_RCC_CFGR_ADCPRE_PCLK2_DIV4);		// Set. 30MHz Max. 40MHz
		 rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);					// Set. 60MHz Max. 60MHz
		 rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);					// Set. 120MHz Max. 120MHz
		 rcc_set_usbpre_gd32(2);									// 120MHz / 2.5 = 48MHz

		 /*
		  * Sysclk runs with 120MHz -> 0 waitstates.
		  */
		 flash_set_ws(FLASH_ACR_LATENCY_0WS);

		 /*
		  * Set the PLL multiplication factor. (x5) 24 * 5 = 120Mhz
		  */
		 rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL5);

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
		 rcc_ahb_frequency = 120000000;
		 rcc_apb1_frequency = 60000000;
		 rcc_apb2_frequency = 120000000;

	}

	void boardInit() {
		rcc_clock_setup_in_hse_24mhz_out_120mhz();

		// enable basic peripherals
		rcc_periph_clock_enable(RCC_GPIOA);
		rcc_periph_clock_enable(RCC_GPIOB);
		rcc_periph_clock_enable(RCC_GPIOC);

		rcc_periph_clock_enable(RCC_AFIO);
		// jtag pins should be used as GPIOs (SWD is used for debugging)
		gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_SPI1_REMAP);
		

		digitalWrite(ili9341_cs, HIGH);
		digitalWrite(xpt2046_cs, HIGH);
		pinMode(ili9341_dc, OUTPUT);
		pinMode(ili9341_cs, OUTPUT);
		pinMode(xpt2046_cs, OUTPUT);

		adc_ratecfg = ADC_SMPR_SMP_7DOT5CYC;
		adc_srate = 30000000/(7.5+12.5);
		adc_period_cycles = (7.5+12.5);
		adc_clk = 30000000;
	}


	// returns an estimate of the HSE frequency in Hz.
	// called by boardInit() to set hseEstimateHz.
	uint32_t detectHSEFreq() {
		rcc_osc_on(RCC_HSE);
		rcc_osc_on(RCC_HSI);
		rtc_auto_awake(RCC_HSE, 1 << 19);
		rtc_exit_config_mode();
		delay(2);
		uint32_t tmp = rtc_get_prescale_div_val();
		delay(20);
		uint32_t tmp2 = rtc_get_prescale_div_val();
		// cycles of a fHSE/128 clock elapsed
		uint32_t cycles = (tmp - tmp2) & ((1 << 20) - 1);
		uint32_t freqHz = cycles * 128 * 50;
		return freqHz;
	}

	void ledPulse() {
		digitalWrite(led2, HIGH);
		delayMicroseconds(1);
		digitalWrite(led2, LOW);
	}
	
	
	int calculateSynthWaitAF(freqHz_t freqHz) {
		if(freqHz < 1200000000) return  8;
		if(freqHz < 2200000000) return 10;
		if(freqHz < 3200000000) return 14;
		return 18;
	}

	int calculateSynthWaitSI(int retval) {
		switch(retval) {
			case 0: return 36;
			case 1: return 120;
			case 2: return 120;
		}
		return 5;
	}

	void lcd_spi_init() {
		dmaChannelSPI.enable();
		gpio_set_mode(lcd_clk.bank(), GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, lcd_clk.mask());
		gpio_set_mode(lcd_mosi.bank(), GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, lcd_mosi.mask());
		gpio_set_mode(lcd_miso.bank(), GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, lcd_miso.mask());

		rcc_periph_clock_enable(RCC_SPI1);
		/* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
		spi_reset(SPI1);

		/* Set up SPI in Master mode with:
		* Clock baud rate: 1/64 of peripheral clock frequency
		* Clock polarity: Idle High
		* Clock phase: Data valid on 1st clock pulse
		* Data frame format: 16-bit
		* Frame format: MSB First
		*/
		spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
						SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

		/*
		* Set NSS management to software.
		*
		* Note:
		* Setting nss high is very important, even if we are controlling the GPIO
		* ourselves this bit needs to be at least set to 1, otherwise the spi
		* peripheral will not send any data out.
		*/
		spi_enable_software_slave_management(SPI1);
		spi_set_nss_high(SPI1);
		
		spi_enable_tx_dma(SPI1);

		// Set to tx+rx mode
		spi_set_unidirectional_mode(SPI1);

		/* Enable SPI1 periph. */
		spi_enable(SPI1);
	}
	void lcd_spi_write() {
		spi_set_baudrate_prescaler(SPI1,SPI_CR1_BR_FPCLK_DIV_4);
	}
	void lcd_spi_read() {
		spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_8);
	}
	void lcd_spi_slow() {
		spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_16);
	}
	void spi_drop_read() {
		(void)SPI_DR(SPI1); // Cleanup read buffers in SPI hardware
		(void)SPI_DR(SPI1);
	}
	bool lcd_spi_isDMAInProgress = false;
	
	void lcd_spi_waitDMA() {
		if(!lcd_spi_isDMAInProgress)
			return;

		// wait for dma finish
		while(!dmaChannelSPI.finished());
		dmaChannelSPI.stop();

		// wait for all ongoing transfers to complete
		while (!(SPI_SR(SPI1) & SPI_SR_TXE));
		while ((SPI_SR(SPI1) & SPI_SR_BSY));
		
		lcd_spi_isDMAInProgress = false;
//		delayMicroseconds(10);
	}
	
	uint32_t lcd_spi_transfer(uint32_t sdi, int bits) {
		if(lcd_spi_isDMAInProgress)
			lcd_spi_waitDMA();
		spi_drop_read();
		uint32_t ret = spi_xfer(SPI1, (uint16_t) sdi);
		if(bits == 16) {
			ret|= uint32_t(spi_xfer(SPI1, (uint16_t) (sdi >> 8))) << 8;
		}
		return ret;
	}

	void lcd_spi_transfer_bulk(uint8_t* buf, int bytes) {
		if(lcd_spi_isDMAInProgress)
			lcd_spi_waitDMA();

		lcd_spi_isDMAInProgress = true;
		DMATransferParams srcParams, dstParams;
		srcParams.address = buf;
		srcParams.bytesPerWord = 1;
		srcParams.increment = true;

		dstParams.address = &SPI_DR(SPI1);
		dstParams.bytesPerWord = 1;
		dstParams.increment = false;

		dmaChannelSPI.setTransferParams(srcParams, dstParams,
								DMADirection::MEMORY_TO_PERIPHERAL,
								bytes, false);
		dmaChannelSPI.start();
		//lcd_spi_waitDMA();
	}

	void lcd_spi_read_bulk(uint8_t* buf, int bytes) {
		if(lcd_spi_isDMAInProgress)
			lcd_spi_waitDMA();
		// Switch to read speed
		lcd_spi_read();
		// Drop old data in rx buffers
		spi_drop_read();
#if 0
		lcd_spi_isDMAInProgress = true;
		DMATransferParams srcParams, dstParams;
		srcParams.address = &SPI_DR(SPI1);
		srcParams.bytesPerWord = 1;
		srcParams.increment = false;

		dstParams.address = buf;
		dstParams.bytesPerWord = 1;
		dstParams.increment = true;

		dmaChannelSPI.setTransferParams(srcParams, dstParams,
								DMADirection::PERIPHERAL_TO_MEMORY,
								bytes, false);
		dmaChannelSPI.start();
		lcd_spi_waitDMA();
#else
		do{
			*buf++= spi_xfer(SPI1, (uint16_t)0);
		}while(--bytes);
#endif
		// Switch back to normal
		lcd_spi_write();
	}
}
