#pragma once
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

#include "rfsw.hpp"

using namespace mculib;
using namespace std;

Pad led = PA9;
Pad led2 = PA10;

array<Pad, 2> RFSW_ECAL = {PC13, PC14};
array<Pad, 2> RFSW_BBGAIN = {PB13, PB12};
Pad RFSW_TXSYNTH = PB9;
Pad RFSW_RXSYNTH = PA4;
Pad RFSW_REFL = PB0;
Pad RFSW_RECV = PB1;

auto RFSW_ECAL_SHORT = RFSWState::RF2;
auto RFSW_ECAL_OPEN = RFSWState::RF3;
auto RFSW_ECAL_LOAD = RFSWState::RF4;
auto RFSW_ECAL_NORMAL = RFSWState::RF1;

int RFSW_TXSYNTH_LF = 0;
int RFSW_TXSYNTH_HF = 1;

int RFSW_RXSYNTH_LF = 1;
int RFSW_RXSYNTH_HF = 0;

int RFSW_REFL_ON = 1;
int RFSW_REFL_OFF = 0;

int RFSW_RECV_REFL = 1;
int RFSW_RECV_PORT2 = 0;


// set by board_init()
uint32_t adc_ratecfg = 0;
uint32_t adc_srate = 0; // Hz
uint32_t adc_period_cycles, adc_clk;
uint8_t registers[32];


DMADriver dma(DMA1);
DMAChannel dmaChannel(dma, 1);
DMAADC dmaADC(dmaChannel, ADC1);


auto i2cDelay = []() {
	/*for(int i=0;i<5000000;i++) {
		asm __volatile__ ( "nop" );
	}*/
	delayMicroseconds(1);
};
auto spiDelay = []() {
	delayMicroseconds(1);
};
// TODO(gabu-chan): get rid of template argument after switch to c++17
SoftI2C<decltype(i2cDelay)> si5351_i2c(i2cDelay);
Si5351::Si5351Driver si5351;

SoftSPI<decltype(spiDelay)> adf4350_tx_spi(spiDelay);
SoftSPI<decltype(spiDelay)> adf4350_rx_spi(spiDelay);
auto adf4350_tx_sendWord = [](uint32_t word) {
	adf4350_tx_spi.beginTransfer();
	adf4350_tx_spi.doTransfer_send(word, 32);
	adf4350_tx_spi.endTransfer();
};
auto adf4350_rx_sendWord = [](uint32_t word) {
	adf4350_rx_spi.beginTransfer();
	adf4350_rx_spi.doTransfer_send(word, 32);
	adf4350_rx_spi.endTransfer();
};
ADF4350::ADF4350Driver<decltype(adf4350_tx_sendWord)>
	adf4350_tx(adf4350_tx_sendWord);
ADF4350::ADF4350Driver<decltype(adf4350_rx_sendWord)>
	adf4350_rx(adf4350_rx_sendWord);


// gain is an integer from 0 to 3, 0 being lowest gain
static inline RFSWState RFSW_BBGAIN_GAIN(int gain) {
	switch(gain) {
		case 0: return RFSWState::RF1;
		case 1: return RFSWState::RF2;
		case 2: return RFSWState::RF3;
		case 3: return RFSWState::RF4;
		default: return RFSWState::RF4;
	}
}

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

static inline void boardInit() {
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

	adf4350_tx_spi.init();
	adf4350_rx_spi.init();

	adc_ratecfg = ADC_SMPR_SMP_7DOT5CYC;
	adc_srate = 6000000/(7.5+12.5);
	adc_period_cycles = (7.5+12.5);
	adc_clk = 6000000;
}


