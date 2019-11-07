#pragma once
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/spi.h>
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

#include "../rfsw.hpp"
#include "../common.hpp"
#include "../xpt2046.hpp"

#define BOARD_NAME "NanoVNA V2_0"

using namespace mculib;
using namespace std;


namespace board {

	// ##### pin assignments #####

	static constexpr Pad led = PA9;
	static constexpr Pad led2 = PA10;

	static constexpr array<Pad, 2> RFSW_ECAL = {PC13, PC14};
	static constexpr array<Pad, 2> RFSW_BBGAIN = {PB13, PB12};
	static constexpr Pad RFSW_TXSYNTH = PB9;
	static constexpr Pad RFSW_RXSYNTH = PA4;
	static constexpr Pad RFSW_REFL = PB0;
	static constexpr Pad RFSW_RECV = PB1;

	
	static constexpr Pad lcd_clk = PB3;
	static constexpr Pad lcd_mosi = PB5;
	static constexpr Pad lcd_miso = PB4;
	static constexpr Pad ili9341_cs = PA15;
	static constexpr Pad ili9341_dc = PB6;
	static constexpr Pad xpt2046_cs = PB7;
	static constexpr Pad xpt2046_irq = PB8;

	static constexpr Pad LEVER_LEFT = PB14;
	static constexpr Pad LEVER_CENTER = PB15;
	static constexpr Pad LEVER_RIGHT = PA8;
	static constexpr bool LEVER_POLARITY = false; // pin level when lever/button is pressed

	// ##### board parameters #####
	extern uint32_t adc_ratecfg;
	extern uint32_t adc_srate; // Hz
	extern uint32_t adc_period_cycles, adc_clk;



	// ##### board peripherals #####

	// baseband ADC

	extern DMADriver dma;
	extern DMAChannel dmaChannelADC;
	extern DMAADC dmaADC;


	// synthesizers

	struct i2cDelay_t {
		void operator()() {
			_delay_8t(10);
		}
	};
	struct spiDelay_t {
		void operator()() {
			_delay_8t(5);
		}
	};

	extern SoftI2C<i2cDelay_t> si5351_i2c;
	extern Si5351::Si5351Driver si5351;

	extern SoftSPI<spiDelay_t> adf4350_tx_spi;
	extern SoftSPI<spiDelay_t> adf4350_rx_spi;

	struct adf4350_sendWord_t {
		SoftSPI<spiDelay_t>& spi;
		void operator()(uint32_t word) {
			spi.beginTransfer();
			spi.doTransfer_send(word, 32);
			spi.endTransfer();
		}
	};
	extern ADF4350::ADF4350Driver<adf4350_sendWord_t> adf4350_tx;
	extern ADF4350::ADF4350Driver<adf4350_sendWord_t> adf4350_rx;



	// lcd display

	extern XPT2046 xpt2046;

	// rf switch positions

	static constexpr auto RFSW_ECAL_SHORT = RFSWState::RF2;
	static constexpr auto RFSW_ECAL_OPEN = RFSWState::RF3;
	static constexpr auto RFSW_ECAL_LOAD = RFSWState::RF4;
	static constexpr auto RFSW_ECAL_NORMAL = RFSWState::RF1;

	static constexpr int RFSW_TXSYNTH_LF = 0;
	static constexpr int RFSW_TXSYNTH_HF = 1;

	static constexpr int RFSW_RXSYNTH_LF = 1;
	static constexpr int RFSW_RXSYNTH_HF = 0;

	static constexpr int RFSW_REFL_ON = 1;
	static constexpr int RFSW_REFL_OFF = 0;

	static constexpr int RFSW_RECV_REFL = 1;
	static constexpr int RFSW_RECV_PORT2 = 0;

	// gain is an integer from 0 to 3, 0 being lowest gain
	static inline RFSWState RFSW_BBGAIN_GAIN(int gain) {
		switch(gain) {
			case 0: return RFSWState::RF1;
			case 1: return RFSWState::RF2;
			case 2: return RFSWState::RF3;
			case 3: return RFSWState::RF4;
			default: return RFSWState::RF4;
		}
		return RFSWState::RF4;
	}

	// call this function at the beginning of main()
	void boardInit();

	// blink the status led
	void ledPulse();

	// initialize and configure si5351
	bool si5351_setup();

	// set si5351 frequency for tx or rx port
	void si5351_set(bool isRX, uint32_t freq_khz);

	// sets up hardware spi for ili9341 and touch.
	// spi peripheral only manages clk, sdi, and sdo.
	void lcd_spi_init();

	// two speed presets for ili9341 and touch controller
	void lcd_spi_fast();
	void lcd_spi_slow();

	// bits must be 16 or 8
	uint32_t lcd_spi_transfer(uint32_t sdi, int bits);

	void lcd_spi_transfer_bulk(uint8_t* buf, int bytes);
	
	// wait for all bulk transfers to complete
	void lcd_spi_waitDMA();
}
