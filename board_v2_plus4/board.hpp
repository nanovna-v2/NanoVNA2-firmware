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

#define BOARD_NAME "NanoVNA V2Plus4"
#define BOARD_REVISION (4)
#define BOARD_REVISION_MAGIC 0xdeadbabf
#define USB_POINTS_MAX 65536
// Plus4 not use ecal mode
#define BOARD_DISABLE_ECAL

using namespace mculib;
using namespace std;

// This not used in Plus4 code, need only for bootloader
#define BOARD_MEASUREMENT_NPERIODS_NORMAL		14
#define BOARD_MEASUREMENT_NPERIODS_CALIBRATING	30
#define BOARD_MEASUREMENT_ECAL_INTERVAL			 5
#define BOARD_MEASUREMENT_NWAIT_SWITCH			 1
#define BOARD_MEASUREMENT_MIN_CALIBRATION_AVG	10
#define BOARD_MEASUREMENT_MAX_CALIBRATION_AVG	255
#define BOARD_MEASUREMENT_FIRST_POINT_WAIT     128

namespace board {

	// ##### pin assignments #####

	static constexpr Pad led = PA6;
	static constexpr Pad led2 = PA7;
	static constexpr Pad USB0_DP = PA12;
	static constexpr Pad USB0_DM = PA11;

	static constexpr array<Pad, 2> RFSW_ECAL = {PC13, PC14};
	static constexpr array<Pad, 2> RFSW_BBGAIN = {PB15, PB14};
	static constexpr Pad RFSW_TXSYNTH = PA5;
	static constexpr Pad RFSW_RXSYNTH = PA10;
	static constexpr Pad RFSW_REFL = PA8;
	static constexpr Pad RFSW_RECV = PA9;

	
	static constexpr Pad lcd_clk = PB3;
	static constexpr Pad lcd_mosi = PB5;
	static constexpr Pad lcd_miso = PB4;
	static constexpr Pad ili9341_cs = PA15;
	static constexpr Pad ili9341_dc = PB6;
	static constexpr Pad xpt2046_cs = PB7;
	static constexpr Pad xpt2046_irq = PB8;

	static constexpr Pad LEVER_LEFT = PB11;
	static constexpr Pad LEVER_CENTER = PB12;
	static constexpr Pad LEVER_RIGHT = PB13;
	static constexpr bool LEVER_POLARITY = false; // pin level when lever/button is pressed

	// ##### board parameters #####
	
	// estimated HSE frequency in Hz, set by boardInit()
	extern uint32_t hseEstimateHz;

	// All boards use a 24Mhz TCXO. It gives best phase noise with the ADF4350
	static constexpr uint32_t xtalFreqHz = 24000000;
	static constexpr freqHz_t DEFAULT_FREQ = 2600000000;

	// ADC parameters, set by boardInit()
	extern uint32_t adc_ratecfg;
	extern uint32_t adc_srate; // Hz
	extern uint32_t adc_period_cycles, adc_clk;
	constexpr int adc_rxChannel = 0;

	// the end of flash memory. User data is stored before this point.
	constexpr uint32_t USERFLASH_END = 0x08000000 + 256*1024;


	// ##### board peripherals #####

	extern DMADriver dma;
	extern DMAChannel dmaChannelADC;
	extern DMAADC dmaADC;

	// synthesizers

	struct i2cDelay_t {
		void operator()() {
			_delay_8t(5);
		}
	};
	struct spiDelay_t {
		void operator()() {
			_delay_8t(2);
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

	constexpr int si5351_rxPLL = 0, si5351_txPLL = 1;
	constexpr int si5351_rxPort = 0, si5351_txPort = 2, si5351_passthruPort = -1;

	// lcd display

	extern XPT2046 xpt2046;

	// rf switch positions

	static constexpr auto RFSW_ECAL_SHORT = RFSWState::RF4;
	static constexpr auto RFSW_ECAL_OPEN = RFSWState::RF3;
	static constexpr auto RFSW_ECAL_LOAD = RFSWState::RF2;
	static constexpr auto RFSW_ECAL_NORMAL = RFSWState::RF1;

	static constexpr int RFSW_TXSYNTH_LF = 0;
	static constexpr int RFSW_TXSYNTH_HF = 1;

	static constexpr int RFSW_RXSYNTH_LF = 1;
	static constexpr int RFSW_RXSYNTH_HF = 0;

	static constexpr int RFSW_REFL_ON = 1;
	static constexpr int RFSW_REFL_OFF = 0;

	static constexpr int RFSW_RECV_REFL = 0;
	static constexpr int RFSW_RECV_PORT2 = 1;

	static constexpr int RFSW_BBGAIN_MAX = 3;

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

	// returns an estimate of the HSE frequency in Hz.
	// called by boardInit() to set hseEstimateHz.
	uint32_t detectHSEFreq();

	// blink the status led
	void ledPulse();
	
	int calculateSynthWaitAF(freqHz_t freqHz);
	int calculateSynthWaitSI(int retval);

	// sets up hardware spi for ili9341 and touch.
	// spi peripheral only manages clk, sdi, and sdo.
	void lcd_spi_init();

	// three speed presets for ili9341 write/read and touch controller
	void lcd_spi_write();
	void lcd_spi_read();
	void lcd_spi_slow();

	// bits must be 16 or 8
	uint32_t lcd_spi_transfer(uint32_t sdi, int bits);

	void lcd_spi_transfer_bulk(uint8_t* buf, int bytes);

	void lcd_spi_read_bulk(uint8_t* buf, int bytes);

	// wait for all bulk transfers to complete
	void lcd_spi_waitDMA();
}
