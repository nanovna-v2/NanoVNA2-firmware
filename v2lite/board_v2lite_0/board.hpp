#pragma once
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/spi.h>
#include <mculib/fastwiring.hpp>
#include <mculib/softi2c.hpp>
#include <mculib/softspi.hpp>
#include <mculib/adf4350.hpp>
#include <mculib/dma_adc.hpp>

#include <array>
#include <stdint.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/adc.h>

#include "../../rfsw.hpp"
#include "../../common.hpp"

#define BOARD_NAME "NanoVNA V2Lite_0"

using namespace mculib;
using namespace std;


namespace board {

	// ##### pin assignments #####

	static constexpr Pad led = PB5;
	static constexpr Pad led2 = PB6;

	static constexpr Pad USB0_DP = PA12;
	static constexpr Pad USB0_DM = PA11;

	static constexpr array<Pad, 2> RFSW_ECAL = {PC13, PC14};

	// ##### board parameters #####
	extern uint32_t adc_ratecfg;
	extern uint32_t adc_srate; // Hz
	extern uint32_t adc_period_cycles, adc_clk;

	constexpr uint32_t xtalFreqHz = 24000000;


	// ##### board peripherals #####

	// baseband ADC

	extern DMADriver dma;
	extern DMAChannel dmaChannelADC;
	extern DMAADC dmaADC;


	// synthesizers
	struct spiDelay_t {
		void operator()() {
			_delay_8t(1);
		}
	};

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



	// rf switch positions

	static constexpr auto RFSW_ECAL_SHORT = RFSWState::RF1;
	static constexpr auto RFSW_ECAL_OPEN = RFSWState::RF2;
	static constexpr auto RFSW_ECAL_LOAD = RFSWState::RF3;
	static constexpr auto RFSW_ECAL_NORMAL = RFSWState::RF4;


	// call this function at the beginning of main()
	void boardInit();

	// blink the status led
	void ledPulse();
}
