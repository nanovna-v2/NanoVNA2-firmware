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

#include "../rfsw.hpp"

#define BOARD_NAME "NanoVNA V2_0"

using namespace mculib;
using namespace std;

static constexpr Pad led = PA9;
static constexpr Pad led2 = PA10;

static constexpr array<Pad, 2> RFSW_ECAL = {PC13, PC14};
static constexpr array<Pad, 2> RFSW_BBGAIN = {PB13, PB12};
static constexpr Pad RFSW_TXSYNTH = PB9;
static constexpr Pad RFSW_RXSYNTH = PA4;
static constexpr Pad RFSW_REFL = PB0;
static constexpr Pad RFSW_RECV = PB1;

static constexpr Pad ili9341_cs = PA15;
static constexpr Pad ili9341_dc = PB6;

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


// set by board_init()
extern uint32_t adc_ratecfg;
extern uint32_t adc_srate; // Hz
extern uint32_t adc_period_cycles, adc_clk;
extern uint8_t registers[32];


extern DMADriver dma;
extern DMAChannel dmaChannel;
extern DMAADC dmaADC;


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

void boardInit();

void ledPulse();

