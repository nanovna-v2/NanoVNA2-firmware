#include "globals.hpp"
#include "ili9341.hpp"

volatile bool sweep_enabled = true;
int16_t vbat;

uint8_t registers[registerSize];

int16_t lastsaveid = 0;

config_t config = {
  .magic =             CONFIG_MAGIC,
  .dac_value =         1922,
  .grid_color =        DEFAULT_GRID_COLOR,
  .menu_normal_color = DEFAULT_MENU_COLOR,
  .menu_active_color = DEFAULT_MENU_ACTIVE_COLOR,
  .trace_color =       { DEFAULT_TRACE_1_COLOR, DEFAULT_TRACE_2_COLOR, DEFAULT_TRACE_3_COLOR, DEFAULT_TRACE_4_COLOR },
#ifndef DISPLAY_ST7796
  .touch_cal =         { 1950, 1900, -90, -120 },  //{ 620, 600, 160, 190 }, // 2.8 display module
#else
  .touch_cal =         { 1977, 1945, -61, -91 },   // 4' display module
#endif
  .default_loadcal =   0,
  .harmonic_freq_threshold = 300000000,
  .ui_options =        0,
  .checksum =          0
};

properties_t current_props;


properties_t* active_props = &current_props;


complexf measuredFreqDomain[2][SWEEP_POINTS_MAX] alignas(8);
complexf measured[2][SWEEP_POINTS_MAX] alignas(8);
complexf measuredEcal[ECAL_CHANNELS][USB_POINTS_MAX] alignas(8);

volatile EcalStates ecalState = ECAL_STATE_MEASURING;

uistat_t uistat = {
 digit: 6,
 digit_mode: 0,
 current_trace: 0,
 value: 0,
 previous_value: 0,
 lever_mode: LM_MARKER,
 previous_marker: 0,
 marker_search_mode: MarkerSearchModes::Max,
 marker_tracking: false,
 marker_delta: false
};

