#pragma once
#include <stdint.h>
#include "common.hpp"

// global variables, to be defined in main.c that can be accessed by all modules.
// DO NOT ADD FUNCTIONS HERE.
// think thrice before adding anything here.

// TODO(gabu-chan): eliminate these variables, make each module provide hooks
// for events that can be acted upon rather than let modules directly
// modify global state.

extern uint16_t redraw_request;
extern int8_t sweep_enabled;


extern int16_t vbat;

extern int16_t lastsaveid;
extern properties_t *active_props;
extern properties_t current_props;

extern int8_t previous_marker;


extern config_t config;


extern complexf measured[2][SWEEP_POINTS_MAX];

extern uistat_t uistat;

#define frequency0 current_props._frequency0
#define frequency1 current_props._frequency1
#define sweep_points current_props._sweep_points
#define cal_status current_props._cal_status
#define frequencies current_props._frequencies
#define cal_data active_props->_cal_data
#define electrical_delay current_props._electrical_delay

#define trace current_props._trace
#define markers current_props._markers
#define active_marker current_props._active_marker
#define domain_mode current_props._domain_mode
#define velocity_factor current_props._velocity_factor



static inline float get_trace_refpos(int t) {
  return trace[t].refpos;
}


static inline float get_trace_scale(int t) {
  return trace[t].scale * trace_info[trace[t].type].scale_unit;
}

static inline const char* get_trace_typename(int t) {
  return trace_info[trace[t].type].name;
}


