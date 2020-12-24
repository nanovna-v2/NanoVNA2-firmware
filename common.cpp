#include "common.hpp"
#include <string.h>

#define TRUE true
#define FALSE false

// NanoVNA Default settings
static const trace_t def_trace[TRACES_MAX] = {//enable, type, channel, reserved, scale, refpos
    { 1, TRC_LOGMAG, 0, 0, 1.0, 7.0 },
    { 1, TRC_LOGMAG, 1, 0, 1.0, 7.0 },
    { 1, TRC_SMITH,  0, 1, 1.0, 0.0 },
    { 1, TRC_PHASE,  0, 0, 1.0, 4.0 }
};

static const marker_t def_markers[MARKERS_MAX] = {
  { 1, 30, 0 }, { 0, 40, 0 }, { 0, 60, 0 }, { 0, 80, 0 }
};

void properties_t::setFieldsToDefault() {
	magic = CONFIG_MAGIC;
	_frequency0   = 100000000;    // start = 100MHz
	_frequency1   = 900000000;    // end   = 900MHz
	_sweep_points = SWEEP_POINTS_MAX;
	_cal_status   = 0;
	_electrical_delay = 0.0;
	_velocity_factor =  0.7;
	_active_marker   = 0;
	_domain_mode     = 0;
	_marker_smith_format = MS_RLC;
	_avg = 1;
	_adf4350_txPower = 3;
	_si5351_txPower = 1;
	_measurement_mode = MEASURE_MODE_FULL;

	setCalDataToDefault();
	memcpy(_trace, def_trace, sizeof(_trace));
	memcpy(_markers, def_markers, sizeof(_markers));
}

void properties_t::setCalDataToDefault() {
  _cal_status = 0;
  do_cal_reset(CAL_LOAD, 0.f);
  do_cal_reset(CAL_OPEN, 1.f);
  do_cal_reset(CAL_SHORT, -1.f);
  do_cal_reset(CAL_THRU, 1.f);
  do_cal_reset(CAL_ISOLN_OPEN, 0.f);
  do_cal_reset(CAL_ISOLN_SHORT, 0.f);
  do_cal_reset(CAL_THRU_REFL, 0.f);
}
void properties_t::do_cal_reset(int calType, complexf val) {
  complexf* arr = _cal_data[calType];
  for(int i=0; i<SWEEP_POINTS_MAX; i++)
    arr[i] = val;
}

float my_atof(const char *p)
{
  int neg = FALSE;
  if (*p == '-')
    neg = TRUE;
  if (*p == '-' || *p == '+')
    p++;
  float x = atoll(p);
  while (isdigit((int)*p))
    p++;
  if (*p == '.') {
    float d = 1.0f;
    p++;
    while (isdigit((int)*p)) {
      d /= 10;
      x += d * (*p - '0');
      p++;
    }
  }
  if (*p == 'e' || *p == 'E') {
    p++;
    int exp = atoi(p);
    while (exp > 0) {
      x *= 10;
      exp--;
    }
    while (exp < 0) {
      x /= 10;
      exp++;
    }
  }
  if (neg)
    x = -x;
  return x;
}
