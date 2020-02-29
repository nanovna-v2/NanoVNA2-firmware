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
	_sweep_points = 101;
	_cal_status   = 0;
	_electrical_delay = 0.0;
	_velocity_factor =  0.7;
	_active_marker   = 0;
	_domain_mode     = 0;
	_marker_smith_format = MS_RLC;

	memset(_cal_data, 0, sizeof(_cal_data));
	memcpy(_trace, def_trace, sizeof(_trace));
	memcpy(_markers, def_markers, sizeof(_markers));
}


float my_atof(const char *p)
{
  int neg = FALSE;
  if (*p == '-')
    neg = TRUE;
  if (*p == '-' || *p == '+')
    p++;
  float x = atoi(p);
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
