#pragma once
#include <stdint.h>
#include <complex>
#include <array>

using namespace std;


// common data structures, constants, and convenience functions.
// DO NOT ADD NON-CONSTANT VARIABLES OR STATEFUL FUNCTIONS HERE.


typedef complex<float> complexf;

// port 1 in, port 1 out, port 2 in
typedef array<complexf, 3> VNAObservation;

// a complete observation set for one frequency point
// (measured waves for excitations at each port).
// currently a single VNAObservation, but will be
// an array of 2 observations for a full two port VNA.
typedef VNAObservation VNAObservationSet;

// S11, S21
typedef array<complexf, 2> VNARawValue;

typedef int64_t freqHz_t;


// constants and data types used by all modules.
// DO NOT DECLARE EXTERNAL FUNCTIONS HERE!!!


#define VERSION "git"
#define FIRMWARE_MAJOR_VERSION 1
#define FIRMWARE_MINOR_VERSION 1
#define CH_KERNEL_VERSION "None"
#define PORT_COMPILER_NAME "gcc"
#define PORT_ARCHITECTURE_NAME "arm"
#define PORT_INFO "GD32F303"
#define PORT_CORE_VARIANT_NAME "N/A"
#define PLATFORM_NAME "BARE METAL"


#define FREQUENCY_MIN 50000
#define FREQUENCY_MAX 4400000000
#define SWEEP_POINTS_MIN 2
#define SWEEP_POINTS_MAX 201
#define USB_POINTS_MAX 1024
#define TRACES_MAX 4
#define MARKERS_MAX 4
#define FFT_SIZE 512
#define ECAL_PARTIAL

#ifdef ECAL_PARTIAL
#define ECAL_CHANNELS 1
#else
#define ECAL_CHANNELS 3
#endif


#define CAL_LOAD 0
#define CAL_OPEN 1
#define CAL_SHORT 2
#define CAL_THRU 3
#define CAL_ISOLN_OPEN 4
#define CAL_ISOLN_SHORT 5

#define CALSTAT_LOAD (1<<0)
#define CALSTAT_OPEN (1<<1)
#define CALSTAT_SHORT (1<<2)
#define CALSTAT_THRU (1<<3)
#define CALSTAT_ISOLN (1<<4)
#define CALSTAT_ES (1<<5)
#define CALSTAT_ER (1<<6)
#define CALSTAT_ET (1<<7)
#define CALSTAT_ED CALSTAT_LOAD
#define CALSTAT_EX CALSTAT_ISOLN
#define CALSTAT_APPLY (1<<8)
#define CALSTAT_INTERPOLATED (1<<9)

#define ETERM_ED 0 /* error term directivity */
#define ETERM_ES 1 /* error term source match */
#define ETERM_ER 2 /* error term refrection tracking */
#define ETERM_ET 3 /* error term transmission tracking */
#define ETERM_EX 4 /* error term isolation */

#define DOMAIN_MODE (1<<0)
#define DOMAIN_FREQ (0<<0)
#define DOMAIN_TIME (1<<0)
#define TD_FUNC (0b11<<1)
#define TD_FUNC_BANDPASS (0b00<<1)
#define TD_FUNC_LOWPASS_IMPULSE (0b01<<1)
#define TD_FUNC_LOWPASS_STEP    (0b10<<1)
#define TD_WINDOW (0b11<<3)
#define TD_WINDOW_NORMAL (0b00<<3)
#define TD_WINDOW_MINIMUM (0b01<<3)
#define TD_WINDOW_MAXIMUM (0b10<<3)


#define REDRAW_CELLS      (1<<0)
#define REDRAW_FREQUENCY  (1<<1)
#define REDRAW_CAL_STATUS (1<<2)
#define REDRAW_MARKER     (1<<3)



constexpr uint32_t BOOTLOADER_DFU_MAGIC = 0xdeadbabe;
static volatile uint32_t& bootloaderDFUIndicator = *(uint32_t*)(0x20000000 + 48*1024 - 4);

constexpr int MEASUREMENT_NPERIODS_NORMAL = 14;
constexpr int MEASUREMENT_NPERIODS_CALIBRATING = 30;
constexpr int MEASUREMENT_ECAL_INTERVAL = 5;


// TODO: name all enums and refer to them by name

enum {
  TRC_LOGMAG, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_OFF
};

enum SweepParameter {
  ST_START, ST_STOP, ST_CENTER, ST_SPAN, ST_CW
};

// lever_mode
enum {
  LM_MARKER, LM_SEARCH, LM_CENTER, LM_SPAN
};

// marker smith value format
enum {
  MS_LIN, MS_LOG, MS_REIM, MS_RX, MS_RLC
};

enum MarkerSearchModes: uint8_t {
	Min,
	Max
};

enum {
	UI_OPTIONS_FLIP = 1
};


typedef struct {
  uint8_t enabled;
  uint8_t type;
  uint8_t channel;
  uint8_t polar;
  float scale;
  float refpos;
} trace_t;

typedef struct {
  int8_t enabled;
  int16_t index;
  freqHz_t frequency;
} marker_t;

struct alignas(4) properties_t {
  uint32_t magic;
  freqHz_t _frequency0; // start
  freqHz_t _frequency1; // stop
  int16_t _sweep_points;
  uint16_t _cal_status;

  complexf _cal_data[6][SWEEP_POINTS_MAX];
  float _electrical_delay; // picoseconds
  
  trace_t _trace[TRACES_MAX];
  marker_t _markers[MARKERS_MAX];
  float _velocity_factor; // %
  int _active_marker;
  uint8_t _domain_mode; /* 0bxxxxxffm : where ff: TD_FUNC m: DOMAIN_MODE */
  uint8_t _marker_smith_format;

  int32_t checksum;

  // overwrite all fields of this instance with factory default values
  void setFieldsToDefault();

  properties_t() { setFieldsToDefault(); }
};


typedef struct {
  uint32_t magic;
  uint16_t dac_value; // unused
  uint16_t grid_color;
  uint16_t menu_normal_color;
  uint16_t menu_active_color;
  uint16_t trace_color[TRACES_MAX];
  int16_t touch_cal[4];
  int8_t default_loadcal;
  uint32_t harmonic_freq_threshold; // unused
  int8_t ui_options;
  int32_t checksum;
} config_t;

struct uistat_t {
  int8_t digit; /* 0~5 */
  int8_t digit_mode;
  int8_t current_trace; /* 0..3 */
  int64_t value; // for editing at numeric input area
  int64_t previous_value;
  uint8_t lever_mode;
  int8_t previous_marker;
  MarkerSearchModes marker_search_mode;
  bool marker_tracking;
  bool marker_delta;
};

#define CONFIG_MAGIC 0x80081235



// convert vbat [mV] to battery indicator
static inline uint8_t vbat2bati(int16_t vbat)
{
	if (vbat < 3200) return 0;
	if (vbat < 3450) return 25;
	if (vbat < 3700) return 50;
	if (vbat < 4100) return 75;
	return 100;
}



static const struct {
  const char *name;
  uint16_t refpos;
  float scale_unit;
} trace_info[] = {
  { "LOGMAG", 7, 10 },
  { "PHASE",  4, 90 },
  { "DELAY",  4,  1 },
  { "SMITH",  0,  1 },
  { "POLAR",  0,  1 },
  { "LINEAR", 0,  0.125 },
  { "SWR",    0,  1 },
  { "REAL",   4,  0.25 },
  { "IMAG",   4,  0.25 },
  { "R",      0, 100 },
  { "X",      4, 100 }
};

static const char * const trc_channel_name[] = {
  "CH0", "CH1"
};


float my_atof(const char *p);


