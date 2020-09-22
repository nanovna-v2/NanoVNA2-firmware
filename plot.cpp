#include <math.h>
#include <string.h>
#include "common.hpp"
#include "globals.hpp"
#include "plot.hpp"
#include "ili9341.hpp"
#include "Font.h"
#include <board.hpp>
#include <mculib/printf.hpp>
#include "ui.hpp"

#define TRUE true
#define FALSE false
#define SWAP(x,y) do { int z=x; x = y; y = z; } while(0)
#define PULSE board::ledPulse()


static void cell_draw_marker_info(int x0, int y0);
void frequency_string(char *buf, size_t len, freqHz_t freq);
void frequency_string_short(char *buf, size_t len, freqHz_t freq, char prefix);
void markmap_all_markers(void);

//#define GRID_COLOR 0x0863
//uint16_t grid_color = 0x1084;

uint16_t redraw_request = 0;

int32_t fgrid = 50000000;
int16_t grid_offset;
int16_t grid_width;

int area_width = AREA_WIDTH_NORMAL;
int area_height = AREA_HEIGHT_NORMAL;

bool plot_canceled = false;

small_function<freqHz_t(int)> plot_getFrequencyAt;
small_function<void()> plot_tick;

// alias for plot_getFrequencyAt
static inline freqHz_t freqAt(int i) {
	return plot_getFrequencyAt(i);
}

#define GRID_RECTANGULAR (1<<0)
#define GRID_SMITH       (1<<1)
#define GRID_ADMIT       (1<<2)
#define GRID_POLAR       (1<<3)


// Depends from spi_buffer size, CELLWIDTH*CELLHEIGHT*sizeof(pixel) <= sizeof(spi_buffer)
#define CELLWIDTH  (32)
#define CELLHEIGHT (32)
// Check buffer size
#if CELLWIDTH*CELLHEIGHT > SPI_BUFFER_SIZE
#error "Too small spi_buffer size SPI_BUFFER_SIZE < CELLWIDTH*CELLHEIGH"
#endif

#define INDEX(x, y) ((((uint32_t)x)<<16)|(((uint32_t)y)))
#define CELL_X(i)  (int)(((i)>>16))
#define CELL_Y(i)  (int)(((i)&0xFFFF))

// indicate dirty cells (not redraw if cell data not changed)
#define MAX_MARKMAP_X    ((LCD_WIDTH+CELLWIDTH-1)/CELLWIDTH)
#define MAX_MARKMAP_Y    ((LCD_HEIGHT+CELLHEIGHT-1)/CELLHEIGHT)
// Define markmap mask size
#if MAX_MARKMAP_X <= 8
typedef uint8_t map_t;
#elif MAX_MARKMAP_X <= 16
typedef uint16_t map_t;
#elif MAX_MARKMAP_X <= 32
typedef uint32_t map_t;
#endif

map_t   markmap[2][MAX_MARKMAP_Y];
uint16_t current_mappage = 0;

uint32_t trace_index[TRACES_MAX][SWEEP_POINTS_MAX];

//#define float2int(v) ((int)(v))
static int
float2int(float v)
{
	if (v < 0) return v - 0.5;
	if (v > 0) return v + 0.5;
	return 0;
}

void recalculate_grid(void)
{
	// no horizontal grid in zero-span mode
	if (frequency1 == 0) {
		grid_offset = 0;
		grid_width = 10 * WIDTH;
		return;
	}

	freqHz_t gdigit = 100000000;
	freqHz_t fstart, fspan;
	freqHz_t grid;
	if (frequency1 > 0) {
		fstart = frequency0;
		fspan = frequency1 - frequency0;
	} else {
		fspan = -frequency1;
		fstart = frequency0 - fspan/2;
	}

	while (gdigit > 100) {
		grid = 5 * gdigit;
		if (fspan / grid >= 4)
			break;
		grid = 2 * gdigit;
		if (fspan / grid >= 4)
			break;
		grid = gdigit;
		if (fspan / grid >= 4)
			break;
		gdigit /= 10;
	}
	fgrid = grid;

	grid_offset = WIDTH * ((fstart % fgrid) / 100) / (fspan / 100);
	grid_width = WIDTH * (fgrid / 100) / (fspan / 1000);
}

void update_grid(void)
{
	recalculate_grid();
	force_set_markmap();
	redraw_request |= REDRAW_FREQUENCY;
}

static inline int
circle_inout(int x, int y, int r)
{
	int d = x*x + y*y - r*r;
	if (d <= -r)
		return 1;
	if (d > r)
		return -1;
	return 0;
}

static int
polar_grid(int x, int y)
{
	int c = config.grid_color;
	int d;

	// offset to center
	x -= P_CENTER_X;
	y -= P_CENTER_Y;

	// outer circle
	d = circle_inout(x, y, P_RADIUS);
	if (d < 0) return 0;
	if (d == 0) return c;

	// vertical and horizontal axis
	if (x == 0 || y == 0)
		return c;

	d = circle_inout(x, y, P_RADIUS / 5);
	if (d == 0) return c;
	if (d > 0) return 0;

	d = circle_inout(x, y, P_RADIUS * 2 / 5);
	if (d == 0) return c;
	if (d > 0) return 0;

	// cross sloping lines
	if (x == y || x == -y)
		return c;

	d = circle_inout(x, y, P_RADIUS * 3 / 5);
	if (d == 0) return c;
	if (d > 0) return 0;

	d = circle_inout(x, y, P_RADIUS * 4 / 5);
	if (d == 0) return c;
	return 0;
}

/*
 * Constant Resistance circle: (u - r/(r+1))^2 + v^2 = 1/(r+1)^2
 * Constant Reactance circle:  (u - 1)^2 + (v-1/x)^2 = 1/x^2
 */
static int
smith_grid(int x, int y)
{
	int d;

	// offset to center
	x -= P_CENTER_X;
	y -= P_CENTER_Y;

	// outer circle
	d = circle_inout(x, y, P_RADIUS);
	if (d < 0) return 0;
	if (d == 0) return 1;

	// horizontal axis
	if (y == 0) return 1;

	// shift circle center to right origin
	x -= P_RADIUS;

	// Constant Reactance Circle: 2j : R/2 = P_RADIUS/2
	if (circle_inout(x, y + P_RADIUS / 2, P_RADIUS / 2) == 0) return 1;
	if (circle_inout(x, y - P_RADIUS / 2, P_RADIUS / 2) == 0) return 1;

	// Constant Resistance Circle: 3 : R/4 = P_RADIUS/4
	d = circle_inout(x + P_RADIUS / 4, y, P_RADIUS / 4);
	if (d > 0) return 0;
	if (d == 0) return 1;

	// Constant Reactance Circle: 1j : R = P_RADIUS
	if (circle_inout(x, y + P_RADIUS, P_RADIUS) == 0) return 1;
	if (circle_inout(x, y - P_RADIUS, P_RADIUS) == 0) return 1;

	// Constant Resistance Circle: 1 : R/2
	d = circle_inout(x + P_RADIUS / 2, y, P_RADIUS / 2);
	if (d > 0) return 0;
	if (d == 0) return 1;

	// Constant Reactance Circle: 1/2j : R*2
	if (circle_inout(x, y + P_RADIUS * 2, P_RADIUS * 2) == 0) return 1;
	if (circle_inout(x, y - P_RADIUS * 2, P_RADIUS * 2) == 0) return 1;

	// Constant Resistance Circle: 1/3 : R*3/4
	if (circle_inout(x + P_RADIUS * 3 / 4, y, P_RADIUS * 3 / 4) == 0) return 1;
	return 0;
}

int
smith_grid2(int x, int y, float scale)
{
	int d;

	// offset to center
	x -= P_CENTER_X;
	y -= P_CENTER_Y;

	// outer circle
	d = circle_inout(x, y, P_RADIUS);
	if (d < 0)
		return 0;
	if (d == 0)
		return 1;

	// shift circle center to right origin
	x -= P_RADIUS * scale;

	// Constant Reactance Circle: 2j : R/2 = 58
	if (circle_inout(x, y+58*scale, 58*scale) == 0)
		return 1;
	if (circle_inout(x, y-58*scale, 58*scale) == 0)
		return 1;
#if 0
	// Constant Resistance Circle: 3 : R/4 = 29
	d = circle_inout(x+29*scale, y, 29*scale);
	if (d > 0) return 0;
	if (d == 0) return 1;
	d = circle_inout(x-29*scale, y, 29*scale);
	if (d > 0) return 0;
	if (d == 0) return 1;
#endif

	// Constant Reactance Circle: 1j : R = 116
	if (circle_inout(x, y+116*scale, 116*scale) == 0)
		return 1;
	if (circle_inout(x, y-116*scale, 116*scale) == 0)
		return 1;

	// Constant Resistance Circle: 1 : R/2 = 58
	d = circle_inout(x+58*scale, y, 58*scale);
	if (d > 0) return 0;
	if (d == 0) return 1;
	d = circle_inout(x-58*scale, y, 58*scale);
	if (d > 0) return 0;
	if (d == 0) return 1;

	// Constant Reactance Circle: 1/2j : R*2 = 232
	if (circle_inout(x, y+232*scale, 232*scale) == 0)
		return 1;
	if (circle_inout(x, y-232*scale, 232*scale) == 0)
		return 1;

#if 0
	// Constant Resistance Circle: 1/3 : R*3/4 = 87
	d = circle_inout(x+87*scale, y, 87*scale);
	if (d > 0) return 0;
	if (d == 0) return 1;
	d = circle_inout(x+87*scale, y, 87*scale);
	if (d > 0) return 0;
	if (d == 0) return 1;
#endif

	// Constant Resistance Circle: 0 : R
	d = circle_inout(x+P_RADIUS*scale, y, P_RADIUS*scale);
	if (d > 0) return 0;
	if (d == 0) return 1;
	d = circle_inout(x-P_RADIUS*scale, y, P_RADIUS*scale);
	if (d > 0) return 0;
	if (d == 0) return 1;

	// Constant Resistance Circle: -1/3 : R*3/2 = 174
	d = circle_inout(x+174*scale, y, 174*scale);
	if (d > 0) return 0;
	if (d == 0) return 1;
	d = circle_inout(x-174*scale, y, 174*scale);
	//if (d > 0) return 0;
	if (d == 0) return 1;
	return 0;
}


const int cirs[][4] = {
	{ 0, 58/2, 58/2, 0 },    // Constant Reactance Circle: 2j : R/2 = 58
	{ 29/2, 0, 29/2, 1 },    // Constant Resistance Circle: 3 : R/4 = 29
	{ 0, 116/2, 116/2, 0 },  // Constant Reactance Circle: 1j : R = 116
	{ 58/2, 0, 58/2, 1 },    // Constant Resistance Circle: 1 : R/2 = 58
	{ 0, 232/2, 232/2, 0 },  // Constant Reactance Circle: 1/2j : R*2 = 232
	{ 87/2, 0, 87/2, 1 },    // Constant Resistance Circle: 1/3 : R*3/4 = 87
	{ 0, 464/2, 464/2, 0 },  // Constant Reactance Circle: 1/4j : R*4 = 464
	{ 116/2, 0, 116/2, 1 },  // Constant Resistance Circle: 0 : R
	{ 174/2, 0, 174/2, 1 },  // Constant Resistance Circle: -1/3 : R*3/2 = 174
	{ 0, 0, 0, 0 } // sentinel
};

int
smith_grid3(int x, int y)
{
	int d;

	// offset to center
	x -= P_CENTER_X;
	y -= P_CENTER_Y;

	// outer circle
	d = circle_inout(x, y, P_RADIUS);
	if (d < 0)
		return 0;
	if (d == 0)
		return 1;

	// shift circle center to right origin
	x -= P_RADIUS /2;

	int i;
	for (i = 0; cirs[i][2]; i++) {
		d = circle_inout(x+cirs[i][0], y+cirs[i][1], cirs[i][2]);
		if (d == 0)
			return 1;
		if (d > 0 && cirs[i][3])
			return 0;
		d = circle_inout(x-cirs[i][0], y-cirs[i][1], cirs[i][2]);
		if (d == 0)
			return 1;
		if (d > 0 && cirs[i][3])
			return 0;
	}
	return 0;
}

#if 0
int
rectangular_grid(int x, int y)
{
	int c = config.grid_color;
	//#define FREQ(x) (((x) * (fspan / 1000) / (WIDTH)) * 1000 + fstart)
	//int32_t n = FREQ(x-1) / fgrid;
	//int32_t m = FREQ(x) / fgrid;
	//if ((m - n) > 0)
	//if (((x * 6) % (WIDTH)) < 6)
	//if (((x - grid_offset) % grid_width) == 0)
	if (x == 0 || x == WIDTH-1)
		return c;
	if ((y % GRIDY) == 0)
		return c;
	if ((((x + grid_offset) * 10) % grid_width) < 10)
		return c;
	return 0;
}
#endif

int
rectangular_grid_x(int x)
{
	x -= CELLOFFSETX;
	if (x < 0)
		return 0;
	if (x == 0 || x == WIDTH)
		return 1;
	if ((((x + grid_offset) * 10) % grid_width) < 10)
		return 1;
	return 0;
}

int
rectangular_grid_y(int y)
{
	if (y < 0)
		return 0;
	if ((y % GRIDY) == 0)
		return 1;
	return 0;
}

/*
 * calculate log10(abs(gamma))
 */
float logmag(complexf v) {
	float re = v.real(), im = v.imag();
	return log10f(re*re + im*im) * 10;
}

/*
 * calculate phase[-2:2] of coefficient
 */
float phase(complexf v) {
	float re = v.real(), im = v.imag();
	return 2 * atan2f(im, re) / M_PI * 90;
}

/*
 * calculate groupdelay
 */
float groupdelay(complexf v, complexf w, float deltaf) {
	// calculate atan(w)-atan(v)
	complexf q = w / v;
	return arg(q) / (2 * M_PI * deltaf);
}

/*
 * calculate abs(gamma)
 */
float linear(complexf v) {
	float re = v.real(), im = v.imag();
	return - sqrtf(re*re + im*im);
}

/*
 * calculate vswr; (1+gamma)/(1-gamma)
 */
float swr(complexf v) {
	float re = v.real(), im = v.imag();
	float x = sqrtf(re*re + im*im);
	if (x > 1)
		return INFINITY;
	return (1 + x)/(1 - x);
}

float resistance(complexf v) {
	float re = v.real(), im = v.imag();
	float z0 = 50;
	float d = z0 / ((1-re)*(1-re)+im*im);
	float zr = ((1+re)*(1-re) - im*im) * d;
	return zr;
}

float reactance(complexf v) {
	float re = v.real(), im = v.imag();
	float z0 = 50;
	float d = z0 / ((1-re)*(1-re)+im*im);
	float zi = 2*im * d;
	return zi;
}

static float
qualityfactor(complexf v)
{
  float re = v.real(), im = v.imag();
  float i = 2*im;
  float r = (1+re)*(1-re) - im*im;
  return fabs(i / r);
}

void
cartesian_scale(float re, float im, int *xp, int *yp, float scale)
{
	//float scale = 4e-3;
	int x = float2int(re * P_RADIUS * scale);
	int y = float2int(im * P_RADIUS * scale);
	if (x < -P_RADIUS) x = -P_RADIUS;
	if (y < -P_RADIUS) y = -P_RADIUS;
	if (x > P_RADIUS) x = P_RADIUS;
	if (y > P_RADIUS) y = P_RADIUS;
	*xp = WIDTH/2 + x;
	*yp = HEIGHT/2 - y;
}

float
groupdelay_from_array(int i, complexf array[SWEEP_POINTS_MAX])
{
	if (i == 0) {
		float deltaf = freqAt(1) - freqAt(0);
		return groupdelay(array[0], array[1], deltaf);
	} else if (i == (SWEEP_POINTS_MAX - 1)) {
		float deltaf = freqAt(i) - freqAt(i-1);
		return groupdelay(array[i-1], array[i], deltaf);
	} else {
		float deltaf = freqAt(i+1) - freqAt(i-1);
		return groupdelay(array[i-1], array[i+1], deltaf);
	}
}

uint32_t
trace_into_index(int x, int t, int i, complexf array[SWEEP_POINTS_MAX])
{
	int y = 0;
	float v = 0;
	float refpos = 8 - get_trace_refpos(t);
	float scale = 1 / get_trace_scale(t);
	complexf coeff = array[i];
	switch (trace[t].type) {
	case TRC_LOGMAG:
		v = refpos - logmag(coeff) * scale;
		break;
	case TRC_PHASE:
		v = refpos - phase(coeff) * scale;
		break;
	case TRC_DELAY:
		v = refpos - groupdelay_from_array(i, array) * scale;
		break;
	case TRC_LINEAR:
		v = refpos + linear(coeff) * scale;
		break;
	case TRC_SWR:
		v = refpos+ (1 - swr(coeff)) * scale;
		break;
	case TRC_REAL:
		v = refpos - coeff.real() * scale;
		break;
	case TRC_IMAG:
		v = refpos - coeff.imag() * scale;
		break;
	case TRC_R:
		v = refpos - resistance(coeff) * scale;
		break;
	case TRC_X:
		v = refpos - reactance(coeff) * scale;
		break;
	case TRC_Q:
		v = refpos - qualityfactor(coeff) * scale;
		break;

	case TRC_SMITH:
	//case TRC_ADMIT:
	case TRC_POLAR:
		cartesian_scale(coeff.real(), coeff.imag(), &x, &y, scale);
		return INDEX(x +CELLOFFSETX, y);
		break;
	}
	if (v < 0) v = 0;
	if (v > 8) v = 8;
	y = float2int(v * GRIDY);
	return INDEX(x +CELLOFFSETX, y);
}

static int
string_value_with_prefix(char *buf, int len, float val, char unit)
{
	char prefix;
	int n = 0;
	if (val < 0) {
		val = -val;
		*buf = '-';
		n++;
		len--;
	}
	if (val < 1e-12) {
		prefix = 'f';
		val *= 1e15;
	} else if (val < 1e-9) {
		prefix = 'p';
		val *= 1e12;
	} else if (val < 1e-6) {
		prefix = 'n';
		val *= 1e9;
	} else if (val < 1e-3) {
		prefix = S_MICRO[0];
		val *= 1e6;
	} else if (val < 1) {
		prefix = 'm';
		val *= 1e3;
	} else if (val < 1e3) {
		prefix = 0;
	} else if (val < 1e6) {
		prefix = 'k';
		val /= 1e3;
	} else if (val < 1e9) {
		prefix = 'M';
		val /= 1e6;
	} else {
		prefix = 'G';
		val /= 1e9;
	}

	if (val < 10) {
		n += chsnprintf(&buf[n], len, "%.2f", val);
	} else if (val < 100) {
		n += chsnprintf(&buf[n], len, "%.1f", val);
	} else {
		n += chsnprintf(&buf[n], len, "%d", (int)val);
	}

	if (prefix)
		buf[n++] = prefix;
	if (unit)
		buf[n++] = unit;
	buf[n] = '\0';
	return n;
}


#define PI2 6.283184

static void
format_smith_value(char *buf, int len, complexf coeff, freqHz_t frequency)
{
	float re = coeff.real(), im = coeff.imag();
	// z = (gamma+1)/(gamma-1) * z0
	float z0 = 50;
	float d = z0 / ((1-re)*(1-re)+im*im);
	float zr = ((1+re)*(1-re) - im*im) * d;
	float zi = 2*im * d;
	int n;

	switch (marker_smith_format) {
	case MS_LIN:
		chsnprintf(buf, len, "%.2f %.1f" S_DEGREE, linear(coeff), phase(coeff));
		break;
		case MS_LOG: {
			float v = logmag(coeff);
			if (v == -INFINITY)
				chsnprintf(buf, len, "-INF dB");
			else
				chsnprintf(buf, len, "%.1fdB %.1f" S_DEGREE, v, phase(coeff));
		}
		break;

	case MS_REIM:
		n = string_value_with_prefix(buf, len, coeff.real(), '\0');
		if (coeff.imag() >= 0) buf[n++] = '+';
		string_value_with_prefix(buf+n, len-n, coeff.imag(), 'j');
		break;

	case MS_RX:
		n = string_value_with_prefix(buf, len, zr, S_OHM[0]);
		if (zi >= 0)
			buf[n++] = ' ';
		string_value_with_prefix(buf+n, len-n, zi, 'j');
		break;

	case MS_RLC:
		n = string_value_with_prefix(buf, len, zr, S_OHM[0]);
		buf[n++] = ' ';

		if (zi < 0) {
			float c = -1 / (PI2 * frequency * zi);
			string_value_with_prefix(buf+n, len-n, c, 'F');
		} else {
			float l = zi / (PI2 * frequency);
			string_value_with_prefix(buf+n, len-n, l, 'H');
		}
		break;
	}
}

static void
gamma2resistance(char *buf, int len, complexf coeff)
{
	float re = coeff.real(), im = coeff.imag();
	float z0 = 50;
	float d = z0 / ((1-re)*(1-re)+im*im);
	float zr = ((1+re)*(1-re) - im*im) * d;
	string_value_with_prefix(buf, len, zr, S_OHM[0]);
}

static void
gamma2reactance(char *buf, int len, complexf coeff)
{
	float re = coeff.real(), im = coeff.imag();
	float z0 = 50;
	float d = z0 / ((1-re)*(1-re)+im*im);
	float zi = 2*im * d;
	string_value_with_prefix(buf, len, zi, S_OHM[0]);
}

static void
trace_get_value_string(int t, char *buf, int len, complexf array[SWEEP_POINTS_MAX], int i)
{
	float v;
	complexf coeff = array[i];
	switch (trace[t].type) {
	case TRC_LOGMAG:
		v = logmag(coeff);
		if (v == -INFINITY)
			chsnprintf(buf, len, "-INF dB");
		else
			chsnprintf(buf, len, "%.2fdB", v);
		break;
	case TRC_PHASE:
		v = phase(coeff);
		chsnprintf(buf, len, "%.2f" S_DEGREE, v);
		break;
	case TRC_DELAY:
		v = groupdelay_from_array(i, array);
		string_value_with_prefix(buf, len, v, 's');
		break;
	case TRC_LINEAR:
		v = linear(coeff);
		chsnprintf(buf, len, "%.2f", v);
		break;
	case TRC_SWR:
		v = swr(coeff);
		chsnprintf(buf, len, "%.2f", v);
		break;
	case TRC_SMITH:
		format_smith_value(buf, len, coeff, freqAt(i));
		break;
	case TRC_REAL:
		chsnprintf(buf, len, "%.2f", coeff.real());
		break;
	case TRC_IMAG:
		chsnprintf(buf, len, "%.2fj", coeff.imag());
		break;
	case TRC_R:
		gamma2resistance(buf, len, coeff);
		break;
	case TRC_X:
		gamma2reactance(buf, len, coeff);
		break;
	case TRC_Q:
		chsnprintf(buf, len, "%.2f", qualityfactor(coeff));
		break;
	//case TRC_ADMIT:
	case TRC_POLAR:
		chsnprintf(buf, len, "%.2f %.2fj", coeff.real(), coeff.imag());
		break;
	}
}


static void
trace_get_value_string_delta(int t, char *buf, int len, complexf array[SWEEP_POINTS_MAX], int index, int index_ref)
{
	complexf coeff = array[index];
	complexf coeff_ref = array[index_ref];
	float v;
	switch (trace[t].type) {
	case TRC_LOGMAG:
		v = logmag(coeff) - logmag(coeff_ref);
		if (v == -INFINITY)
			chsnprintf(buf, len, S_DELTA "-INF dB");
		else
			chsnprintf(buf, len, S_DELTA "%.2fdB", v);
		break;
	case TRC_PHASE:
		v = phase(coeff) - phase(coeff_ref);
		chsnprintf(buf, len, S_DELTA "%.2f" S_DEGREE, v);
		break;
	case TRC_DELAY:
		v = groupdelay_from_array(index, array) - groupdelay_from_array(index_ref, array);
		string_value_with_prefix(buf, len, v, 's');
		break;
	case TRC_LINEAR:
		v = linear(coeff) - linear(coeff_ref);
		chsnprintf(buf, len, S_DELTA "%.2f", v);
		break;
	case TRC_SWR:
		v = swr(coeff) - swr(coeff_ref);
		chsnprintf(buf, len, S_DELTA "%.2f", v);
		break;
	case TRC_SMITH:
		format_smith_value(buf, len, coeff, freqAt(index));
		break;
	case TRC_REAL:
		chsnprintf(buf, len, S_DELTA "%.2f", coeff.real() - coeff_ref.real());
		break;
	case TRC_IMAG:
		chsnprintf(buf, len, S_DELTA "%.2fj", coeff.imag() - coeff_ref.imag());
		break;
	case TRC_R:
		gamma2resistance(buf, len, coeff);
		break;
	case TRC_X:
		gamma2reactance(buf, len, coeff);
		break;
	case TRC_Q:
		chsnprintf(buf, len, S_DELTA "%.2f", qualityfactor(coeff) - qualityfactor(coeff_ref));
		break;
	//case TRC_ADMIT:
	case TRC_POLAR:
		chsnprintf(buf, len, "%.2f %.2fj", coeff.real(), coeff.imag());
		break;
	}
}


void
trace_get_info(int t, char *buf, int len)
{
	const char *type = get_trace_typename(t);
	int n;
	switch (trace[t].type) {
	case TRC_LOGMAG:
		chsnprintf(buf, len, "%s %ddB/", type, (int)get_trace_scale(t));
		break;
	case TRC_PHASE:
		chsnprintf(buf, len, "%s %d" S_DEGREE "/", type, (int)get_trace_scale(t));
		break;
	case TRC_SMITH:
	//case TRC_ADMIT:
	case TRC_POLAR:
		chsnprintf(buf, len, "%s %.1fFS", type, get_trace_scale(t));
		break;
	default:
		n = chsnprintf(buf, len, "%s ", type);
		string_value_with_prefix(buf+n, len-n, get_trace_scale(t), '/');
		break;
	}
}

static float time_of_index(int idx) {
	 return 1.0 / (float)(plot_getFrequencyAt(1) - plot_getFrequencyAt(0)) / (float)FFT_SIZE * idx;
}

static float distance_of_index(int idx) {
#define SPEED_OF_LIGHT 299792458
	 float distance = ((float)idx * (float)SPEED_OF_LIGHT) / ( (float)(plot_getFrequencyAt(1) - plot_getFrequencyAt(0)) * (float)FFT_SIZE * 2.0);
	 return distance * velocity_factor;
}


static inline void
mark_map(int x, int y)
{
	if (y >= 0 && y < MAX_MARKMAP_Y && x >= 0 && x < MAX_MARKMAP_X)
		markmap[current_mappage][y] |= 1 << x;
}

static inline void
swap_markmap(void)
{
	current_mappage^= 1;
}

static inline void
clear_markmap(void)
{
	memset(markmap[current_mappage], 0, sizeof markmap[current_mappage]);
}

void
force_set_markmap(void)
{
	memset(markmap[current_mappage], 0xff, sizeof markmap[current_mappage]);
}

void
invalidate_rect(int x0, int y0, int x1, int y1)
{
	x0 /= CELLWIDTH;
	x1 /= CELLWIDTH;
	y0 /= CELLHEIGHT;
	y1 /= CELLHEIGHT;
	int x, y;
	for (y = y0; y <= y1; y++)
		for (x = x0; x <= x1; x++)
			mark_map(x, y);
}

static inline void
markmap_upperarea(void)
{
	// Hardcoded, Text info from upper area
	invalidate_rect(0, 0, AREA_WIDTH_NORMAL, 3*FONT_STR_HEIGHT);
}

static void
mark_cells_from_index(void)
{
	int t, i, j;
	/* mark cells between each neighber points */
	map_t *map = &markmap[current_mappage][0];
	for (t = 0; t < TRACES_MAX; t++) {
		if (!trace[t].enabled)
			continue;
		uint32_t *index = &trace_index[t][0];
		int m0 = CELL_X(index[0]) / CELLWIDTH;
		int n0 = CELL_Y(index[0]) / CELLHEIGHT;
		map[n0] |= 1 << m0;
		for (i = 1; i < sweep_points; i++) {
		int m1 = CELL_X(index[i]) / CELLWIDTH;
		int n1 = CELL_Y(index[i]) / CELLHEIGHT;
		if (m0 == m1 && n0 == n1)
			continue;
		int x0 = m0; int x1 = m1; if (x0>x1) SWAP(x0, x1); m0 = m1;
		int y0 = n0; int y1 = n1; if (y0>y1) SWAP(y0, y1); n0 = n1;
		for (; y0 <= y1; y0++)
			for (j = x0; j <= x1; j++) {
				if(y0 >= 0 && y0 < MAX_MARKMAP_Y)
					map[y0] |= 1 << j;
			}
		}
	}
}

void plot_into_index(complexf measured[2][SWEEP_POINTS_MAX])
{
	mark_cells_from_index();
	int i, t;
	for (i = 0; i < sweep_points; i++) {
		int x = i * WIDTH / (sweep_points-1);
		for (t = 0; t < TRACES_MAX; t++) {
			if (!trace[t].enabled)
				continue;
			int n = trace[t].channel;
			trace_index[t][i] = trace_into_index(x, t, i, measured[n]);
		}
	}

	mark_cells_from_index();
	markmap_all_markers();
	redraw_request |= REDRAW_CELLS;
}

//
// in most cases _compute_outcode clip calculation not give render line speedup
//
static inline void
cell_drawline(int x0, int y0, int x1, int y1, int c)
{
	if (x0 < 0 && x1 < 0) return;
	if (y0 < 0 && y1 < 0) return;
	if (x0 >= CELLWIDTH && x1 >= CELLWIDTH) return;
	if (y0 >= CELLHEIGHT && y1 >= CELLHEIGHT) return;

	// modifed Bresenham's line algorithm, see https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	if (x1 < x0) { SWAP(x0, x1); SWAP(y0, y1); }
	int dx = x1 - x0;
	int dy = y1 - y0, sy = 1; if (dy < 0) { dy = -dy; sy = -1; }
	int err = (dx > dy ? dx : -dy) / 2;

	while (1) {
		if (y0 >= 0 && y0 < CELLHEIGHT && x0 >= 0 && x0 < CELLWIDTH)
			ili9341_spi_buffer[y0 * CELLWIDTH + x0] |= c;
		if (x0 == x1 && y0 == y1)
			return;
		int e2 = err;
		if (e2 > -dx) { err -= dy; x0++;  }
		if (e2 <  dy) { err += dx; y0+=sy;}
	}
}

// Give a little speedup then draw rectangular plot (50 systick on all calls, all render req 700 systick)
// Write more difficult algoritm for seach indexes not give speedup
static int
search_index_range_x(int x1, int x2, uint32_t *index, int *i0, int *i1)
{
	int i, j;
	int head = 0;
	int tail = sweep_points;
	int idx_x;

	// Search index point in cell
	while (1) {
		i = (head + tail) / 2;
		idx_x = CELL_X(index[i]);
		if (idx_x >= x2) { // index after cell
			if (tail == i)
				return false;
			tail = i;
		}
		else if (idx_x < x1) {    // index before cell
			if (head == i)
				return false;
			head = i;
		}
		else  // index in cell (x =< idx_x < cell_end)
			break;
	}
	j = i;
	// Search index left from point
	do {
		j--;
	} while (j > 0 && x1 <= CELL_X(index[j]));
	*i0 = j;
	// Search index right from point
	do {
		i++;
	} while (i < sweep_points-1 && CELL_X(index[i]) < x2);
	*i1 = i;

	return TRUE;
}

#define REFERENCE_WIDTH    6
#define REFERENCE_HEIGHT   5
#define REFERENCE_X_OFFSET 5
#define REFERENCE_Y_OFFSET 2

// Reference bitmap
static const uint8_t reference_bitmap[]={
  0b11000000,
  0b11110000,
  0b11111100,
  0b11110000,
  0b11000000,
};

static void
draw_refpos(int x, int y, int c)
{
	int y0 = y, j;
	for (j = 0; j < REFERENCE_HEIGHT; j++, y0++) {
		if (y0 < 0 || y0 >= CELLHEIGHT) continue;
			int x0 = x;
			uint8_t bits = reference_bitmap[j];
			while (bits) {
			if (x0 >= 0 && x0 < CELLWIDTH)
				ili9341_spi_buffer[y0 * CELLWIDTH + x0] = (bits & 0x80) ? c : 0x0000;
			x0++;
			bits <<= 1;
		}
	}
}

#ifndef DISPLAY_ST7796
#define MARKER_WIDTH  7
#define MARKER_HEIGHT 10
#define X_MARKER_OFFSET 3
#define Y_MARKER_OFFSET 10
#define MARKER_DRAW_MASK 0x80
typedef uint8_t m_bitmap_t;
static const m_bitmap_t marker_bitmap[]={
  // Marker 1
  0b11111110,
  0b11101110,
  0b11001110,
  0b11101110,
  0b11101110,
  0b11101110,
  0b11000110,
  0b01111100,
  0b00111000,
  0b00010000,
  // Marker 2
  0b11111110,
  0b11000110,
  0b10111010,
  0b11111010,
  0b11000110,
  0b10111110,
  0b10000010,
  0b01111100,
  0b00111000,
  0b00010000,
  // Marker 3
  0b11111110,
  0b11000110,
  0b10111010,
  0b11100110,
  0b11111010,
  0b10111010,
  0b11000110,
  0b01111100,
  0b00111000,
  0b00010000,
  // Marker 4
  0b11111110,
  0b11110110,
  0b11100110,
  0b11010110,
  0b10110110,
  0b10110110,
  0b10000010,
  0b01110100,
  0b00111000,
  0b00010000,
};
#else
#define MARKER_WIDTH  10
#define MARKER_HEIGHT 13
#define X_MARKER_OFFSET 4
#define Y_MARKER_OFFSET 13
#define MARKER_DRAW_MASK 0x8000
typedef uint16_t m_bitmap_t;
static const m_bitmap_t marker_bitmap[]={
  // Marker 1
  0b1111111110000000,
  0b1111001110000000,
  0b1110001110000000,
  0b1101001110000000,
  0b1111001110000000,
  0b1111001110000000,
  0b1111001110000000,
  0b1111001110000000,
  0b1111001110000000,
  0b0110000100000000,
  0b0011111000000000,
  0b0001110000000000,
  0b0000100000000000,
  // Marker 2
  0b1111111110000000,
  0b1110000110000000,
  0b1100110010000000,
  0b1100110010000000,
  0b1111100110000000,
  0b1111001110000000,
  0b1110011110000000,
  0b1100111110000000,
  0b1100000010000000,
  0b0111111100000000,
  0b0011111000000000,
  0b0001110000000000,
  0b0000100000000000,
  // Marker 3
  0b1111111110000000,
  0b1100000110000000,
  0b1001110010000000,
  0b1001110010000000,
  0b1111110010000000,
  0b1111000110000000,
  0b1111110010000000,
  0b1001110010000000,
  0b1001110010000000,
  0b0100000100000000,
  0b0011111000000000,
  0b0001110000000000,
  0b0000100000000000,
  // Marker 4
  0b1111111110000000,
  0b1111100110000000,
  0b1111000110000000,
  0b1110000110000000,
  0b1100100110000000,
  0b1001100110000000,
  0b1001100110000000,
  0b1000000010000000,
  0b1111100110000000,
  0b0111100100000000,
  0b0011111000000000,
  0b0001110000000000,
  0b0000100000000000,
};
#endif
static void
draw_marker(int x, int y, int c, int ch)
{
	int y0 = y, j;
	for (j = 0; j < MARKER_HEIGHT; j++, y0++) {
		int x0 = x;
		m_bitmap_t bits = marker_bitmap[ch * MARKER_HEIGHT + j];
		bool force_color = false;
		while (bits) {
			if (bits & MARKER_DRAW_MASK) force_color = true;
			if (x0 >= 0 && x0 < CELLWIDTH && y0 >= 0 && y0 < CELLHEIGHT) {
				if (bits & MARKER_DRAW_MASK)
					ili9341_spi_buffer[y0 * CELLWIDTH + x0] = c;
				else if (force_color)
					ili9341_spi_buffer[y0 * CELLWIDTH + x0] = 0x0000;
			}
			x0++;
			bits <<= 1;
		}
	}
}

void
marker_position(int m, int t, int *x, int *y)
{
	uint32_t index = trace_index[t][markers[m].index];
	*x = CELL_X(index);
	*y = CELL_Y(index);
}

typedef int (*compare_t)(int x, int y);
static int greater(int x, int y) { return x > y; }
static int lesser(int x, int y) { return x < y; }

compare_t marker_comparator(MarkerSearchModes mode) {
	if (mode == MarkerSearchModes::Max)
		return ::greater;
	else
		return ::lesser;
}


int
marker_search(MarkerSearchModes mode)
{
	compare_t compare = marker_comparator(mode);
	int i;
	int found = 0;

	if (uistat.current_trace == -1)
		return -1;

	int value = CELL_Y(trace_index[uistat.current_trace][0]);
	for (i = 0; i < sweep_points; i++) {
		uint32_t index = trace_index[uistat.current_trace][i];
		if ((*compare)(value, CELL_Y(index))) {
			value = CELL_Y(index);
			found = i;
		}
	}

	return found;
}

// TODO: merge marker_search_left and marker_search_right into one function

int
marker_search_left(MarkerSearchModes mode, int from)
{
	compare_t compare = marker_comparator(mode);
	int i;
	int found = -1;

	if (uistat.current_trace == -1)
		return -1;

	int value = CELL_Y(trace_index[uistat.current_trace][from]);
	for (i = from - 1; i >= 0; i--) {
		uint32_t index = trace_index[uistat.current_trace][i];
		if ((*compare)(value, CELL_Y(index)))
			break;
		value = CELL_Y(index);
	}

	for (; i >= 0; i--) {
		uint32_t index = trace_index[uistat.current_trace][i];
		if ((*compare)(CELL_Y(index), value)) {
			break;
		}
		found = i;
		value = CELL_Y(index);
	}
	return found;
}

int
marker_search_right(MarkerSearchModes mode, int from)
{
	compare_t compare = marker_comparator(mode);
	int i;
	int found = -1;

	if (uistat.current_trace == -1)
		return -1;

	int value = CELL_Y(trace_index[uistat.current_trace][from]);
	for (i = from + 1; i < sweep_points; i++) {
		uint32_t index = trace_index[uistat.current_trace][i];
		if ((*compare)(value, CELL_Y(index)))
			break;
		value = CELL_Y(index);
	}

	for (; i < sweep_points; i++) {
		uint32_t index = trace_index[uistat.current_trace][i];
		if ((*compare)(CELL_Y(index), value)) {
			break;
		}
		found = i;
		value = CELL_Y(index);
	}
	return found;
}


int
search_nearest_index(int x, int y, int t)
{
	uint32_t *index = trace_index[t];
	int min_i = -1;
	int min_d = 1000;
	int i;
	for (i = 0; i < sweep_points; i++) {
		int16_t dx = x - CELL_X(index[i]);
		int16_t dy = y - CELL_Y(index[i]);
		if (dx < 0) dx = -dx;
		if (dy < 0) dy = -dy;
		if (dx > 20 || dy > 20)
			continue;
		int d = dx*dx + dy*dy;
		if (d < min_d) {
			min_d = d;
			min_i = i;
		}
	}
	return min_i;
}

static void
markmap_marker(int marker)
{
	int t;
	if (!markers[marker].enabled)
		return;
	for (t = 0; t < TRACES_MAX; t++) {
		if (!trace[t].enabled)
			continue;
		uint32_t index = trace_index[t][markers[marker].index];
		int x = CELL_X(index) - X_MARKER_OFFSET;
		int y = CELL_Y(index) - Y_MARKER_OFFSET;
		invalidate_rect(x, y, x+MARKER_WIDTH-1, y+MARKER_HEIGHT-1);
	}
}

void
markmap_all_markers(void)
{
	int i;
	for (i = 0; i < MARKERS_MAX; i++) {
		if (!markers[i].enabled)
			continue;
		markmap_marker(i);
	}
	markmap_upperarea();
}

bool plot_checkerBoard = false;
bool plot_shadeCells = false;
static void
draw_cell(int m, int n)
{
	int x0 = m * CELLWIDTH;
	int y0 = n * CELLHEIGHT;
	int w = CELLWIDTH;
	int h = CELLHEIGHT;
	int x, y;
	int i0, i1;
	int i;
	int t;
	bool shade = plot_shadeCells;
	if(plot_checkerBoard)
		shade |= (((m + n) % 2) == 0);

	uint16_t bg = shade ? RGB565(40,40,40) : DEFAULT_BG_COLOR;

	// Clip cell by area
	if (x0 + w > area_width)
		w = area_width - x0;
	if (y0 + h > area_height)
		h = area_height - y0;
	if (w <= 0 || h <= 0)
		return;

	uint16_t grid_mode = 0;
	for (t = 0; t < TRACES_MAX; t++) {
		if (!trace[t].enabled)
			continue;

		if (trace[t].type == TRC_SMITH)
			grid_mode |= GRID_SMITH;
		//else if (trace[t].type == TRC_ADMIT)
		//  grid_mode |= GRID_ADMIT;
		else if (trace[t].type == TRC_POLAR)
			grid_mode |= GRID_POLAR;
		else
			grid_mode |= GRID_RECTANGULAR;
	}

//	PULSE;
	  // Clear buffer ("0 : height" lines)
	#if 0
	  // use memset 350 system ticks for all screen calls
	  // as understand it use 8 bit set, slow down on 32 bit systems
	  memset(spi_buffer, bg, (h*CELLWIDTH)*sizeof(uint16_t));
	#else
	  // use direct set  35 system ticks for all screen calls
	#if CELLWIDTH%8 != 0
	#error "CELLWIDTH % 8 should be == 0 for speed, or need rewrite cell cleanup"
	#endif
	// Set DEFAULT_BG_COLOR for 8 pixels in one cycle
	int count = h*CELLWIDTH / (16/sizeof(uint16_t));
	uint32_t *p = (uint32_t *)ili9341_spi_buffer;
	while (count--) {
		p[0] = bg | (bg << 16);
		p[1] = bg | (bg << 16);
		p[2] = bg | (bg << 16);
		p[3] = bg | (bg << 16);
		p += 4;
	}
	#endif

	uint16_t c = config.grid_color;
	if (grid_mode & (GRID_RECTANGULAR)){
		for (x = 0; x < w; x++) {
			if (rectangular_grid_x(x + x0)) {
				for (y = 0; y < h; y++) ili9341_spi_buffer[y * CELLWIDTH + x] = c;
			}
		}
		for (y = 0; y < h; y++) {
			if (rectangular_grid_y(y + y0)) {
				for (x = 0; x < w; x++)
					if (x + x0 >= CELLOFFSETX && x + x0 <= WIDTH + CELLOFFSETX)
						ili9341_spi_buffer[y * CELLWIDTH + x] = c;
			}
		}
	}
	if (grid_mode & GRID_SMITH) {
		for (y = 0; y < h; y++)
			for (x = 0; x < w; x++)
				if (smith_grid(x + x0, y + y0)) ili9341_spi_buffer[y * CELLWIDTH + x] = c;
	}
	else if (grid_mode & GRID_POLAR) {
		for (y = 0; y < h; y++)
			for (x = 0; x < w; x++)
				if (polar_grid(x + x0, y + y0)) ili9341_spi_buffer[y * CELLWIDTH + x] = c;
	}
	else if (grid_mode & GRID_ADMIT) {
		for (y = 0; y < h; y++)
			for (x = 0; x < w; x++)
				if (smith_grid3(x+x0, y+y0))// smith_grid2(x+x0, y+y0, 0.5))
					ili9341_spi_buffer[y * CELLWIDTH + x] = c;
	}
//	PULSE;
	for (t = 0; t < TRACES_MAX; t++) {
		if (!trace[t].enabled)
			continue;
		c = config.trace_color[t];
		// draw polar plot (check all points)
		i0 = 0;
		i1 = 0;
		uint32_t trace_type = (1 << trace[t].type);
		if (trace_type & ((1 << TRC_SMITH) | (1 << TRC_POLAR)))
			i1 = sweep_points - 1;
		else
			search_index_range_x(x0, x0 + w, trace_index[t], &i0, &i1);
		uint32_t *index = trace_index[t];
		for (i = i0; i < i1; i++) {
			int x1 = CELL_X(index[i]) - x0;
			int y1 = CELL_Y(index[i]) - y0;
			int x2 = CELL_X(index[i + 1]) - x0;
			int y2 = CELL_Y(index[i + 1]) - y0;
			cell_drawline(x1, y1, x2, y2, c);
		}
	}

//	PULSE;
	// draw marker symbols on each trace (<10 system ticks for all screen calls)
#if 1
	for (i = 0; i < MARKERS_MAX; i++) {
		if (!markers[i].enabled)
			continue;
		for (t = 0; t < TRACES_MAX; t++) {
			if (!trace[t].enabled)
				continue;
			uint32_t index = trace_index[t][markers[i].index];
			int x = CELL_X(index) - x0 - X_MARKER_OFFSET;
			int y = CELL_Y(index) - y0 - Y_MARKER_OFFSET;
			// Check marker icon on cell
			if (x + MARKER_WIDTH >= 0 && x - MARKER_WIDTH < CELLWIDTH &&
				y + MARKER_HEIGHT >= 0 && y - MARKER_HEIGHT < CELLHEIGHT)
				draw_marker(x, y, config.trace_color[t], i);
		}
	}
#endif

	// draw trace and marker info on the top
	if (n <= (3*FONT_STR_HEIGHT)/CELLHEIGHT)
		cell_draw_marker_info(x0, y0);

//	PULSE;

	// Draw reference position (<10 system ticks for all screen calls)
	for (t = 0; t < TRACES_MAX; t++) {
		if (!trace[t].enabled)
			continue;
		uint32_t trace_type = (1 << trace[t].type);
		if (trace_type & ((1 << TRC_SMITH) | (1 << TRC_POLAR)))
			continue;
		int x = 0 - x0 + CELLOFFSETX - REFERENCE_X_OFFSET;
		if (x + REFERENCE_WIDTH >= 0 && x - REFERENCE_WIDTH < CELLWIDTH) {
			int y = HEIGHT - float2int((get_trace_refpos(t) * GRIDY)) - y0 - REFERENCE_Y_OFFSET;
			if (y + REFERENCE_HEIGHT >= 0 && y - REFERENCE_HEIGHT < CELLHEIGHT)
				draw_refpos(x, y, config.trace_color[t]);
		}
	}

#if 1
	if (w < CELLWIDTH) {
		uint16_t *src = ili9341_spi_buffer + CELLWIDTH;
		uint16_t *dst = ili9341_spi_buffer + w;
		for (y = h; --y; src += CELLWIDTH - w)
			for (x = w; x--;)
				*dst++ = *src++;
		}
#endif
	ili9341_bulk(OFFSETX + x0, OFFSETY + y0, w, h);
}

void
draw_all_cells(bool flush_markmap)
{
	int m, n;
	for (m = 0; m < (area_width+CELLWIDTH-1) / CELLWIDTH; m++)
		for (n = 0; n < (area_height+CELLHEIGHT-1) / CELLHEIGHT; n++) {
			if ((markmap[0][n] | markmap[1][n]) & (1 << m)) {
				draw_cell(m, n);
				plot_tick();
				if(plot_canceled)
					return;
//				ili9341_fill(m*CELLWIDTH+OFFSETX, n*CELLHEIGHT, 2, 2, RGB565(255,0,0));
			}
//			else
//				ili9341_fill(m*CELLWIDTH+OFFSETX, n*CELLHEIGHT, 2, 2, RGB565(0,255,0));
		}
	if (flush_markmap) {
		// keep current map for update
		swap_markmap();
		// clear map for next plotting
		clear_markmap();
	}
}

void
draw_all(bool flush)
{
	plot_canceled = false;
	if (redraw_request & REDRAW_MARKER)
		markmap_upperarea();
	if (redraw_request & (REDRAW_CELLS | REDRAW_MARKER))
		draw_all_cells(flush);
	if (redraw_request & REDRAW_FREQUENCY)
		draw_frequencies();
	if (redraw_request & REDRAW_CAL_STATUS)
		draw_cal_status();
	redraw_request = 0;
}
void
request_to_redraw_marker(int marker)
{
	if (marker < 0)
		return;
	// mark map on new position of marker
	redraw_request |= REDRAW_MARKER;
	markmap_marker(marker);
}
void
redraw_marker(int marker)
{
	request_to_redraw_marker(marker);
	draw_all_cells(true);
}

void
request_to_draw_cells_behind_menu(void)
{
  // Values Hardcoded from ui.c
  invalidate_rect(LCD_WIDTH-MENU_BUTTON_WIDTH-OFFSETX, 0, LCD_WIDTH-OFFSETX, LCD_HEIGHT-1);
  redraw_request |= REDRAW_CELLS;
}

void
request_to_draw_cells_behind_numeric_input(void)
{
  // Values Hardcoded from ui.c
  invalidate_rect(0, LCD_HEIGHT-NUM_INPUT_HEIGHT, LCD_WIDTH-1, LCD_HEIGHT-1);
  redraw_request |= REDRAW_CELLS;
}

static int
cell_drawchar(uint8_t ch, int x, int y)
{
	uint8_t bits;
	int c, r, ch_size;
	const uint8_t *char_buf = FONT_GET_DATA(ch);
	ch_size = FONT_GET_WIDTH(ch);
	//  if (y <= -FONT_GET_HEIGHT || y >= CELLHEIGHT || x <= -ch_size || x >= CELLWIDTH)
	//    return ch_size;
	if (x <= -ch_size)
		return ch_size;
	for (c = 0; c < FONT_GET_HEIGHT; c++) {
		bits = *char_buf++;
		if ((y + c) < 0 || (y + c) >= CELLHEIGHT)
			continue;
		for (r = 0; r < ch_size; r++) {
			if ((x+r) >= 0 && (x+r) < CELLWIDTH && (0x80 & bits))
				ili9341_spi_buffer[(y+c)*CELLWIDTH + (x+r)] = foreground_color;
			bits <<= 1;
		}
	}
	return ch_size;
}

void
cell_drawstring(char *str, int x, int y)
{
	if (y <= -FONT_GET_HEIGHT || y >= CELLHEIGHT)
		return;
	while (*str) {
		if (x >= CELLWIDTH)
			return;
		x += cell_drawchar(*str++, x, y);
	}
}

static void
cell_draw_marker_info(int x0, int y0)
{
	char buf[24];
	int t;
	if (active_marker < 0)
		return;
	int idx = markers[active_marker].index;
	int j = 0;
	if (active_marker != -1 && previous_marker != -1 && uistat.current_trace != -1) {
		int t = uistat.current_trace;
		int mk;
		for (mk = 0; mk < MARKERS_MAX; mk++) {
			if (!markers[mk].enabled)
				continue;
			ili9341_set_foreground(config.trace_color[t]);
			int xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
			int ypos = 1 + (j/2)*(FONT_STR_HEIGHT) - y0;
			strcpy(buf, " M1"); if (mk == active_marker) buf[0] = S_SARROW[0];
			buf[2] += mk;
			cell_drawstring(buf, xpos, ypos);
			xpos += 3*FONT_WIDTH + 3;
			//trace_get_info(t, buf, sizeof buf);
			freqHz_t freq = freqAt(markers[mk].index);
			if (uistat.marker_delta && mk != active_marker) {
				freq -= freqAt(markers[active_marker].index);
				frequency_string_short(buf, sizeof buf, freq, S_DELTA[0]);
			} else {
				frequency_string_short(buf, sizeof buf, freq, 0);
			}
			cell_drawstring(buf, xpos, ypos);
			xpos += 11*FONT_WIDTH + 11;
			if (uistat.marker_delta && mk != active_marker)
				trace_get_value_string_delta(t, buf, sizeof buf, measured[trace[t].channel], markers[mk].index, markers[active_marker].index);
			else
				trace_get_value_string(t, buf, sizeof buf, measured[trace[t].channel], markers[mk].index);
			ili9341_set_foreground(0xFFFF);
			cell_drawstring(buf, xpos, ypos);
			j++;
		}

		// draw marker delta
		if (!uistat.marker_delta && previous_marker >= 0 && active_marker != previous_marker && markers[previous_marker].enabled) {
			int idx0 = markers[previous_marker].index;
			int xpos = (WIDTH/2+30) + CELLOFFSETX - x0;
			int ypos = 1 + (j/2)*(FONT_STR_HEIGHT) - y0;
			strcpy(buf, S_DELTA "1:"); if (mk == active_marker) buf[0] = S_SARROW[0];
			buf[1] += previous_marker;
			ili9341_set_foreground(0xFFFF);
			cell_drawstring(buf, xpos, ypos);
			xpos += 3*FONT_WIDTH + 3;
			if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
				frequency_string(buf, sizeof buf, freqAt(idx) - freqAt(idx0));
			} else {
				//chsnprintf(buf, sizeof buf, "%d ns %.1f m", (uint16_t)(time_of_index(idx) * 1e9 - time_of_index(idx0) * 1e9),
				//                                            distance_of_index(idx) - distance_of_index(idx0));
				int n = string_value_with_prefix(buf, sizeof buf, time_of_index(idx) - time_of_index(idx0), 's');
				buf[n++] = ' ';
				string_value_with_prefix(&buf[n], sizeof buf - n, distance_of_index(idx) - distance_of_index(idx0), 'm');
			}
			cell_drawstring(buf, xpos, ypos);
		}
	} else {
		for (t = 0; t < TRACES_MAX; t++) {
			if (!trace[t].enabled)
				continue;
			int xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
			int ypos = 1 + (j/2)*(FONT_STR_HEIGHT) - y0;
			strcpy(buf, " CH0"); if (t == uistat.current_trace) buf[0] = S_SARROW[0];
			buf[3] += trace[t].channel;
			ili9341_set_foreground(config.trace_color[t]);
			//chsnprintf(buf, sizeof buf, "CH%d", trace[t].channel);
			cell_drawstring(buf, xpos, ypos); // invert
			xpos += 4*FONT_WIDTH + 4;
			trace_get_info(t, buf, sizeof buf);
			cell_drawstring(buf, xpos, ypos);
			xpos += 11*FONT_WIDTH + 5;
			trace_get_value_string(t, buf, sizeof buf, measured[trace[t].channel], idx);
			ili9341_set_foreground(0xFFFF);
			cell_drawstring(buf, xpos, ypos);
			j++;
		}

		// draw marker frequency
		int xpos = (WIDTH/2+40) + CELLOFFSETX - x0;
		int ypos = 1 + (j/2)*(FONT_STR_HEIGHT) - y0;
		strcpy(buf, " 1:");if (uistat.lever_mode == LM_MARKER) buf[0] = S_SARROW[0];
		buf[0] += active_marker;
		xpos += FONT_WIDTH;
		ili9341_set_foreground(0xFFFF);
		cell_drawstring(buf, xpos, ypos);
		xpos += 3*FONT_WIDTH;
		if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
			frequency_string(buf, sizeof buf, plot_getFrequencyAt(idx));
		} else {
			//chsnprintf(buf, sizeof buf, "%d ns %.1f m", (uint16_t)(time_of_index(idx) * 1e9), distance_of_index(idx));
			int n = string_value_with_prefix(buf, sizeof buf, time_of_index(idx), 's');
			buf[n++] = ' ';
			string_value_with_prefix(&buf[n], sizeof buf-n, distance_of_index(idx), 'm');
		}
		cell_drawstring(buf, xpos, ypos);
	}
	if (electrical_delay != 0) {
		// draw electrical delay
		int xpos = 21 + CELLOFFSETX - x0;
		int ypos = 1 + ((j+1)/2)*(FONT_STR_HEIGHT) - y0;
		chsnprintf(buf, sizeof buf, "Edelay");
		ili9341_set_foreground(0xFFFF);
		cell_drawstring(buf, xpos, ypos);
		xpos += 7*FONT_WIDTH + 7;
		int n = string_value_with_prefix(buf, sizeof buf, electrical_delay * 1e-12, 's');
		cell_drawstring(buf, xpos, ypos);
		xpos += n*FONT_WIDTH + n;
		float light_speed_ps = 299792458e-12; //(m/ps)
		string_value_with_prefix(buf, sizeof buf, electrical_delay * light_speed_ps * velocity_factor, 'm');
		cell_drawstring(buf, xpos, ypos);
	}
}

/* Prints a full frequency:
 * 500.000 Khz
 * 10.000 000 Mhz
 * 3000.000 000 Mhz
 */
void
frequency_string(char *buf, size_t len, freqHz_t freq)
{
	if (freq < 0) {
		freq = -freq;
		*buf++ = '-';
		len -= 1;
	}
	if (freq < 1000) {
		chsnprintf(buf, len, "%d Hz", (int)freq);
	} else if (freq < 1000000) {
		chsnprintf(buf, len, "%d.%03d kHz",
						 (int)(freq / 1000),
						 (int)(freq % 1000));
	} else {
		chsnprintf(buf, len, "%d.%03d %03d MHz",
						 (int)(freq / 1000000),
						 (int)((freq / 1000) % 1000),
						 (int)(freq % 1000));
	}
}

/* Prints a shorter/compacter frequency:
 * 500.000KHz
 * 10.000000Mhz
 * 3000.0000Mhz
 */
void
frequency_string_short(char *b, size_t len, freqHz_t freq, char prefix)
{
	char *buf = b;
	if (prefix) {
		*buf++ = prefix;
		len -= 1;
	}
	if (freq < 0) {
		freq = -freq;
		*buf++ = '-';
		len -= 1;
	}
	if (freq < 1000) {
		chsnprintf(buf, len, "%d Hz", (int)freq);
	} else if (freq < 1000000) {
		chsnprintf(buf, len, "%d.%03dkHz",
						 (int)(freq / 1000),
						 (int)(freq % 1000));
	}
	size_t wr = chsnprintf(buf, len, "%d.%06d",
			(int)(freq / 1000000),
			(int)(freq % 1000000));
	// Note that wr is either:
	// 8: 1 - 9 Mhz
	// 9: 10 - 99 Mhz
	// 10: 100 - 999 Mhz
	// 11: 1000 - 4400 Mhz
	// Do not add 'Mhz' if it would overwrite all characters
	if (len < 4)
		return;
	// Overwrite last digits, if >= 10Mhz (aka 9 digits)
	if (wr > 9) {
		wr = 9;
	}
	// Make sure we have space left for 'Mhz'
	if (wr > len - 4) {
		wr = len - 4;
	}

	strcpy(buf + wr, "MHz");
}

void padString(char* s, int len, char c = ' ') {
	int oldLen = strlen(s);
	for(int i = oldLen; i < (len - 1); i++)
		s[i] = c;
	s[len - 1] = 0;
}

void
draw_frequencies(void)
{
	char buf[24];
	ili9341_set_foreground(DEFAULT_FG_COLOR);
	ili9341_set_background(DEFAULT_BG_COLOR);

	ili9341_fill(0, FREQUENCIES_YPOS, LCD_WIDTH, FONT_GET_HEIGHT, DEFAULT_BG_COLOR);
	// draw sweep points
	chsnprintf(buf, sizeof(buf), "%3d P  %2dx AVG", (int)sweep_points, (int)current_props._avg);
	ili9341_drawstring(buf, FREQUENCIES_XPOS3, FREQUENCIES_YPOS);

	if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
		if (frequency1 > 0) {
			auto start = frequency0;
			auto stop = frequency1;

			strcpy(buf, " START ");
			frequency_string(buf+7, 24-7, start);
			ili9341_drawstring(buf, FREQUENCIES_XPOS1, FREQUENCIES_YPOS);
			strcpy(buf, " STOP ");
			frequency_string(buf+6, 24-6, stop);
			ili9341_drawstring(buf, FREQUENCIES_XPOS2, FREQUENCIES_YPOS);
		} else if (frequency1 < 0) {
			auto fcenter = frequency0;
			auto fspan = -frequency1;
			strcpy(buf, " CENTER ");if (uistat.lever_mode == LM_CENTER) buf[0] = S_SARROW[0];
			frequency_string(buf+8, 24-8, fcenter);
			ili9341_drawstring(buf, FREQUENCIES_XPOS1, FREQUENCIES_YPOS);
			strcpy(buf, " SPAN "); if (uistat.lever_mode == LM_SPAN) buf[0] = S_SARROW[0];
			frequency_string(buf+6, 24-6, fspan);
			ili9341_drawstring(buf, FREQUENCIES_XPOS2, FREQUENCIES_YPOS);
		} else {
			strcpy(buf, " CW ");if (uistat.lever_mode == LM_CENTER) buf[0] = S_SARROW[0];
			frequency_string(buf+4, 24-4, frequency0);
			ili9341_drawstring(buf, FREQUENCIES_XPOS1, FREQUENCIES_YPOS);
		}
	} else {
		strcpy(buf, " START 0s");
		ili9341_drawstring(buf, FREQUENCIES_XPOS1, FREQUENCIES_YPOS);

		strcpy(buf, " STOP ");
		chsnprintf(buf+6, 24-6, "%d ns", (uint16_t)(time_of_index(current_props._sweep_points) * 1e9));
		ili9341_drawstring(buf, FREQUENCIES_XPOS2, FREQUENCIES_YPOS);
	}
}

void
draw_cal_status(void)
{
  int x = 0;
  int y = 100;
  char c[3];
  ili9341_set_foreground(DEFAULT_CAL_INACTIVE_COLOR);
  ili9341_set_background(DEFAULT_BG_COLOR);
  ili9341_fill(0, y, OFFSETX, 6*(FONT_STR_HEIGHT), DEFAULT_BG_COLOR);
  if (cal_status & CALSTAT_APPLY) {
	ili9341_set_foreground(DEFAULT_CAL_ACTIVE_COLOR);
    c[0] = cal_status & CALSTAT_INTERPOLATED ? 'c' : 'C';
    c[1] = active_props == &current_props ? '*' : '0' + lastsaveid;
    c[2] = 0;
    ili9341_drawstring(c, x, y);
  }
  y += FONT_STR_HEIGHT;
  int i;
  static const struct {char text[2]; uint16_t mask;} calibration_text[]={
    {"S", CALSTAT_SHORT},
    {"O", CALSTAT_OPEN},
    {"L", CALSTAT_LOAD},
    {"T", CALSTAT_THRU},
    {"E", CALSTAT_ENHANCED_RESPONSE},
  };
  for (i = 0; i < sizeof(calibration_text)/sizeof(*calibration_text); i++, y+=FONT_STR_HEIGHT)
    if (cal_status & calibration_text[i].mask)
      ili9341_drawstring(calibration_text[i].text, x, y);
}

void
draw_battery_status(void)
{
		int w = 10, h = 14;
		int x = 0, y = 0;
		int i, c;
		uint16_t *buf = ili9341_spi_buffer;
		uint8_t vbati = vbat2bati(vbat);
		uint16_t col = vbati == 0 ?  DEFAULT_LOW_BAT_COLOR : DEFAULT_NORMAL_BAT_COLOR;
		memset(ili9341_spi_buffer, 0, w * h * 2);

		// battery head
		x = 3;
		buf[y * w + x++] = col;
		buf[y * w + x++] = col;
		buf[y * w + x++] = col;
		buf[y * w + x++] = col;

		y++;
		x = 3;
		buf[y * w + x++] = col;
		x++; x++;
		buf[y * w + x++] = col;

		y++;
		x = 1;
		for (i = 0; i < 8; i++)
				buf[y * w + x++] = col;

		for (c = 0; c < 3; c++) {
				y++;
				x = 1;
				buf[y * w + x++] = col;
				x++; x++; x++; x++; x++; x++;
				buf[y * w + x++] = col;

				y++;
				x = 1;
				buf[y * w + x++] = col;
				x++;
				for (i = 0; i < 4; i++)
						buf[y * w + x++] = ( ((c+1) * 25) >= (100 - vbati)) ? col : 0;
				x++;
				buf[y * w + x++] = col;

				y++;
				x = 1;
				buf[y * w + x++] = col;
				x++;
				for (i = 0; i < 4; i++)
						buf[y * w + x++] = ( ((c+1) * 25) >= (100 - vbati)) ? col : 0;
				x++;
				buf[y * w + x++] = col;
		}

		// battery foot
		y++;
		x = 1;
		buf[y * w + x++] = col;
		x++; x++; x++; x++; x++; x++;
		buf[y * w + x++] = col;

		y++;
		x = 1;
		for (i = 0; i < 8; i++)
				buf[y * w + x++] = col;

		ili9341_bulk(0, 1, w, h);
}

void
request_to_redraw_grid(void)
{
	force_set_markmap();
	redraw_request |= REDRAW_CELLS;
}

void
redraw_frame(void)
{
	ili9341_set_background(DEFAULT_BG_COLOR);
	ili9341_clear_screen();
	draw_frequencies();
	draw_cal_status();
}

void
plot_init(void)
{
	force_set_markmap();
}

void plot_cancel() {
	plot_canceled = true;
}
