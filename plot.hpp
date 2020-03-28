#pragma once
#include "common.hpp"
#include <mculib/small_function.hpp>


// Offset of plot area
#define OFFSETX 10
#define OFFSETY  0

#define WIDTH  300

// HEIGHT = 8*GRIDY
#define HEIGHT 232

#define NGRIDY 8

#define FREQUENCIES_XPOS1 OFFSETX
#define FREQUENCIES_XPOS2 200
#define FREQUENCIES_YPOS  (240-7)

// GRIDX is calculated at runtime depending on frequency span
#define GRIDY (HEIGHT / NGRIDY)


#define CELLOFFSETX 5

#define AREA_WIDTH_NORMAL  (CELLOFFSETX + WIDTH  + 1 + 4)
#define AREA_HEIGHT_NORMAL (              HEIGHT + 1)


// Smith/polar chart
#define P_CENTER_X (CELLOFFSETX + WIDTH/2)
#define P_CENTER_Y (HEIGHT/2)
#define P_RADIUS   (HEIGHT/2)


#define TRACES_MAX 4



#define DEFAULT_FG_COLOR			RGB565(255,255,255)
#define DEFAULT_BG_COLOR			RGB565(  0,  0,  0)
#define DEFAULT_GRID_COLOR			RGB565(128,128,128)
#define DEFAULT_MENU_COLOR			RGB565(255,255,255)
#define DEFAULT_MENU_TEXT_COLOR		RGB565(  0,  0,  0)
#define DEFAULT_MENU_ACTIVE_COLOR	RGB565(180,255,180)
#define DEFAULT_TRACE_1_COLOR		RGB565(255,255,  0)
#define DEFAULT_TRACE_2_COLOR		RGB565(  0,255,255)
#define DEFAULT_TRACE_3_COLOR		RGB565(  0,255,  0)
#define DEFAULT_TRACE_4_COLOR		RGB565(255,  0,255)
#define DEFAULT_NORMAL_BAT_COLOR	RGB565( 31,227,  0)
#define DEFAULT_LOW_BAT_COLOR		RGB565(255,  0,  0)
#define	DEFAULT_SPEC_INPUT_COLOR	RGB565(128,255,128);


extern int area_width;
extern int area_height;

// for debugging plots
extern bool plot_checkerBoard; // draw a checkerboard pattern that indicates where the cells are
extern bool plot_shadeCells; // shade all drawn cells from now on

// this function is called to determine frequency in hz at a marker point
extern small_function<freqHz_t(int index)> plot_getFrequencyAt;

// this function is called periodically during plotting and can be used
// to process events in the event queue.
extern small_function<void()> plot_tick;

void plot_init(void);

// mark a cell for redraw. x: 0 to 15; y: 0 to 7
void mark_map(int x, int y);

// cancel ongoing draw operations further up the stack
void plot_cancel();
void update_grid(void);
void request_to_redraw_grid(void);
void redraw_frame(void);
//void redraw_all(void);
void request_to_draw_cells_behind_menu(void);
void request_to_draw_cells_behind_numeric_input(void);
void request_to_redraw_marker(int marker, int update_info);
void redraw_marker(int marker, int update_info);
void trace_get_info(int t, char *buf, int len);
float groupdelay_from_array(int i, complexf array[SWEEP_POINTS_MAX]);
void plot_into_index(complexf measured[2][SWEEP_POINTS_MAX]);
void force_set_markmap(void);
void draw_all(bool flush);
void draw_all_cells(bool flush_markmap);
void draw_frequencies(void);
void draw_cal_status(void);

void markmap_all_markers(void);

void marker_position(int m, int t, int *x, int *y);
int search_nearest_index(int x, int y, int t);

int marker_search(MarkerSearchModes mode);
int marker_search_left(MarkerSearchModes mode, int from);
int marker_search_right(MarkerSearchModes mode, int from);
