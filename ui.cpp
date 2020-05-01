/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "common.hpp"
#include "main.hpp"
#include "flash.hpp"
#include "globals.hpp"
#include "ili9341.hpp"
#include "Font5x7.h"
#include "plot.hpp"
#include "ui.hpp"
#include "board.hpp"
#include <stdlib.h>
#include <string.h>
#include <mculib/message_log.hpp>
#include <mculib/printf.hpp>
#include <mculib/printk.hpp>
#include "gitversion.hpp"

using UIHW::UIEvent;
using UIHW::UIEventButtons;
using UIHW::UIEventTypes;
using namespace UIActions;


#define TRUE true
#define FALSE false


int8_t previous_marker = -1;

enum {
  UI_NORMAL, UI_MENU, UI_NUMERIC, UI_KEYPAD, UI_USB_MODE
};

enum {
  KM_START, KM_STOP, KM_CENTER, KM_SPAN, KM_POINTS, KM_CW, KM_SCALE, KM_REFPOS, KM_EDELAY, KM_VELOCITY_FACTOR, KM_SCALEDELAY
};

uint8_t ui_mode = UI_NORMAL;
uint8_t keypad_mode;
int8_t selection = 0;
bool ui_disabled = false;

typedef struct {
  uint8_t type;
  const char *label;
  void (*callback)(UIEvent evt, int item);
  const void* reference;
} menuitem_t;


int awd_count;
//int touch_x, touch_y;

#define NUMINPUT_LEN 10

#define KP_CONTINUE 0
#define KP_DONE 1
#define KP_CANCEL 2

char kp_buf[11];
int8_t kp_index = 0;

bool uiEventsEnabled = true;
UIEvent lastUIEvent = {};

void ui_mode_menu(void);
void ui_mode_numeric(int _keypad_mode);
void ui_mode_keypad(int _keypad_mode);
void draw_menu(void);
void leave_ui_mode(void);
void erase_menu_buttons(void);
void ui_process_keypad(UIEvent evt);
static void ui_process_numeric(UIEvent evt);

static void menu_push_submenu(const menuitem_t *submenu);
static int touch_pickup_marker(void);



void
touch_prepare_sense(void)
{
}

// Disable or enable ui_process_* callbacks in order to do synchronous event polling.
// New uses of these functions are heavily discouraged.
// Please use event driven UI processing instead.

void uiEnableProcessing(void) {
  uiEventsEnabled = true;
}
void uiDisableProcessing() {
  uiEventsEnabled = false;
}

// wait for an UI event when UI processing is disabled.
UIEvent uiWaitEvent() {
  while(lastUIEvent.type == UIEventTypes::None)
    application_doSingleEvent();

  UIEvent ret = lastUIEvent;
  lastUIEvent = {};
  return ret;
}



void
touch_cal_exec(void)
{
  int status;
  uint16_t x1, x2, y1, y2;
  UIEvent evt;
  
  uiDisableProcessing();

  ili9341_fill(0, 0, 320, 240, 0);
  ili9341_line(0, 0, 0, 32, 0xffff);
  ili9341_line(0, 0, 32, 0, 0xffff);
  ili9341_drawstring_5x7("TOUCH UPPER LEFT", 10, 10, 0xffff, 0x0000);

  do {
    evt = uiWaitEvent();
    if(evt.isTouchPress())
      UIHW::touchPosition(x1, y1);
  } while(!evt.isTouchRelease());


  ili9341_fill(0, 0, 320, 240, 0);
  ili9341_line(320-1, 240-1, 320-1, 240-32, 0xffff);
  ili9341_line(320-1, 240-1, 320-32, 240-1, 0xffff);
  ili9341_drawstring_5x7("TOUCH LOWER RIGHT", 230, 220, 0xffff, 0x0000);

  do {
    evt = uiWaitEvent();
     if(evt.isTouchPress())
      UIHW::touchPosition(x2, y2);
  } while(!evt.isTouchRelease());

  config.touch_cal[0] = x1;
  config.touch_cal[1] = y1;
  config.touch_cal[2] = (x2 - x1) * 16 / 320;
  config.touch_cal[3] = (y2 - y1) * 16 / 240;

  UIActions::printTouchCal();

  uiEnableProcessing();
}

void
touch_draw_test(void)
{
  UIEvent evt;
  int x0, y0;
  int x1, y1;
  
  uiDisableProcessing();

  ili9341_fill(0, 0, 320, 240, 0);
  ili9341_drawstring_5x7("TOUCH TEST: DRAG PANEL", OFFSETX, 233, 0xffff, 0x0000);

  do {
    evt = uiWaitEvent();
  } while(!evt.isTouchPress());
  touch_position(&x0, &y0);

  while(true) {
    if(!touch_position(&x1, &y1))
      break;
    ili9341_line(x0, y0, x1, y1, 0xffff);
    x0 = x1;
    y0 = y1;
    delay(50);
  }

  uiEnableProcessing();
}


bool touch_position(int *x, int *y)
{
  uint16_t touchX, touchY;
  if(!UIHW::touchPosition(touchX, touchY))
    return false;
  *x = (int(touchX) - config.touch_cal[0]) * 16 / config.touch_cal[2];
  *y = (int(touchY) - config.touch_cal[1]) * 16 / config.touch_cal[3];
  if(config.ui_options & UI_OPTIONS_FLIP) {
    *x = 320 - *x;
    *y = 240 - *y;
  }
  return true;
}

bool touch_position(int *x, int *y, UIEvent evt) {
  return touch_position(x, y);
}


void
show_version(void)
{
  int x = 5, y = 5;
  
  uiDisableProcessing();
  ili9341_fill(0, 0, 320, 240, 0);

  ili9341_drawstring_size(BOARD_NAME, x, y, 0xffff, 0x0000, 4);
  y += 25;

  ili9341_drawstring_5x7("Software copyright @edy555 et al", x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("Hardware designed by OwOComm", x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("Licensed under GPL. ", x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("https://github.com/ttrftech/NanoVNA", x + 10, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("https://github.com/nanovna/NanoVNA-V2-firmware", x + 10, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("Version: " GITVERSION, x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("Build Time: " __DATE__ " - " __TIME__, x, y += 10, 0xffff, 0x0000);
  y += 5;
  ili9341_drawstring_5x7("Kernel: " CH_KERNEL_VERSION, x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("Compiler: " PORT_COMPILER_NAME, x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("Architecture: " PORT_ARCHITECTURE_NAME " Core Variant: " PORT_CORE_VARIANT_NAME, x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("Port Info: " PORT_INFO, x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("Platform: " PLATFORM_NAME, x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("Board: " BOARD_NAME, x, y += 10, 0xffff, 0x0000);

  while (true) {
    UIEvent evt = uiWaitEvent();
    if(evt.isTouchPress() || evt.isLeverClick())
      break;
  }

  uiEnableProcessing();
}


void
show_dmesg(void)
{
  int x = 5, y = 5;
  
  uiDisableProcessing();
  ili9341_fill(0, 0, 320, 240, 0);

  int maxLines = 23;
  
  const char* msg = dmesg();
  int len = strlen(msg);
  const char* end = msg + len;

  // leave only last maxLines lines
  const char* pos = end - 1;
  const char* lines[maxLines];
  int nextLineIndex = maxLines - 1;
  while(pos >= msg && pos < end) {
    if((*pos) == '\n') {
      lines[nextLineIndex] = pos + 1;
      nextLineIndex--;
      if(nextLineIndex < 0) break;
    }
    pos--;
  }

  if(nextLineIndex >= 0) {
    lines[nextLineIndex] = msg;
    nextLineIndex--;
  }

  for(int i = nextLineIndex+1; i<maxLines; i++) {
    int len = (i < (maxLines-1)) ? (lines[i+1] - lines[i]) : (end - lines[i]);
    if(len > 0) len--;
    ili9341_drawstring_5x7(lines[i], len, x, y, 0xffff, 0x0000);
    y += 10;
  }

  while (true) {
    UIEvent evt = uiWaitEvent();
    if(evt.isTouchPress() || evt.isLeverClick())
      break;
  }

  uiEnableProcessing();
}


void ui_mode_usb(void) {
  int x = 5, y = 5;

  ili9341_fill(0, 0, 320, 240, 0);

  ili9341_drawstring_size(BOARD_NAME, x, y, 0xffff, 0x0000, 4);
  y += 50;

  ili9341_drawstring_size("USB MODE", x, y, 0xffff, 0x0000, 4);
  ui_mode = UI_USB_MODE;
}


void
show_message(const char* title, const char* message, int fg, int bg)
{
  int x = 5, y = 5;

  ili9341_fill(0, 0, 320, 240, bg);

  ili9341_drawstring_size(title, x, y, fg, bg, 4);
  y += 50;

  ili9341_drawstring_size(message, x, y, fg, bg, 1);
}

void
ui_enter_dfu(void)
{
  uiDisableProcessing();

  int x = 5, y = 5;

  // leave a last message 
  ili9341_fill(0, 0, 320, 240, 0);
  ili9341_drawstring_5x7("DFU: Device Firmware Update Mode", x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_5x7("To exit DFU mode, please reset device yourself.", x, y += 10, 0xffff, 0x0000);

  enterDFU();
}


// type of menu item 
enum {
  MT_NONE,
  MT_BLANK,
  MT_SUBMENU,
  MT_CALLBACK,
  MT_CANCEL,
  MT_CLOSE
};

typedef void (*menuaction_cb_t)(UIEvent evt, int item);


static void menu_move_back(void);


static void
menu_calop_cb(UIEvent evt, int item)
{
  switch (item) {
  case 0: // OPEN
    cal_collect(CAL_OPEN);
    break;
  case 1: // SHORT
    cal_collect(CAL_SHORT);
    break;
  case 2: // LOAD
    cal_collect(CAL_LOAD);
    break;
  case 3: // THRU
    cal_collect(CAL_THRU);
    break;
  default:
    return;
  }
  //selection = item+1;
  ui_disabled = true;
  draw_cal_status();
  draw_menu();
}

void ui_cal_collected() {
  ui_disabled = false;
  draw_menu();
}

extern const menuitem_t menu_save[];

static void menu_caldone_cb(UIEvent evt, int item)
{
  (void)item;
  cal_done();
  draw_cal_status();
  menu_move_back();
  menu_push_submenu(menu_save);
}

static void
menu_cal2_cb(UIEvent evt, int item)
{
  switch (item) {
  case 2: // RESET
    cal_status = 0;
    break;
  case 3: // CORRECTION
    // toggle applying correction
    if (cal_status)
      cal_status ^= CALSTAT_APPLY;
    break;
  }
  draw_menu();
  draw_cal_status();
}

static void
menu_recall_cb(UIEvent evt, int item)
{
  if (item < 0 || item >= 5)
    return;
  if (caldata_recall(item) == 0) {
    menu_move_back();
    ui_mode_normal();
    draw_cal_status();
  } else {
    show_dmesg();
    redraw_frame();
    request_to_redraw_grid();
    draw_menu();
  }
}

static void
menu_config_cb(UIEvent evt, int item)
{
  switch (item) {
  case 0:
      touch_cal_exec();
      redraw_frame();
      request_to_redraw_grid();
      draw_menu();
      uiEnableProcessing();
      break;
  case 1:
      touch_draw_test();
      redraw_frame();
      request_to_redraw_grid();
      draw_menu();
      break;
  case 2:
      config_save();
      menu_move_back();
      ui_mode_normal();
      break;
  case 3:
      show_version();
      redraw_frame();
      request_to_redraw_grid();
      draw_menu();
      break;
  case 4:
      show_dmesg();
      redraw_frame();
      request_to_redraw_grid();
      draw_menu();
      break;
  }
}

static void
menu_dfu_cb(UIEvent evt, int item)
{
  switch (item) {
  case 0:
      ui_enter_dfu();
  }
}

static void
menu_save_cb(UIEvent evt, int item)
{
  if (item < 0 || item >= 5)
    return;
  if (caldata_save(item) == 0) {
    menu_move_back();
    ui_mode_normal();
    draw_cal_status();
  } else {
    show_dmesg();
    redraw_frame();
    request_to_redraw_grid();
    draw_menu();
  }
}

static void 
choose_active_trace(void)
{
  int i;
  if (trace[uistat.current_trace].enabled)
    // do nothing
    return;
  for (i = 0; i < 4; i++)
    if (trace[i].enabled) {
      uistat.current_trace = i;
      return;
    }
}

static void
menu_trace_cb(UIEvent evt, int item)
{
  if (item < 0 || item >= 4)
    return;
  if (trace[item].enabled) {
    if (item == uistat.current_trace) {
      // disable if active trace is selected
      trace[item].enabled = FALSE;
      choose_active_trace();
    } else {
      // make active selected trace
      uistat.current_trace = item;
    }
  } else {
    trace[item].enabled = TRUE;
    uistat.current_trace = item;
  }
  request_to_redraw_grid();
  draw_menu();
}

static void
menu_format_cb(UIEvent evt, int item)
{
  switch (item) {
  case 0:
    set_trace_type(uistat.current_trace, TRC_LOGMAG);
    break;
  case 1:
    set_trace_type(uistat.current_trace, TRC_PHASE);
    break;
  case 2:
    set_trace_type(uistat.current_trace, TRC_DELAY);
    break;
  case 3:
    set_trace_type(uistat.current_trace, TRC_SMITH);
    break;
  case 4:
    set_trace_type(uistat.current_trace, TRC_SWR);
    break;
  }

  request_to_redraw_grid();
  ui_mode_normal();
  //redraw_all();
}

static void
menu_format2_cb(UIEvent evt, int item)
{
  switch (item) {
  case 0:
    set_trace_type(uistat.current_trace, TRC_POLAR);
    break;
  case 1:
    set_trace_type(uistat.current_trace, TRC_LINEAR);
    break;
  case 2:
    set_trace_type(uistat.current_trace, TRC_REAL);
    break;
  case 3:
    set_trace_type(uistat.current_trace, TRC_IMAG);
    break;
  case 4:
    set_trace_type(uistat.current_trace, TRC_R);
    break;
  case 5:
    set_trace_type(uistat.current_trace, TRC_X);
    break;
  }

  request_to_redraw_grid();
  ui_mode_normal();
}

static void
menu_channel_cb(UIEvent evt, int item)
{
  if (item < 0 || item >= 2)
    return;
  set_trace_channel(uistat.current_trace, item);
  menu_move_back();
  ui_mode_normal();
}

static void
menu_transform_window_cb(UIEvent evt, int item)
{
  // TODO
  switch (item) {
    case 0:
      domain_mode = (domain_mode & ~TD_WINDOW) | TD_WINDOW_MINIMUM;
      ui_mode_normal();
      break;
    case 1:
      domain_mode = (domain_mode & ~TD_WINDOW) | TD_WINDOW_NORMAL;
      ui_mode_normal();
      break;
    case 2:
      domain_mode = (domain_mode & ~TD_WINDOW) | TD_WINDOW_MAXIMUM;
      ui_mode_normal();
      break;
  }
}

static void
menu_transform_cb(UIEvent evt, int item)
{
  int status;
  switch (item) {
    case 0:
      if ((domain_mode & DOMAIN_MODE) == DOMAIN_TIME) {
          domain_mode = (domain_mode & ~DOMAIN_MODE) | DOMAIN_FREQ;
      } else {
          domain_mode = (domain_mode & ~DOMAIN_MODE) | DOMAIN_TIME;
      }
      uistat.lever_mode = LM_MARKER;
      draw_frequencies();
      ui_mode_normal();
      break;
    case 1:
      domain_mode = (domain_mode & ~TD_FUNC) | TD_FUNC_LOWPASS_IMPULSE;
      ui_mode_normal();
      break;
    case 2:
      domain_mode = (domain_mode & ~TD_FUNC) | TD_FUNC_LOWPASS_STEP;
      ui_mode_normal();
      break;
    case 3:
      domain_mode = (domain_mode & ~TD_FUNC) | TD_FUNC_BANDPASS;
      ui_mode_normal();
      break;
    case 5:
      if (evt.isLeverLongPress()) {
        ui_mode_numeric(KM_VELOCITY_FACTOR);
        ui_process_numeric(evt);
      } else {
        ui_mode_keypad(KM_VELOCITY_FACTOR);
        ui_process_keypad(evt);
      }
      break;
  }
}


static void
menu_display_cb(UIEvent evt, int item)
{
  switch (item) {
    case 5:   // FLIP DISPLAY
      if(config.ui_options & UI_OPTIONS_FLIP)
        config.ui_options &= ~UI_OPTIONS_FLIP;
      else config.ui_options |= UI_OPTIONS_FLIP;

      if(config.ui_options & UI_OPTIONS_FLIP)
        ili9341_set_flip(true, true);
      else ili9341_set_flip(false, false);
      redraw_request |= 0xff;
      force_set_markmap();
      ili9341_fill(0, 0, 320, 240, 0);
      draw_all(true);
      draw_menu();
      plot_cancel();
  }
}

static void 
choose_active_marker(void)
{
  int i;
  for (i = 0; i < 4; i++)
    if (markers[i].enabled) {
      active_marker = i;
      return;
    }
  active_marker = -1;
}

static void
menu_scale_cb(UIEvent evt, int item)
{
  int km = KM_SCALE + item;
  if (km == KM_SCALE && trace[uistat.current_trace].type == TRC_DELAY) {
    km = KM_SCALEDELAY;
  }
  if (evt.isLeverLongPress()) {
    ui_mode_numeric(km);
    //ui_process_numeric(evt);
  } else {
    ui_mode_keypad(km);
    //ui_process_keypad(evt);
  }
}

static void
menu_stimulus_cb(UIEvent evt, int item)
{
  switch (item) {
  case 0: /* START */
  case 1: /* STOP */
  case 2: /* CENTER */
  case 3: /* SPAN */
  case 4: /* SWEEP POINTS */
  case 5: /* CW */
  {
    if(item != 4)
      uistat.lever_mode = item == 3 ? LM_SPAN : LM_CENTER;
    if (evt.isLeverLongPress()) {
      ui_mode_numeric(item);
    } else {
      ui_mode_keypad(item);
    }
    break;
  }
  }
}

static void 
menu_top_cb(UIEvent evt, int item)
{
  switch (item) {
  case 6: /* PAUSE */
    toggle_sweep();
    draw_menu();
    break;
  }
}

static freqHz_t
get_marker_frequency(int marker)
{
  if (marker < 0 || marker >= 4)
    return -1;
  if (!markers[marker].enabled)
    return -1;
  return frequencyAt(markers[marker].index);
}

static void
menu_marker_op_cb(UIEvent evt, int item)
{
  freqHz_t freq = get_marker_frequency(active_marker);
  if (freq < 0)
    return; // no active marker

  switch (item) {
  case 0: /* MARKER->START */
    set_sweep_frequency(ST_START, freq);
    break;
  case 1: /* MARKER->STOP */
    set_sweep_frequency(ST_STOP, freq);
    break;
  case 2: /* MARKER->CENTER */
    set_sweep_frequency(ST_CENTER, freq);
    break;
  case 3: /* MARKERS->SPAN */
    {
      if (previous_marker == -1 || active_marker == previous_marker) {
        // if only 1 marker is active, keep center freq and make span the marker comes to the edge  
        freqHz_t center = get_sweep_frequency(ST_CENTER);
        freqHz_t span = center - freq;
       if (span < 0) span = -span;
        set_sweep_frequency(ST_SPAN, span * 2);
      } else {
        // if 2 or more marker active, set start and stop freq to each marker
        freqHz_t freq2 = get_marker_frequency(previous_marker);
        if (freq2 < 0)
          return;
        if (freq > freq2) {
          freq2 = freq;
          freq = get_marker_frequency(previous_marker);
        }
        set_sweep_frequency(ST_START, freq);
        set_sweep_frequency(ST_STOP, freq2);
      }
    }
    break;
  case 4: /* MARKERS->EDELAY */
    { 
      if (uistat.current_trace == -1)
        break;
      complexf* array = measured[trace[uistat.current_trace].channel];
      float v = groupdelay_from_array(markers[active_marker].index, array);
      set_electrical_delay(electrical_delay + (v / 1e-12));
    }
    break;
  }
  ui_mode_normal();
  draw_cal_status();
  //redraw_all();
}

static void
menu_marker_search_cb(UIEvent evt, int item)
{
  int i;
  if (active_marker == -1)
    return;

  request_to_redraw_marker(active_marker, TRUE);
  switch (item) {
  case 0: /* maximum */
  case 1: /* minimum */
    uistat.marker_search_mode = (item == 0) ? MarkerSearchModes::Max : MarkerSearchModes::Min;
    i = marker_search(uistat.marker_search_mode);
    if (i != -1)
      markers[active_marker].index = i;
    draw_menu();
    break;
  case 2: /* search Left */
    i = marker_search_left(uistat.marker_search_mode, markers[active_marker].index);
    if (i != -1)
      markers[active_marker].index = i;
    draw_menu();
    break;
  case 3: /* search right */
    i = marker_search_right(uistat.marker_search_mode, markers[active_marker].index);
    if (i != -1)
      markers[active_marker].index = i;
    draw_menu();
    break;
  case 4: /* tracking */
    uistat.marker_tracking = !uistat.marker_tracking;
    draw_menu();
    break;
  }
  redraw_marker(active_marker, TRUE);
  uistat.lever_mode = LM_SEARCH;
}

void ui_marker_track() {
  if (uistat.marker_tracking) {
    int i = marker_search(uistat.marker_search_mode);
    if (i != -1 && active_marker != -1) {
      markers[active_marker].index = i;
      redraw_request |= REDRAW_MARKER;
    }
  }
}

static void
menu_marker_smith_cb(UIEvent evt, int item)
{
  marker_smith_format = item;
  redraw_marker(active_marker, TRUE);
  draw_menu();
}


void 
active_marker_select(UIEvent evt, int item)
{
  if (item == -1) {
    active_marker = previous_marker;
    previous_marker = -1;
    if (active_marker == -1) {
      choose_active_marker();
    }
  } else {
    if (previous_marker != active_marker)
      previous_marker = active_marker;
    active_marker = item;
  }
}

static void
menu_marker_sel_cb(UIEvent evt, int item)
{
  if (item >= 0 && item < 4) {
    if (markers[item].enabled) {
      if (item == active_marker) {
        // disable if active trace is selected
        markers[item].enabled = FALSE;
        active_marker_select(evt, -1);
      } else {
        active_marker_select(evt, item);
      }
    } else {
      markers[item].enabled = TRUE;
      active_marker_select(evt, item);
    }
  } else if (item == 4) { /* all off */
      markers[0].enabled = FALSE;
      markers[1].enabled = FALSE;
      markers[2].enabled = FALSE;
      markers[3].enabled = FALSE;
      previous_marker = -1;
      active_marker = -1;     
  } else if (item == 5) { /* marker delta */
    uistat.marker_delta = !uistat.marker_delta;
  }
  request_to_redraw_marker(active_marker, TRUE);
  draw_menu();
  uistat.lever_mode = LM_MARKER;
}

const menuitem_t menu_calop[] = {
  { MT_CALLBACK, "OPEN", menu_calop_cb },
  { MT_CALLBACK, "SHORT", menu_calop_cb },
  { MT_CALLBACK, "LOAD", menu_calop_cb },
  { MT_CALLBACK, "THRU", menu_calop_cb },
  { MT_CALLBACK, "DONE", menu_caldone_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_save[] = {
  { MT_CALLBACK, "SAVE 0", menu_save_cb },
  { MT_CALLBACK, "SAVE 1", menu_save_cb },
  { MT_CALLBACK, "SAVE 2", menu_save_cb },
  { MT_CALLBACK, "SAVE 3", menu_save_cb },
  { MT_CALLBACK, "SAVE 4", menu_save_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_cal[] = {
  { MT_SUBMENU, "CALIBRATE", NULL, menu_calop },
  { MT_SUBMENU, "SAVE", NULL, menu_save },
  { MT_CALLBACK, "RESET", menu_cal2_cb },
  { MT_CALLBACK, "CORRECTION", menu_cal2_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_trace[] = {
  { MT_CALLBACK, "TRACE 0", menu_trace_cb },
  { MT_CALLBACK, "TRACE 1", menu_trace_cb },
  { MT_CALLBACK, "TRACE 2", menu_trace_cb },
  { MT_CALLBACK, "TRACE 3", menu_trace_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_format2[] = {
  { MT_CALLBACK, "POLAR", menu_format2_cb },
  { MT_CALLBACK, "LINEAR", menu_format2_cb },
  { MT_CALLBACK, "REAL", menu_format2_cb },
  { MT_CALLBACK, "IMAG", menu_format2_cb },
  { MT_CALLBACK, "RESISTANCE", menu_format2_cb },
  { MT_CALLBACK, "REACTANCE", menu_format2_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_format[] = {
  { MT_CALLBACK, "LOGMAG", menu_format_cb },
  { MT_CALLBACK, "PHASE", menu_format_cb },
  { MT_CALLBACK, "DELAY", menu_format_cb },
  { MT_CALLBACK, "SMITH", menu_format_cb },
  { MT_CALLBACK, "SWR", menu_format_cb },
  { MT_SUBMENU, S_RARROW" MORE", NULL, menu_format2 },  
  //{ MT_CALLBACK, "LINEAR", menu_format_cb },
  //{ MT_CALLBACK, "SWR", menu_format_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_scale[] = {
  { MT_CALLBACK, "SCALE/DIV", menu_scale_cb },
  { MT_CALLBACK, "\2REFERENCE\0POSITION", menu_scale_cb },
  { MT_CALLBACK, "\2ELECTRICAL\0DELAY", menu_scale_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};


const menuitem_t menu_channel[] = {
  { MT_CALLBACK, "\2CH0\0REFLECT", menu_channel_cb },
  { MT_CALLBACK, "\2CH1\0THROUGH", menu_channel_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_transform_window[] = {
  { MT_CALLBACK, "MINIMUM", menu_transform_window_cb },
  { MT_CALLBACK, "NORMAL", menu_transform_window_cb },
  { MT_CALLBACK, "MAXIMUM", menu_transform_window_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_transform[] = {
  { MT_CALLBACK, "\2TRANSFORM\0ON", menu_transform_cb },
  { MT_CALLBACK, "\2LOW PASS\0IMPULSE", menu_transform_cb },
  { MT_CALLBACK, "\2LOW PASS\0STEP", menu_transform_cb },
  { MT_CALLBACK, "BANDPASS", menu_transform_cb },
  { MT_SUBMENU, "WINDOW", NULL, menu_transform_window },
  { MT_CALLBACK, "\2VELOCITY\0FACTOR", menu_transform_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_display[] = {
  { MT_SUBMENU, "TRACE", NULL, menu_trace },
  { MT_SUBMENU, "FORMAT", NULL, menu_format },
  { MT_SUBMENU, "SCALE", NULL, menu_scale },
  { MT_SUBMENU, "CHANNEL", NULL, menu_channel },
  { MT_SUBMENU, "TRANSFORM", NULL, menu_transform },
  { MT_CALLBACK, "\2FLIP\0DISPLAY", menu_display_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_stimulus[] = {
  { MT_CALLBACK, "START", menu_stimulus_cb },
  { MT_CALLBACK, "STOP", menu_stimulus_cb },
  { MT_CALLBACK, "CENTER", menu_stimulus_cb },
  { MT_CALLBACK, "SPAN", menu_stimulus_cb },
  { MT_CALLBACK, "\2SWEEP\0POINTS", menu_stimulus_cb },
  { MT_CALLBACK, "CW FREQ", menu_stimulus_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_sel[] = {
  { MT_CALLBACK, "MARKER 1", menu_marker_sel_cb },
  { MT_CALLBACK, "MARKER 2", menu_marker_sel_cb },
  { MT_CALLBACK, "MARKER 3", menu_marker_sel_cb },
  { MT_CALLBACK, "MARKER 4", menu_marker_sel_cb },
  { MT_CALLBACK, "ALL OFF", menu_marker_sel_cb },
  { MT_CALLBACK, "DELTA", menu_marker_sel_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_ops[] = {
  { MT_CALLBACK, S_RARROW"START", menu_marker_op_cb },
  { MT_CALLBACK, S_RARROW"STOP", menu_marker_op_cb },
  { MT_CALLBACK, S_RARROW"CENTER", menu_marker_op_cb },
  { MT_CALLBACK, S_RARROW"SPAN", menu_marker_op_cb },
  { MT_CALLBACK, S_RARROW"EDELAY", menu_marker_op_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_search[] = {
  //{ MT_CALLBACK, "OFF", menu_marker_search_cb },
  { MT_CALLBACK, "MAXIMUM", menu_marker_search_cb },
  { MT_CALLBACK, "MINIMUM", menu_marker_search_cb },
  { MT_CALLBACK, "\2SEARCH\0" S_LARROW" LEFT", menu_marker_search_cb },
  { MT_CALLBACK, "\2SEARCH\0" S_RARROW" RIGHT", menu_marker_search_cb },
  { MT_CALLBACK, "TRACKING", menu_marker_search_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_smith[] = {
  { MT_CALLBACK, "LIN", menu_marker_smith_cb },
  { MT_CALLBACK, "LOG", menu_marker_smith_cb },
  { MT_CALLBACK, "Re+Im", menu_marker_smith_cb },
  { MT_CALLBACK, "R+Xj", menu_marker_smith_cb },
  { MT_CALLBACK, "R+L/C", menu_marker_smith_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_marker[] = {
  { MT_SUBMENU, "\2SELECT\0MARKER", NULL, menu_marker_sel },
  { MT_SUBMENU, "SEARCH", NULL, menu_marker_search },
  { MT_SUBMENU, "OPERATIONS", NULL, menu_marker_ops },
  { MT_SUBMENU, "\2SMITH\0VALUE", NULL, menu_marker_smith },
  { MT_CANCEL, S_LARROW" BACK", NULL, NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_recall[] = {
  { MT_CALLBACK, "RECALL 0", menu_recall_cb },
  { MT_CALLBACK, "RECALL 1", menu_recall_cb },
  { MT_CALLBACK, "RECALL 2", menu_recall_cb },
  { MT_CALLBACK, "RECALL 3", menu_recall_cb },
  { MT_CALLBACK, "RECALL 4", menu_recall_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_dfu[] = {
  { MT_CALLBACK, "\2RESET AND\0ENTER DFU", menu_dfu_cb },
  { MT_CANCEL, S_LARROW"CANCEL", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_config[] = {
  { MT_CALLBACK, "TOUCH CAL", menu_config_cb },
  { MT_CALLBACK, "TOUCH TEST", menu_config_cb },
  { MT_CALLBACK, "SAVE", menu_config_cb },
  { MT_CALLBACK, "VERSION", menu_config_cb },
  { MT_CALLBACK, "DMESG", menu_config_cb },
  { MT_SUBMENU, S_RARROW"DFU", NULL, menu_dfu },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_top[] = {
  { MT_SUBMENU, "DISPLAY", NULL, menu_display },
  { MT_SUBMENU, "MARKER", NULL, menu_marker },
  { MT_SUBMENU, "STIMULUS", NULL, menu_stimulus },
  { MT_SUBMENU, "CAL", NULL, menu_cal },
  { MT_SUBMENU, "RECALL", NULL, menu_recall },
  { MT_SUBMENU, "CONFIG", NULL, menu_config },
  { MT_CALLBACK, "\2PAUSE\0SWEEP", menu_top_cb },
  { MT_NONE, NULL, NULL } // sentinel
};

#define MENU_STACK_DEPTH_MAX 4
uint8_t menu_current_level = 0;
const menuitem_t *menu_stack[4] = {
  menu_top, NULL, NULL, NULL
};

static void
ensure_selection(void)
{
  const menuitem_t *menu = menu_stack[menu_current_level];
  int i;
  for (i = 0; menu[i].type != MT_NONE; i++)
    ;
  if (selection >= i)
    selection = i-1;
}

static void menu_move_back(void)
{
  if (menu_current_level == 0)
    return;
  menu_current_level--;
  ensure_selection();
  erase_menu_buttons();
  draw_menu();
}

static void menu_push_submenu(const menuitem_t *submenu)
{
  if (menu_current_level < MENU_STACK_DEPTH_MAX-1)
    menu_current_level++;
  menu_stack[menu_current_level] = submenu;
  ensure_selection();
  erase_menu_buttons();
  draw_menu();
}

/*
static void menu_move_top(void)
{
  if (menu_current_level == 0)
    return;
  menu_current_level = 0;
  ensure_selection();
  erase_menu_buttons();
  draw_menu();
}
*/

void menu_invoke(UIEvent evt, int item)
{
  const menuitem_t *menu = menu_stack[menu_current_level];
  menu = &menu[item];

  switch (menu->type) {
  case MT_NONE:
  case MT_BLANK:
  case MT_CLOSE:
    ui_mode_normal();
    break;

  case MT_CANCEL:
    menu_move_back();
    break;

  case MT_CALLBACK: {
    menuaction_cb_t cb = (menuaction_cb_t)menu->callback;
    if (cb == NULL)
      return;
    (*cb)(evt, item);
    break;
  }

  case MT_SUBMENU:
    menu_push_submenu((const menuitem_t*)menu->reference);
    break;
  }
}

#define KP_X(x) (48*(x) + 2 + (320-64-192))
#define KP_Y(y) (48*(y) + 2)

#define KP_PERIOD 10
#define KP_MINUS 11
#define KP_X1 12
#define KP_K 13
#define KP_M 14
#define KP_G 15
#define KP_BS 16
#define KP_INF 17
#define KP_DB 18
#define KP_PLUSMINUS 19
#define KP_KEYPAD 20
#define KP_N 21
#define KP_P 22

typedef struct {
  uint16_t x, y;
  int8_t c;
} keypads_t;

const keypads_t *keypads;
uint8_t keypads_last_index;

const keypads_t keypads_freq[] = {
  { KP_X(1), KP_Y(3), KP_PERIOD },
  { KP_X(0), KP_Y(3), 0 },
  { KP_X(0), KP_Y(2), 1 },
  { KP_X(1), KP_Y(2), 2 },
  { KP_X(2), KP_Y(2), 3 },
  { KP_X(0), KP_Y(1), 4 },
  { KP_X(1), KP_Y(1), 5 },
  { KP_X(2), KP_Y(1), 6 },
  { KP_X(0), KP_Y(0), 7 },
  { KP_X(1), KP_Y(0), 8 },
  { KP_X(2), KP_Y(0), 9 },
  { KP_X(3), KP_Y(0), KP_G },
  { KP_X(3), KP_Y(1), KP_M },
  { KP_X(3), KP_Y(2), KP_K },
  { KP_X(3), KP_Y(3), KP_X1 },
  { KP_X(2), KP_Y(3), KP_BS },
  { 0, 0, -1 }
};

const keypads_t keypads_scale[] = {
  { KP_X(1), KP_Y(3), KP_PERIOD },
  { KP_X(0), KP_Y(3), 0 },
  { KP_X(0), KP_Y(2), 1 },
  { KP_X(1), KP_Y(2), 2 },
  { KP_X(2), KP_Y(2), 3 },
  { KP_X(0), KP_Y(1), 4 },
  { KP_X(1), KP_Y(1), 5 },
  { KP_X(2), KP_Y(1), 6 },
  { KP_X(0), KP_Y(0), 7 },
  { KP_X(1), KP_Y(0), 8 },
  { KP_X(2), KP_Y(0), 9 },
  { KP_X(3), KP_Y(3), KP_X1 },
  { KP_X(2), KP_Y(3), KP_BS },
  { 0, 0, -1 }
};

const keypads_t keypads_time[] = {
  { KP_X(1), KP_Y(3), KP_PERIOD },
  { KP_X(0), KP_Y(3), 0 },
  { KP_X(0), KP_Y(2), 1 },
  { KP_X(1), KP_Y(2), 2 },
  { KP_X(2), KP_Y(2), 3 },
  { KP_X(0), KP_Y(1), 4 },
  { KP_X(1), KP_Y(1), 5 },
  { KP_X(2), KP_Y(1), 6 },
  { KP_X(0), KP_Y(0), 7 },
  { KP_X(1), KP_Y(0), 8 },
  { KP_X(2), KP_Y(0), 9 },
  { KP_X(3), KP_Y(1), KP_N },
  { KP_X(3), KP_Y(2), KP_P },
  { KP_X(3), KP_Y(3), KP_MINUS },
  { KP_X(2), KP_Y(3), KP_BS },
  { 0, 0, -1 }
};

const keypads_t * const keypads_mode_tbl[] = {
  keypads_freq, // start
  keypads_freq, // stop
  keypads_freq, // center
  keypads_freq, // span
  keypads_scale, // sweep points
  keypads_freq, // cw freq
  keypads_scale, // scale
  keypads_scale, // refpos
  keypads_time, // electrical delay
  keypads_scale, // velocity factor
  keypads_time // scale of delay
};

const char * const keypad_mode_label[] = {
  "START", "STOP", "CENTER", "SPAN", "POINTS", "CW FREQ", "SCALE", "REFPOS", "EDELAY", "VELOCITY%", "DELAY"
};

void
draw_keypad(void)
{
  int i = 0;
  while (keypads[i].x) {
    uint16_t bg = config.menu_normal_color;
    if (i == selection)
      bg = config.menu_active_color;
    ili9341_fill(keypads[i].x, keypads[i].y, 44, 44, bg);
    ili9341_drawfont(keypads[i].c, &NF20x22, keypads[i].x+12, keypads[i].y+10, 0x0000, bg);
    i++;
  }
}

void
draw_numeric_area_frame(void)
{
  ili9341_fill(0, 208, 320, 32, 0xffff);
  ili9341_drawstring_5x7(keypad_mode_label[keypad_mode], 10, 220, 0x0000, 0xffff);
  ili9341_drawfont(KP_KEYPAD, &NF20x22, 300, 216, 0x0000, 0xffff);
}

void
draw_numeric_input(const char *buf)
{
  int i = 0;
  int x = 64;
  int focused = FALSE;
  const uint16_t xsim[] = { 8, 0, 0, 8, 0, 0, 8, 0, 0, 0, 0 };
  for (i = 0; i < (NUMERIC_INPUT_DIGITS + 1) && buf[i]; i++) {
    uint16_t fg = 0x0000;
    uint16_t bg = 0xffff;
    int c = buf[i];
    if (c == '.')
      c = KP_PERIOD;
    else if (c == '-')
      c = KP_MINUS;
    else if (c >= '0' && c <= '9')
      c = c - '0';
    else
      c = -1;

    if (uistat.digit == NUMERIC_INPUT_DIGITS-i-1) {
      fg = RGB565(128,255,128);
      focused = TRUE;
      if (uistat.digit_mode)
        bg = 0x0000;
    }

    if (c >= 0)
      ili9341_drawfont(c, &NF20x22, x, 208+4, fg, bg);
    else if (focused)
      ili9341_drawfont(0, &NF20x22, x, 208+4, fg, bg);
    else
      ili9341_fill(x, 208+4, 20, 24, bg);
      
    x += 20;
    if (xsim[i] > 0) {
      //ili9341_fill(x, 208+4, xsim[i], 20, bg);
      x += xsim[i];
    }
  }
  if (i <= NUMERIC_INPUT_DIGITS) {
      ili9341_fill(x, 208+4, 20*(NUMERIC_INPUT_DIGITS+1-i), 24, 0xffff);
  }
}

static int
menu_is_multiline(const char *label, const char **l1, const char **l2)
{
  if (label[0] != '\2')
    return FALSE;

  *l1 = &label[1];
  *l2 = &label[1] + strlen(&label[1]) + 1;
  return TRUE;
}

static void
menu_item_modify_attribute(const menuitem_t *menu, int item,
                           uint16_t *fg, uint16_t *bg)
{
  if (menu == menu_trace && item < 4) {
    if (trace[item].enabled)
      *bg = config.trace_color[item];
  } else if (menu == menu_marker_sel) {
    if (item < 4) {
      if (markers[item].enabled) {
        *bg = 0x0000;
        *fg = 0xffff;
      }
    } else if (item == 5) {
      if (uistat.marker_delta) {
        *bg = 0x0000;
        *fg = 0xffff;
      }
    }
  } else if (menu == menu_marker_search) {
    if (item == 4 && uistat.marker_tracking) {
      *bg = 0x0000;
      *fg = 0xffff;
    }
  } else if (menu == menu_marker_smith) {
    if (marker_smith_format == item) {
      *bg = 0x0000;
      *fg = 0xffff;
    }
  } else if (menu == menu_calop) {
    if ((item == 0 && (cal_status & CALSTAT_OPEN))
        || (item == 1 && (cal_status & CALSTAT_SHORT))
        || (item == 2 && (cal_status & CALSTAT_LOAD))
        || (item == 3 && (cal_status & CALSTAT_THRU))) {
      domain_mode = (domain_mode & ~DOMAIN_MODE) | DOMAIN_FREQ;
      *bg = 0x0000;
      *fg = 0xffff;
    }
  } else if (menu == menu_top) {
    if (item == 6 /* PAUSE */ && !sweep_enabled) {
      *bg = 0x0000;
      *fg = 0xffff;
    }
  } else if (menu == menu_cal) {
    if (item == 3 /* CORRECTION */ && (cal_status & CALSTAT_APPLY)) {
      *bg = 0x0000;
      *fg = 0xffff;
    }
  } else if (menu == menu_transform) {
      if ((item == 0 && (domain_mode & DOMAIN_MODE) == DOMAIN_TIME)
       || (item == 1 && (domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_IMPULSE)
       || (item == 2 && (domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_STEP)
       || (item == 3 && (domain_mode & TD_FUNC) == TD_FUNC_BANDPASS)
       ) {
        *bg = 0x0000;
        *fg = 0xffff;
      }
  } else if (menu == menu_transform_window) {
      if ((item == 0 && (domain_mode & TD_WINDOW) == TD_WINDOW_MINIMUM)
       || (item == 1 && (domain_mode & TD_WINDOW) == TD_WINDOW_NORMAL)
       || (item == 2 && (domain_mode & TD_WINDOW) == TD_WINDOW_MAXIMUM)
       ) {
        *bg = 0x0000;
        *fg = 0xffff;
      }
  }
  if(*bg == 0x0000 && ui_disabled)
    *bg = 0x6666;
  if(*fg == 0x0000 && ui_disabled)
    *fg = 0x6666;
}

void
draw_menu_buttons(const menuitem_t *menu)
{
  int i = 0;
  for (i = 0; i < 7; i++) {
    const char *l1, *l2;
    if (menu[i].type == MT_NONE)
      break;
    if (menu[i].type == MT_BLANK) 
      continue;
    int y = 32*i;
    uint16_t bg = config.menu_normal_color;
    uint16_t fg = 0x0000;
    // focus only in MENU mode but not in KEYPAD mode
    if (ui_mode == UI_MENU && i == selection)
      bg = config.menu_active_color;
    ili9341_fill(320-60, y, 60, 30, bg);
    
    menu_item_modify_attribute(menu, i, &fg, &bg);
    if (menu_is_multiline(menu[i].label, &l1, &l2)) {
      ili9341_drawstring_5x7(l1, 320-54, y+8, fg, bg);
      ili9341_drawstring_5x7(l2, 320-54, y+15, fg, bg);
    } else {
      ili9341_drawstring_5x7(menu[i].label, 320-54, y+12, fg, bg);
    }
  }
}

void
menu_select_touch(UIEvent evt, int i)
{
  selection = i;
  draw_menu();
  uiDisableProcessing();
  while(uiWaitEvent().type != UIEventTypes::Up);
  uiEnableProcessing();
  selection = -1;
  menu_invoke(evt, i);
}

void
menu_apply_touch(UIEvent evt)
{
  int touch_x, touch_y;
  const menuitem_t *menu = menu_stack[menu_current_level];
  int i;

  if(!touch_position(&touch_x, &touch_y, evt))
    return;
  for (i = 0; i < 7; i++) {
    if (menu[i].type == MT_NONE)
      break;
    if (menu[i].type == MT_BLANK) 
      continue;
    int y = 32*i;
    if (y-2 < touch_y && touch_y < y+30+2
        && 320-60 < touch_x) {
      menu_select_touch(evt, i);
      return;
    }
  }

  ui_mode_normal();
}

void
draw_menu(void)
{
  draw_menu_buttons(menu_stack[menu_current_level]);
}

void
erase_menu_buttons(void)
{
  uint16_t bg = 0;
  ili9341_fill(320-60, 0, 60, 32*7, bg);
}

void
erase_numeric_input(void)
{
  uint16_t bg = 0;
  ili9341_fill(0, 240-32, 320, 32, bg);
}

void
leave_ui_mode()
{
  if (ui_mode == UI_MENU) {
    request_to_draw_cells_behind_menu();
    erase_menu_buttons();
  } else if (ui_mode == UI_NUMERIC) {
    request_to_draw_cells_behind_numeric_input();
    erase_numeric_input();
    draw_frequencies();
  }
}

void
fetch_numeric_target(void)
{
  switch (keypad_mode) {
  case KM_START:
    uistat.value = get_sweep_frequency(ST_START);
    break;
  case KM_STOP:
    uistat.value = get_sweep_frequency(ST_STOP);
    break;
  case KM_CENTER:
    uistat.value = get_sweep_frequency(ST_CENTER);
    break;
  case KM_SPAN:
    uistat.value = get_sweep_frequency(ST_SPAN);
    break;
  case KM_POINTS:
    uistat.value = sweep_points;
    break;
  case KM_CW:
    uistat.value = get_sweep_frequency(ST_CW);
    break;
  case KM_SCALE:
    uistat.value = get_trace_scale(uistat.current_trace) * 1000;
    break;
  case KM_REFPOS:
    uistat.value = get_trace_refpos(uistat.current_trace) * 1000;
    break;
  case KM_EDELAY:
    uistat.value = get_electrical_delay();
    break;
  case KM_VELOCITY_FACTOR:
    uistat.value = velocity_factor * 100;
    break;
  case KM_SCALEDELAY:
    uistat.value = get_trace_scale(uistat.current_trace) * 1e12;
    break;
  }
  
  {
    uint32_t x = uistat.value;
    int n = 0;
    for (; x >= 10 && n < 9; n++)
      x /= 10;
    uistat.digit = n;
  }
  uistat.previous_value = uistat.value;
}

void set_numeric_value(void)
{
  switch (keypad_mode) {
  case KM_START:
    set_sweep_frequency(ST_START, uistat.value);
    break;
  case KM_STOP:
    set_sweep_frequency(ST_STOP, uistat.value);
    break;
  case KM_CENTER:
    set_sweep_frequency(ST_CENTER, uistat.value);
    break;
  case KM_SPAN:
    set_sweep_frequency(ST_SPAN, uistat.value);
    break;
  case KM_POINTS:
    set_sweep_points(uistat.value);
    break;
  case KM_CW:
    set_sweep_frequency(ST_CW, uistat.value);
    break;
  case KM_SCALE:
    set_trace_scale(uistat.current_trace, uistat.value / 1000.0);
    break;
  case KM_REFPOS:
    set_trace_refpos(uistat.current_trace, uistat.value / 1000.0);
    break;
  case KM_EDELAY:
    set_electrical_delay(uistat.value);
    break;
  case KM_VELOCITY_FACTOR:
    velocity_factor = uistat.value / 100.f;
    break;
  }
}

void
draw_numeric_area(void)
{
  char buf[NUMERIC_INPUT_DIGITS+1];

  // if NUMERIC_INPUT_DIGITS is changed, the below format string
  // must be changed as well.
  static_assert(NUMERIC_INPUT_DIGITS == 10);
  chsnprintf(buf, sizeof buf, "%10ld", uistat.value);

  draw_numeric_input(buf);
}


void
ui_mode_menu(void)
{
  if (ui_mode == UI_MENU) 
    return;

  ui_mode = UI_MENU;
  /* narrowen plotting area */
  area_width = AREA_WIDTH_NORMAL - (64-8);
  area_height = HEIGHT;
  ensure_selection();
  draw_menu();
}

void
ui_mode_numeric(int _keypad_mode)
{
  if (ui_mode == UI_NUMERIC) 
    return;

  leave_ui_mode();
  
  // keypads array
  keypad_mode = _keypad_mode;
  ui_mode = UI_NUMERIC;
  area_width = AREA_WIDTH_NORMAL;
  area_height = 240-32;//HEIGHT - 32;

  draw_numeric_area_frame();
  fetch_numeric_target();
  draw_numeric_area();
  enable_refresh(false);
}

void
ui_mode_keypad(int _keypad_mode)
{
  if (ui_mode == UI_KEYPAD) 
    return;

  kp_index = 0;

  // keypads array
  keypad_mode = _keypad_mode;
  keypads = keypads_mode_tbl[_keypad_mode];
  int i;
  for (i = 0; keypads[i+1].c >= 0; i++)
    ;
  keypads_last_index = i;

  ui_mode = UI_KEYPAD;
  area_width = AREA_WIDTH_NORMAL - (64-8);
  area_height = HEIGHT - 32;
  draw_menu();
  draw_keypad();
  draw_numeric_area_frame();
  fetch_numeric_target();
  draw_numeric_area();
  enable_refresh(false);
  plot_cancel();
}

void
ui_mode_normal(void)
{
  if (ui_mode == UI_NORMAL) 
    return;

  area_width = AREA_WIDTH_NORMAL;
  area_height = HEIGHT;
  leave_ui_mode();
  ui_mode = UI_NORMAL;
}

static void
lever_move_marker(UIEvent evt)
{
  if (active_marker >= 0 && markers[active_marker].enabled) {
    request_to_redraw_marker(active_marker, TRUE);
    auto& am = markers[active_marker];
    int step = evt.isTick() ? 3 : 1;
    if (evt.isJogLeft()) {
      am.index -= step;
      if(am.index < 0)
        am.index = 0;
    }
    if (evt.isJogRight()) {
      am.index += step;
      if(am.index >= current_props._sweep_points)
        am.index = current_props._sweep_points - 1;
    }
    am.frequency = frequencyAt(am.index);
    request_to_redraw_marker(active_marker, TRUE);
  }
}

static void
lever_search_marker(UIEvent evt)
{
  if (active_marker >= 0) {
    request_to_redraw_marker(active_marker, TRUE);
    if (evt.isJogLeft()) {
      int i = marker_search_left(uistat.marker_search_mode, markers[active_marker].index);
      if (i != -1)
        markers[active_marker].index = i;
    } else if (evt.isJogRight()) {
      int i = marker_search_right(uistat.marker_search_mode, markers[active_marker].index);
      if (i != -1)
        markers[active_marker].index = i;
    }
    redraw_marker(active_marker, TRUE);
  }
}

// ex. 10942 -> 10000
//      6791 ->  5000
//       341 ->   200
static freqHz_t
step_round(freqHz_t v)
{
  // decade step
  freqHz_t x = 1;
  for (x = 1; x*10 < v; x *= 10)
    ;
  
  // 1-2-5 step
  if (x * 2 > v)
    return x;
  else if (x * 5 > v)
    return x * 2;
  else 
    return x * 5;
}

static void
lever_zoom_span(UIEvent evt)
{
  freqHz_t span = get_sweep_frequency(ST_SPAN);
  if (evt.isJogLeft()) {
    span = step_round(span - 1);
    set_sweep_frequency(ST_SPAN, span);
  } else if (evt.isJogRight()) {
    span = step_round(span + 1);
    span = step_round(span * 3);
    set_sweep_frequency(ST_SPAN, span);
  }
}

static void
lever_move_center(UIEvent evt)
{
  freqHz_t center = get_sweep_frequency(ST_CENTER);
  freqHz_t span = get_sweep_frequency(ST_SPAN);
  span = step_round(span / 3);
  if (evt.isJogRight()) {
    set_sweep_frequency(ST_CENTER, center + span);
  } else if (evt.isJogLeft()) {
    set_sweep_frequency(ST_CENTER, center - span);
  }
}


static void
ui_process_normal(UIEvent evt)
{
  if (evt.isLeverClick()) {
    ui_mode_menu();
  }
  if(evt.isJog()) {
#ifdef ENABLE_LEVER_MODES
    switch (uistat.lever_mode) {
      case LM_MARKER: lever_move_marker(evt);   break;
      case LM_SEARCH: lever_search_marker(evt); break;
      case LM_CENTER: lever_move_center(evt);   break;
      case LM_SPAN:   lever_zoom_span(evt);     break;      
    }
#else
    lever_move_marker(evt);
#endif
  }
  if(evt.isJogEnd()) {
    if (active_marker >= 0)
      request_to_redraw_marker(active_marker, TRUE);
  }
  if(evt.isTouchPress()) {
    if (touch_pickup_marker()) {
      return;
    }
    // switch menu mode
    selection = -1;
    ui_mode_menu();
  }
}

static void
ui_process_menu(UIEvent evt)
{
  if (evt.isLeverClick() || evt.isLeverLongPress()) {
    if(selection < 0)
      goto menuclose;
    menu_invoke(evt, selection);
    return;
  }
  if (evt.isJogRight()) {
    if(menu_stack[menu_current_level][selection+1].type == MT_NONE)
      goto menuclose;
    selection++;
    draw_menu();
  }
  if (evt.isJogLeft()) {
    if (selection == 0)
      goto menuclose;
    if(selection < 0)
      return;
    selection--;
    draw_menu();
  }
  if(evt.isTouchPress()) {
    menu_apply_touch(evt);
  }
  return;
menuclose:
  ui_mode_normal();
}

static int
keypad_click(int key) 
{
  int c = keypads[key].c;
  if ((c >= KP_X1 && c <= KP_G) || c == KP_N || c == KP_P) {
    float scale = 1;
    if (c >= KP_X1 && c <= KP_G) {
      int n = c - KP_X1;
      while (n-- > 0)
        scale *= 1000;
    } else if (c == KP_N) {
      scale *= 1000;
    }
    /* numeric input done */
    float value = my_atof(kp_buf) * scale;
    switch (keypad_mode) {
    case KM_START:
      set_sweep_frequency(ST_START, value);
      break;
    case KM_STOP:
      set_sweep_frequency(ST_STOP, value);
      break;
    case KM_CENTER:
      set_sweep_frequency(ST_CENTER, value);
      break;
    case KM_SPAN:
      set_sweep_frequency(ST_SPAN, value);
      break;
    case KM_POINTS:
      set_sweep_points(value);
      break;
    case KM_CW:
      set_sweep_frequency(ST_CW, value);
      break;
    case KM_SCALE:
      set_trace_scale(uistat.current_trace, value);
      break;
    case KM_REFPOS:
      set_trace_refpos(uistat.current_trace, value);
      break;
    case KM_EDELAY:
      set_electrical_delay(value); // pico seconds
      break;
    case KM_VELOCITY_FACTOR:
      velocity_factor = value / 100.f;
      break;
    case KM_SCALEDELAY:
      set_trace_scale(uistat.current_trace, value * 1e-12); // pico second
      break;
    }

    return KP_DONE;
  } else if (c <= 9 && kp_index < NUMINPUT_LEN)
    kp_buf[kp_index++] = '0' + c;
  else if (c == KP_PERIOD && kp_index < NUMINPUT_LEN) {
    // check period in former input
    int j;
    for (j = 0; j < kp_index && kp_buf[j] != '.'; j++)
      ;
    // append period if there are no period
    if (kp_index == j)
      kp_buf[kp_index++] = '.';
  } else if (c == KP_MINUS) {
    if (kp_index == 0)
      kp_buf[kp_index++] = '-';
  } else if (c == KP_BS) {
    if (kp_index == 0) {
      return KP_CANCEL;
    }
    --kp_index;
  }
  kp_buf[kp_index] = '\0';
  draw_numeric_input(kp_buf);
  return KP_CONTINUE;
}

static int
keypad_apply_touch(UIEvent evt)
{
  int touch_x, touch_y;
  int i = 0;

  if(!touch_position(&touch_x, &touch_y, evt))
    return -1;

  while (keypads[i].x) {
    if (keypads[i].x-2 < touch_x && touch_x < keypads[i].x+44+2
        && keypads[i].y-2 < touch_y && touch_y < keypads[i].y+44+2) {
      // draw focus
      selection = i;
      draw_keypad();
      uiDisableProcessing();
      uiWaitEvent();
      uiEnableProcessing();
      // erase focus
      selection = -1;
      draw_keypad();
      return i;
    }
    i++;
  }
  if (touch_y > 48 * 4) {
    // exit keypad mode
    return -2;
  }
  return -1;
}

static void
numeric_apply_touch(UIEvent evt)
{
  int touch_x, touch_y;
  if(!touch_position(&touch_x, &touch_y, evt))
    return;

  if (touch_x < 64) {
    ui_mode_normal();
    return;
  }
  if (touch_x > 64+9*20+8+8) {
    ui_mode_keypad(keypad_mode);
    ui_process_keypad(evt);
    return;
  }

  if (touch_y > 240-40) {
    int n = 9 - (touch_x - 64) / 20;
    uistat.digit = n;
    uistat.digit_mode = TRUE;
  } else {
    int step, n;
    if (touch_y < 100) {
      step = 1;
    } else {
      step = -1;
    }

    for (n = uistat.digit; n > 0; n--)
      step *= 10;
    uistat.value += step;
  }
  draw_numeric_area();
  
  uiWaitEvent();
  uistat.digit_mode = FALSE;
  draw_numeric_area();
  
  return;
}

static void
ui_process_numeric(UIEvent evt)
{
  if (evt.isLeverClick() || evt.isLeverLongPress()) {
    if (uistat.digit_mode) {
      uistat.digit_mode = FALSE;
      draw_numeric_area();
    } else {
      if (evt.type == UIEventTypes::LongPress) {
        uistat.digit_mode = TRUE;
        draw_numeric_area();
      } else {
        set_numeric_value();
        ui_mode_normal();
        enable_refresh(true);
      }
    }
  }

  if(evt.isJog()) {
    if (uistat.digit_mode) {
      if (evt.isJogLeft()) {
        if (uistat.digit < 8) {
          uistat.digit++;
          draw_numeric_area();
        } else {
          goto exit;
        }
      }
      if (evt.isJogRight()) {
        if (uistat.digit > 0) {
          uistat.digit--;
          draw_numeric_area();
        } else {
          goto exit;
        }
      }
    } else {
      int32_t step = 1;
      int n;
      for (n = uistat.digit; n > 0; n--)
        step *= 10;
      if (evt.isJogRight()) {
        uistat.value += step;
        draw_numeric_area();
      }
      if (evt.isJogLeft()) {
        uistat.value -= step;
        draw_numeric_area();
      }
    }
  }

  if(evt.isTouchPress()) {
    numeric_apply_touch(evt);
  }

  return;

 exit:
  // cancel operation
  ui_mode_normal();
  enable_refresh(true);
}

void
ui_process_keypad(UIEvent evt)
{
  if (evt.isJogLeft()) {
    selection--;
    if (selection < 0)
      selection = keypads_last_index;
    draw_keypad();
    return;
  }
  if (evt.isJogRight()) {
    selection++;
    if (keypads[selection].c < 0) {
      // reaches to tail
      selection = 0;
    }
    draw_keypad();
    return;
  }

  if (evt.isLeverClick()) {
    if (keypad_click(selection))
      /* exit loop on done or cancel */
      goto return_to_normal;
  }

  if (evt.isTouchPress()) {
    int key = keypad_apply_touch(evt);
    if (key >= 0 && keypad_click(key))
      /* exit loop on done or cancel */
      goto return_to_normal;
    else if (key == -2) {
      //xxx;
      return;
    }
  }
  return;

return_to_normal:
  redraw_frame();
  request_to_redraw_grid();
  ui_mode_normal();
  //redraw_all();
  uiEnableProcessing();
  enable_refresh(true);
}


static void
drag_marker(int t, int m)
{
  int status;
  /* wait touch release */
  while(true) {
    int touch_x, touch_y;
    int index;
    if(!touch_position(&touch_x, &touch_y))
      break;
    touch_x -= OFFSETX;
    touch_y -= OFFSETY;
    index = search_nearest_index(touch_x, touch_y, t);
    if (index >= 0) {
      markers[m].index = index;
      markers[m].frequency = frequencyAt(index);
      redraw_marker(m, TRUE);
    }
  }
}

static int 
sq_distance(int x0, int y0)
{
  return x0*x0 + y0*y0;
}

static int
touch_pickup_marker(void)
{
  int touch_x, touch_y;
  int m, t;
  if(!touch_position(&touch_x, &touch_y))
    return FALSE;
  touch_x -= OFFSETX;
  touch_y -= OFFSETY;

  for (m = 0; m < 4; m++) {
    if (!markers[m].enabled)
      continue;

    for (t = 0; t < 4; t++) {
      int x, y;
      if (!trace[t].enabled)
        continue;

      marker_position(m, t, &x, &y);

      if (sq_distance(x - touch_x, y - touch_y) < 400) {
        if (active_marker != m) {
          previous_marker = active_marker;
          active_marker = m;
          request_to_redraw_marker(active_marker, TRUE);
        }
        // select trace
        uistat.current_trace = t;
        
        // drag marker until release
        drag_marker(t, m);
        return TRUE;
      }
    }
  }

  return FALSE;
}



void
ui_process(UIEvent evt)
{
  // if the display is flipped, flip jog left/right too
  if(config.ui_options & UI_OPTIONS_FLIP) {
    if(evt.button == UIEventButtons::LeverLeft)
      evt.button = UIEventButtons::LeverRight;
    else if(evt.button == UIEventButtons::LeverRight)
      evt.button = UIEventButtons::LeverLeft;
  }

  if(uiEventsEnabled)
    lastUIEvent = {};
  else {
    lastUIEvent = evt;
    return;
  }

  if(ui_disabled) return;

  if(evt.isTouchPress())
    awd_count++;

  switch (ui_mode) {
  case UI_NORMAL:
    ui_process_normal(evt);
    break;    
  case UI_MENU:
    ui_process_menu(evt);
    break;    
  case UI_NUMERIC:
    ui_process_numeric(evt);
    break;    
  case UI_KEYPAD:
    ui_process_keypad(evt);
    break;
  case UI_USB_MODE:
    if(evt.isLeverLongPress()) {
      UIActions::reconnectUSB();
    }
    break;
  }
}
