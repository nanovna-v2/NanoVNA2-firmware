#pragma once

#include <stdint.h>
#include "uihw.hpp"


// only process one event
void ui_process(UIHW::UIEvent evt);

void uiEnableProcessing();
void uiDisableProcessing();

#define TOUCH_THRESHOLD 2000
#define NUMERIC_INPUT_DIGITS 10

void touch_cal_exec(void);
void touch_draw_test(void);
void touch_start_watchdog(void);
bool touch_position(int *x, int *y);
void ui_enter_dfu(void);

void ui_mode_normal(void);
void ui_mode_menu(void);
void ui_mode_usb(void);
void draw_numeric_input(const char *buf);
void draw_menu();

void ui_cal_collected();

// update marker position if marker tracking is enabled
void ui_marker_track();

void show_dmesg();
void show_message(const char* title, const char* message, int fg = 0xffff, int bg = 0x0000);
