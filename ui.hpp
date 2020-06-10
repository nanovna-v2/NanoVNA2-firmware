#pragma once

#include <stdint.h>
#include "uihw.hpp"

#ifndef DISPLAY_ST7796
// Maximum menu buttons count
#define MENU_BUTTON_MAX     7
// Menu buttons size
#define MENU_BUTTON_WIDTH  60
#define MENU_BUTTON_HEIGHT 30
// Height of numerical input field (at bottom)
#define NUM_INPUT_HEIGHT   30

#define KP_WIDTH     48
#define KP_HEIGHT    48
// Key x, y position (0 - 15) on screen
#define KP_GET_X(posx) ((posx)*KP_WIDTH + (LCD_WIDTH-64-KP_WIDTH*4))
#define KP_GET_Y(posy) ((posy)*KP_HEIGHT + 12 )
#else
// Maximum menu buttons count
#define MENU_BUTTON_MAX     7
// Menu buttons size
#define MENU_BUTTON_WIDTH  80
#define MENU_BUTTON_HEIGHT 38
// Height of numerical input field (at bottom)
#define NUM_INPUT_HEIGHT   30

#define KP_WIDTH     64
#define KP_HEIGHT    64
// Key x, y position (0 - 15) on screen
#define KP_GET_X(posx) ((posx)*KP_WIDTH + (LCD_WIDTH-128-KP_WIDTH*4))
#define KP_GET_Y(posy) ((posy)*KP_HEIGHT + 20 )
#endif

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
