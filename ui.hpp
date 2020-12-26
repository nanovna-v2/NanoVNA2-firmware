#pragma once

#include <stdint.h>
#include "uihw.hpp"

#ifndef DISPLAY_ST7796
// Maximum menu buttons count
#define MENU_BUTTON_MAX         9
// Menu buttons size
#define MENU_BUTTON_WIDTH      68
#define MENU_BUTTON_HEIGHT     26
#define MENU_BUTTON_BORDER      1
#define KEYBOARD_BUTTON_BORDER  2
// Height of numerical input field (at bottom)
#define NUM_INPUT_HEIGHT       32
// Distance for marker start grab
#define MARKER_PICKUP_DISTANCE 30
// On screen keyboard button size
#if 1
#define KP_WIDTH                  ((LCD_WIDTH - MENU_BUTTON_WIDTH) / 4) // numeric keypad button width
#define KP_HEIGHT                 ((LCD_HEIGHT - NUM_INPUT_HEIGHT) / 4) // numeric keypad button height
// Key x, y position (0 - 15) on screen
#define KP_GET_X(posx)            ((posx) * KP_WIDTH)                   // numeric keypad left
#define KP_GET_Y(posy)            ((posy) * KP_HEIGHT)                  // numeric keypad top
#else
#define KP_WIDTH     48
#define KP_HEIGHT    48
// Key x, y position (0 - 15) on screen
#define KP_GET_X(posx) ((posx)*KP_WIDTH + (LCD_WIDTH-MENU_BUTTON_WIDTH-16-KP_WIDTH*4))
#define KP_GET_Y(posy) ((posy)*KP_HEIGHT + 12 )
#endif
#else
// Maximum menu buttons count
#define MENU_BUTTON_MAX         8
// Menu buttons size
#define MENU_BUTTON_WIDTH      84
#define MENU_BUTTON_HEIGHT     36
#define MENU_BUTTON_BORDER      1
#define KEYBOARD_BUTTON_BORDER  2
// Height of numerical input field (at bottom)
#define NUM_INPUT_HEIGHT   32
// Distance for marker start grab
#define MARKER_PICKUP_DISTANCE 20
#if 1
#define KP_WIDTH                  ((LCD_WIDTH - MENU_BUTTON_WIDTH) / 4) // numeric keypad button width
#define KP_HEIGHT                 ((LCD_HEIGHT - NUM_INPUT_HEIGHT) / 4) // numeric keypad button height
// Key x, y position (0 - 15) on screen
#define KP_GET_X(posx)            ((posx) * KP_WIDTH)                   // numeric keypad left
#define KP_GET_Y(posy)            ((posy) * KP_HEIGHT)                  // numeric keypad top
#else
#define KP_WIDTH     64
#define KP_HEIGHT    64
// Key x, y position (0 - 15) on screen
#define KP_GET_X(posx) ((posx)*KP_WIDTH + (LCD_WIDTH-MENU_BUTTON_WIDTH-16-KP_WIDTH*4))
#define KP_GET_Y(posy) ((posy)*KP_HEIGHT + 20 )
#endif
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
