#pragma once

#include <stdint.h>
#include "uihw.hpp"


void ui_init(void);

// only process one event
void ui_process(UIHW::UIEvent evt);


#define TOUCH_THRESHOLD 2000

void touch_cal_exec(void);
void touch_draw_test(void);
void touch_start_watchdog(void);
bool touch_position(int *x, int *y);
void ui_enter_dfu(void);

void ui_mode_menu(void);
void show_usb_data_mode(void);
void draw_numeric_input(const char *buf);
void draw_menu();

void ui_cal_collected();

void show_dmesg();
void show_message(const char* title, const char* message, int fg = 0xffff, int bg = 0x0000);
