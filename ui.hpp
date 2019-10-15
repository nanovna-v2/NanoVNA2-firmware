#pragma once

#include <stdint.h>

/*
 * ui.c
 */
void ui_init(void);
void ui_process(void);


extern uint8_t operation_requested;

void handle_touch_interrupt(void);

#define TOUCH_THRESHOLD 2000

void touch_cal_exec(void);
void touch_draw_test(void);
void touch_start_watchdog(void);
void touch_position(int *x, int *y);
void enter_dfu(void);

void ui_mode_menu(void);
