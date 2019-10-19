#pragma once

#include <stdint.h>
#include <mculib/small_function.hpp>

// DO NOT INCLUDE THIS FILE OTHER THAN FROM main2.cpp and ui.cpp!!!!!
// ONLY ui.cpp IS ALLOWED TO CALL FUNCTIONS DEFINED IN main2.cpp!!!!!

// The following are the application callback functions that the
// UI code will call to perform actions.

void cal_collect(int type);
void cal_done(void);


void set_sweep_frequency(int type, int32_t frequency);
uint32_t get_sweep_frequency(int type);



void toggle_sweep(void);



void set_trace_type(int t, int type);
void set_trace_channel(int t, int channel);
void set_trace_scale(int t, float scale);
void set_trace_refpos(int t, float refpos);

void set_electrical_delay(float picoseconds);
float get_electrical_delay(void);

void apply_edelay_at(int i);

void set_frequencies(uint32_t start, uint32_t stop, int16_t points);
void update_frequencies(void);

// process up to one outstanding event in the main event queue.
// This can lead to ui_process() being called.
void application_doSingleEvent();

// register a callback to be called from the main thread at earliest opportunity.
void enqueueEvent(const small_function<void()>& cb);

