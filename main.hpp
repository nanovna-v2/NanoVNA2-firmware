#pragma once

#include <stdint.h>
#include <mculib/small_function.hpp>
#include "common.hpp"

// DO NOT INCLUDE THIS FILE OTHER THAN FROM main2.cpp and ui.cpp!!!!!
// ONLY ui.cpp IS ALLOWED TO CALL FUNCTIONS DEFINED IN main2.cpp!!!!!

// The following are the application callback functions that the
// UI code will call to perform actions.

namespace UIActions {

	void cal_collect(int type);
	void cal_done(void);

	void set_sweep_frequency(SweepParameter type, freqHz_t frequency);
	void set_sweep_points(int points);
	freqHz_t get_sweep_frequency(int type);
	freqHz_t frequencyAt(int index);

	void toggle_sweep(void);
	void enable_refresh(bool enable);

	void set_trace_type(int t, int type);
	void set_trace_channel(int t, int channel);
	void set_trace_scale(int t, float scale);
	void set_trace_refpos(int t, float refpos);

	void set_electrical_delay(float picoseconds);
	float get_electrical_delay(void);

	void apply_edelay_at(int i);


	int caldata_save(int id);
	int caldata_recall(int id);

	int config_save();
	int config_recall();

	void printTouchCal();

	void enterDFU();

	void reconnectUSB();

	// process all outstanding events in the main event queue.
	void application_doEvents();

	// process up to one outstanding event in the main event queue.
	// This can lead to ui_process() being called.
	void application_doSingleEvent();

	// register a callback to be called from the main thread at earliest opportunity.
	void enqueueEvent(const small_function<void()>& cb);
}

