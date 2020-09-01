#include "vna_measurement.hpp"
#include "ui.hpp"
#include "fifo.hpp"
#include "main.hpp"
#include "ili9341.hpp"
#include "globals.hpp"
#include <board.hpp>
#include <mculib/printf.hpp>
#include <mculib/message_log.hpp>
#include <mculib/printk.hpp>
#include <libopencm3/cm3/scb.h>

using namespace mculib;
using namespace board;


template<int fifoSize>
static void discardPoints(FIFO<complexf, fifoSize>& dpFIFO, int n) {
	dpFIFO.clear();

	// skip n data points
	for(int i=0; i<n; i++) {
		while(!dpFIFO.readable());
		dpFIFO.dequeue();
	}
}


void performGainCal(VNAMeasurement& vnaMeasurement, float* gainTable, int maxGain) {
	// measure the reference channel at all gain settings to determine baseband gains

	auto old_emitDataPoint = vnaMeasurement.emitDataPoint;
	auto old_phaseChanged = vnaMeasurement.phaseChanged;
	FIFO<complexf, 32> dpFIFO;
	volatile int currGain = 0;

	vnaMeasurement.emitDataPoint = [&](int freqIndex, freqHz_t freqHz, const VNAObservation& v, const complexf* ecal) {
		digitalWrite(board::led, vnaMeasurement.clipFlag?1:0);
		dpFIFO.enqueue(v[1]);
	};

	// override phaseChanged, set bbgain to desired value
	vnaMeasurement.phaseChanged = [&](VNAMeasurementPhases ph) {
		old_phaseChanged(ph);
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(currGain));
	};

	vnaMeasurement.ecalIntervalPoints = 10000;
	vnaMeasurement.nPeriods = MEASUREMENT_NPERIODS_NORMAL;

	// only > 2.5GHz can we be sure that the max gain setting will not clip when measuring the reference channel
	vnaMeasurement.setSweep(2600000000, 0, 1, 1);

	for(currGain=0; currGain <= maxGain; currGain++) {
		discardPoints(dpFIFO, 3);
		float mag = 0;
		for(int i=0; i<10; i++) {
			while(!dpFIFO.readable());
			auto& dp = dpFIFO.read();
			mag += abs(dp);
			dpFIFO.dequeue();
		}
		gainTable[currGain] = 1.f/mag;
	}

	// normalize gain table
	for(currGain=1; currGain <= maxGain; currGain++) {
		gainTable[currGain] /= gainTable[0];
	}
	gainTable[0] = 1.f;

	// reset callbacks
	vnaMeasurement.emitDataPoint = old_emitDataPoint;
	vnaMeasurement.phaseChanged = old_phaseChanged;
}
