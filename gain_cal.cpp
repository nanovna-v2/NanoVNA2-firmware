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


// measure the attenuation at each gain setting
void performGainCal(VNAMeasurement& vnaMeasurement, float* gainTable, int maxGain) {
	auto old_emitDataPoint = vnaMeasurement.emitDataPoint;
	auto old_phaseChanged = vnaMeasurement.phaseChanged;
	FIFO<complexf, 32> dpFIFO;
	volatile int currGain = 0;


	// override phaseChanged, set bbgain to desired value
	vnaMeasurement.phaseChanged = [&](VNAMeasurementPhases ph) {
		old_phaseChanged(ph);
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(currGain));
	};

	// disable ecal during gain cal
	vnaMeasurement.ecalIntervalPoints = 10000;
	vnaMeasurement.nPeriods = MEASUREMENT_NPERIODS_NORMAL;
	vnaMeasurement.setSweep(DEFAULT_FREQ, 0, 1, 1);
	vnaMeasurement.emitDataPoint = [&](int freqIndex, freqHz_t freqHz, const VNAObservation& v, const complexf* ecal) {
		dpFIFO.enqueue(v[1]);
	};

	// use averaging 10x
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

	// normalize first entry to 1.0
	for(currGain=1; currGain <= maxGain; currGain++) {
		gainTable[currGain] /= gainTable[0];
	}
	gainTable[0] = 1.f;

	// reset callbacks
	vnaMeasurement.emitDataPoint = old_emitDataPoint;
	vnaMeasurement.phaseChanged = old_phaseChanged;
}
