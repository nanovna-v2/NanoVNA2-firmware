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
	int j, i;
	volatile int currGain = 0;
	auto old_emitDataPoint = vnaMeasurement.emitDataPoint;
	auto old_phaseChanged = vnaMeasurement.phaseChanged;
	auto old_avg = current_props._avg;
	auto old_pow = current_props._adf4350_txPower;
	FIFO<complexf, 32> dpFIFO;
	current_props._avg = 1;
	current_props._adf4350_txPower = 0;

	// override phaseChanged, set bbgain to desired value
	vnaMeasurement.phaseChanged = [&](VNAMeasurementPhases ph) {
		rfsw(RFSW_REFL, RFSW_REFL_ON);
		rfsw(RFSW_RECV, RFSW_RECV_REFL);
		rfsw(RFSW_ECAL, RFSW_ECAL_OPEN);
		rfsw(RFSW_BBGAIN, RFSW_BBGAIN_GAIN(currGain));
	};

	// disable ecal during gain cal
	vnaMeasurement.ecalIntervalPoints = 10000;
	vnaMeasurement.nPeriods = MEASUREMENT_NPERIODS_NORMAL;
	vnaMeasurement.setSweep(DEFAULT_FREQ, 0, 1, 1);
	vnaMeasurement.emitDataPoint = [&](int freqIndex, freqHz_t freqHz, const VNAObservation& v, const complexf* ecal) {
		dpFIFO.enqueue(v[1]);
	};

	for(j = 0; j <= maxGain; j++) {
		currGain = j;
		discardPoints(dpFIFO, 3);
		int nValues = 10;
		if(currGain == 3)
			nValues = 40;

		float mag = 0;
		for(i=0; i<nValues; i++) {
			while(!dpFIFO.readable());
			auto& dp = dpFIFO.read();
			mag += abs(dp);
			dpFIFO.dequeue();
		}
		gainTable[j] = mag / nValues;
	}

	// normalize first entry to 1.0
	float norm = gainTable[0];
	gainTable[0] = 1.f; // norm / gainTable[0];
	for(j = 1; j <= maxGain; j++) {
		gainTable[j] = norm / gainTable[j];
	}

	current_props._avg = old_avg;
	current_props._adf4350_txPower = old_pow;
	// reset callbacks
	vnaMeasurement.emitDataPoint = old_emitDataPoint;
	vnaMeasurement.phaseChanged = old_phaseChanged;
}
