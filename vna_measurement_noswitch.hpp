#pragma once
#include <mculib/small_function.hpp>
#include "common.hpp"
#include "sample_processor.hpp"



// implements sweep and dsp for no-switch VNAs (dedicated receiver
// for each of reference, reflected, and thru).
// given synthesizer controls and adc data feed, emit a stream
// of data points.
// nChannels is either 2 or 3. 2 => reference and reflect. 3 => reference, reflect, and thru.
template<int nChannels>
class VNAMeasurementNoSwitch {
public:
	typedef complex<int32_t> complexi;

	// how many periods to wait after changing synthesizer frequency
	uint32_t nWaitSynth = 100;

	// how many periods to average over
	uint32_t nPeriods = 20;

	// called when a new data point is available
	small_function<void(int freqIndex, uint64_t freqHz, const VNAObservationSet& v)> emitDataPoint;

	// called to change synthesizer frequency
	small_function<void(uint64_t freqHz)> frequencyChanged;

	VNAMeasurementNoSwitch(): sampleProcessor(_emitValue_t {this}) {}

	void init() { sampleProcessor.init(); }
	void setCorrelationTable(const int16_t* table, int length) {
		sampleProcessor.setCorrelationTable(table, length);
	}
	void processSamples(uint16_t* buf, int len) { sampleProcessor.process(buf, len); }

	// if points is 1, sets frequency to startFreqHz and disables sweep
	void setSweep(uint64_t startFreqHz, uint64_t stepFreqHz, int points, int dataPointsPerFreq=1) {
		sweepStartHz = startFreqHz;
		sweepStepHz = stepFreqHz;
		sweepPoints = points;
		sweepCurrPoint = 0;
		sweepDataPointsPerFreq = dataPointsPerFreq;

		currFreq = startFreqHz;
		periodCounterSynth = nWaitSynth;
		dpCounterSynth = 0;
		frequencyChanged(startFreqHz);
	}


	struct _emitValue_t {
		VNAMeasurementNoSwitch* m;
		void operator()(int32_t* valRe, int32_t* valIm);
	};
	
	SampleProcessor<_emitValue_t, nChannels> sampleProcessor;

public:
	// state variables

	// number of periods so far in this data point
	uint32_t periodCounter = 0;

	// number of periods left to wait
	uint32_t periodCounterSynth = 0;

	// number of data points since synthesizer frequency change
	uint32_t dpCounterSynth = 0;

	// number of frequency points since start of sweep
	int sweepCurrPoint = 0;

	// current data point variables
	complexi currDP[3];

	// sweep params
	uint64_t sweepStartHz = 0, sweepStepHz = 0;
	int sweepPoints = 1;
	int sweepDataPointsPerFreq = 1;

	uint64_t currFreq;


	void sweepAdvance() {
		sweepCurrPoint++;
		if(sweepCurrPoint >= sweepPoints)
			sweepCurrPoint = 0;

		currFreq = sweepStartHz + sweepStepHz*sweepCurrPoint;
		frequencyChanged(currFreq);

		periodCounterSynth = nWaitSynth;
	}
	void sampleProcessor_emitValue(int32_t* valRe, int32_t* valIm) {
		if(periodCounterSynth > 0) {
			periodCounterSynth--;
			return;
		}
		for(int i=0; i<nChannels; i++)
			currDP[i] += complexi{valRe[i], valIm[i]};
		periodCounter++;

		if(periodCounter == nPeriods) {
			periodCounter = 0;
			// emit new data point
			VNAObservationSet value = {
					to_complexf(currDP[1]),
					to_complexf(currDP[0]),
					nChannels >= 3 ? to_complexf(currDP[2]) : 0.};

			emitDataPoint(sweepCurrPoint, currFreq, value);

			currDP[0] = currDP[1] = currDP[2] = 0;
			dpCounterSynth++;
			if(dpCounterSynth >= sweepDataPointsPerFreq && sweepPoints > 1) {
				dpCounterSynth = 0;
				sweepAdvance();
			}
		}
	}


	static inline complexf to_complexf(VNAMeasurementNoSwitch::complexi value) {
		return {(float) value.real(), (float) value.imag()};
	}
};


template<int nChannels>
void VNAMeasurementNoSwitch<nChannels>::_emitValue_t::operator()(int32_t* valRe, int32_t* valIm) {
	m->sampleProcessor_emitValue(valRe, valIm);
}

