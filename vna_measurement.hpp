#pragma once
#include <mculib/small_function.hpp>
#include "common.hpp"
#include "sample_processor.hpp"


enum class VNAMeasurementPhases {
	REFERENCE,
	REFL,
	THRU,

	ECALLOAD,
	ECALSHORT,
	ECALTHRU
};

// implements sweep, rf switch timing, and dsp for single-receiver
// switched path VNAs (one receiver with switches to select reference,
// reflected, and thru paths).
// given switch & synthesizer controls and adc data feed, emit a stream
// of data points.
class VNAMeasurement {
public:
	typedef complex<int32_t> complexi;

	// how many periods to wait after changing rf switches
	uint16_t nWaitSwitch = 1;

	// how many periods to wait after changing synthesizer frequency
	uint16_t nWaitSynth = 30;

	// how many periods to average over
	uint16_t nMeasureCount = 0;
	uint16_t nPeriods = 14;
	uint16_t nPeriodsCalibrating = 28;
	uint16_t nPeriodsMultiplier = 1;

	// every ecalIntervalPoints we will measure one frequency point for ecal
	uint16_t ecalIntervalPoints = 8;

	// AGC parameters; VNAMeasurement will detect ADC clip events and inform the
	// host when baseband/rf gain needs to be changed.
	uint8_t gainMin = 0, gainMax = 3;

	float adcFullScale = 0;

	// automatically reset before each measurement; indicates whether the current
	// S11 data point is corrupted when emitDataPoint() is called.
	bool clipFlag = false;

	// same as clipFlag, but for S21
//	bool clipFlag2 = false;

	// called when a new data point is available.
	// ecal is load, short, thru.
	small_function<void(int freqIndex, freqHz_t freqHz, const VNAObservationSet& v, const complexf* ecal)> emitDataPoint;

	// called to change rf switch direction;
	// the function may assume the phase progression is always:
	// REFERENCE, REFL1, REFL2, THRU,
	// except that REFERENCE may be switched to at any time and from any phase.
	small_function<void(VNAMeasurementPhases ph)> phaseChanged;

	// called to change synthesizer frequency
	small_function<void(freqHz_t freqHz)> frequencyChanged;

	// called when sweep setup change is processed in measurement 'thread'
	small_function<void(freqHz_t start, freqHz_t stop)> sweepSetupChanged;

	// called to change overall system gain; gain is a user defined value
	// and VNAMeasurement will only increment or decrement it if ADC
	// clips occur or signal value is too low.
	// the gain applies to THRU measurements only.
	small_function<void(int gain)> gainChanged;

	VNAMeasurement();

	void init();
	void setCorrelationTable(const int16_t* table, int length);
	void processSamples(uint16_t* buf, int len);

	// if points is 1, sets frequency to startFreqHz and disables sweep
	void setSweep(freqHz_t startFreqHz, freqHz_t stepFreqHz, int points, int dataPointsPerFreq=1);

	void resetSweep();

	struct _emitValue_t {
		VNAMeasurement* m;
		void operator()(int32_t* valRe, int32_t* valIm);
	};

	SampleProcessor<_emitValue_t> sampleProcessor;

public:
	// state variables
	VNAMeasurementPhases measurementPhase = VNAMeasurementPhases::REFERENCE;

	// number of periods left to wait
	uint32_t periodCounterSynth = 0;

	// number of periods since changing rf switches
	uint32_t periodCounterSwitch = 0;

	// number of data points since synthesizer frequency change
	uint32_t dpCounterSynth = 0;

	// counts up every data point; resets when it reaches ecalIntervalPoints
	uint32_t ecalCounter = 0;
	uint32_t ecalCounterOffset = 0;

	// What measurements to make
	enum MeasurementMode measurement_mode = MEASURE_MODE_FULL;

	// number of frequency points since start of sweep
	volatile int sweepCurrPoint = 0;

	uint16_t currThruGain = 0;
	uint16_t currReflGain = 0;
	bool gainChangeOccurred = false;


	// current data point variables
	int64_t currDP_re, currDP_im;
	complexf currFwd, currRefl, currThru;

	// sweep params
	freqHz_t sweepStartHz = 0, sweepStepHz = 0;
	int sweepPoints = 1;
	uint32_t sweepDataPointsPerFreq = 1;

	freqHz_t currFreq;

	complexf ecal[ECAL_CHANNELS];


	void setMeasurementPhase(VNAMeasurementPhases ph);
	void sweepAdvance();
	void sampleProcessor_emitValue(int32_t valRe, int32_t valIm, bool clipped);
	void doEmitValue(bool ecal);
};
