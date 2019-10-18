#include "vna_measurement.hpp"

VNAMeasurement::VNAMeasurement(): sampleProcessor(_emitValue_t {this}) {
	
}

void VNAMeasurement::init() {
	sampleProcessor.init();
}

void VNAMeasurement::processSamples(uint16_t* buf, int len) {
	sampleProcessor.process(buf, len);
}

void VNAMeasurement::setSweep(uint64_t startFreqHz, uint64_t stepFreqHz, int points, int dataPointsPerFreq) {
	sweepStartHz = startFreqHz;
	sweepStepHz = stepFreqHz;
	sweepPoints = points;
	sweepCurrPoint = 0;
	sweepDataPointsPerFreq = dataPointsPerFreq;

	currFreq = startFreqHz;
	periodCounterSynth = dpCounterSynth = 0;
	setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
	frequencyChanged(startFreqHz);
}

void VNAMeasurement::setMeasurementPhase(VNAMeasurementPhases ph) {
	phaseChanged(ph);
	measurementPhase = ph;
	periodCounterSwitch = 0;
	currDP = {0, 0};
}
static inline complexf to_complexf(VNAMeasurement::complexi value) {
	return {(float) value.real(), (float) value.imag()};
}

void VNAMeasurement::sweepAdvance() {
	sweepCurrPoint++;
	if(sweepCurrPoint >= sweepPoints)
		sweepCurrPoint = 0;

	currFreq = sweepStartHz + sweepStepHz*sweepCurrPoint;
	frequencyChanged(currFreq);

	periodCounterSynth = 0;
}

void VNAMeasurement::sampleProcessor_emitValue(int32_t valRe, int32_t valIm) {
	if(periodCounterSynth < nWaitSynth) {
		periodCounterSynth++;
		periodCounterSwitch = 0;
		return;
	}
	if(periodCounterSwitch >= nWaitSwitch) {
		currDP += complexi{valRe, valIm};
	}
	periodCounterSwitch++;

	if(measurementPhase == VNAMeasurementPhases::REFERENCE
		&& periodCounterSwitch == (nWaitSwitch + nPeriods*2)) {
		currFwd = currDP;
		setMeasurementPhase(VNAMeasurementPhases::REFL1);
	} else if(measurementPhase == VNAMeasurementPhases::REFL1
		&& periodCounterSwitch == (nWaitSwitch + nPeriods)) {
		currRefl = currDP;
		setMeasurementPhase(VNAMeasurementPhases::REFL2);
	} else if(measurementPhase == VNAMeasurementPhases::REFL2
		&& periodCounterSwitch == (nWaitSwitch + nPeriods)) {
		//currRefl = (currRefl - currDP) * int32_t(2);
		currRefl += currDP;
		setMeasurementPhase(VNAMeasurementPhases::THRU);
	} else if(measurementPhase == VNAMeasurementPhases::THRU
		&& periodCounterSwitch == (nWaitSwitch + nPeriods*2)) {
		currThru = currDP;
		setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
		
		// emit new data point
		VNAObservationSet value = {to_complexf(currRefl), to_complexf(currFwd), to_complexf(currThru)};
		emitDataPoint(currFreq, value);

		dpCounterSynth++;
		if(dpCounterSynth >= sweepDataPointsPerFreq && sweepPoints > 1) {
			dpCounterSynth = 0;
			sweepAdvance();
		}
	}
}

void VNAMeasurement::_emitValue_t::operator()(int32_t valRe, int32_t valIm) {
	m->sampleProcessor_emitValue(valRe, valIm);
}
