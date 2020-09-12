#include "vna_measurement.hpp"

VNAMeasurement::VNAMeasurement(): sampleProcessor(_emitValue_t {this}) {

}

void VNAMeasurement::init() {
	sampleProcessor.init();
}
void VNAMeasurement::setCorrelationTable(const int16_t* table, int length) {
	sampleProcessor.setCorrelationTable(table, length);
	sampleProcessor.emitValue = _emitValue_t {this};
}
void VNAMeasurement::processSamples(uint16_t* buf, int len) {
	sampleProcessor.process(buf, len);
}

void VNAMeasurement::setSweep(freqHz_t startFreqHz, freqHz_t stepFreqHz, int points, int dataPointsPerFreq) {
	sweepStartHz = startFreqHz;
	sweepStepHz = stepFreqHz;
	sweepPoints = points;
	sweepDataPointsPerFreq = dataPointsPerFreq;
	resetSweep();
}

void VNAMeasurement::resetSweep() {
	__sync_synchronize();
	sweepCurrPoint = -1;
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

	periodCounterSynth = nWaitSynth;

	ecalCounterOffset++;
	if(ecalCounterOffset >= ecalIntervalPoints)
		ecalCounterOffset = 0;
	ecalCounter = ecalCounterOffset;
	if(sweepCurrPoint == 0) {
		periodCounterSynth *= 2;
		currGain = gainMax;
	}
}

void VNAMeasurement::sampleProcessor_emitValue(int32_t valRe, int32_t valIm, bool clipped) {
	auto currPoint = sweepCurrPoint;
	if(currPoint == -1) {
		freqHz_t start = sweepStartHz;
		freqHz_t stop = start + sweepStepHz*sweepPoints;
		sweepSetupChanged(start, stop);
		dpCounterSynth = 0;
		setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
		sweepAdvance();
		periodCounterSynth *= 2;
		return;
	}
	if(periodCounterSynth > 0) {
		// still waiting for synthesizer
		periodCounterSynth--;
		periodCounterSwitch = 0;
		gainChangeOccurred = false;
		return;
	}
	if(periodCounterSwitch >= nWaitSwitch) {
		currDP += complexi{valRe, valIm};

		if(measurementPhase == VNAMeasurementPhases::THRU) {
			if(clipped) {
				// ADC clip occurred during a measurement period
				if(currGain > gainMin) {
					// decrease gain and redo measurement
					currGain--;
					gainChanged(currGain);
					periodCounterSwitch = 0;
					currDP = {0, 0};
					sampleProcessor.clipFlag = false;
					gainChangeOccurred = true;
					return;
				}
			}
		}

		if(measurementPhase == VNAMeasurementPhases::THRU)
			clipFlag2 |= clipped;
		else clipFlag |= clipped;
	} else {
		sampleProcessor.clipFlag = false;
	}
	periodCounterSwitch++;

	if(measurementPhase == VNAMeasurementPhases::REFERENCE
		&& periodCounterSwitch >= (nWaitSwitch + nPeriods*nPeriodsMultiplier)) {
		currFwd = currDP;
		setMeasurementPhase(VNAMeasurementPhases::REFL);
	} else if(measurementPhase == VNAMeasurementPhases::REFL
		&& periodCounterSwitch >= (nWaitSwitch + nPeriods*nPeriodsMultiplier)) {
		currRefl = currDP;
		setMeasurementPhase(VNAMeasurementPhases::THRU);
		gainChanged(currGain);
	} else if(measurementPhase == VNAMeasurementPhases::THRU
		&& periodCounterSwitch >= (nWaitSwitch + nPeriods*nPeriodsMultiplier)) {
		currThru = currDP;

		float mag = abs(to_complexf(currThru));
		float fullScale = float(adcFullScale) * nPeriods*nPeriodsMultiplier;
		if(mag < (fullScale * 0.15) && currGain < gainMax && !gainChangeOccurred) {
			// signal level too low; increase gain and retry
			currGain++;
			gainChanged(currGain);
			gainChangeOccurred = true;
			periodCounterSwitch = 0;
			currDP = {0, 0};
			return;
		}

		if(ecalCounter == 0) {
#ifdef ECAL_PARTIAL
			setMeasurementPhase(VNAMeasurementPhases::ECALLOAD);
#else
			setMeasurementPhase(VNAMeasurementPhases::ECALTHRU);
#endif
		} else {
			setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
			doEmitValue(false);
		}
		ecalCounter++;
		if(ecalCounter >= ecalIntervalPoints)
			ecalCounter = 0;
	} else if(measurementPhase == VNAMeasurementPhases::ECALTHRU
		&& periodCounterSwitch >= (nWaitSwitch + nPeriods*nPeriodsMultiplier)) {

		ecal[2] = to_complexf(currDP);
		setMeasurementPhase(VNAMeasurementPhases::ECALLOAD);
	} else if(measurementPhase == VNAMeasurementPhases::ECALLOAD
		&& periodCounterSwitch >= (nWaitSwitch + nPeriods*nPeriodsMultiplier)) {
		ecal[0] = to_complexf(currDP);
#ifdef ECAL_PARTIAL
		setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
		doEmitValue(true);
#else
		setMeasurementPhase(VNAMeasurementPhases::ECALSHORT);
#endif
	} else if(measurementPhase == VNAMeasurementPhases::ECALSHORT
		&& periodCounterSwitch >= (nWaitSwitch + nPeriods*nPeriodsMultiplier)) {
		ecal[1] = to_complexf(currDP);
		setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
		doEmitValue(true);
	}
}
void VNAMeasurement::doEmitValue(bool ecal) {
	// emit new data point
	VNAObservationSet value = {to_complexf(currRefl), to_complexf(currFwd), to_complexf(currThru)};
	emitDataPoint(sweepCurrPoint, currFreq, value, ecal ? this->ecal : nullptr);

	clipFlag = false;
	clipFlag2 = false;
	dpCounterSynth++;
	if(int(dpCounterSynth) >= sweepDataPointsPerFreq && sweepPoints > 1) {
		dpCounterSynth = 0;
		sweepAdvance();
	}
}

void VNAMeasurement::_emitValue_t::operator()(int32_t* valRe, int32_t* valIm) {
	m->sampleProcessor_emitValue(*valRe, *valIm, m->sampleProcessor.clipFlag);
}
