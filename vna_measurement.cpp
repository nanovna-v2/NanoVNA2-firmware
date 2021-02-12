#include "vna_measurement.hpp"
#include <board.hpp>

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
	currDP_re = 0;
	currDP_im = 0;
	gainChangeOccurred = false;
#ifdef BOARD_DISABLE_ECAL
	// Disabled ecal, use only nPeriods
	nMeasureCount = nPeriods * nPeriodsMultiplier;
#elif 1
	// For ecal use nPeriodsCalibrating always, for other use nPeriods
    if (ph > VNAMeasurementPhases::THRU) nMeasureCount = nPeriodsCalibrating * nPeriodsMultiplier;
	else  	                             nMeasureCount = nPeriods * nPeriodsMultiplier;
#else
	// On calibration or first step (ecalIntervalPoints == 1) use nPeriodsCalibrating, for other use nPeriods
	nMeasureCount = ((ecalIntervalPoints == 1) ? nPeriodsCalibrating : nPeriods) * nPeriodsMultiplier;
#endif
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
	periodCounterSwitch = 0;
	if(sweepCurrPoint == 0) {
		periodCounterSynth = BOARD_MEASUREMENT_FIRST_POINT_WAIT; // for first point need more wait
		currThruGain = gainMax;
		ecalCounter = ecalCounterOffset;
		ecalCounterOffset++;
		if(ecalCounterOffset >= ecalIntervalPoints)
			ecalCounterOffset = 0;
	}
}

void VNAMeasurement::sampleProcessor_emitValue(int32_t valRe, int32_t valIm, bool clipped) {
	auto currPoint = sweepCurrPoint;
	/* If -1 then we restart */
	if(currPoint == -1) {
		freqHz_t start = sweepStartHz;
		freqHz_t stop = start + sweepStepHz*sweepPoints;
		sweepSetupChanged(start, stop);
		dpCounterSynth = 0;
		setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
		ecalCounterOffset = 0;
		sweepAdvance();
		return;
	}
	/* If periodCounterSynth not elapsed, decrement and wait for it */
	if(periodCounterSynth > 0) {
		// still waiting for synthesizer
		periodCounterSynth--;
		gainChangeOccurred = false;
		return;
	}
	if(periodCounterSwitch >= nWaitSwitch) {
		currDP_re+= valRe;
		currDP_im+= valIm;

		if(measurementPhase == VNAMeasurementPhases::THRU) {
			if(clipped) {
				// ADC clip occurred during a measurement period
				if(currThruGain > gainMin) {
					// decrease gain and redo measurement
					currThruGain--;
					gainChanged(currThruGain);
					periodCounterSwitch = 0;
					currDP_re = 0;
					currDP_im = 0;
					sampleProcessor.clipFlag = false;
					gainChangeOccurred = true;
					return;
				}
			}
		}
		else // not show clippederror on thru measure
			clipFlag |= clipped;
	} else {
		sampleProcessor.clipFlag = false;
	}
	periodCounterSwitch++;

	/* If switch time not elapsed, wait some more */
	if(periodCounterSwitch < (nWaitSwitch + nMeasureCount)) {
		return;
	}
	// Real measure count
	periodCounterSwitch-=nWaitSwitch;
	// Get current point measured data (not depend from measure count)
	complexf currDP = complexf{(float)currDP_re/periodCounterSwitch, (float)currDP_im/periodCounterSwitch};

	// Loop through measurement phase
	switch(measurementPhase) {
		case VNAMeasurementPhases::REFERENCE:
			currFwd = currDP;
			setMeasurementPhase(VNAMeasurementPhases::REFL);
			break;
		case VNAMeasurementPhases::REFL:
			currRefl = currDP;
			setMeasurementPhase(VNAMeasurementPhases::THRU);
			break;
		case VNAMeasurementPhases::THRU:
			if(currThruGain < gainMax && !gainChangeOccurred) {
				float mag = abs(currDP);
				if(mag < (adcFullScale * 0.15f)) {
					// signal level too low; increase gain and retry
					currThruGain++;
					gainChanged(currThruGain);
					gainChangeOccurred = true;
					periodCounterSwitch = 0;
					currDP_re = 0;
					currDP_im = 0;
					return;
				}
			}
			currThru = currDP;
			switch(measurement_mode) {
				case MEASURE_MODE_FULL:
#ifdef BOARD_DISABLE_ECAL
					setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
					doEmitValue(false);
#else
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
#endif
					break;
				case MEASURE_MODE_REFL_THRU_REFRENCE: /* AKA no ECAL */
					/* Go back to the start: REFERENCE */
					setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
					doEmitValue(false);
					break;
				case MEASURE_MODE_REFL_THRU:
					/* aka CW mode
					 * And keep the signal on the ouput */
					setMeasurementPhase(VNAMeasurementPhases::REFL);
					doEmitValue(false);
					break;
			}
			break;

		case VNAMeasurementPhases::ECALTHRU:
			ecal[2] = currDP;
			setMeasurementPhase(VNAMeasurementPhases::ECALLOAD);
			break;

		case VNAMeasurementPhases::ECALLOAD:
			ecal[0] = currDP;
#ifdef ECAL_PARTIAL
			/* Go back to the start: REFERENCE */
			setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
			doEmitValue(true);
#else
			setMeasurementPhase(VNAMeasurementPhases::ECALSHORT);
#endif
			break;
		case VNAMeasurementPhases::ECALSHORT:
			ecal[1] = currDP;
			/* Go back to the start: REFERENCE */
			setMeasurementPhase(VNAMeasurementPhases::REFERENCE);
			doEmitValue(true);
			break;
	}
}

void VNAMeasurement::doEmitValue(bool ecal) {
	// emit new data point
	VNAObservationSet value = {currRefl, currFwd, currThru};
	emitDataPoint(sweepCurrPoint, currFreq, value, ecal ? this->ecal : nullptr);

	clipFlag = false;

	dpCounterSynth++;
	if(dpCounterSynth >= sweepDataPointsPerFreq && sweepPoints > 1) {
		dpCounterSynth = 0;
		sweepAdvance();
	}
}

void VNAMeasurement::_emitValue_t::operator()(int32_t* valRe, int32_t* valIm) {
	m->sampleProcessor_emitValue(*valRe, *valIm, m->sampleProcessor.clipFlag);
}
