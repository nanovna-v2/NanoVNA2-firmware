

// nStreams specifies the number of interleaved streams in the incoming data.
// emitValue_t must have the signature:
// void(int32_t* re, int32_t* im)
// where re and im point to arrays of size nStreams.
template<class emitValue_t, int nStreams=1>
class SampleProcessor {
public:
	uint32_t accumPhase;
	int32_t accumRe[nStreams],accumIm[nStreams];

	// integration time for each output value
	int accumPeriod = 50;

	// sinusoid table, must be accumPeriod*2 entries (interleaved real & imag)
	const int16_t* correlationTable = nullptr;

	bool clipFlag = false;

	// void emitValue(int32_t valRe, int32_t valIm)
	emitValue_t emitValue;

	SampleProcessor(const emitValue_t& cb): emitValue(cb) {}
	void init() {
		accumPhase = 0;
		for(int streamNum = 0; streamNum < nStreams; streamNum++)
			accumRe[streamNum] = accumIm[streamNum] = 0;
		clipFlag = false;
	}
	void setCorrelationTable(const int16_t* table, int length) {
		accumPhase = 0;
		correlationTable = table;
		accumPeriod = length;
	}
	// len specifies the number of aggregates (i.e. when nStreams > 1,
	// the number of words in the array must be len * nStreams).
	// returns whether we completed a cycle.
	bool process(uint16_t* samples, int len) {
		uint16_t* end = samples+len*nStreams;
		bool ret = false;
		while(samples < end) {
			int32_t lo_im = correlationTable[accumPhase*2];
			int32_t lo_re = correlationTable[accumPhase*2 + 1];
			for(int streamNum = 0; streamNum < nStreams; streamNum++) {
				int16_t sample = int16_t((samples[streamNum]) - 2048);
				if(sample > 2000 || sample < -2000) {clipFlag = true;}    // Overflow

				accumRe[streamNum] += lo_re*sample/512;
				accumIm[streamNum] += lo_im*sample/512;
			}
			
			accumPhase++;
			samples += nStreams;
			if(int(accumPhase) >= accumPeriod) {
				emitValue(accumRe, accumIm);
				clipFlag = false;
				for(int streamNum = 0; streamNum < nStreams; streamNum++)
					accumRe[streamNum] = accumIm[streamNum] = 0;
				accumPhase = 0;
				ret = true;
			}
		}
		return ret;
	}
};

