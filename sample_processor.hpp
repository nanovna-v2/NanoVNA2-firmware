
// Each output data point comes from taking the dot product
// between this function and the history of measured values.
// The table contains sin and cos multiplied by a window function
// that is a convolution between a rect with width of one IF period
// and a gaussian window.
static const int16_t correlationTable[100] = {
464, 0,
1018, -261,
1558, -856,
1939, -1821,
1987, -3132,
1524, -4691,
397, -6316,
-1479, -7751,
-4089, -8690,
-7294, -8817,
-10824, -7864,
-14296, -5660,
-17255, -2180,
-19235, 2430,
-19826, 7850,
-18739, 13615,
-15856, 19167,
-11257, 23922,
-5216, 27342,
1825, 29000,
9304, 28634,
16605, 26165,
23120, 21711,
28307, 15562,
31738, 8149,
32303, 0,
30720, -7888,
27156, -14929,
21947, -20610,
15570, -24534,
8601, -26472,
1660, -26386,
-4661, -24435,
-9862, -20959,
-13592, -16430,
-15685, -11396,
-16170, -6402,
-15254, -1927,
-13274, 1677,
-10640, 4213,
-7770, 5645,
-5030, 6080,
-2695, 5726,
-924, 4845,
233, 3702,
822, 2530,
952, 1501,
766, 719,
407, 223,
0, 0}; //*/

// 6kHz
/*
const int16_t correlationTable[100] = {
32767, 0,
32509, 4107,
31738, 8149,
30466, 12062,
28714, 15786,
26509, 19260,
23886, 22431,
20886, 25247,
17557, 27666,
13952, 29648,
10126, 31163,
6140, 32187,
2057, 32702,
-2057, 32702,
-6140, 32187,
-10126, 31163,
-13952, 29648,
-17557, 27666,
-20886, 25247,
-23886, 22431,
-26509, 19260,
-28714, 15786,
-30466, 12062,
-31738, 8149,
-32509, 4107,
-32767, 0,
-32509, -4107,
-31738, -8149,
-30466, -12062,
-28714, -15786,
-26509, -19260,
-23886, -22431,
-20886, -25247,
-17557, -27666,
-13952, -29648,
-10126, -31163,
-6140, -32187,
-2057, -32702,
2057, -32702,
6140, -32187,
10126, -31163,
13952, -29648,
17557, -27666,
20886, -25247,
23886, -22431,
26509, -19260,
28714, -15786,
30466, -12062,
31738, -8149,
32509, -4107}; //*/

// nStreams specifies the number of interleaved streams in the incoming data.
// emitValue_t must have the signature:
// void(int32_t* re, int32_t* im)
// where re and im point to arrays of size nStreams.
template<class emitValue_t, int nStreams=1>
class SampleProcessor {
public:
	uint32_t accumPhase;
	int32_t accumRe[nStreams],accumIm[nStreams];
	
	static constexpr int accumPeriod = 50;
	bool clipFlag;

	// void emitValue(int32_t valRe, int32_t valIm)
	emitValue_t emitValue;
	
	int _phMask;

	SampleProcessor(const emitValue_t& cb): emitValue(cb) {}
	void init() {
		accumPhase = 0;
		for(int streamNum = 0; streamNum < nStreams; streamNum++)
			accumRe[streamNum] = accumIm[streamNum] = 0;
		_phMask = 2047;
		clipFlag = false;
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
				int16_t sample = int16_t((samples[streamNum])*16 - 32768);
				if(sample > 30000) clipFlag = true;
				if(sample < -30000) clipFlag = true;
			
				accumRe[streamNum] += lo_re*sample/256;
				accumIm[streamNum] += lo_im*sample/256;
			}
			
			accumPhase++;
			samples += nStreams;
			if(accumPhase == accumPeriod) {
				emitValue(accumRe, accumIm);
				for(int streamNum = 0; streamNum < nStreams; streamNum++)
					accumRe[streamNum] = accumIm[streamNum] = 0;
				accumPhase = 0;
				ret = true;
			}
		}
		return ret;
	}
};

