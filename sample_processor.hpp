// Cortex M4 DSP instructions assembly

// __smlabb inserts a SMLABB instruction. __smlabb returns the equivalent of
//  int32_t res = x[0] * y[0] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits. This operation sets the Q flag if overflow occurs on the addition.
__attribute__((always_inline)) static inline int32_t __smlabb(int32_t x, int32_t y, int32_t acc)
{
  int32_t r;
  __asm volatile ("smlabb %[r], %[x], %[y], %[a]" : [r] "=r" (r) : [x] "r" (x), [y] "r" (y), [a] "r" (acc) : );
  return r;
}

// __smlabt inserts a SMLABT instruction. __smlabt returns the equivalent of
//  int32_t res = x[0] * y[1] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits. This operation sets the Q flag if overflow occurs on the addition.
__attribute__((always_inline)) static inline int32_t __smlabt(int32_t x, int32_t y, int32_t acc)
{
  int32_t r;
  __asm volatile ("smlabt %[r], %[x], %[y], %[a]" : [r] "=r" (r) : [x] "r" (x), [y] "r" (y), [a] "r" (acc) : );
  return r;
}

// __smlalbb inserts a SMLALBB instruction. __smlalbb returns the equivalent of
//  int64_t res = x[0] * y[0] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits.
__attribute__((always_inline)) static inline int64_t __smlalbb(int64_t acc, int32_t x, int32_t y)
{
  union {
    struct { uint32_t lo; uint32_t hi; } s_rep;
    int64_t i_rep;
  } r;
  r.i_rep = acc;
  __asm volatile  ("smlalbb %[r_lo], %[r_hi], %[x], %[y]"
   : [r_lo] "+r" (r.s_rep.lo), [r_hi] "+r" (r.s_rep.hi)
   : [x] "r" (x), [y] "r" (y) : );
  return r.i_rep;
}

// __smlalbt inserts a SMLALBT instruction. __smlalbt returns the equivalent of
//  int64_t res = x[0] * y[1] + acc
//  where [0] is the lower 16 bits and [1] is the upper 16 bits.
__attribute__((always_inline)) static inline int64_t __smlalbt(int64_t acc, int32_t x, int32_t y)
{
  union {
    struct { uint32_t lo; uint32_t hi; } s_rep;
    int64_t i_rep;
  } r;
  r.i_rep = acc;
  __asm volatile  ("smlalbt %[r_lo], %[r_hi], %[x], %[y]"
   : [r_lo] "+r" (r.s_rep.lo), [r_hi] "+r" (r.s_rep.hi)
   : [x] "r" (x), [y] "r" (y) : );
  return r.i_rep;
}

// nStreams specifies the number of interleaved streams in the incoming data.
// emitValue_t must have the signature:
// void(int32_t* re, int32_t* im)
// where re and im point to arrays of size nStreams.
template<class emitValue_t, int nStreams=1>
class SampleProcessor {
public:
	uint32_t accumPhase;
	int64_t accumRe[nStreams],accumIm[nStreams];

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
		bool ret = false;
		len*=nStreams;
		while(len--) {
#if 1
			int32_t sc_t = ((int32_t *)correlationTable)[accumPhase];
			for(int streamNum = 0; streamNum < nStreams; streamNum++){
				int32_t sample = *samples++ - 2048;                              // 12bit ADC
				if(sample > 2000 || sample < -2000) {clipFlag = true;}           // Overflow
				accumRe[streamNum] = __smlalbt(accumRe[streamNum], sample, sc_t); // accumRe[streamNum]+= sample[0] * sc_t[1];
				accumIm[streamNum] = __smlalbb(accumIm[streamNum], sample, sc_t); // accumIm[streamNum]+= sample[0] * sc_t[0];
			}
#else
			int32_t lo_im = correlationTable[accumPhase*2];
			int32_t lo_re = correlationTable[accumPhase*2 + 1];
			for(int streamNum = 0; streamNum < nStreams; streamNum++) {
				int32_t sample = *samples++ - 2048;                       // 12bit ADC
				if(sample > 2000 || sample < -2000) {clipFlag = true;}    // Overflow
				accumRe[streamNum] += lo_re*sample;
				accumIm[streamNum] += lo_im*sample;
			}
#endif
			accumPhase++;
			if(int(accumPhase) >= accumPeriod) {
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

