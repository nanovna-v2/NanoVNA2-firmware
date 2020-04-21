#include <stdio.h>
#include <math.h>
#include <stdint.h>

int FFT_SIZE = 512;

int main() {
	for (int i = 0; i < FFT_SIZE - (FFT_SIZE / 4); i++) {
		printf("% .8f,%c", sin(2 * M_PI * i / FFT_SIZE), i % 8 == 7 ? '\n' : ' ');
	}
}
