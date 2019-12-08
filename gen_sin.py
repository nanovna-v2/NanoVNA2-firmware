#!/usr/bin/python
from math import *
from scipy import signal

# sine period in samples
N = 25

# integration period
accumPeriod = 50

# window decay length
windowN = 25

scale = (2**15 - 1)

if windowN > 0:
	window = signal.gaussian(windowN, std=7)
	#window = signal.bartlett(N)

	rectWindow = signal.boxcar(windowN)
	finalWindow = signal.convolve(window, rectWindow)

	finalWindow.resize(len(finalWindow) + 1)

	print len(finalWindow)
	assert len(finalWindow) == accumPeriod

	windowMax = 0
	for x in finalWindow:
		if abs(x) > windowMax:
			windowMax = abs(x)

	finalWindow /= windowMax

print 'const int16_t sinTable[%d] = {' % (accumPeriod*2)
for i in range(accumPeriod):
	arg = float(i) / N * 2*pi
	re = cos(arg)
	im = -sin(arg)
	vals = [re, im]
	if windowN > 0:
		vals = [x*finalWindow[i] for x in vals]
	vals = [int(round(x*scale)) for x in vals]
	vals = [str(x) for x in vals]
	print vals[0] + ', ' + vals[1] + ','
