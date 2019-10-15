#!/usr/bin/python
from math import *
from scipy import signal

N = 25
scale = (2**15 - 1)

accumPeriod = N*2

window = signal.gaussian(N, std=7)
#window = signal.bartlett(N)

rectWindow = signal.boxcar(N)
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
	im = sin(arg)
	vals = [re, im]
	vals = [x*finalWindow[i] for x in vals]
	vals = [int(round(x*scale)) for x in vals]
	vals = [str(x) for x in vals]
	print vals[0] + ', ' + vals[1] + ','
