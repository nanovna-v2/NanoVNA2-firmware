#!/usr/bin/env python3
import serial, tty, sys

# sets the flash address to begin writing at
def sendWriteAddr(ser, addr):
	# reset protocol to known state
	ser.write([0,0,0,0,0,0,0,0])

	# write register 0xe0, 4 bytes
	cmd = b"\x22\xe0" + int.to_bytes(addr, 4, 'little')

	ser.write(cmd)

def sendBytes(ser, data):
	cmd = bytearray()
	offs = 0
	while offs < len(data):
		sendLen = len(data) - offs
		if sendLen > 255:
			sendLen = 255

		# cmd: write FIFO, addr 0xe4
		cmd += (b'\x28\xe4')
		cmd.append(sendLen)
		cmd += (data[offs:offs+sendLen])
		offs += sendLen

	# cmd: echo version
	cmd.append(0x0d)
	ser.write(cmd)

# wait for one sent buffer to complete
def waitSend(ser):
	ser.read(1)
	sys.stdout.write('.')
	sys.stdout.flush()

ser = serial.Serial('/dev/ttyACM0')
tty.setraw(ser.fd)
ser.timeout = 3


sendWriteAddr(ser, 0x08004000)

src = sys.stdin
outstanding = 0
while True:
	buf = src.buffer.read(1200)
	if len(buf) == 0:
		break
	sendBytes(ser, buf)
	outstanding += 1
	if outstanding >= 2:
		waitSend(ser)
		outstanding -= 1

while outstanding > 0:
	waitSend(ser)
	outstanding -= 1
