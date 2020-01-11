#!/usr/bin/env python3
import serial, tty, sys
import argparse

# returns true if device is in dfu mode
def detectDFU(ser):
	# reset protocol to known state
	ser.write([0,0,0,0,0,0,0,0])

	# read register 0xf3 (firmware major version)
	cmd = b"\x10\xf3"
	ser.write(cmd)

	resp = ser.read(1)
	if len(resp) < 1:
		print('Read timeout')
		return False

	# firmware major version is always 0xff in dfu mode
	if resp[0] == 0xff:
		return True

	print('Firmware major version: 0x%02x' % resp[0])
	return False

# sets the flash address to begin writing at
def sendWriteAddr(ser, addr):
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




parser = argparse.ArgumentParser()
parser.add_argument('-s', '--serial', default='/dev/ttyACM0', 
	help="Path to USB serial device")
parser.add_argument('-f', '--file',
	help="Path to .bin image file")
parser.add_argument('-r', '--reboot', action='store_true', 
	help="Reset device without flashing any image")
parser.add_argument('-a', '--argument', type=lambda x:int(x,0), default=-1, 
	help="User argument (u32) passed to application after reset")
args = parser.parse_args()

if args.file != None and args.reboot:
	print('-f and -r can not both be specified at the same time')
	exit(1)

if args.file == None and not args.reboot:
	print('Either -f or -r required')
	exit(1)


ser = serial.Serial(args.serial)
tty.setraw(ser.fd)
ser.timeout = 0.1
ser.read(4096)
ser.timeout = 2

if not detectDFU(ser):
	print('Device not in DFU mode')
	print('Please enter DFU mode through menu CONFIG -> DFU or hold down the JOG LEFT button and power cycle the device.')
	exit(1)

if args.file != None:
	sendWriteAddr(ser, 0x08004000)

	src = open(args.file)
	outstanding = 0
	while True:
		buf = src.buffer.read(1200)
		if len(buf) == 0:
			break
		sendBytes(ser, buf)
		outstanding += 1
		if outstanding >= 5:
			waitSend(ser)
			outstanding -= 1

	while outstanding > 0:
		waitSend(ser)
		outstanding -= 1

	print('')
	print('done. Resetting')

# set user argument
if args.argument >= 0:
	cmd = b"\x22\xe8" + int.to_bytes(args.argument, 4, 'little')
	ser.write(cmd)

# reboot device
ser.write([0x20, 0xef, 0x5e])

