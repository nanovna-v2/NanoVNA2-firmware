#!/usr/bin/env python3
import argparse
import os
from sys import exit
import time
import tty

import serial

try:
    from progress.bar import Bar
except ImportError:
    Bar = None


class DummyBar:
    """Dummy progress bar, to be used if progress is not available"""
    def __init__(self, max=100, **kwargs):
        self.max = max
        self.index = 0

    def start(self):
        pass

    def next(self, n=1):
        self.index += n
        print(".", sep="", end="", flush=True)

    def finish(self):
        print(flush=True)


if Bar is None:
    Bar = DummyBar


class DummySerial:
    """Dummy serial port, useful for testing purpose"""
    def read(self, size=1):
        time.sleep(0.05)
        # return 0xFF so detect_fpu() is happy
        return b"\xff" * size

    def write(self, data):
        pass


def detect_dfu(ser):
    """returns true if device is in dfu mode"""

    # reset protocol to known state
    ser.write([0] * 8)

    # read registers 0xf1->0xf4
    # f0: device variant
    # f1: protocol version (01)
    # f2: hardware revision (always 0 in dfu mode)
    # f3: firmware major version (ff => dfu mode)
    # f4: firmware minor version (bootloader version)

    cmd = b"\x10\xf0\x10\xf1\x10\xf2\x10\xf3\x10\xf4"
    ser.write(cmd)
    resp = ser.read(5)

    if resp and len(resp) == 5:
        # firmware major version is always 0xff in dfu mode
        if resp[3] == 0xFF:
            return True
        print(f"Informations:")
        print(f"  Device variant:   0x{resp[0]:02x}")
        print(f"  Protocol version: 0x{resp[1]:02x}")
        print(f"  Hardware version: 0x{resp[2]:02x}")
        print(f"  Firmware major version: 0x{resp[3]:02x}")
        print(f"  Firmware minor version: 0x{resp[4]:02x}")
    else:
        print("Read timeout")
    return False


def send_write_addr(addr):
    "returns the cmd to set the flash address to begin writing at"
    # write register 0xe0, 4 bytes
    return b"\x22\xe0" + addr.to_bytes(length=4, byteorder="little")


def send_bytes(data):
    "returns the bytes to send as block of data"
    cmd = []
    while data:
        # send data by blocks of 255 bytes
        block = data[:255]
        cmd.append(b"\x28\xe4")
        cmd.append(bytes([len(block)]))
        cmd.append(block)
        data = data[255:]  # remaining data to send, if any
    # cmd: echo version
    cmd.append(b"\x0d")
    return b"".join(cmd)


def block_reader(fobj, blocksize=1275):
    "generates blocks of data of size blocksize reading from fobj"
    while True:
        data = fobj.read(blocksize)
        if data:
            yield data
        else:
            break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-s", "--serial", default="/dev/ttyACM0", help="Path to USB serial device"
    )
    parser.add_argument(
        "-f", "--file", type=argparse.FileType("rb"), help="Path to .bin image file"
    )
    parser.add_argument(
        "-r",
        "--reboot",
        action="store_true",
        help="Reset device without flashing any image",
    )
    parser.add_argument(
        "-a",
        "--argument",
        type=int,
        default=-1,
        help="User argument (u32) passed to application after reset",
    )
    parser.add_argument(
        "-d", "--dummy", default=False, action="store_true", help="Dummy mode",
    )
    args = parser.parse_args()
    if args.file is not None and args.reboot:
        print("-f and -r can not both be specified at the same time")
        exit(1)

    if args.file is None and not args.reboot:
        print("Either -f or -r required")
        exit(1)

    if not args.dummy:
        ser = serial.Serial(args.serial)
        tty.setraw(ser.fd)
        # flush the serial port buffer, if any
        ser.timeout = 0.1
        ser.read(4096)

        ser.timeout = 2
    else:
        ser = DummySerial()

    if not detect_dfu(ser):
        print("Device not in DFU mode")
        print(
            "Please enter DFU mode through menu CONFIG -> DFU or hold down "
            "the JOG LEFT button and power cycle the device."
        )
        exit(1)

    if args.file is not None:
        print(f"Uploading firmware file {args.file.name}")
        # set the flash address where the firmware is to be written
        ser.write(send_write_addr(0x08004000))
        size = os.stat(args.file.name).st_size
        pbar = Bar(message=os.path.basename(args.file.name), max=size)
        pbar.start()
        # write the firmware
        for data in block_reader(args.file, blocksize=1200):
            if data:
                ser.write(send_bytes(data))
                pbar.next(len(data))
                ser.read(1)  # wait for block write to complete
        pbar.finish()

    # set user argument
    if args.argument >= 0:
        print("Sending user argument")
        cmd = b"\x22\xe8" + int.to_bytes(args.argument, 4, "little")
        ser.write(cmd)

    print("Resetting")
    # reboot device
    ser.write([0x20, 0xEF, 0x5E])


if __name__ == "__main__":
    main()
