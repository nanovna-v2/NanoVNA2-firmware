## NanoVNA V2 Firmware

This repository contains the source code to build the firmware for the NanoVNA V2 (S-A-A-2).

See https://nanorfe.com/nanovna-v2.html for more info.

Developers chat room: https://discord.gg/DUH5Xk5

Below is information for building and uploading the firmware.

## Installing the compiler

The ARM GCC compiler is maintained by ARM, and is also available by other methods.

### Debian Linux based systems

On any recent Debian based installation:
``` 
sudo apt install gcc-arm-none-eabi
```

### Installing the upstream toolchain direct from ARM

If you want to install the latest version of the gnu ARM toolchain:

1. Get the latest version of the toolchain from https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads .
You can download it using your browser or a command line tool like `wget`:
```
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
```

2. untar it, eg. in `/opt/toolchains`:
```
sudo mkdir -p /opt/toolchains
sudo tar xvf -C /opt/toolchains gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
```

3. set your PATH environment variable:
```
export PATH=/opt/toolchains/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH
```

## Downloading this source code

The code is spread out over 3 repositories, 2 of which are submodules of the main NanoVNA-V2-firmware one:
```
git clone --recursive https://github.com/nanovna/NanoVNA-V2-firmware.git
cd NanoVNA-V2-firmware
```

## Building
Now you can build the firmware by running make in the firmware sources directory:
```
cd NanoVNA-V2-firmware
make BOARDNAME=board_v2_2 EXTRA_CFLAGS="-DSWEEP_POINTS_MAX=201 -DSAVEAREA_MAX=7"
```
Note that `SWEEP_POINTS_MAX` and `SAVEAREA_MAX` can be customized depending on hardware target.
Since Plus4 ECAL is no longer needed, and the extra RAM can be used to increase `SWEEP_POINTS_MAX` to 301 points (warning: experimental! there may not be enough stack space if ram usage is near full).

`BOARDNAME` should be set to:
- `board_v2_2` for V2.2 hardware
- `board_v2_plus` for V2 Plus hardware
- `board_v2_plus4` for V2 Plus4 hardware

For Plus4, a different linker script and display driver needs to be used. The build command line for the Plus4 is:
```
make BOARDNAME=board_v2_plus4 EXTRA_CFLAGS="-DSWEEP_POINTS_MAX=201 -DSAVEAREA_MAX=7 -DDISPLAY_ST7796" LDSCRIPT=./gd32f303cc_with_bootloader_plus4.ld
```

The first time you build the firmware on a fresh repository, there is a libopencm3 bug that sometimes causes the linker script to be overwritten with one that will not work. If the built firmware does not boot, try running the following commands, then rebuild:
```
git checkout -- gd32f303cc_with_bootloader.ld
git checkout -- gd32f303cc_with_bootloader_plus4.ld
```

## To upload the firmware

The GD32F303 processor does not support [USB DFU](https://www.usb.org/sites/default/files/DFU_1.1.pdf) mode like the STM32 chips do.
Instead, a serial bootloader program is installed. Please do not attempt to use a standard USB DFU tool.

All V2 devices contain a USB serial bootloader, and you can upload the firmware through USB without needing extra hardware.

## Updating firmware using the USB serial bootloader

You must restart the device in BOOTLOAD mode:

```
Switch the device off.
Press and hold down the left button (the one closest to the Port 1 or the On/Off switch).
Switch the device on (screen stays white), release the button.
```

You will see a blank white screen. This indicates that it is waiting for new firmware.
Check for the existence of the USB serial port device. This should be the device-special file /dev/ttyACM0
On a *nix system, the current user probably needs to be part of the dialout group to allow access to the device.

Note that depending on your installation's *udev* rules, the new serial device initially appear as an Mobile Modem (3G/4G/etc)
and it will not create the /dev/ttyACM0 port.  After a while the modem manager will give up and you can access the port.
If this is too much of a burden, you can add udev rules to block modem manager from trying to control this port.


Once the device is in BOOTLOAD mode, you can update the firmware using either [NanoVNA-QT](https://github.com/nanovna-v2/NanoVNA-QT) or the [bootload_firmware.py](bootload_firmware.py) Python script.


#### Option 1: Upload firmware using NanoVNA-QT

Select the serial port device /dev/ttyACM0 under the Device menu in NanoVNA-QT. A dialog will appear asking asking if you would like to flash a new firmware. Click yes, and select the firmware .bin file to flash.


#### Option 2: Upload firmware using bootload_firmware.py
Ensure you have Python version 3, and install [pyserial](https://github.com/pyserial/pyserial).

On a Debian based system, you can get pyserial using:
```
sudo apt install python3-serial
```

Flashing can be done by running:
```
python bootload_firmware.py -f binary.bin
```

On some systems you may need to explicitly invoke python3 instead:
```
python3 bootload_firmware.py -f binary.bin
```


## Updating the firmware using an ST-Link

### WARNING: It is imperative to back up the entire flash memory of the GD32 before updating the firmware using this method. Some devices have factory calibration data stored between the bootloader and application.

The NanoVNA V2 firmware is installed at address 0x8004000 (V2/V2Plus) and 0x8008000 (V2Plus4).
You can upload the firmware binary to that address using an ST-Link.

The [bootloader](bootloader/binary.bin) is located at address 0x8000000, the start of the GD32F303 flash section.

You can flash the firmware image using an [ST-Link](https://www.st.com/en/development-tools/st-link-v2.html) device, of which many inexpensive clones are available.
Some reports indicate that the NanoVNAv2 cannot be powered via the 3.2v supply from the ST-Link, but should be powered from its own battery.
