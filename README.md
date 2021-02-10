
This repository contains the source code to build the firmware for the NanoVNA V2 (S-A-A-2).

See https://nanorfe.com/nanovna-v2.html for more info.

Developers chat room: https://discord.gg/DUH5Xk5

Below is information for building the firmware on Linux.

## Installing the compiler

### Debian based systems

On any recent Debian based installation:
``` 
sudo apt install gcc-arm-none-eabi
```

### Installing the upstream toolchain

If you want to install the latest version of the gnu ARM toolchain:

1. get the latest version from https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads . You can download it using your browser or a command line tool like `wget`:
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

## Installing dependencies for the firmware upload tools

The GD32F303 processor does not support the normal [USB DFU](https://www.usb.org/sites/default/files/DFU_1.1.pdf) mode. Instead, a serial bootloader program is installed. This serial tool was unfortunately named "dfu" even though it has no relationship to DFU. Please do not attempt to use a standard USB DFU tool.

To install the bootloader on a new GD32F303, you will need to use an [ST-Link](https://www.st.com/en/development-tools/st-link-v2.html) device, of which many inexpensive clones are available. The [bootloader](bootloader/binary.bin) should be loaded at address 0x8000000, the start of the GD32F303 flash section.

If you are using an ST-Link adapter, you should load the firmware binary at address 0x8004000. Some reports indicate that the NanoVNAv2 cannot be powered via the 3.2v supply from the ST-Link, but should be powered from its own battery.

If you have an intact bootloader already installed in your NanoVNAv2, you have options to upload the firmware:

- Using (NanoVNAv2-QT](https://github.com/nanovna-v2/NanoVNA-QT)

- Using Python and the [dfu.py](dfu.py) script. You need Python version 3, and [pyserial](https://github.com/pyserial/pyserial)

On a Debian based system, you can get pyserial using:

```
sudo apt install python3-serial
```

## Getting the source code

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

For Plus4, a different linker script needs to be used. The build command line for the Plus4 is:
```
make BOARDNAME=board_v2_plus4 EXTRA_CFLAGS="-DSWEEP_POINTS_MAX=201 -DSAVEAREA_MAX=7 -DDISPLAY_ST7796" LDSCRIPT=./gd32f303cc_with_bootloader_plus4.ld
```

The first time you build the firmware on a fresh repository, there is a libopencm3 bug that sometimes causes the linker script to be overwritten with a nonworking one. If the built firmware does not boot, try running:
```
git reset --hard gd32f303cc_with_bootloader.ld
git reset --hard gd32f303cc_with_bootloader_plus4.ld
```
And rebuilding.


## Flashing the firmware
There are two options to update the firmware when using the regular USB interface:
 - Use NanoVNA-QT
 - Command line
For this to work the device must stay in the bootloader and enter _DFU mode_

Another option is using a debugger using the debug pins.

### Entering serial firmware update mode
```
Switch the device off
Press and hold down the left button (the one closest to the Port 1 or the On/Off switch)
Switch the device on (screen stays white), release the button
```

On a *nix system, the current user probably needs to be part of the dialout group to allow access so the serial port.

Flashing can be done by running:
```
python dfu.py -f binary.bin
```
On some systems you may need to invoke python3 instead:
```
python3 dfu.py -f binary.bin
```

Note that depending on your installation's *udev* rules, the new serial device might be seen as an Mobile Modem (3G/4G/etc) and it will not create the /dev/ttyACM0 port. 
After a while the modem manager will give up and you can access the device.
If this is too much of a burden, you need to add udev rules to block modem manager from doing so.

