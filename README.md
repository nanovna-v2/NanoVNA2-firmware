
This repository contains the source code to build the firmware for the NanoVNA2 (or SAA2)
See https://www.nanorfe.com for more info.

Below is information for building the firmware on Linux.

## Installing the compiler
On any recent Debian based installation:
``` 
apt install gcc-arm-none-eabi python3-serial
```

## Getting all source code
The code is spread out over 3 repositories:
```
git clone https://github.com/nanovna/libopencm3-gd32f3.git
git clone https://github.com/gabriel-tenma-white/mculib.git
git clone https://github.com/nanovna/NanoVNA-V2-firmware.git
```
The libopencm3 is not build automatically and must be build before building the project.
This is a one time step.
Build the libopencm3:
```
cd libopencm3-gd32f3
make
cd ..
```

Now edit the Makefile to point to the correct library
```
cd NanoVNA-V2-firmware
```
open the Makefile in your favorite editor and change the two lines with
```
MCULIB         ?= /persist/mculib
OPENCM3_DIR    ?= /persist/libopencm3
```
So they point to the directories that where create during the checkout:
```
MCULIB         ?= /home/YOUR-USER/mculib
OPENCM3_DIR    ?= /home/YOUR-USER/libopencm3-gd32f3
```

## Building
Now you can build the firmware by running make:
```
make
```

## Flashing the firmware
There are two options to update the firmware when using the regular USB interface:
 - Use NanoVNA-QT
 - Command line
For this to work the device must stay in the bootloader and enter _DFU mode_

Another option is using a debugger using the debug pins.

### Entering DFU mode
```
Switch the device off
Press and hold down the left button (the one closest to the Port 1 or the On/Off switch)
Switch the device on (screen stays white), release the button
```


The current user probably needs to be part of the dialout group to allow access.

Flashing can be done by running:
```
python dfu.py -f binary.bin
```
On some systems you may need to invoke python3 instead:
```
python3 dfu.py -f binary.bin
```

Note that depending on your installation the device might be seen as an Mobile Modem (3G/4G/etc) and it will not open the /dev/ttyACM0 port. 
After a while the modem manager will give up and you can access the device.
If this is too much of a burden, you need to add udev rules to block modem manager from doing so.

