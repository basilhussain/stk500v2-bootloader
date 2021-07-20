# STK500v2-compatible Bootloader for ATmegaXXM1

This is a fork of Peter Fleury's STK500v2 bootloader for Atmel AVR microcontrollers. It features modifications to allow it to be used on the ATmega16M1, ATmega32M1 and ATmega64M1 microcontrollers.

Described briefly, the following changes have been made:

- Added support for the LIN-based UART on the ATmegaXXM1 chips.
- LED indicator pin now has configurable polarity (active-low or active-high).
- Device signature string is configurable to be either "AVRISP_2" or "STK500_2"; latter is default for Atmel Studio 7 compatibility.
- Changed status response to unhandled commands to 'unknown' rather than 'failed'.
- Added configurable handling for responses to `VTARGET` and `VADJUST` parameter queries. Can optionally be disabled.
- Fixed bug with flash page erasure when writing; address of page being erased was being maintained separately and could in certain circumstances get out of sync.
- Corrected command sequence number behaviour. Should just echo back the given number in answer.
- Added proper handling of commands received with invalid checksum; will now respond with `ANSWER_CKSUM_ERROR` in that case.
- Added option to use avr-libc library functions for EEPROM reading and writing (should be more general-purpose).
- Changed code that reads device signature to actually read values from chip, rather than respond with compiled-in constants.
- Added support for oscillator calibration read command.
- Implemented chip erase command; erases entire application flash and also EEPROM if `EESAVE` fuse is not set. Can optionally be disabled.
- Modified makefile to have `MCU`, `F_CPU` and `BOOTLOADER_ADDRESS` variables be overridable by command-line arguments.
- Corrected various spelling mistakes in comments.

## Building

Edit the `stk500boot.c` source file and modify `PROG_PORT`, `PROG_DDR`, `PROG_IN`, and `PROG_PIN` definitions to match your hardware configuration. Also similarly modify `PROGLED_POLARITY`, `PROGLED_PORT`, `PROGLED_DDR`, and `PROGLED_PIN`.

Build by running `make all`, passing in arguments that specify the target microcontroller model, CPU speed, and bootloader address (in bytes, not words!) according to `BOOTSZ` fuse settings. For example, to build for an ATmega32M1 running at 16 MHz and with bootloader section size fuse settings of 512 words:

```
make MCU=atmega32m1 F_CPU=16000000 BOOTLOADER_ADDRESS=0x7C00 all
```

**Please note:** enabling additional options (see below) may mean that the compiled size of the bootloader code is larger than the original version, so will not fit within 512 words, and the next-largest bootloader section size of 1024 words may need to be used. In that case, the `BOOTLOADER_ADDRESS` parameter will need to be adjusted accordingly.

## Additional Configuration Options

The following defines have been added to support optional toggling of various extra features or behaviours:

- `REMOVE_VOLTAGE_PARAMS` - Disables responses to queries for `VTARGET` and `VADJUST` parameters.
- `REMOVE_CMD_CHIP_ERASE_ISP` - Disables chip erase, making that bootloader command a no-op.
- `USE_LIBC_EEPROM_FUNCS` - Use avr-libc library functions instead for reading/writing EEPROM.
- `PROGLED_POLARITY` - Sets LED polarity; 1 or 0 to indicate whether pin is active-high or active-low, respectively.
- `CONFIG_SIGN_ON_SIG` - Determines the signature string the bootloader responds with to the 'sign on' command. Change this to be one of the two defined strings: `CONFIG_SIGN_ON_SIG_AVRISP` or `CONFIG_SIGN_ON_SIG_STK500`. Either options works with avrdude; Atmel Studio works only with the latter.
- `CONFIG_PARAM_VOLTAGE` - Determines the voltage value that is given in response to queries for `VTARGET` and `VADJUST` parameters (if enabled). Value is volts x10 in decimal - for example: 5.0V => `50`, 3.3V => `33`.

To configure these options, edit the `stk500boot.c` source file and modify values or comment/un-comment relevant `#define` lines as appropriate.

## License

Copyright (C) 2006 Peter Fleury

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.