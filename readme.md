# STM32F405 Stub Project
## Dependencies (Required)
- make
- openocd
- arm-none-eabi-gcc
- ChibiOS 21.11.03
- FMPP - FreeMark Pre-Processor

## Dependencies (Optional)
- Cortex Debug
- arm-none-eabi-gdb

## Setup
- Clone the ChibiOS source repo locally.
- Define the "CHIBIOS_SOURCE_PATH" environment variable to point to the location the ChibiOS repo.

## Usage
### Compilation
- Use 'make' to compile the program. Files that have not been modified will not re-compiled.
- Use 'make clean' to delete all build files.

### Programming
- Use 'make flash' to call OpenOCD. If modifications were made, the program will be recompiled.

### Debugging
- In VS-code, use 'Run Debugger'.

## Filesystem
- build - Compilation output, includes the .elf file (the application file).
- config - ChibiOS configuration files.
- src - Project source code.
