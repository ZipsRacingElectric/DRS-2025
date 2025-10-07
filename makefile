# Project name
PROJECT = drs_2025

# Imported files
CHIBIOS  := $(CHIBIOS_SOURCE_PATH)

# Directories
CONFDIR  	:= ./config
BUILDDIR 	:= ./build
DEPDIR   	:= ./build/dep
BOARDDIR	:= ./build/board
COMMONDIR	:= ./common

# Source files
CSRC =	$(ALLCSRC)		\
		src/main.c		\
		src/servo.c		\
		src/peripherals.c

# Common library includes
include common/src/fault_handler.mk


# REVIEW(Barach): We are including "lerp.h" so "lerp.mk" must also be included.
# include common/src/controls/lerp.mk
include common/src/debug.mk

include common/src/peripherals/adc/analog_linear.mk
include common/src/peripherals/adc/stm_adc.mk

include common/src/can/can_thread.mk
include common/src/can/eeprom_can.mk

include common/src/peripherals/i2c/mc24lc32.mk

# REVIEW(Barach): -O0 shouldn't ever be committed, just to be temporarily
# enabled when testing. We always want to compile with -Og.

# Compiler flags
# USE_OPT = -Og -Wall -Wextra
USE_OPT = -O0 -Wall -Wextra

# C macro definitions
UDEFS =

# ASM definitions
UADEFS =

# Include directories
UINCDIR =

# Library directories
ULIBDIR =

# Libraries
ULIBS =

# Common toolchain includes
include common/common.mk
include common/make/openocd.mk

# ChibiOS compilation hooks
PRE_MAKE_ALL_RULE_HOOK: $(BOARD_FILES) $(CLANGD_FILE)