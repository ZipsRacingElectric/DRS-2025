# Project name
PROJECT = drs_2025

# Imported files
CHIBIOS  := $(CHIBIOS_SOURCE_PATH)

# Directories
CONFDIR  := ./config
BUILDDIR := ./build
DEPDIR   := ./build/dep
BOARDDIR := ./build/board

# Source files
CSRC =	$(ALLCSRC)		\
		src/main.c		\
		src/servo.c

# Common library includes
include common/src/fault_handler.mk
# include common/src/controls/lerp.mk
include common/src/debug.mk
include common/src/peripherals/analog_linear.mk
include common/src/peripherals/stm_adc.mk

# Compiler flags
# USE_OPT = -Og
USE_OPT = -O0

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

# ChibiOS extra includes
include $(CHIBIOS)/os/hal/lib/streams/streams.mk

# Common toolchain includes
include common/makefile

# ChibiOS compilation hooks
PRE_MAKE_ALL_RULE_HOOK: $(BOARD_FILES) $(CLANGD_FILE)