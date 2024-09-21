# Source: https://github.com/ChibiOS/ChibiOS/blob/master/demos/STM32/RT-STM32F407-DISCOVERY/Makefile

# Build Options ---------------------------------------------------------------------------------------------------------------

# Compiler options here.
USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16

# C specific options here (added to USE_OPT).
USE_COPT = 

# C++ specific options here (added to USE_OPT).
USE_CPPOPT = -fno-rtti

# Enable this if you want the linker to remove unused code and data.
USE_LINK_GC = yes

# Linker extra options here.
USE_LDOPT = 

# Enable this if you want link time optimizations (LTO).
USE_LTO = yes

# Enable this if you want to see the full log while compiling.
USE_VERBOSE_COMPILE = no

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
USE_SMART_BUILD = yes

# Architecture / Project Specific Options -------------------------------------------------------------------------------------

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
USE_PROCESS_STACKSIZE = 0x400

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
USE_EXCEPTIONS_STACKSIZE = 0x400

# Enables the use of FPU (no, softfp, hard).
# TODO(Barach): This should be enabled
USE_FPU = no

# FPU-related options.
USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16

# Project, Target, Sources & Paths --------------------------------------------------------------------------------------------

# Define project name here
PROJECT = stub

# Target settings.
MCU  = cortex-m4

# Imported source files and paths.
CHIBIOS  := $(CHIBIOS_SOURCE_PATH)
CONFDIR  := ./config
BUILDDIR := ./build
DEPDIR   := ./build/dep
BOARDDIR := ./build/board

# Licensing files.
include $(CHIBIOS)/os/license/license.mk

# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk

# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk

# Board Files
ALLCSRC += $(BOARDDIR)/board.c
ALLINC += $(BOARDDIR)

# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk

# Auto-build files in ./source recursively.
include $(CHIBIOS)/tools/mk/autobuild.mk

# Other files (optional).
include $(CHIBIOS)/os/test/test.mk
include $(CHIBIOS)/test/rt/rt_test.mk
include $(CHIBIOS)/test/oslib/oslib_test.mk

# Define linker script file here
LDSCRIPT= $(STARTUPLD)/STM32F407xG.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(TESTSRC) \
       src/main.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC)

# List ASM source files here.
ASMSRC = $(ALLASMSRC)

# List ASM with preprocessor source files here.
ASMXSRC = $(ALLXASMSRC)

# Inclusion directories.
INCDIR = $(CONFDIR) $(ALLINC) $(TESTINC)

# Define C warning options here.
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here.
CPPWARN = -Wall -Wextra -Wundef

# User section ----------------------------------------------------------------------------------------------------------------

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

# Common Rules ----------------------------------------------------------------------------------------------------------------

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

# Board Files -----------------------------------------------------------------------------------------------------------------

BOARD_CONF := $(CONFDIR)/board.chcfg $(CONFDIR)/board.fmpp
BOARD_FILES := $(BOARDDIR)/board.h $(BOARDDIR)/board.c

PRE_MAKE_ALL_RULE_HOOK: $(BOARD_FILES)

$(BOARD_FILES) &: $(BOARD_CONF)
	mkdir -p $(BOARDDIR)
	fmpp -C $(CONFDIR)/board.fmpp --data-root=$(CONFDIR) -S									\
		$(CHIBIOS_SOURCE_PATH)/tools/ftl/processors/boards/stm32f4xx/templates				\
		--freemarker-links=lib:$(CHIBIOS_SOURCE_PATH)/tools/ftl/libs -O $(BOARDDIR)

board_files: $(BOARD_FILES)

# Board Programming -----------------------------------------------------------------------------------------------------------

flash: build/$(PROJECT).elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/$(PROJECT).elf verify reset exit"
