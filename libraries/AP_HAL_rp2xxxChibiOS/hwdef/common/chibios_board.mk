##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = -Os
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -std=gnu++11
endif

# Assembly specific options here (added to USE_OPT).
ifeq ($(USE_ASOPT),)
  USE_ASOPT =
endif

# Assembly specific options here (added to USE_ASXOPT).
ifeq ($(USE_ASXOPT),)
  USE_ASXOPT =
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk
USE_FATFS = yes
FATFS_FLAGS = -DUSE_POSIX
ifeq ($(USE_FATFS),yes)
# Use rp2xxxChibiOS bindings but the extracted FatFS core from modules/ChibiOS
CHIBIOS_FATFS_CORE = $(AP_HAL)/../../modules/ChibiOS
FATFSSRC = $(CHIBIOS)/os/various/fatfs_bindings/fatfs_diskio.c \
           $(CHIBIOS)/os/various/fatfs_bindings/fatfs_syscall.c \
           $(CHIBIOS_FATFS_CORE)/ext/fatfs/source/ff.c \
           $(CHIBIOS_FATFS_CORE)/ext/fatfs/source/ffunicode.c
FATFSINC = $(CHIBIOS_FATFS_CORE)/ext/fatfs/source
ALLCSRC += $(FATFSSRC)
ALLINC  += $(FATFSINC)
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#
HWDEF = $(AP_HAL)/hwdef
# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = softfp
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#
CONFDIR := $(HWDEF)/common

# Define project name here
PROJECT = ch

# Target settings.
MCU  = cortex-m0plus

# Imported source files and paths
# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_rp2040.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/RP/RP2040/platform.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv6-M/compilers/GCC/mk/port_rp2.mk
# Auto-build files in ./source recursively.
include $(CHIBIOS)/tools/mk/autobuild.mk
# Other files (optional).
include $(CHIBIOS)/os/hal/lib/streams/streams.mk


# Define linker script file here
LDSCRIPT= $(STARTUPLD)/RP2xxx_FLASH.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.

CSRC = $(sort $(ALLCSRC))

CSRC += $(HWDEF)/common/hrt.c \
     $(HWDEF)/common/malloc.c \
     $(HWDEF)/common/board.c \
     $(HWDEF)/common/rp2xxx_util.c \
     $(HWDEF)/common/usbcfg.c \
     $(HWDEF)/common/bouncebuffer.c \
     $(HWDEF)/common/stubs.c \
     $(HWDEF)/common/pio.c \
     $(HWDEF)/common/rp2040_rom_float_init.c

# RP2040 ROM floating-point shims: added to ALLXASMSRC so they survive the
# ASMXSRC = $(ALLXASMSRC) reset below.
ALLXASMSRC += $(HWDEF)/common/float_aeabi_rp2040.S
ALLXASMSRC += $(HWDEF)/common/double_aeabi_rp2040.S
# B0 silicon shims: always compiled (PICO_*_SUPPORT_ROM_V1=1 is hardcoded in
# rp2040_rom_float_asm.h). On B1 the shims exist in flash but are never called.
ALLXASMSRC += $(HWDEF)/common/float_v1_rom_shim_rp2040.S
ALLXASMSRC += $(HWDEF)/common/double_v1_rom_shim_rp2040.S
#	   $(TESTSRC) \
#	   test.c
ifneq ($(CRASHCATCHER),)
LIBCC_CSRC = $(CRASHCATCHER)/Core/src/CrashCatcher.c \
             $(HWDEF)/common/crashdump.c

LIBCC_ASMXSRC = $(CRASHCATCHER)/Core/src/CrashCatcher_armv7m.S
endif

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(sort $(ALLCPPSRC))

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

INCDIR = $(CHIBIOS)/os/license \
         $(ALLINC) $(HWDEF)/common

ifneq ($(CRASHCATCHER),)
INCDIR += $(CRASHCATCHER)/include
endif
#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes -Werror

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef -Werror

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = $(ENV_UDEFS) $(FATFS_FLAGS) -DHAL_BOARD_NAME=\"$(HAL_BOARD_NAME)\" \
        -DHAL_MAX_STACK_FRAME_SIZE=$(HAL_MAX_STACK_FRAME_SIZE) \
        -DCRT0_VTOR_INIT=1 -DCRT0_EXTRA_CORES_NUMBER=1 -DPICO_NO_FPGA_CHECK -DNDEBUG -DHAL_SDCARD_SPI_HOOK=TRUE

ifeq ($(ENABLE_ASSERTS),yes)
 UDEFS += -DHAL_CHIBIOS_ENABLE_ASSERTS
 ASXFLAGS += -DHAL_CHIBIOS_ENABLE_ASSERTS
endif

ifeq ($(ENABLE_MALLOC_GUARD),yes)
 UDEFS += -DHAL_CHIBIOS_ENABLE_MALLOC_GUARD
 ASXFLAGS += -DHAL_CHIBIOS_ENABLE_MALLOC_GUARD
endif

ifeq ($(ENABLE_STATS),yes)
 UDEFS += -DHAL_ENABLE_THREAD_STATISTICS
 ASXFLAGS += -DHAL_ENABLE_THREAD_STATISTICS
endif

# Define ASM defines here
UADEFS = -DCRT0_VTOR_INIT=1 -DCRT0_EXTRA_CORES_NUMBER=1

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user section
##############################################################################

##############################################################################
# Common rules
#
#
# Common rules
##############################################################################

##############################################################################
# Custom rules
#
#
# Custom rules
##############################################################################

include $(HWDEF)/common/chibios_common.mk
