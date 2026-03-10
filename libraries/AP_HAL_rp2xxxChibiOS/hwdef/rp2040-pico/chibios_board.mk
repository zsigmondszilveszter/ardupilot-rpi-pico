# RP2040-specific ChibiOS board configuration for rp2040-pico

HWDEF = $(AP_HAL)/hwdef
BOARDDIR = $(HWDEF)/rp2040-pico

# Target settings.
MCU = cortex-m0plus

# Startup files (ARMv6-M, RP2040).
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_rp2040.mk
# Platform (RP2040).
include $(CHIBIOS)/os/hal/ports/RP/RP2040/platform.mk
# Port (ARMv6-M).
include $(CHIBIOS)/os/common/ports/ARMv6-M/compilers/GCC/mk/port_rp2.mk

# Linker script file.
LDSCRIPT = $(STARTUPLD)/RP2040_FLASH.ld

# RP2040 ROM floating-point shims: software-float calls routed through RP2040 ROM.
ALLCSRC += $(BOARDDIR)/board.c
ALLCSRC += $(BOARDDIR)/rp2040_rom_float_init.c
ALLXASMSRC += $(BOARDDIR)/float_aeabi_rp2040.S
ALLXASMSRC += $(BOARDDIR)/double_aeabi_rp2040.S
# B0 silicon shims (always compiled; hardcoded in rp2040_rom_float_asm.h).
# On B1 the shims exist in flash but are never called.
ALLXASMSRC += $(BOARDDIR)/float_v1_rom_shim_rp2040.S
ALLXASMSRC += $(BOARDDIR)/double_v1_rom_shim_rp2040.S

# Add board directory to include path (for board.h, mcuconf.h, rp2040_rom_float_asm.h).
ALLINC += $(BOARDDIR)

# Signal that MCU-specific config is loaded, then include shared configuration.
MCU_CONFIG_LOADED = 1
include $(HWDEF)/common/chibios_board.mk
