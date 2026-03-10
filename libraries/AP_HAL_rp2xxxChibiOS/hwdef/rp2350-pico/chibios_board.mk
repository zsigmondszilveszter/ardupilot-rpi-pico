# RP2350-specific ChibiOS board configuration for rp2350-pico

HWDEF = $(AP_HAL)/hwdef
BOARDDIR = $(HWDEF)/rp2350-pico

# Target settings.
MCU = cortex-m33

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = softfp
endif

# FPU-related options (RP2350 has FPv5-SP single-precision FPU).
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-sp-d16
endif

# Startup files (ARMv8-M, RP2350).
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_rp2350.mk
# Override the ChibiOS startup default to also enable DCP (CP4) on RP2350.
DADEFS += -UCRT0_CPACR_INIT -DCRT0_CPACR_INIT=0x00F0C300
# Trim EKF3 features that are enabled by generic "large flash" heuristics but
# are unlikely to be needed on this minimal Copter target. This reduces code
# size and hot-path pressure in the AHRS/EKF update loop.
UDEFS += -DEK3_FEATURE_BODY_ODOM=0
UDEFS += -DEK3_FEATURE_EXTERNAL_NAV=0
UDEFS += -DEK3_FEATURE_DRAG_FUSION=0
# Platform (RP2350).
include $(CHIBIOS)/os/hal/ports/RP/RP2350/platform.mk
# Port (ARMv8-M mainline).
include $(CHIBIOS)/os/common/ports/ARMv8-M-ML-ALT/compilers/GCC/mk/port_rp2.mk

# Define linker script file here.
LDSCRIPT = $(BOARDDIR)/RP2350_FLASH.ld

ALLCSRC += $(BOARDDIR)/board.c
ALLCSRC += $(BOARDDIR)/double_math.c
ALLXASMSRC += $(BOARDDIR)/double_aeabi_dcp.S
ALLXASMSRC += $(BOARDDIR)/double_fma_dcp.S
ALLXASMSRC += $(BOARDDIR)/double_sci_m33.S
ALLXASMSRC += $(BOARDDIR)/double_conv_m33.S

# Add board directory to include path (for board.h and mcuconf.h).
ALLINC += $(BOARDDIR)

# Signal that MCU-specific config is loaded, then include shared configuration.
MCU_CONFIG_LOADED = 1
include $(HWDEF)/common/chibios_board.mk
