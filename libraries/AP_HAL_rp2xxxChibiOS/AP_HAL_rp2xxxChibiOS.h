#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

/**
 * Umbrella header for AP_HAL_rp2xxxChibiOS module.
 * The module header exports singleton instances which must conform the
 * AP_HAL::HAL interface. It may only expose implementation details (class
 * names, headers) via the rp2xxxChibiOS namespace.
 * The class implementing AP_HAL::HAL should be called HAL_rp2xxxChibiOS and exist
 * in the global namespace. There should be a single const instance of the
 * HAL_rp2xxxChibiOS class called AP_HAL_rp2xxxChibiOS, instantiated in the HAL_rp2xxxChibiOS_Class.cpp
 * and exported as `extern const HAL_rp2xxxChibiOS AP_HAL_rp2xxxChibiOS;` in HAL_rp2xxxChibiOS_Class.h
 *
 * All declaration and compilation should be guarded by CONFIG_HAL_BOARD macros.
 * In this case, we're using CONFIG_HAL_BOARD == HAL_BOARD_RP2xxxCHIBIOS.
 * When creating a new HAL, declare a new HAL_BOARD_ in AP_HAL/AP_HAL_Boards.h
 */

#include "HAL_rp2xxxChibiOS_Class.h"
