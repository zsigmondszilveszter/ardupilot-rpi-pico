#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

/**
 * Umbrella header for AP_HAL_rp2040ChibiOS module.
 * The module header exports singleton instances which must conform the
 * AP_HAL::HAL interface. It may only expose implementation details (class
 * names, headers) via the rp2040ChibiOS namespace.
 * The class implementing AP_HAL::HAL should be called HAL_rp2040ChibiOS and exist
 * in the global namespace. There should be a single const instance of the
 * HAL_rp2040ChibiOS class called AP_HAL_rp2040ChibiOS, instantiated in the HAL_rp2040ChibiOS_Class.cpp
 * and exported as `extern const HAL_rp2040ChibiOS AP_HAL_rp2040ChibiOS;` in HAL_rp2040ChibiOS_Class.h
 *
 * All declaration and compilation should be guarded by CONFIG_HAL_BOARD macros.
 * In this case, we're using CONFIG_HAL_BOARD == HAL_BOARD_RP2040CHIBIOS.
 * When creating a new HAL, declare a new HAL_BOARD_ in AP_HAL/AP_HAL_Boards.h
 */

#include "HAL_rp2040ChibiOS_Class.h"
