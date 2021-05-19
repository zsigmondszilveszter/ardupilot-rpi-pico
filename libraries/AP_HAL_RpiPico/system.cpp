/**
 * Ardupilot port to Raspberry Pi Pico
 * Copyright (C) 2021  Szilveszter Zsigmond
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Szilveszter Zsigmond <email@zsigmondszilveszter.website>, May 2021
 */

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>
#include "pico/time.h"

// const AP_HAL::HAL& hal = AP_HAL::get_HAL();
extern const AP_HAL::HAL& hal;

namespace AP_HAL {

    void init(){}

    /**
     * 
     */
    uint32_t millis()
    {
        return to_ms_since_boot(get_absolute_time());
    }

    uint32_t micros()
    {
        return (uint32_t)to_us_since_boot(get_absolute_time());
    }

    uint64_t micros64()
    {
        return to_us_since_boot(get_absolute_time());
    }

    uint64_t millis64()
    {
        return (uint64_t)to_ms_since_boot(get_absolute_time());
    }

    void panic(const char *errormsg, ...)
    {
    #if !defined(HAL_BOOTLOADER_BUILD) && !defined(HAL_NO_LOGGING)
        // INTERNAL_ERROR(AP_InternalError::error_t::panic);
        va_list ap;

        va_start(ap, errormsg);
        vprintf(errormsg, ap);
        va_end(ap);

        hal.scheduler->delay_microseconds(10000);
        while (1) {
            va_start(ap, errormsg);
            vprintf(errormsg, ap);
            va_end(ap);
            hal.scheduler->delay(500);
        }
    #else
        // we don't support variable args in bootlaoder
        chSysHalt(errormsg);
        // we will never get here, this just to silence a warning
        while (1) {}
    #endif
    }

}