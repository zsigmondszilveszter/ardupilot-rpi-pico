/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <stdarg.h>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#if AP_CRASHDUMP_ENABLED
#include <CrashCatcher.h>
#endif
#include <ch.h>
#include "hal.h"
#include <hrt.h>

extern const AP_HAL::HAL& hal;

void *__dso_handle;

namespace AP_HAL {

void init()
{
    // TODO
}

void panic(const char *errormsg, ...)
{
#if !defined(HAL_BOOTLOADER_BUILD) && !APM_BUILD_TYPE(APM_BUILD_iofirmware)
    INTERNAL_ERROR(AP_InternalError::error_t::panic);
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

__FASTRAMFUNC__ uint32_t micros()
{
    return hrt_micros32();
}

uint16_t micros16()
{
    return hrt_micros32() & 0xFFFF;
}
    
__FASTRAMFUNC__ uint32_t millis()
{
    return hrt_millis32();
}

__FASTRAMFUNC__ uint16_t millis16()
{
    return hrt_millis32() & 0xFFFF;
}

__FASTRAMFUNC__ uint64_t micros64()
{
    return hrt_micros64();
}

__FASTRAMFUNC__ uint64_t millis64()
{
    return hrt_micros64() / 1000U;
}


__FASTRAMFUNC__ uint32_t native_micros()
{
    return micros();
}

__FASTRAMFUNC__ uint32_t native_millis()
{
    return millis();
}

__FASTRAMFUNC__ uint16_t native_millis16()
{
    return millis16();
}

__FASTRAMFUNC__ uint64_t native_micros64()
{
    return micros64();
}

__FASTRAMFUNC__ uint64_t native_millis64()
{
    return millis64();
}


} // namespace AP_HAL
