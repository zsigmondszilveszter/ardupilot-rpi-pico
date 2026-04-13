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
#include "board.h"
#if AP_CRASHDUMP_ENABLED
#include <CrashCatcher.h>

#define STR(x) #x
#define XSTR(x) STR(x)

#define RP2XXX_FAULT_BREADCRUMB_ADDR   (0x400D800CU)
#define RP2XXX_FAULT_BREADCRUMB_MAGIC  (0x43524348U) /* "CRCH" */
#define RP2XXX_FAULT_STAGE_BUS         (0x42555346U) /* "BUSF" */
#define RP2XXX_FAULT_STAGE_USAGE       (0x55534746U) /* "USGF" */
#define RP2XXX_FAULT_STAGE_MEM         (0x4D454D46U) /* "MEMF" */
#define RP2XXX_FAULT_STAGE_SECURE      (0x53454346U) /* "SECF" */
/*
 * On RP2040 (Cortex-M0+, ARMv6-M) HardFault_Handler is the only fault
 * vector; it is defined by CrashCatcher_armv6m.S.
 *
 * On RP2350 (Cortex-M33, ARMv8-M) separate BusFault/UsageFault/MemManage
 * vectors exist. CrashCatcher_armv7m.S only provides HardFault_Handler,
 * so we tail-branch the other fault vectors to that same assembly entry.
 *
 * These wrappers must be naked and use a plain branch, not a C call:
 * CrashCatcher expects to see the original EXC_RETURN value in LR and the
 * original exception stack frame untouched.
 */
#if defined(RP2350)
extern "C" {
void HardFault_Handler(void);
void BusFault_Handler(void) __attribute__((naked));
void UsageFault_Handler(void) __attribute__((naked));
void MemManage_Handler(void) __attribute__((naked));
void SecureFault_Handler(void) __attribute__((naked));
}

extern "C" void BusFault_Handler(void)
{
    __asm volatile(
        "ldr r0, =" XSTR(RP2XXX_FAULT_BREADCRUMB_ADDR) "\n"
        "ldr r1, =" XSTR(RP2XXX_FAULT_BREADCRUMB_MAGIC) "\n"
        "str r1, [r0]\n"
        "ldr r1, =" XSTR(RP2XXX_FAULT_STAGE_BUS) "\n"
        "str r1, [r0, #4]\n"
        "b HardFault_Handler\n");
}

extern "C" void UsageFault_Handler(void)
{
    __asm volatile(
        "ldr r0, =" XSTR(RP2XXX_FAULT_BREADCRUMB_ADDR) "\n"
        "ldr r1, =" XSTR(RP2XXX_FAULT_BREADCRUMB_MAGIC) "\n"
        "str r1, [r0]\n"
        "ldr r1, =" XSTR(RP2XXX_FAULT_STAGE_USAGE) "\n"
        "str r1, [r0, #4]\n"
        "b HardFault_Handler\n");
}

extern "C" void MemManage_Handler(void)
{
    __asm volatile(
        "ldr r0, =" XSTR(RP2XXX_FAULT_BREADCRUMB_ADDR) "\n"
        "ldr r1, =" XSTR(RP2XXX_FAULT_BREADCRUMB_MAGIC) "\n"
        "str r1, [r0]\n"
        "ldr r1, =" XSTR(RP2XXX_FAULT_STAGE_MEM) "\n"
        "str r1, [r0, #4]\n"
        "b HardFault_Handler\n");
}

extern "C" void SecureFault_Handler(void)
{
    __asm volatile(
        "ldr r0, =" XSTR(RP2XXX_FAULT_BREADCRUMB_ADDR) "\n"
        "ldr r1, =" XSTR(RP2XXX_FAULT_BREADCRUMB_MAGIC) "\n"
        "str r1, [r0]\n"
        "ldr r1, =" XSTR(RP2XXX_FAULT_STAGE_SECURE) "\n"
        "str r1, [r0, #4]\n"
        "b HardFault_Handler\n");
}
#endif /* RP2350 */
#endif /* AP_CRASHDUMP_ENABLED */
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


} // namespace AP_HAL
