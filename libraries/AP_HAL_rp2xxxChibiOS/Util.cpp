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

#include <AP_Math/AP_Math.h>
#include <AP_Common/ExpandingString.h>
#include <AP_InternalError/AP_InternalError.h>
#include "Util.h"

#include "hwdef/common/rp2xxx_util.h"

using namespace Rp2xxxChibiOS;

const uint8_t rp2xxx_udid[12] = {};

extern "C" {
    uint32_t __main_stack_base__;
    uint32_t __main_stack_end__;
    uint32_t __main_thread_stack_base__;
    uint32_t __main_thread_stack_end__;
}

static uint32_t stack_free(void *stack_base)
{
    const uint32_t *p = static_cast<const uint32_t *>(stack_base);
    const uint32_t canary_word = 0x55555555;
    while (*p == canary_word) {
        p++;
    }
    return uintptr_t(p) - uintptr_t(stack_base);
}

Util::Util() {}

/*
    Special Allocation Routines
*/
void* Util::malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    if (mem_type == AP_HAL::Util::MEM_DMA_SAFE) {
        return malloc_dma(size);
    } else if (mem_type == AP_HAL::Util::MEM_FAST) {
        return malloc_fastmem(size);
    } else {
        return calloc(1, size);
    }
}

void Util::free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    if (ptr != NULL) {
        free(ptr);
    }
}

#ifdef ENABLE_HEAP
void * Util::allocate_heap_memory(size_t size)
{
    memory_heap_t *heap = (memory_heap_t *)malloc(size + sizeof(memory_heap_t));
    if (heap == nullptr) {
        return nullptr;
    }
    chHeapObjectInit(heap, heap + 1U, size);
    return heap;
}

/*
  realloc implementation thanks to wolfssl, used by AP_Scripting
 */
void * Util::std_realloc(void *addr, uint32_t size)
{
    if (size == 0) {
       free(addr);
       return nullptr;
    }
    if (addr == nullptr) {
        return malloc(size);
    }
    void *new_mem = malloc(size);
    if (new_mem != nullptr) {
        memcpy(new_mem, addr, chHeapGetSize(addr) > size ? size : chHeapGetSize(addr));
        free(addr);
    }
    return new_mem;
}

void * Util::heap_realloc(void *heap, void *ptr, size_t old_size, size_t new_size)
{
    if (heap == nullptr) {
        return nullptr;
    }
    if (new_size == 0) {
        if (ptr != nullptr) {
            chHeapFree(ptr);
        }
        return nullptr;
    }
    if (ptr == nullptr) {
        return chHeapAlloc((memory_heap_t *)heap, new_size);
    }
    void *new_mem = chHeapAlloc((memory_heap_t *)heap, new_size);
    if (new_mem != nullptr) {
        const size_t old_size2 = chHeapGetSize(ptr);
#ifdef HAL_DEBUG_BUILD
        if (new_size != 0 && old_size2 != old_size) {
            INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        }
#endif
        memcpy(new_mem, ptr, old_size2 > new_size ? new_size : old_size2);
        chHeapFree(ptr);
    }
    return new_mem;
}
#endif // ENABLE_HEAP

/*
  set HW RTC in UTC microseconds
*/
void Util::set_hw_rtc(uint64_t time_utc_usec)
{
    rp2040_set_utc_usec(time_utc_usec);
}

/*
  get system clock in UTC microseconds
*/
uint64_t Util::get_hw_rtc() const
{
    return rp2040_get_utc_usec();
}

// return true if the reason for the reboot was a watchdog reset
bool Util::was_watchdog_reset() const
{
    return (&WDGD1)->wdg->REASON;
}

#if AP_CRASHDUMP_ENABLED

extern "C" {
    uint32_t rp2xxx_crash_dump_addr(void);
    uint32_t rp2xxx_crash_dump_max_size(void);
    uint32_t rp2xxx_crash_dump_size(void);
}

size_t Util::last_crash_dump_size() const
{
    const uint32_t size = rp2xxx_crash_dump_size();
    // 0x00000000 = never written; 0xFFFFFFFF = erased flash
    if (size == 0U || size == 0xFFFFFFFFU) {
        return 0;
    }
    if (size > rp2xxx_crash_dump_max_size()) {
        return 0;
    }
    // Validate CrashCatcher "cC" signature at the start of the dump
    const uint8_t *ptr = (const uint8_t *)rp2xxx_crash_dump_addr();
    if (ptr[0] != 0x63U || ptr[1] != 0x43U) {
        return 0;
    }
    return (size_t)size;
}

void *Util::last_crash_dump_ptr() const
{
    if (last_crash_dump_size() == 0) {
        return nullptr;
    }
    return (void *)rp2xxx_crash_dump_addr();
}

#endif // AP_CRASHDUMP_ENABLED

/**
   how much free memory do we have in bytes.
*/
uint32_t Util::available_memory(void)
{
    // from malloc.c in hwdef
    return mem_available();
}

void Util::thread_info(ExpandingString &str)
{
    const uint32_t isr_stack_size = uintptr_t(&__main_stack_end__) - uintptr_t(&__main_stack_base__);
    str.printf("ThreadsV2\nISR           PRI=255 sp=%p STACK=%u/%u\n",
               &__main_stack_base__,
               unsigned(stack_free(&__main_stack_base__)),
               unsigned(isr_stack_size));

    for (thread_t *tp = chRegFirstThread(); tp; tp = chRegNextThread(tp)) {
        uint32_t total_stack;
        if (tp->wabase == (void *)&__main_thread_stack_base__) {
            total_stack = uintptr_t(&__main_thread_stack_end__) - uintptr_t(&__main_thread_stack_base__);
        } else {
            total_stack = uintptr_t(tp) - uintptr_t(tp->wabase);
        }

        str.printf("%-13.13s PRI=%3u sp=%p STACK=%u/%u\n",
                   tp->name ? tp->name : "?",
                   unsigned(tp->realprio),
                   tp->wabase,
                   unsigned(stack_free(tp->wabase)),
                   unsigned(total_stack));
    }
}
