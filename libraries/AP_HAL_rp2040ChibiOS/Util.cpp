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

#include "Util.h"

#include "hwdef/common/rp2040_util.h"

using namespace Rp2040ChibiOS;

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
void * Util::std_realloc(void *addr, size_t size)
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

/**
   how much free memory do we have in bytes.
*/
uint32_t Util::available_memory(void)
{
    // from malloc.c in hwdef
    return mem_available();
}