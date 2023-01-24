#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_rp2040ChibiOS_Namespace.h"

#include "hal.h"


// Watchdog deadline set to X ms.
static const WDGConfig wdgcfg = { .rlr = RP2040_WATCHDOG_TIMEOUT };


class Rp2040ChibiOS::Util : public AP_HAL::Util {
public:
    Util();
    bool run_debug_shell(AP_HAL::BetterStream *stream) override { return true; }
    uint32_t available_memory() override;

    // Special Allocation Routines
    void *malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type) override;
    void free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type) override;

#ifdef ENABLE_HEAP
    // heap functions, note that a heap once alloc'd cannot be dealloc'd
    virtual void *allocate_heap_memory(size_t size) override;
    virtual void *heap_realloc(void *heap, void *ptr, size_t old_size, size_t new_size) override;
    virtual void *std_realloc(void *ptr, size_t new_size) override;
#endif // ENABLE_HEAP

    // return true if the reason for the reboot was a watchdog reset
    bool was_watchdog_reset() const override;

    /*
      set HW RTC in UTC microseconds
     */
    void set_hw_rtc(uint64_t time_utc_usec) override;

    /*
      get system clock in UTC microseconds
     */
    uint64_t get_hw_rtc() const override;
};
