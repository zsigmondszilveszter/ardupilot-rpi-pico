#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_RpiPico_Namespace.h"

class RpiPico::Util : public AP_HAL::Util {
public:
    Util();
    bool run_debug_shell(AP_HAL::BetterStream *stream) override { return true; }

#ifdef ENABLE_HEAP
    // heap functions, note that a heap once alloc'd cannot be dealloc'd
    virtual void *allocate_heap_memory(size_t size) override;
    virtual void *heap_realloc(void *heap, void *ptr, size_t new_size) override;
#if USE_LIBC_REALLOC
    virtual void *std_realloc(void *ptr, size_t new_size) { return realloc(ptr, new_size); }
#else
    virtual void *std_realloc(void *ptr, size_t new_size) override;
#endif // USE_LIBC_REALLOC
#endif // ENABLE_HEAP
};
