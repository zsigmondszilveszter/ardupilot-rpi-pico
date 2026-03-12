#pragma once

#include "AP_HAL_rp2xxxChibiOS.h"
#include <AP_HAL/AP_HAL.h>

#define RP2xxx_STORAGE_LINE_SHIFT   9
#define RP2xxx_STORAGE_LINE_SIZE    (1<<RP2xxx_STORAGE_LINE_SHIFT)
#define RP2xxx_STORAGE_NUM_LINES    (HAL_STORAGE_SIZE/RP2xxx_STORAGE_LINE_SIZE)

#ifndef HAL_STORAGE_FILE
#define HAL_STORAGE_FILE "/APM/ArduPilot.stg"
#endif

class Rp2xxxChibiOS::Storage : public AP_HAL::Storage {
public:
    Storage() {}
    void init() override {}
    void read_block(void *dst, uint16_t src, size_t n) override;
    void write_block(uint16_t dst, const void* src, size_t n) override;
    void _timer_tick(void) override;
    bool healthy(void) override;

private:
    enum class ErrorCode : uint8_t {
        None = 0,
        OpenFailed,
        ReadFailed,
        ExtendFailed,
        ExtendFsyncFailed,
        ReadFallbackZero,
        WriteSkippedUnavailable,
        FlushLseekFailed,
        FlushWriteFailed,
        FlushFsyncFailed,
    };

    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    void _set_error(ErrorCode err);
    void _report_error_later();
    const char *_error_string(ErrorCode err) const;

    uint8_t _buffer[HAL_STORAGE_SIZE] __attribute__((aligned(4)));
    uint32_t _dirty_mask;          // 1 bit per line (4 lines)
    int _storage_fd = -1;
    bool _initialised = false;
    HAL_Semaphore sem;
    uint8_t tmpline[RP2xxx_STORAGE_LINE_SIZE];
    uint32_t _last_empty_ms = 0;
    volatile ErrorCode _last_error = ErrorCode::None;
    uint32_t _error_count = 0;
    uint32_t _last_report_ms = 0;
};
