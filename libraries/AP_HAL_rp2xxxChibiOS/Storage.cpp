
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_RP2xxxCHIBIOS

#include <string.h>
#include <fcntl.h>
#include "Storage.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>

using namespace Rp2xxxChibiOS;

extern const AP_HAL::HAL& hal;

#ifndef RP2XXX_STORAGE_DEBUG
#define RP2XXX_STORAGE_DEBUG 0
#endif

#if RP2XXX_STORAGE_DEBUG
#define STORAGE_DEBUG(fmt, args...) ::printf("rp2xxx-storage: " fmt "\n", ##args)
#else
#define STORAGE_DEBUG(fmt, args...)
#endif

void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }

    // ensure APM directory exists
    AP::FS().mkdir("/APM");

    _storage_fd = AP::FS().open(HAL_STORAGE_FILE, O_RDWR|O_CREAT);
    if (_storage_fd == -1) {
        STORAGE_DEBUG("open failed for %s", HAL_STORAGE_FILE);
        _set_error(ErrorCode::OpenFailed);
        return;
    }

    memset(_buffer, 0, sizeof(_buffer));

    // read existing content
    ssize_t ret = AP::FS().read(_storage_fd, _buffer, sizeof(_buffer));
    if (ret < 0) {
        STORAGE_DEBUG("read failed for %s", HAL_STORAGE_FILE);
        _set_error(ErrorCode::ReadFailed);
        ret = 0;
    }
    STORAGE_DEBUG("opened %s fd=%d read=%ld", HAL_STORAGE_FILE, _storage_fd, (long)ret);

    // pre-fill file to full size if shorter
    if ((size_t)ret < sizeof(_buffer)) {
        if (AP::FS().lseek(_storage_fd, ret, SEEK_SET) != ret ||
            AP::FS().write(_storage_fd, &_buffer[ret], sizeof(_buffer) - ret) != (ssize_t)(sizeof(_buffer) - ret)) {
            STORAGE_DEBUG("extend failed at %ld for %s", (long)ret, HAL_STORAGE_FILE);
            _set_error(ErrorCode::ExtendFailed);
            AP::FS().close(_storage_fd);
            _storage_fd = -1;
            return;
        }
        if (AP::FS().fsync(_storage_fd) != 0) {
            STORAGE_DEBUG("extend fsync failed for %s", HAL_STORAGE_FILE);
            _set_error(ErrorCode::ExtendFsyncFailed);
            AP::FS().close(_storage_fd);
            _storage_fd = -1;
            return;
        }
    }

    _dirty_mask = 0;
    _initialised = true;
    STORAGE_DEBUG("storage initialised size=%u", (unsigned)sizeof(_buffer));
}

void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
    if (length == 0) {
        return;
    }
    uint16_t end = loc + length - 1;
    for (uint16_t line = loc >> RP2xxx_STORAGE_LINE_SHIFT;
         line <= end >> RP2xxx_STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask |= (1U << line);
    }
}

void Storage::read_block(void* dst, uint16_t src, size_t n)
{
    if (n == 0 || (size_t)src + n > sizeof(_buffer)) {
        STORAGE_DEBUG("read_block rejected src=%u len=%u", (unsigned)src, (unsigned)n);
        return;
    }
    _storage_open();
    if (!_initialised) {
        STORAGE_DEBUG("read_block using zeros src=%u len=%u", (unsigned)src, (unsigned)n);
        _set_error(ErrorCode::ReadFallbackZero);
        memset(dst, 0, n);
        return;
    }
    memcpy(dst, &_buffer[src], n);
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
    if (n == 0 || (size_t)loc + n > sizeof(_buffer)) {
        STORAGE_DEBUG("write_block rejected dst=%u len=%u", (unsigned)loc, (unsigned)n);
        return;
    }
    if (memcmp(src, &_buffer[loc], n) == 0) {
        return;
    }
    _storage_open();
    if (!_initialised) {
        STORAGE_DEBUG("write_block skipped, storage unavailable dst=%u len=%u", (unsigned)loc, (unsigned)n);
        _set_error(ErrorCode::WriteSkippedUnavailable);
        return;
    }
    WITH_SEMAPHORE(sem);
    memcpy(&_buffer[loc], src, n);
    _mark_dirty(loc, n);
    STORAGE_DEBUG("queued write dst=%u len=%u dirty=0x%08lx", (unsigned)loc, (unsigned)n, (unsigned long)_dirty_mask);
}

void Storage::_timer_tick(void)
{
    if (!_initialised) {
        _storage_open();
        return;
    }

    if (_dirty_mask == 0) {
        _last_empty_ms = AP_HAL::millis();
        return;
    }

    // find first dirty line
    uint16_t i;
    for (i = 0; i < RP2xxx_STORAGE_NUM_LINES; i++) {
        if (_dirty_mask & (1U << i)) {
            break;
        }
    }
    if (i >= RP2xxx_STORAGE_NUM_LINES) {
        return;
    }

    // copy the line under semaphore
    {
        WITH_SEMAPHORE(sem);
        memcpy(tmpline, &_buffer[RP2xxx_STORAGE_LINE_SIZE * i], RP2xxx_STORAGE_LINE_SIZE);
    }

    // write to file
    uint32_t offset = RP2xxx_STORAGE_LINE_SIZE * i;
    if (AP::FS().lseek(_storage_fd, offset, SEEK_SET) != (off_t)offset) {
        STORAGE_DEBUG("flush lseek failed line=%u ofs=%lu", (unsigned)i, (unsigned long)offset);
        _set_error(ErrorCode::FlushLseekFailed);
        return;
    }
    if (AP::FS().write(_storage_fd, tmpline, RP2xxx_STORAGE_LINE_SIZE) != RP2xxx_STORAGE_LINE_SIZE) {
        STORAGE_DEBUG("flush write failed line=%u ofs=%lu", (unsigned)i, (unsigned long)offset);
        _set_error(ErrorCode::FlushWriteFailed);
        return;
    }
    if (AP::FS().fsync(_storage_fd) != 0) {
        STORAGE_DEBUG("flush fsync failed line=%u ofs=%lu", (unsigned)i, (unsigned long)offset);
        _set_error(ErrorCode::FlushFsyncFailed);
        return;
    }

    // clear dirty bit if buffer hasn't changed since we copied it
    WITH_SEMAPHORE(sem);
    if (memcmp(tmpline, &_buffer[RP2xxx_STORAGE_LINE_SIZE * i], RP2xxx_STORAGE_LINE_SIZE) == 0) {
        _dirty_mask &= ~(1U << i);
        STORAGE_DEBUG("flushed line=%u remaining=0x%08lx", (unsigned)i, (unsigned long)_dirty_mask);
    }

    _report_error_later();
}

bool Storage::healthy(void)
{
    return _initialised && (_last_empty_ms != 0) &&
           (AP_HAL::millis() - _last_empty_ms < 30000U);
}

void Storage::_set_error(ErrorCode err)
{
    _last_error = err;
    _error_count++;
    _report_error_later();
}

void Storage::_report_error_later()
{
    if (_last_error == ErrorCode::None || !hal.scheduler->is_system_initialized()) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    if (now - _last_report_ms < 2000U) {
        return;
    }
    _last_report_ms = now;
    hal.console->printf("rp2xxx-storage: %s count=%lu init=%u fd=%d dirty=0x%08lx\n",
                        _error_string(_last_error),
                        (unsigned long)_error_count,
                        (unsigned)_initialised,
                        _storage_fd,
                        (unsigned long)_dirty_mask);
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Storage: %s", _error_string(_last_error));
}

const char *Storage::_error_string(ErrorCode err) const
{
    switch (err) {
    case ErrorCode::None:
        return "none";
    case ErrorCode::OpenFailed:
        return "open failed";
    case ErrorCode::ReadFailed:
        return "read failed";
    case ErrorCode::ExtendFailed:
        return "extend failed";
    case ErrorCode::ExtendFsyncFailed:
        return "extend fsync failed";
    case ErrorCode::ReadFallbackZero:
        return "read fallback zero";
    case ErrorCode::WriteSkippedUnavailable:
        return "write skipped unavailable";
    case ErrorCode::FlushLseekFailed:
        return "flush lseek failed";
    case ErrorCode::FlushWriteFailed:
        return "flush write failed";
    case ErrorCode::FlushFsyncFailed:
        return "flush fsync failed";
    }
    return "unknown";
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_RP2xxxCHIBIOS
