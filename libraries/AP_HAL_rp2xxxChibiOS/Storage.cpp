
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_RP2xxxCHIBIOS

#include <string.h>
#include <fcntl.h>
#include "Storage.h"
#include <AP_Filesystem/AP_Filesystem.h>

using namespace Rp2xxxChibiOS;

extern const AP_HAL::HAL& hal;

void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }

    // ensure APM directory exists
    AP::FS().mkdir("/APM");

    _storage_fd = AP::FS().open(HAL_STORAGE_FILE, O_RDWR|O_CREAT);
    if (_storage_fd == -1) {
        return;
    }

    memset(_buffer, 0, sizeof(_buffer));

    // read existing content
    ssize_t ret = AP::FS().read(_storage_fd, _buffer, sizeof(_buffer));
    if (ret < 0) {
        ret = 0;
    }

    // pre-fill file to full size if shorter
    if ((size_t)ret < sizeof(_buffer)) {
        if (AP::FS().lseek(_storage_fd, ret, SEEK_SET) != ret ||
            AP::FS().write(_storage_fd, &_buffer[ret], sizeof(_buffer) - ret) != (ssize_t)(sizeof(_buffer) - ret)) {
            AP::FS().close(_storage_fd);
            _storage_fd = -1;
            return;
        }
        if (AP::FS().fsync(_storage_fd) != 0) {
            AP::FS().close(_storage_fd);
            _storage_fd = -1;
            return;
        }
    }

    _dirty_mask = 0;
    _initialised = true;
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
        return;
    }
    _storage_open();
    if (!_initialised) {
        memset(dst, 0, n);
        return;
    }
    memcpy(dst, &_buffer[src], n);
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
    if (n == 0 || (size_t)loc + n > sizeof(_buffer)) {
        return;
    }
    if (memcmp(src, &_buffer[loc], n) == 0) {
        return;
    }
    _storage_open();
    if (!_initialised) {
        return;
    }
    WITH_SEMAPHORE(sem);
    memcpy(&_buffer[loc], src, n);
    _mark_dirty(loc, n);
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
        return;
    }
    if (AP::FS().write(_storage_fd, tmpline, RP2xxx_STORAGE_LINE_SIZE) != RP2xxx_STORAGE_LINE_SIZE) {
        return;
    }
    if (AP::FS().fsync(_storage_fd) != 0) {
        return;
    }

    // clear dirty bit if buffer hasn't changed since we copied it
    WITH_SEMAPHORE(sem);
    if (memcmp(tmpline, &_buffer[RP2xxx_STORAGE_LINE_SIZE * i], RP2xxx_STORAGE_LINE_SIZE) == 0) {
        _dirty_mask &= ~(1U << i);
    }
}

bool Storage::healthy(void)
{
    return _initialised && (_last_empty_ms != 0) &&
           (AP_HAL::millis() - _last_empty_ms < 30000U);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_RP2xxxCHIBIOS
