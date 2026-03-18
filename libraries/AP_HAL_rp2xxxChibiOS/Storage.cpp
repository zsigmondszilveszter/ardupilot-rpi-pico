
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
        hal.console->printf("Storage: open failed for %s (errno=%d)\n", HAL_STORAGE_FILE, errno);
        return;
    }

    memset(_buffer, 0, sizeof(_buffer));

    // read existing content
    ssize_t ret = AP::FS().read(_storage_fd, _buffer, sizeof(_buffer));
    if (ret < 0) {
        hal.console->printf("Storage: read failed (errno=%d), treating as empty\n", errno);
        ret = 0;
    }
    hal.console->printf("Storage: opened %s, read %d bytes\n", HAL_STORAGE_FILE, (int)ret);

    // pre-fill file to full size if shorter
    if ((size_t)ret < sizeof(_buffer)) {
        hal.console->printf("Storage: extending file from %d to %u bytes\n", (int)ret, (unsigned)sizeof(_buffer));
        if (AP::FS().lseek(_storage_fd, ret, SEEK_SET) != ret ||
            AP::FS().write(_storage_fd, &_buffer[ret], sizeof(_buffer) - ret) != (ssize_t)(sizeof(_buffer) - ret)) {
            hal.console->printf("Storage: pre-fill write failed (errno=%d)\n", errno);
            AP::FS().close(_storage_fd);
            _storage_fd = -1;
            return;
        }
        if (AP::FS().fsync(_storage_fd) != 0) {
            hal.console->printf("Storage: pre-fill fsync failed (errno=%d)\n", errno);
            AP::FS().close(_storage_fd);
            _storage_fd = -1;
            return;
        }
    }

    _dirty_mask = 0;
    _initialised = true;
    hal.console->printf("Storage: initialised OK\n");
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
        if (!_write_error_logged) {
            hal.console->printf("Storage: lseek failed at offset %u (errno=%d)\n", (unsigned)offset, errno);
            _write_error_logged = true;
        }
        return;
    }
    if (AP::FS().write(_storage_fd, tmpline, RP2xxx_STORAGE_LINE_SIZE) != RP2xxx_STORAGE_LINE_SIZE) {
        if (!_write_error_logged) {
            hal.console->printf("Storage: write failed for line %u (errno=%d)\n", (unsigned)i, errno);
            _write_error_logged = true;
        }
        return;
    }
    if (AP::FS().fsync(_storage_fd) != 0) {
        if (!_write_error_logged) {
            hal.console->printf("Storage: fsync failed for line %u (errno=%d)\n", (unsigned)i, errno);
            _write_error_logged = true;
        }
        return;
    }
    _write_error_logged = false;

    // clear dirty bit if buffer hasn't changed since we copied it
    WITH_SEMAPHORE(sem);
    if (memcmp(tmpline, &_buffer[RP2xxx_STORAGE_LINE_SIZE * i], RP2xxx_STORAGE_LINE_SIZE) == 0) {
        _dirty_mask &= ~(1U << i);
    }
}

bool Storage::healthy(void)
{
    if (!_initialised) {
        return false;
    }
    if (_last_empty_ms == 0) {
        // Never seen a clean dirty mask yet — storage ticked but no flush cycle completed,
        // or _timer_tick has not run with an empty dirty mask even once.
        hal.console->printf("Storage: unhealthy - not yet flushed (fd=%d dirty=0x%lx)\n",
                            _storage_fd, (unsigned long)_dirty_mask);
        return false;
    }
    uint32_t now = AP_HAL::millis();
    uint32_t age = now - _last_empty_ms;
    if (age >= 30000U) {
        hal.console->printf("Storage: unhealthy - last flush was %lums ago (dirty=0x%lx)\n",
                            (unsigned long)age, (unsigned long)_dirty_mask);
        return false;
    }
    return true;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_RP2xxxCHIBIOS
