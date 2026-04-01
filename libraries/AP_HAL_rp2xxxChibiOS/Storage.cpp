
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_RP2xxxCHIBIOS

#include <string.h>
#include <fcntl.h>
#include "Storage.h"
#include <AP_Filesystem/AP_Filesystem.h>

using namespace Rp2xxxChibiOS;

extern const AP_HAL::HAL& hal;

static constexpr uint32_t STORAGE_OPEN_RETRY_MS = 3000;
static constexpr uint16_t STORAGE_OPEN_LOG_EVERY_NTH_RETRY = 15;

void Storage::init(void)
{
    const uint32_t start_ms = AP_HAL::millis();
    while (!_initialised && AP_HAL::millis() - start_ms < 4000U) {
        _storage_open(true);
        if (_initialised) {
            break;
        }
        hal.scheduler->delay(100);
    }
}

void Storage::_storage_open(bool force_retry)
{
    if (_initialised) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (!force_retry &&
        _last_open_attempt_ms != 0 &&
        now - _last_open_attempt_ms < STORAGE_OPEN_RETRY_MS) {
        return;
    }
    _last_open_attempt_ms = now;

    const auto note_failure = [&](const char *step, int err) {
        _open_fail_count++;
        const bool should_log = !_open_failed ||
            (_open_fail_count % STORAGE_OPEN_LOG_EVERY_NTH_RETRY) == 0;
        _open_failed = true;
        if (should_log) {
            hal.console->printf("Storage: %s failed for %s (errno=%d, retry=%u)\n",
                                step, HAL_STORAGE_FILE, err,
                                (unsigned)_open_fail_count);
        }
    };

    // ensure APM directory exists
    AP::FS().mkdir("/APM");

    _storage_fd = AP::FS().open(HAL_STORAGE_FILE, O_RDWR|O_CREAT);
    if (_storage_fd == -1) {
        note_failure("open", errno);
        return;
    }

    if (!_buffer_loaded) {
        // First open: read existing content from disk
        memset(_buffer, 0, sizeof(_buffer));

        ssize_t ret = AP::FS().read(_storage_fd, _buffer, sizeof(_buffer));
        if (ret < 0) {
            note_failure("read", errno);
            ret = 0;
        }
        hal.console->printf("Storage: opened %s, read %d bytes\n", HAL_STORAGE_FILE, (int)ret);

        // pre-fill file to full size if shorter
        if ((size_t)ret < sizeof(_buffer)) {
            hal.console->printf("Storage: extending file from %d to %u bytes\n", (int)ret, (unsigned)sizeof(_buffer));
            if (AP::FS().lseek(_storage_fd, ret, SEEK_SET) != ret ||
                AP::FS().write(_storage_fd, &_buffer[ret], sizeof(_buffer) - ret) != (ssize_t)(sizeof(_buffer) - ret)) {
                note_failure("pre-fill write", errno);
                AP::FS().close(_storage_fd);
                _storage_fd = -1;
                return;
            }
            if (AP::FS().fsync(_storage_fd) != 0) {
                note_failure("pre-fill fsync", errno);
                AP::FS().close(_storage_fd);
                _storage_fd = -1;
                return;
            }
        }

        _dirty_mask = 0;
        _buffer_loaded = true;
    } else {
        // Recovery open: buffer already holds the latest state, skip disk read.
        // _dirty_mask is already set to all-lines-dirty by the recovery trigger.
        hal.console->printf("Storage: recovery open OK (fd=%d dirty=0x%lx)\n",
                            _storage_fd, (unsigned long)_dirty_mask);
    }

    _initialised = true;
    if (_open_failed) {
        hal.console->printf("Storage: recovered after %u failed retries\n",
                            (unsigned)_open_fail_count);
        _open_failed = false;
        _open_fail_count = 0;
    }
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

// Number of consecutive write failures before triggering a recovery open.
// Each failure from our perspective may already include a FATFS-level remount
// attempt (100ms delay), so this is intentionally a small number.
#define STORAGE_WRITE_FAIL_THRESHOLD 5

void Storage::_timer_tick(void)
{
    if (!_initialised) {
        _storage_open();
        return;
    }

    if (_dirty_mask == 0) {
        _last_empty_ms = AP_HAL::millis();
        _write_fail_count = 0;
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
    bool ok = true;

    if (AP::FS().lseek(_storage_fd, offset, SEEK_SET) != (off_t)offset) {
        if (!_write_error_logged) {
            hal.console->printf("Storage: lseek failed at offset %u (errno=%d)\n", (unsigned)offset, errno);
            _write_error_logged = true;
        }
        ok = false;
    } else if (AP::FS().write(_storage_fd, tmpline, RP2xxx_STORAGE_LINE_SIZE) != RP2xxx_STORAGE_LINE_SIZE) {
        if (!_write_error_logged) {
            hal.console->printf("Storage: write failed for line %u (errno=%d)\n", (unsigned)i, errno);
            _write_error_logged = true;
        }
        ok = false;
    } else if (AP::FS().fsync(_storage_fd) != 0) {
        if (!_write_error_logged) {
            hal.console->printf("Storage: fsync failed for line %u (errno=%d)\n", (unsigned)i, errno);
            _write_error_logged = true;
        }
        ok = false;
    }

    if (!ok) {
        if (++_write_fail_count >= STORAGE_WRITE_FAIL_THRESHOLD) {
            // The FIL struct is likely zeroed from a failed FATFS remount
            // attempt, making lseek/write permanently return wrong error
            // codes that bypass the FATFS auto-recovery path. Force a full
            // reopen via _storage_open() which goes through CHECK_REMOUNT()
            // in AP::FS().open(), allowing FATFS to properly reopen the fd.
            // _buffer holds the latest state so we skip the disk read.
            hal.console->printf("Storage: triggering recovery after %u failures\n",
                                (unsigned)_write_fail_count);
            AP::FS().close(_storage_fd);
            _storage_fd = -1;
            _dirty_mask = (1U << RP2xxx_STORAGE_NUM_LINES) - 1;
            _initialised = false;
            _write_fail_count = 0;
            _write_error_logged = false;
        }
        return;
    }

    _write_error_logged = false;
    _write_fail_count = 0;

    // clear dirty bit if buffer hasn't changed since we copied it
    WITH_SEMAPHORE(sem);
    if (memcmp(tmpline, &_buffer[RP2xxx_STORAGE_LINE_SIZE * i], RP2xxx_STORAGE_LINE_SIZE) == 0) {
        _dirty_mask &= ~(1U << i);
    }
}

bool Storage::erase(void)
{
    // Firmware-change erase happens very early in boot. On RP2xxx the backing
    // store is the SD-backed storage file, so do a bounded synchronous retry
    // here instead of relying on the later timer-driven reopen path.
    const uint32_t start_ms = AP_HAL::millis();
    while (!_initialised && AP_HAL::millis() - start_ms < 4000U) {
        _storage_open(true);
        if (_initialised) {
            break;
        }
        hal.scheduler->delay(100);
    }
    if (!_initialised) {
        return false;
    }
    // Base class zeros storage via write_block() in 16-byte chunks,
    // marking all lines dirty so _timer_tick() flushes them to disk.
    return AP_HAL::Storage::erase();
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
