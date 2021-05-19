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

#include "Console.h"

// Raspbery Pi Pico SDK headers
#include "tusb.h"
#include "pico/time.h"
#include "pico/mutex.h"
#include "hardware/irq.h"
#include "pico/stdio_usb.h"


static_assert(PICO_STDIO_USB_LOW_PRIORITY_IRQ > RTC_IRQ, ""); // note RTC_IRQ is currently the last one
static mutex_t stdio_usb_mutex;

static void low_priority_worker_irq(void) {
    // if the mutex is already owned, then we are in user code
    // in this file which will do a tud_task itself, so we'll just do nothing
    // until the next tick; we won't starve
    if (mutex_try_enter(&stdio_usb_mutex, NULL)) {
        tud_task();
        mutex_exit(&stdio_usb_mutex);
    }
}

static int64_t timer_task(__unused alarm_id_t id, __unused void *user_data) {
    irq_set_pending(PICO_STDIO_USB_LOW_PRIORITY_IRQ);
    return PICO_STDIO_USB_TASK_INTERVAL_US;
}

static void usb_out_chars(const char *buf, int length) {
    static uint64_t last_avail_time;
    uint32_t owner;
    if (!mutex_try_enter(&stdio_usb_mutex, &owner)) {
        if (owner == get_core_num()) return; // would deadlock otherwise
        mutex_enter_blocking(&stdio_usb_mutex);
    }
    if (tud_cdc_connected()) {
        for (int i = 0; i < length;) {
            int n = length - i;
            int avail = tud_cdc_write_available();
            if (n > avail) n = avail;
            if (n) {
                int n2 = tud_cdc_write(buf + i, n);
                tud_task();
                tud_cdc_write_flush();
                i += n2;
                last_avail_time = time_us_64();
            } else {
                tud_task();
                tud_cdc_write_flush();
                if (!tud_cdc_connected() ||
                    (!tud_cdc_write_available() && time_us_64() > last_avail_time + PICO_STDIO_USB_STDOUT_TIMEOUT_US)) {
                    break;
                }
            }
        }
    } else {
        // reset our timeout
        last_avail_time = 0;
    }
    mutex_exit(&stdio_usb_mutex);
}

int usb_in_chars(char *buf, int length) {
    uint32_t owner;
    if (!mutex_try_enter(&stdio_usb_mutex, &owner)) {
        if (owner == get_core_num()) return PICO_ERROR_NO_DATA; // would deadlock otherwise
        mutex_enter_blocking(&stdio_usb_mutex);
    }
    int rc = PICO_ERROR_NO_DATA;
    if (tud_cdc_connected() && tud_cdc_available()) {
        int count = tud_cdc_read(buf, length);
        rc =  count ? count : PICO_ERROR_NO_DATA;
    }
    mutex_exit(&stdio_usb_mutex);
    return rc;
}

static bool last_ended_with_cr = false;
static void usb_out_chars_crlf(const char *s, int len) {
    int first_of_chunk = 0;
    static const char crlf_str[] = {'\r', '\n'};
    for (int i = 0; i < len; i++) {
        bool prev_char_was_cr = i > 0 ? s[i - 1] == '\r' : last_ended_with_cr;
        if (s[i] == '\n' && !prev_char_was_cr) {
            if (i > first_of_chunk) {
                usb_out_chars(&s[first_of_chunk], i - first_of_chunk);
            }
            usb_out_chars(crlf_str, 2);
            first_of_chunk = i + 1;
        }
    }
    if (first_of_chunk < len) {
        usb_out_chars(&s[first_of_chunk], len - first_of_chunk);
    }
    if (len > 0) {
        last_ended_with_cr = s[len - 1] == '\r';
    }
}


RpiPico::Console::Console() 
{
    tusb_init(); // initialize tinyusb stack
    irq_set_exclusive_handler(PICO_STDIO_USB_LOW_PRIORITY_IRQ, low_priority_worker_irq);
    irq_set_enabled(PICO_STDIO_USB_LOW_PRIORITY_IRQ, true);

    mutex_init(&stdio_usb_mutex);
    add_alarm_in_us(PICO_STDIO_USB_TASK_INTERVAL_US, timer_task, NULL, true);
}

void RpiPico::Console::begin(uint32_t b) {
    //
}
void RpiPico::Console::begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    //
}
void RpiPico::Console::end() {
    //
}
void RpiPico::Console::flush() {
    //
}
bool RpiPico::Console::is_initialized() {
    return tusb_inited();
}
void RpiPico::Console::set_blocking_writes(bool blocking) {
    //
}
bool RpiPico::Console::tx_pending() { 
    return false; 
}

/* Rpi Pico implementations of Stream virtual methods */
uint32_t RpiPico::Console::available() {
    return tud_cdc_available();
}
uint32_t RpiPico::Console::txspace() { 
    return tud_cdc_write_available(); 
}
int16_t RpiPico::Console::read() {
    uint8_t c = 0;
    int16_t res = usb_in_chars((char *)c, 1);
    return res < PICO_OK ? -1 : c;
}
ssize_t RpiPico::Console::read(uint8_t *buffer, uint16_t count) {
    int16_t res = usb_in_chars((char *)buffer, count);
    return res < PICO_OK ? -1 : res;
}
bool RpiPico::Console::discard_input() { return false; }

/* Rpi Pico implementations of Print virtual methods */
size_t RpiPico::Console::write(uint8_t c) {
    write(&c, 1);
    return 1;
}

size_t RpiPico::Console::write(const uint8_t *buffer, size_t size)
{
    usb_out_chars_crlf((const char *)buffer, size);
    return size;
}
