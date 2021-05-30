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
#include "BgThread.h"

// Raspbery Pi Pico SDK headers
#include "tusb.h"
#include "pico/time.h"
#include "pico/mutex.h"

RpiPico::BgThread& bgthread_pointer_console = RpiPico::getBgThread();
static mutex_t usb_console_mutex;


RpiPico::Console::Console() 
{}

void RpiPico::Console::init() 
{
    if (is_initialized()) 
        return;
    // initialize tinyusb stack
    tusb_init(); 
    mutex_init(&usb_console_mutex);

    // register 1khz background tasks
    bgthread_pointer_console.add_periodic_background_task_us(1000, FUNCTOR_BIND_MEMBER(&Console::tusb_task, void), PR1);
    bgthread_pointer_console.add_periodic_background_task_us(1000, FUNCTOR_BIND_MEMBER(&Console::flush, void), PR1);

    initialized_flag = true;
}

void RpiPico::Console::begin(uint32_t b) {
    //
}
void RpiPico::Console::begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    //
}
void RpiPico::Console::end() {
    // cannot deinit
}

void RpiPico::Console::tusb_task() {
    if (!is_initialized()) return;

    // we don't force taking the USB source
    // if the mutex is already owned, then we are in ardupilot code
    // in this file which will do a tud_task itself, so we'll just do nothing
    // until the next tick; we won't starve
    if (mutex_try_enter(&usb_console_mutex, NULL)) {
        tud_task();
        mutex_exit(&usb_console_mutex);
    }
}

void RpiPico::Console::flush() {
    if (!is_initialized()) return;

    // we don't force taking the USB source
    // if the mutex is already owned, then we are in ardupilot code
    // in this file which will do a tud_task itself, so we'll just do nothing
    // until the next tick; we won't starve
    if (mutex_try_enter(&usb_console_mutex, NULL)) {
        if (tud_cdc_connected()) {
            tud_cdc_write_flush();
        }
        mutex_exit(&usb_console_mutex);
    }
}


void RpiPico::Console::set_blocking_writes(bool blocking) {
    // not supported
}
bool RpiPico::Console::tx_pending() {
    return false;
}

uint32_t RpiPico::Console::available() {
    if (!is_initialized()) return 0;

    mutex_enter_blocking(&usb_console_mutex);
    uint32_t ret_val = tud_cdc_available();
    mutex_exit(&usb_console_mutex);
    return ret_val;
}

uint32_t RpiPico::Console::txspace() {
    if (!is_initialized()) return 0;

    uint32_t space = 0;
    // the usb is locked, we wait max 10 ms to take the resource
    if (!mutex_enter_timeout_ms(&usb_console_mutex, 10)) {
        // and return 0, we couldn't access the USB resource
        return space;
    }
    space = tud_cdc_write_available();
    mutex_exit(&usb_console_mutex);
    return space;
}

int16_t RpiPico::Console::read() {
    if (!is_initialized()) return 0;

    // we must wait until we can obtain the USB resource
    mutex_enter_blocking(&usb_console_mutex);
    int16_t ret = (int16_t) tud_cdc_read_char();
    mutex_exit(&usb_console_mutex);
    return ret;
}

ssize_t RpiPico::Console::read(uint8_t *buffer, uint16_t count) {
    if (!is_initialized()) return 0;

    // we must wait until we can obtain the USB resource
    mutex_enter_blocking(&usb_console_mutex);
    ssize_t ret = (ssize_t) tud_cdc_read(buffer, count);
    mutex_exit(&usb_console_mutex);
    return ret;
}

bool RpiPico::Console::discard_input() {
    if (!is_initialized()) return false;

    // we must wait until we can obtain the USB resource
    mutex_enter_blocking(&usb_console_mutex);
    tud_cdc_read_flush();
    mutex_exit(&usb_console_mutex);
    return true; 
}

size_t RpiPico::Console::write(uint8_t c) {
    static uint64_t last_avail_time;
    if (!is_initialized()) return 0;

    // we must wait until we can obtain the USB resource
    mutex_enter_blocking(&usb_console_mutex);

    size_t ret = 0;
    if (tud_cdc_connected()) {
        if (tud_cdc_write_available()) {
            ret = (size_t)tud_cdc_write_char(c);
            last_avail_time = time_us_64();
        } else {
            tud_task();
            tud_cdc_write_flush();
        }
    } else {
        // reset our timeout
        last_avail_time = 0;
    }
    (void)last_avail_time;
    mutex_exit(&usb_console_mutex);
    return ret;
}

size_t RpiPico::Console::write(const uint8_t *buffer, size_t size){
    static uint64_t last_avail_time;
    if (!is_initialized()) return 0;

    // we must wait until we can obtain the USB resource
    mutex_enter_blocking(&usb_console_mutex);

    int i = 0, len = (int) size;
    if (tud_cdc_connected()) {
        for (i = 0; i < len;) {
            int n = len - i;
            int avail = tud_cdc_write_available();
            if (n > avail) n = avail;
            if (n) {
                int n2 = tud_cdc_write(buffer + i, n);
                tud_task();
                tud_cdc_write_flush();
                i += n2;
                last_avail_time = time_us_64();
            } else {
                tud_task();
                tud_cdc_write_flush();
                if (!tud_cdc_connected() ||
                    (!tud_cdc_write_available() && time_us_64() > last_avail_time + PICO_USB_OUT_TIMEOUT_US)) {
                    break;
                }
            }
        }
    } else {
        // reset our timeout
        last_avail_time = 0;
    }
    mutex_exit(&usb_console_mutex);
    return i;
}