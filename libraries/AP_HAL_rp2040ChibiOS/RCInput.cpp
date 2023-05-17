/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include <hal.h>
#include "RCInput.h"
#include "hal.h"
// #include "hwdef/common/ppm.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_RP2040CHIBIOS

#include <AP_Math/AP_Math.h>

#ifndef HAL_NO_UARTDRIVER
#include <GCS_MAVLink/GCS.h>
#endif

#define SIG_DETECT_TIMEOUT_US 500000
using namespace Rp2040ChibiOS;
extern const AP_HAL::HAL& hal;

// constructor
RCInput::RCInput(UARTDriver *_dev_inverted, UARTDriver *_dev_115200) : AP_HAL::RCInput(), 
    dev_inverted(_dev_inverted),
    dev_115200(_dev_115200)
{}

void RCInput::init()
{
    if (dev_inverted) {
        open_sbus(dev_inverted);
        fd_inverted = true;
    } else {
        fd_inverted = false;
    }
    inverted_is_115200 = false;
    if (dev_115200) {
        open_115200(dev_115200);
        fd_115200 = true;
    } else {
        fd_115200 = false;
    }
    AP::RC().init();
    _init = true;
}

/*
  open a SBUS UART
 */
void RCInput::open_sbus(UARTDriver *uart)
{
    (void)uart;
}

/*
  open a 115200 UART
 */
void RCInput::open_115200(UARTDriver *uart)
{
    uart->begin(115200, 128, 0);
}

/*
  enable or disable pulse input for RC input. This is used to reduce
  load when we are decoding R/C via a UART
*/
void RCInput::pulse_input_enable(bool enable)
{
    pulse_input_enabled = enable;
}

bool RCInput::new_input()
{
    if (!_init) {
        return false;
    }
    bool valid;
    {
        WITH_SEMAPHORE(rcin_mutex);
        valid = _rcin_timestamp_last_signal != _last_read;
        _last_read = _rcin_timestamp_last_signal;
    }
    return valid;
}

uint8_t RCInput::num_channels()
{
    if (!_init) {
        return 0;
    }
    return _num_channels;
}

uint16_t RCInput::read(uint8_t channel)
{
    if (!_init || (channel >= MIN(RC_INPUT_MAX_CHANNELS, _num_channels))) {
        return 0;
    }
    uint16_t v;
    {
        WITH_SEMAPHORE(rcin_mutex);
        v = _rc_values[channel];
    }
    return v;
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    if (!_init) {
        return false;
    }

    if (len > RC_INPUT_MAX_CHANNELS) {
        len = RC_INPUT_MAX_CHANNELS;
    }
    {
        WITH_SEMAPHORE(rcin_mutex);
        memcpy(periods, _rc_values, len*sizeof(periods[0]));
    }
    return len;
}

void RCInput::_timer_tick(void)
{
    if (!_init) {
        return;
    }
uint8_t b[80];

    if (fd_inverted) {
        ssize_t n = dev_inverted->read(&b[0], sizeof(b));
        if (n > 0) {
            for (uint8_t i=0; i<n; i++) {
                AP::RC().process_byte(b[i], inverted_is_115200 ? 115200 : 100000);
            }
        }
    }
    if (fd_115200) {
        ssize_t n = dev_115200->read(&b[0], sizeof(b));
        if (n > 0 && !inverted_is_115200) {
            for (uint8_t i=0; i<n; i++) {
                AP::RC().process_byte(b[i], 115200);
            }
        }
    }

    if (AP::RC().new_input()) {
        last_frame_ms = AP_HAL::millis();
        uint8_t n = AP::RC().num_channels();
        for (uint8_t i=0; i<n; i++) {
            _rc_values[i] = AP::RC().read(i);
        }
        _num_channels = n;
    }

    uint32_t now = AP_HAL::millis();
    if (fd_inverted && now - last_frame_ms > 2000) {
        // no inverted data for 2s, flip baudrate
        inverted_is_115200 = !inverted_is_115200;
        if (inverted_is_115200) {
            open_115200(dev_inverted);
            fd_inverted = true;
        } else {
            open_sbus(dev_inverted);
            fd_inverted = true;
        }
        last_frame_ms = now;
    }
}

/*
  start a bind operation, if supported
 */
bool RCInput::rc_bind(int dsmMode)
{
#ifndef HAL_BUILD_AP_PERIPH
    // ask AP_RCProtocol to start a bind
    AP::RC().start_bind();
#endif
    return true;
}
#endif //#if CONFIG_HAL_BOARD == HAL_BOARD_RP2040CHIBIOS
