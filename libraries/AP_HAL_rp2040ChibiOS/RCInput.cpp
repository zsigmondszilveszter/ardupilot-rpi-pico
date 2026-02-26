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

#include "hwdef/common/pio.h"
#include "hwdef/common/rc_rx_sbus_uart.pio.h"
#include "hwdef/common/rc_rx_ibus_uart.pio.h"

#ifndef HAL_NO_UARTDRIVER
#include <GCS_MAVLink/GCS.h>
#endif

#define SIG_DETECT_TIMEOUT_US 500000
using namespace Rp2040ChibiOS;
extern const AP_HAL::HAL& hal;

// constructor
RCInput::RCInput(RCProtocol protocol) : AP_HAL::RCInput(),
    _protocol(protocol)
{}

void RCInput::init()
{
    hal_lld_peripheral_reset(RESETS_ALLREG_PIO0);
    hal_lld_peripheral_unreset(RESETS_ALLREG_PIO0);

    if (_protocol == RCProtocol::IBUS) {
        open_ibus();
    } else if (_protocol == RCProtocol::SBUS) {
        open_sbus();
    }
    
    AP::RC().init();
    _init = true;
}

/*
  open an SBUS UART
 */
void RCInput::open_sbus()
{
    WITH_SEMAPHORE(rcin_mutex);
    uint32_t offset = pio_add_program(_pio, &rc_rx_sbus_uart_pio_program);
    rc_rx_uart_pio_program_init(_pio, _sm, offset, RP2040_RC_RX_PIN, SBUS_BAUD, SBUS);
}

/*
  open an IBUS UART
 */
void RCInput::open_ibus()
{
    WITH_SEMAPHORE(rcin_mutex);
    uint32_t offset = pio_add_program(_pio, &rc_rx_ibus_uart_pio_program);
    rc_rx_uart_pio_program_init(_pio, _sm, offset, RP2040_RC_RX_PIN, IBUS_BAUD, IBUS);
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

    if (_protocol != RCProtocol::NONE) {
        const uint32_t baud = (_protocol == RCProtocol::SBUS) ? SBUS_BAUD : IBUS_BAUD;
        WITH_SEMAPHORE(rcin_mutex);
        uint32_t nr_of_bytes = pio_sm_get_rx_fifo_level(_pio, _sm);
        if (nr_of_bytes > 0) {
            for (uint8_t i=0; i<nr_of_bytes; i++) {
                uint8_t b = rc_rx_uart_pio_program_getc(_pio, _sm);
                AP::RC().process_byte(b, baud);
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
