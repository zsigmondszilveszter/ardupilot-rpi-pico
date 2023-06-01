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
 * and Szilveszter Zsigmond
 */
#pragma once

#include "AP_HAL_rp2040ChibiOS.h"
#include "Semaphores.h"
#include "hwdef/common/pio.h"

#include <AP_RCProtocol/AP_RCProtocol.h>


#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18
#endif

#define SBUS_BAUD 100000
#define IBUS_BAUD 115200

namespace Rp2040ChibiOS {

class RCInput : public AP_HAL::RCInput {
public:
    RCInput(bool sbus, bool ibus);
    void init() override;
    bool new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;

    /* enable or disable pulse input for RC input. This is used to
       reduce load when we are decoding R/C via a UART */
    void pulse_input_enable(bool enable) override;

    int16_t get_rssi(void) override {
        return _rssi;
    }
    int16_t get_rx_link_quality(void) override {
        return _rx_link_quality;
    }
    const char *protocol() const override { return last_protocol; }

    virtual void _timer_tick(void);
    bool rc_bind(int dsmMode) override;

protected:
    void open_sbus();
    void open_ibus();

    uint16_t _rc_values[RC_INPUT_MAX_CHANNELS] = {0};

    uint64_t _last_read;
    uint8_t _num_channels;
    Semaphore rcin_mutex;
    int16_t _rssi = -1;
    int16_t _rx_link_quality = -1;
    uint32_t _rcin_timestamp_last_signal;

    PIO ibus_pio = pio0;
    uint ibus_sm = 0;
    PIO sbus_pio = pio0;
    uint sbus_sm = 1;

    bool enable_sbus = false;
    bool enable_ibus = false;
    uint32_t last_frame_ms;

    bool _init;
    const char *last_protocol;

    enum class RCSource {
        NONE = 0,
        IOMCU = 1,
        RCPROT_PULSES = 2,
        RCPROT_BYTES = 3,
        APRADIO = 4,
    } last_source;

    bool pulse_input_enabled;
};

}