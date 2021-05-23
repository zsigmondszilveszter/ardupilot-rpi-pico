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

#pragma once

#include "AP_HAL_RpiPico.h"
#include "UARTDriver.h"
#include "pico/mutex.h"
#include <queue>

// max allowed is 255 (uint8_t limits)
#define RPI_PICO_USB_TX_FIFO_SIZE 64
#define RPI_PICO_USB_RX_FIFO_SIZE 64

// do not change, unless you know what you are doing 
// bigger than 255, requires changing some uint8_t members
#define RPI_PICO_USB_MAX_ALLOWED_BUFFER_SIZE 255

class RpiPico::Console : public RpiPico::UARTDriver {
public:
    Console();
    void init();
    void tusb_task();

    void begin(uint32_t b) override;
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void end() override;

    void flush() override;
    void async_read() override;
    
protected:
    // software FIFO buffers
    uint8_t maxTxFIFO = RPI_PICO_USB_TX_FIFO_SIZE <= RPI_PICO_USB_MAX_ALLOWED_BUFFER_SIZE ? RPI_PICO_USB_TX_FIFO_SIZE : RPI_PICO_USB_MAX_ALLOWED_BUFFER_SIZE;
    uint8_t maxRxFIFO = RPI_PICO_USB_RX_FIFO_SIZE <= RPI_PICO_USB_MAX_ALLOWED_BUFFER_SIZE ? RPI_PICO_USB_RX_FIFO_SIZE : RPI_PICO_USB_MAX_ALLOWED_BUFFER_SIZE;
};
