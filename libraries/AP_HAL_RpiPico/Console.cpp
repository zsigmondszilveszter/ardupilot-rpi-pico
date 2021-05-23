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

static mutex_t usb_console_mutex;
static mutex_t rpiPico_usbTxFifoMutex;
static mutex_t rpiPico_usbRxFifoMutex;


RpiPico::Console::Console() 
{}

void RpiPico::Console::init() 
{
    if (is_initialized()) 
        return;
    // initialize tinyusb stack
    tusb_init(); 
    mutex_init(&usb_console_mutex);

    // the pointers to those mutexes are a bit useless, but I wanted be consistent across the USB and UART devices
    txFifoMutex = &rpiPico_usbTxFifoMutex;
    rxFifoMutex = &rpiPico_usbRxFifoMutex;
    mutex_init(txFifoMutex);
    mutex_init(rxFifoMutex);

    initialized_flag = true;
}

void RpiPico::Console::begin(uint32_t b) {
    //
}
void RpiPico::Console::begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    // max allowed fifo size is RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE
    maxTxFIFO = txS <= RPI_PICO_USB_MAX_ALLOWED_BUFFER_SIZE ? txS : RPI_PICO_USB_MAX_ALLOWED_BUFFER_SIZE;
    maxRxFIFO = rxS <= RPI_PICO_USB_MAX_ALLOWED_BUFFER_SIZE ? rxS : RPI_PICO_USB_MAX_ALLOWED_BUFFER_SIZE;
    begin(b);
}
void RpiPico::Console::end() {
    // cannot deinit
}

void RpiPico::Console::tusb_task() {
    if (!is_initialized()) return;

    if (mutex_try_enter(&usb_console_mutex, NULL)) {
        tud_task();
        tud_cdc_write_flush();
        mutex_exit(&usb_console_mutex);
    }
}

void RpiPico::Console::flush() {
    if (!is_initialized()) return;
    if (!tud_cdc_connected()) return;

    if (!mutex_try_enter(txFifoMutex, NULL)){
        // the transfer FIFO is locked.
        // the flush function is called periodically by the background thread
        // therefore we leave now and try in the next round
        return;
    }
    if (!mutex_try_enter(&usb_console_mutex, NULL)) {
        mutex_exit(txFifoMutex);
        return;
    }
    while (!txFIFO.empty() && tud_cdc_write_available()) {
        uint8_t c = txFIFO.front();
        tud_cdc_write_char(c);
        txFIFO.pop();
    }
    mutex_exit(txFifoMutex);
    mutex_exit(&usb_console_mutex);
}

void RpiPico::Console::async_read() {
    if (!is_initialized()) return;
    if (!tud_cdc_connected()) return;

    if (!mutex_try_enter(rxFifoMutex, NULL)){
        // the receive FIFO is locked.
        // the async_read function is called periodically by the background thread
        // therefore we leave now and try in the next round
        return;
    }
    if (!mutex_try_enter(&usb_console_mutex, NULL)) {
        mutex_exit(rxFifoMutex);
        return;
    }
    // we read max 64 bytes at once
    uint8_t steps = 0;
    while(tud_cdc_available() && steps < 64) {
        uint8_t c = tud_cdc_read_char();
        if (rxFIFO.size() == maxRxFIFO){
            // we start to delete the oldest characters from buffer, 
            // because we ran out of space
            rxFIFO.pop();
        }
        rxFIFO.push(c);
        steps++;
    }
    mutex_exit(rxFifoMutex);
    mutex_exit(&usb_console_mutex);
}