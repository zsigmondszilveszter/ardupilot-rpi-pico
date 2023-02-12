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
#include "hal.h"

#define RP2040_USB_CDC_MAX_ALLOWED_BUFFER_SIZE 1024

#define MAX_USB_CDC_TX_FIFO_SIZE (RP2040_UART_TX_FIFO_SIZE <= RP2040_USB_CDC_MAX_ALLOWED_BUFFER_SIZE ? RP2040_UART_TX_FIFO_SIZE : RP2040_USB_CDC_MAX_ALLOWED_BUFFER_SIZE)
#define MAX_USB_CDC_RX_FIFO_SIZE (RP2040_UART_RX_FIFO_SIZE <= RP2040_USB_CDC_MAX_ALLOWED_BUFFER_SIZE ? RP2040_UART_RX_FIFO_SIZE : RP2040_USB_CDC_MAX_ALLOWED_BUFFER_SIZE)

void usb_initialise(void);

class Rp2040ChibiOS::UsbCdcConsole : public AP_HAL::UARTDriver {
public:
    UsbCdcConsole();

    void writeThread(void);
    void readThread(void);

    /* rp2040 implementations of USB CDC Serial Console virtual methods */
    void begin(uint32_t b) override;
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void end() override;
    void flush() override;
    bool is_initialized() override;
    void set_blocking_writes(bool blocking) override;
    virtual bool is_blocking_writes();
    bool tx_pending() override;

    /* rp2040 implementations of Stream virtual methods */
    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;
    ssize_t read(uint8_t *buffer, uint16_t count) override;
    bool discard_input() override;

    /* rp2040 implementations of Print virtual methods */
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    virtual void clearTxFIFO();
    virtual void async_read();

protected:
    bool initialized_flag = false;
    bool _blocking_writes;

    // software FIFO buffers
    ByteBuffer rxFIFO{0};
    ByteBuffer txFIFO{0};
    HAL_Semaphore _usbMutex;
    HAL_Semaphore _txUsbMutex;
    HAL_Semaphore _rxUsbMutex;

    thread_t* _usb_cdc_write_thread_ctx;
    thread_t* _usb_cdc_read_thread_ctx;
    static void _usb_cdc_write_thread(void *arg);
    static void _usb_cdc_read_thread(void *arg);
};
