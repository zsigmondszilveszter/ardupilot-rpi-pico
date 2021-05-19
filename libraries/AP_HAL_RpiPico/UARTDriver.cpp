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

#include "UARTDriver.h"

// Raspbery Pi Pico SDK headers
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define rpi_uart(x) uart##x

RpiPico::UARTDriver::UARTDriver(uint8_t serial_num) {
    switch (serial_num) {
        case 0: 
            uart_inst = rpi_uart(0); 
            tx_pin = UART0_TX_GPIO_PIN;
            rx_pin = UART0_RX_GPIO_PIN;
            break;
        case 1: 
            uart_inst = rpi_uart(1);
            tx_pin = UART1_TX_GPIO_PIN;
            rx_pin = UART1_RX_GPIO_PIN;
            break;
        default: /* TODO handle exception */ return;
    }
}

void RpiPico::UARTDriver::begin(uint32_t b) {
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_init(uart_inst, b);
#if PICO_UART_ENABLE_CRLF_SUPPORT
    uart_set_translate_crlf(uart_inst, PICO_UART_DEFAULT_CRLF);
#endif
    initialized_flag = true;
}
void RpiPico::UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    // pico doesn't support initializing UARTs with buffer size
    // the PL011 has separate 32x8 Tx and 32x8 Rx buffer
    begin(b);
}
void RpiPico::UARTDriver::end() {
    uart_deinit(uart_inst);
    initialized_flag = false;
}
void RpiPico::UARTDriver::flush() {
    uart_tx_wait_blocking(uart_inst);
}
bool RpiPico::UARTDriver::is_initialized() { 
    return initialized_flag;
}
void RpiPico::UARTDriver::set_blocking_writes(bool blocking) {
    blocking_writes = blocking;
}
bool RpiPico::UARTDriver::tx_pending() { 
    return false; 
}

/* Rpi Pico implementations of Stream virtual methods */
uint32_t RpiPico::UARTDriver::available() { 
    return uart_is_readable(uart_inst);
}
uint32_t RpiPico::UARTDriver::txspace() {
    if (!uart_is_writable(uart_inst)) {
        return 0;
    }
    return 32; 
}
int16_t RpiPico::UARTDriver::read() {
    if (available()) {
        return (int16_t) uart_getc(uart_inst);
    }
    return -1;
}
bool RpiPico::UARTDriver::discard_input() {
    // probably a bad idea looping through the Rx buffer, 
    // but no better option available right now
    while (available()) {
        read();
    }
    return true; 
}

/* Rpi Pico implementations of Print virtual methods */
size_t RpiPico::UARTDriver::write(uint8_t c) { 
    uart_putc(uart_inst, c);
    return (size_t) 1; 
}

size_t RpiPico::UARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}
