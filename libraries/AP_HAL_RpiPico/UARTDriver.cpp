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

static mutex_t rpiPico_uartTxFifoMutex[4];
static mutex_t rpiPico_uartRxFifoMutex[4];

RpiPico::UARTDriver::UARTDriver() {}
RpiPico::UARTDriver::UARTDriver(uint8_t serial_num) {
    switch (serial_num) {
        case 0: 
            uart_inst = rpi_uart(0); 
            tx_pin = RPI_PICO_UART0_TX_GPIO_PIN;
            rx_pin = RPI_PICO_UART0_RX_GPIO_PIN;
            break;
        case 1: 
            uart_inst = rpi_uart(1);
            tx_pin = RPI_PICO_UART1_TX_GPIO_PIN;
            rx_pin = RPI_PICO_UART1_RX_GPIO_PIN;
            break;
        default: /* TODO handle exception */ return;
    }
}

void RpiPico::UARTDriver::begin(uint32_t b) {
    if (initialized_flag) {
        // it is already initialized, reset it
        end();
    }
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_init(uart_inst, b);

    txFifoMutex = &rpiPico_uartTxFifoMutex[uart_get_index(uart_inst)];
    rxFifoMutex = &rpiPico_uartRxFifoMutex[uart_get_index(uart_inst)];
    mutex_init(txFifoMutex);
    mutex_init(rxFifoMutex);
    initialized_flag = true;
}

/**
 * Note that there is a maximum allowed rx and tx buffer size
 */
void RpiPico::UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    // max allowed fifo size is RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE
    maxTxFIFO = txS <= RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE ? txS : RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE;
    maxRxFIFO = rxS <= RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE ? rxS : RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE;
    begin(b);
}
void RpiPico::UARTDriver::end() {
    discard_input();
    clearTxFIFO();
    uart_deinit(uart_inst);
    initialized_flag = false;
}

void RpiPico::UARTDriver::flush() {
    if (!is_initialized()) return;
    if (!mutex_try_enter(txFifoMutex, NULL)){
        // the transfer FIFO is locked.
        // the flush function is called periodically by the background thread
        // therefore we leave now and try in the next round
        return;
    }
    while (!txFIFO.empty()){
        if (uart_is_writable(uart_inst)) {
            uint8_t c = txFIFO.front();
            uart_putc_raw(uart_inst, c);
            txFIFO.pop();
        } else {
            // don't force, try later
            break;
        }
    }
    mutex_exit(txFifoMutex);
}

void RpiPico::UARTDriver::async_read() {
    if (!is_initialized()) return;
    if (!mutex_try_enter(rxFifoMutex, NULL)){
        // the receive FIFO is locked.
        // the async_read function is called periodically by the background thread
        // therefore we leave now and try in the next round
        return;
    }
    // we read max the hardcoded 32 bytes (the size of PL011's RX buffer) at once
    uint8_t steps = 0;
    while(uart_is_readable(uart_inst) && steps < 32) {
        uint8_t c = uart_getc(uart_inst);
        if (rxFIFO.size() == maxRxFIFO){
            // we start to delete the oldest characters from buffer, 
            // because we ran out of space
            rxFIFO.pop();
        }
        rxFIFO.push(c);
        steps++;
    }
    mutex_exit(rxFifoMutex);
}

void RpiPico::UARTDriver::set_blocking_writes(bool blocking) {
    // TODO, think and research about this
    blocking_writes = blocking;
}
bool RpiPico::UARTDriver::tx_pending() {
    if (!is_initialized()) return false;

    if (!mutex_try_enter(txFifoMutex, NULL)){
        // the transfer FIFO is locked.
        return true;
    }
    bool ret = !txFIFO.empty();
    mutex_exit(txFifoMutex);
    return ret; 
}

uint32_t RpiPico::UARTDriver::available() {
    if (!is_initialized()) return 0;

    // we must wait until we can obtain the rxFIFO
    mutex_enter_blocking(rxFifoMutex);
    uint32_t ret_val = rxFIFO.size();
    mutex_exit(rxFifoMutex);
    return ret_val;
}

uint32_t RpiPico::UARTDriver::txspace() {
    if (!is_initialized()) return 0;

    uint32_t space = 0;
    // the transfer FIFO is locked, we wait max 10 ms to take the resource
    if (!mutex_enter_timeout_ms(txFifoMutex, 10)) {
        // and return 0, we couldn't access the FIFO
        return space;
    }
    space = maxTxFIFO - txFIFO.size();
    mutex_exit(txFifoMutex);
    return space;
}

int16_t RpiPico::UARTDriver::read() {
    if (!is_initialized()) return 0;

    // we must wait until we can obtain the rxFIFO
    mutex_enter_blocking(rxFifoMutex);
    int16_t ret = -1;
    if (!rxFIFO.empty()) {
        ret = (int16_t) rxFIFO.front();
        rxFIFO.pop();
    }
    mutex_exit(rxFifoMutex);
    return ret;
}

bool RpiPico::UARTDriver::discard_input() {
    if (!is_initialized()) return false;

    // read the hardware fifos into software fifos
    async_read();
    // and then clear the software fifo
    mutex_enter_blocking(rxFifoMutex);
    std::queue<uint8_t>().swap(rxFIFO);
    mutex_exit(rxFifoMutex);

    return true; 
}

void RpiPico::UARTDriver::clearTxFIFO() {
    if (!is_initialized()) return;

    // clear the software fifo
    mutex_enter_blocking(txFifoMutex);
    std::queue<uint8_t>().swap(txFIFO);
    mutex_exit(txFifoMutex);
}

size_t RpiPico::UARTDriver::write(uint8_t c) {
    if (!is_initialized()) return 0;

    // we must write this to the txFIFO
    mutex_enter_blocking(txFifoMutex);

    if (txFIFO.size() == maxTxFIFO){
        // start to remove old data from buffer if there is no more space left
        txFIFO.pop();
    }
    txFIFO.push(c);
    mutex_exit(txFifoMutex);
    return 1;
}

size_t RpiPico::UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!is_initialized()) return 0;

    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}
