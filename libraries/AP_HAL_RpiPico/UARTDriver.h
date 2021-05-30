#pragma once

#include "AP_HAL_RpiPico.h"

// Raspbery Pi Pico SDK headers
#include "hardware/uart.h"
#include "pico/mutex.h"
#include <queue>

// do not change, unless you know what you are doing 
// bigger than 255, requires changing some uint8_t members
#define RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE 255

class RpiPico::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(uint8_t serial_num);

    /*  Rpi Pico implementations of UARTDriver virtual methods */
    void begin(uint32_t b) override;
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void end() override;
    void flush() override;
    bool is_initialized() override { return initialized_flag; };
    void set_blocking_writes(bool blocking) override;
    bool tx_pending() override;

    /*  Rpi Pico implementations of Stream virtual methods */
    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;
    bool discard_input() override;

    /*  Rpi Pico implementations of Print virtual methods */
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    virtual void clearTxFIFO();
    virtual void async_read();
protected:
    uart_inst_t * uart_inst;
    bool initialized_flag = false;
    bool blocking_writes;
    uint tx_pin;
    uint rx_pin;
    // software FIFO buffers
    uint8_t maxTxFIFO = RPI_PICO_UART_TX_FIFO_SIZE <= RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE ? RPI_PICO_UART_TX_FIFO_SIZE : RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE;
    uint8_t maxRxFIFO = RPI_PICO_UART_RX_FIFO_SIZE <= RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE ? RPI_PICO_UART_RX_FIFO_SIZE : RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE;
    std::queue<uint8_t> txFIFO;
    std::queue<uint8_t> rxFIFO;
    mutex_t * txFifoMutex;
    mutex_t * rxFifoMutex;
};
