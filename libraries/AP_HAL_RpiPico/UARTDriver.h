#pragma once

#include "AP_HAL_RpiPico.h"

// Raspbery Pi Pico SDK headers
#include "hardware/uart.h"

class RpiPico::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(uint8_t serial_num);

    /*  Rpi Pico implementations of UARTDriver virtual methods */
    void begin(uint32_t b) override;
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void end() override;
    void flush() override;
    bool is_initialized() override;
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
private:
    uart_inst_t * uart_inst;
    bool initialized_flag;
    bool blocking_writes;
    uint tx_pin;
    uint rx_pin;
};
