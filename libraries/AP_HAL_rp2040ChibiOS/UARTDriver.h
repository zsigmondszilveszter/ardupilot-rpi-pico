#pragma once

#include "AP_HAL_rp2040ChibiOS.h"
#include "hal.h"
#include <AP_HAL/utility/RingBuffer.h>

#define RP2040_UART_MAX_ALLOWED_BUFFER_SIZE 1024

#define MAX_UART_TX_FIFO_SIZE (RP2040_UART_TX_FIFO_SIZE <= RP2040_UART_MAX_ALLOWED_BUFFER_SIZE ? RP2040_UART_TX_FIFO_SIZE : RP2040_UART_MAX_ALLOWED_BUFFER_SIZE)
#define MAX_UART_RX_FIFO_SIZE (RP2040_UART_RX_FIFO_SIZE <= RP2040_UART_MAX_ALLOWED_BUFFER_SIZE ? RP2040_UART_RX_FIFO_SIZE : RP2040_UART_MAX_ALLOWED_BUFFER_SIZE)

class Rp2040ChibiOS::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(int8_t serial_num);

    void writeThread(void);
    void readThread(void);

    /* rp2040 implementations of UARTDriver virtual methods */
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
    bool discard_input() override;

    /* rp2040 implementations of Print virtual methods */
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    virtual void clearTxFIFO();
    virtual void async_read();

    virtual int8_t driverSerialNr();

    // returns the number of elements in txFIFO we couldn't flush into hardware buffers
    virtual uint8_t _flush(void);

protected: 
    SIODriver * uart_driver_inst;
    int8_t _serial_num = -1;
    bool initialized_flag = false;
    bool _blocking_writes;
    uint tx_pin;
    uint rx_pin;
    // software FIFO buffers
    ByteBuffer rxFIFO{0};
    ByteBuffer txFIFO{0};
    HAL_Semaphore _uartMutex;
    HAL_Semaphore _txUartMutex;
    HAL_Semaphore _rxUartMutex;

    thread_t* _uart_write_thread_ctx;
    thread_t* _uart_read_thread_ctx;
    static void _uart_write_thread(void *arg);
    static void _uart_read_thread(void *arg);
};
