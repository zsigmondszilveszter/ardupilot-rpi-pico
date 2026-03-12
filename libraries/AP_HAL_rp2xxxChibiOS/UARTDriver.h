#pragma once

#include "AP_HAL_rp2xxxChibiOS.h"
#include "hal.h"
#include <AP_HAL/utility/RingBuffer.h>
#include "Semaphores.h"

#define RP2xxx_UART_MAX_ALLOWED_BUFFER_SIZE 1024

class Rp2xxxChibiOS::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(int8_t serial_num);

    void writeThread(void);
    void readThread(void);

    /* rp2040 implementations of UARTDriver virtual methods */
    bool is_initialized() override;
    bool tx_pending() override;

    /* rp2040 implementations of BetterStream virtual methods */
    uint32_t txspace() override;

    virtual void clearTxFIFO();
    virtual void async_read();

    virtual int8_t driverSerialNr();

protected:
    /* backend implementations called by AP_HAL::UARTDriver base class */
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    bool _discard_input() override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;

    SIODriver * uart_driver_inst;
    SIOConfig _uart_config;
    int8_t _serial_num = -1;
    bool initialized_flag = false;
    volatile bool _restart_pending = false;
    uint tx_pin;
    uint rx_pin;
    // software FIFO buffers
    ByteBuffer rxFIFO{0};
    ByteBuffer txFIFO{0};
    HAL_Semaphore _uartMutex;
    HAL_Semaphore _txUartMutex;
    HAL_Semaphore _rxUartMutex;
    Rp2xxxChibiOS::BinarySemaphore _txWakeSem{false};
    Rp2xxxChibiOS::BinarySemaphore _rxWakeSem{false};

    thread_t* _uart_write_thread_ctx;
    thread_t* _uart_read_thread_ctx;
    static void _uart_write_thread(void *arg);
    static void _uart_read_thread(void *arg);
    static void _sio_rx_callback(SIODriver *siop);
};
