#pragma once

#include "AP_HAL_rp2040ChibiOS.h"
#include "hal.h"
#include <queue>

// do not change, unless you know what you are doing 
// bigger than 255, requires changing some uint8_t members
#define RP2040_UART_MAX_ALLOWED_BUFFER_SIZE 255

class Rp2040ChibiOS::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(int8_t serial_num);

    void startThreadOnCore1(void);

    /* rp2040 implementations of UARTDriver virtual methods */
    void begin(uint32_t b) override;
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void end() override;
    void flush() override;
    bool is_initialized() override { return initialized_flag; };
    void set_blocking_writes(bool blocking) override { blocking_writes = blocking; };
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

    int8_t driverSerialNr() {return _serial_num; }

    bool lockMutexIfInitialized(void);

    uint8_t _flush(void);

#if HAL_UART_STATS_ENABLED
    // request information on uart I/O for one uart
    void uart_info(ExpandingString &str) override;
#endif

protected: 
    SIODriver * uart_driver_inst;
    int8_t _serial_num = -1;
    bool initialized_flag = false;
    bool blocking_writes;
    uint tx_pin;
    uint rx_pin;
    // software FIFO buffers
    uint8_t maxTxFIFO = RP2040_UART_TX_FIFO_SIZE <= RP2040_UART_MAX_ALLOWED_BUFFER_SIZE ? RP2040_UART_TX_FIFO_SIZE : RP2040_UART_MAX_ALLOWED_BUFFER_SIZE;
    uint8_t maxRxFIFO = RP2040_UART_RX_FIFO_SIZE <= RP2040_UART_MAX_ALLOWED_BUFFER_SIZE ? RP2040_UART_RX_FIFO_SIZE : RP2040_UART_MAX_ALLOWED_BUFFER_SIZE;
    std::queue<uint8_t> txFIFO;
    std::queue<uint8_t> rxFIFO;
    mutex_t uartMutex;

    thread_t* _uart_thread_ctx;
    static void _uart_thread(void *arg);
};
