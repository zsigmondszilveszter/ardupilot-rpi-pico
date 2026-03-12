#include "UARTDriver.h"
#include "Scheduler.h"
#include "rp2xxx_util.h"

#ifndef UART_WRITE_THD_WA_SIZE
#define UART_WRITE_THD_WA_SIZE 256
#endif

#ifndef UART_READ_THD_WA_SIZE
#define UART_READ_THD_WA_SIZE 256
#endif

extern const AP_HAL::HAL& hal;

Rp2xxxChibiOS::UARTDriver::UARTDriver(int8_t serial_num) {
    _serial_num = serial_num;
    switch (_serial_num) {
        case 0:
            uart_driver_inst = &SIOD0;
            palSetLineMode(RP2xxx_UART0_TX_GPIO_PIN, PAL_MODE_ALTERNATE_UART);
            palSetLineMode(RP2xxx_UART0_RX_GPIO_PIN, PAL_MODE_ALTERNATE_UART);
            break;
        case 1:
            uart_driver_inst = &SIOD1;
            palSetLineMode(RP2xxx_UART1_TX_GPIO_PIN, PAL_MODE_ALTERNATE_UART);
            palSetLineMode(RP2xxx_UART1_RX_GPIO_PIN, PAL_MODE_ALTERNATE_UART);
            break;
        default: break;
    }
}

void Rp2xxxChibiOS::UARTDriver::writeThread() {
    // setup the uart worker thread to flush the TX FIFO
    if (_uart_write_thread_ctx == nullptr) {
        _uart_write_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(UART_WRITE_THD_WA_SIZE),
                "UART_TX",
                APM_UART_PRIORITY,        /* Initial priority.    */
                _uart_write_thread,             /* Thread function.     */
                this,
                &ch1);
    }
}

void Rp2xxxChibiOS::UARTDriver::readThread() {
    // setup the uart worker thread to read the RX FIFO
    if (_uart_read_thread_ctx == nullptr) {
        _uart_read_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(UART_READ_THD_WA_SIZE),
                "UART_RX",
                APM_UART_PRIORITY,        /* Initial priority.    */
                _uart_read_thread,             /* Thread function.     */
                this,
                &ch1);
    }
}

void Rp2xxxChibiOS::UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    if (rxS == 0) {
        rxS = RP2xxx_UART_RX_FIFO_SIZE;
    }
    if (txS == 0) {
        txS = RP2xxx_UART_TX_FIFO_SIZE;
    }

    {
        WITH_SEMAPHORE(_uartMutex);
        if (rxS > RP2xxx_UART_MAX_ALLOWED_BUFFER_SIZE ) {
            rxS = RP2xxx_UART_MAX_ALLOWED_BUFFER_SIZE ;
        }
        if (txS > RP2xxx_UART_MAX_ALLOWED_BUFFER_SIZE ) {
            txS = RP2xxx_UART_MAX_ALLOWED_BUFFER_SIZE ;
        }
        if (rxS != rxFIFO.get_size()) {
            rxFIFO.set_size(rxS);
        }
        if (txS != txFIFO.get_size()) {
            txFIFO.set_size(txS);
        }
        _uart_config = {
            .baud         = b,
            .UARTLCR_H    = UART_UARTLCR_H_WLEN_8BITS | UART_UARTLCR_H_FEN,
            .UARTCR       = 0U,
            .UARTIFLS     = UART_UARTIFLS_RXIFLSEL_1_2F | UART_UARTIFLS_TXIFLSEL_1_2E,
            .UARTDMACR    = 0U
        };
    }

    if (initialized_flag) {
        // Already running: signal the write thread to restart sio with the new
        // config. sioStop/sioStart must run on core1 (where the IRQ was
        // registered), so we delegate to the write thread and wait.
        _restart_pending = true;
        while (_restart_pending) {
            hal.scheduler->delay_microseconds(100);
        }
        return;
    }

    // First-time init: spawn threads. The write thread calls sioStart on core1.
    // initialized_flag is set by the write thread after sioStart completes.
    writeThread();
    readThread();
}

bool Rp2xxxChibiOS::UARTDriver::is_initialized() {
    WITH_SEMAPHORE(_uartMutex);
    return initialized_flag;
}
int8_t Rp2xxxChibiOS::UARTDriver::driverSerialNr() {
    WITH_SEMAPHORE(_uartMutex);
    return _serial_num;
}

void Rp2xxxChibiOS::UARTDriver::_end() {
    WITH_SEMAPHORE(_uartMutex);
    WITH_SEMAPHORE(_txUartMutex);
    WITH_SEMAPHORE(_rxUartMutex);

    rxFIFO.set_size(0);
    txFIFO.set_size(0);
    sioStop(uart_driver_inst);
    initialized_flag = false;
}

void Rp2xxxChibiOS::UARTDriver::_flush(void) {
    if (!is_initialized()) return;
    
    WITH_SEMAPHORE(_txUartMutex);

    ByteBuffer::IoVec vec[2];
    const auto n_vec = txFIFO.peekiovec(vec, txFIFO.available());

    for (int i = 0; i < n_vec; i++) {
        size_t ret = sioAsyncWrite(uart_driver_inst, vec[i].data, vec[i].len);

        if (!ret) {
            break;
        }
        txFIFO.advance(ret);

        /* We wrote less than we asked for, stop (no place in the buffer) */
        if ((unsigned)ret != vec[i].len) {
            break;
        }
    }
}

void Rp2xxxChibiOS::UARTDriver::async_read() {
    if (!is_initialized()) return;

    WITH_SEMAPHORE(_rxUartMutex);

    // try to fill the read buffer
    ByteBuffer::IoVec vec[2];
    const auto n_vec = rxFIFO.reserve(vec, rxFIFO.space());

    for (uint32_t i = 0; i < n_vec; i++) {
        size_t ret = sioAsyncRead(uart_driver_inst, vec[i].data, vec[i].len);

        if (!ret) {
            break;
        }
        rxFIFO.commit((unsigned)ret);

        /* stop reading as we read less than we asked for */
        if ((unsigned)ret < vec[i].len) {
            break;
        }
    }
}

bool Rp2xxxChibiOS::UARTDriver::tx_pending() {
    if (!is_initialized()) return false;

    if (!_txUartMutex.take_nonblocking()) {
        // the thread FIFO is locked, assume pending
        return true;
    }
    bool pending = txFIFO.available() > 0;
    _txUartMutex.give();
    return pending;
}

uint32_t Rp2xxxChibiOS::UARTDriver::_available() {
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUartMutex);

    return rxFIFO.available();
}

uint32_t Rp2xxxChibiOS::UARTDriver::txspace() {
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_txUartMutex);

    return txFIFO.space();
}

ssize_t Rp2xxxChibiOS::UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUartMutex);

    return rxFIFO.read(buffer, count);
}

bool Rp2xxxChibiOS::UARTDriver::_discard_input() {
    if (!is_initialized()) {
        return true;
    }

    WITH_SEMAPHORE(_rxUartMutex);

    rxFIFO.clear();
    return true;
}

void Rp2xxxChibiOS::UARTDriver::clearTxFIFO() {
    if (!is_initialized()) {
        return;
    }

    WITH_SEMAPHORE(_txUartMutex);
    // clear the software fifo
    txFIFO.clear();
}

size_t Rp2xxxChibiOS::UARTDriver::_write(const uint8_t *buffer, size_t size) {
    if (!is_initialized()) return 0;

    size_t written;
    {
        WITH_SEMAPHORE(_txUartMutex);
        written = txFIFO.write(buffer, size);
    }
    if (written > 0) {
        _txWakeSem.signal();
    }
    return written;
}

void Rp2xxxChibiOS::UARTDriver::_sio_rx_callback(SIODriver *siop)
{
    UARTDriver *uart = (UARTDriver *)siop->arg;
    sioevents_t events = sioGetAndClearEventsX(siop);
    if (events & SIO_EV_RXNOTEMPY) {
        uart->_rxWakeSem.signal_ISR();
    }
}

void Rp2xxxChibiOS::UARTDriver::_uart_write_thread(void *arg)
{
    UARTDriver * uart = (UARTDriver *)arg;
    // sioStart here so the UART IRQ is registered on core1's NVIC,
    // keeping all SIO driver access on the same core.
    sioStart(uart->uart_driver_inst, &uart->_uart_config);
    uart->uart_driver_inst->arg = uart;
    sioSetCallbackX(uart->uart_driver_inst, _sio_rx_callback);
    sioSetEnableFlags(uart->uart_driver_inst, SIO_EV_RXNOTEMPY);
    uart->initialized_flag = true;

    // add the number of uart interface to the name of thread
    char * thread_name;
    asprintf(&thread_name, "uart_write_thread_%d", uart->driverSerialNr());
    chRegSetThreadName(thread_name);

    while (true) {
        if (uart->_restart_pending) {
            sioStop(uart->uart_driver_inst);
            sioStart(uart->uart_driver_inst, &uart->_uart_config);
            uart->uart_driver_inst->arg = uart;
            sioSetCallbackX(uart->uart_driver_inst, _sio_rx_callback);
            sioSetEnableFlags(uart->uart_driver_inst, SIO_EV_RXNOTEMPY);
            uart->_restart_pending = false;
        }
        uart->_txWakeSem.wait(5000);
        uart->flush();
    }
}

void Rp2xxxChibiOS::UARTDriver::_uart_read_thread(void *arg)
{
    UARTDriver * uart = (UARTDriver *)arg;
    // add the number of uart interface to the name of thread
    char * thread_name;
    asprintf(&thread_name, "uart_read_thread_%d", uart->driverSerialNr());
    chRegSetThreadName(thread_name);
    while (!uart->is_initialized()) {
        hal.scheduler->delay_microseconds(10);
    }

    while (true) {
        uart->_rxWakeSem.wait(5000);
        uart->async_read();
    }
}
