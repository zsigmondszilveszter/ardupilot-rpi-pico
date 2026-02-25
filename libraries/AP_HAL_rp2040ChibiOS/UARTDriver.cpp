#include "UARTDriver.h"
#include "Scheduler.h"
#include "rp2xxx_util.h"

#ifndef UART_WRITE_THD_WA_SIZE
#define UART_WRITE_THD_WA_SIZE RP2040_UART_TX_FIFO_SIZE + 32
#endif

#ifndef UART_READ_THD_WA_SIZE
#define UART_READ_THD_WA_SIZE RP2040_UART_RX_FIFO_SIZE + 32
#endif

extern const AP_HAL::HAL& hal;

Rp2040ChibiOS::UARTDriver::UARTDriver(int8_t serial_num) {
    _serial_num = serial_num;
    switch (_serial_num) {
        case 0:
            uart_driver_inst = &SIOD0;
            palSetLineMode(RP2040_UART0_TX_GPIO_PIN, PAL_MODE_ALTERNATE_UART);
            palSetLineMode(RP2040_UART0_RX_GPIO_PIN, PAL_MODE_ALTERNATE_UART);
            break;
        case 1:
            uart_driver_inst = &SIOD1;
            palSetLineMode(RP2040_UART1_TX_GPIO_PIN, PAL_MODE_ALTERNATE_UART);
            palSetLineMode(RP2040_UART1_RX_GPIO_PIN, PAL_MODE_ALTERNATE_UART);
            break;
        default: break;
    }
}

void Rp2040ChibiOS::UARTDriver::writeThread() {
    // setup the uart worker thread to flush the TX FIFO
    if (_uart_write_thread_ctx == nullptr) {
        _uart_write_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(UART_WRITE_THD_WA_SIZE),
                "UART_TX",
                UART_THREAD_PRIORITY,        /* Initial priority.    */
                _uart_write_thread,             /* Thread function.     */
                this,
                &ch1);
    }
}

void Rp2040ChibiOS::UARTDriver::readThread() {
    // setup the uart worker thread to read the RX FIFO
    if (_uart_read_thread_ctx == nullptr) {
        _uart_read_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(UART_READ_THD_WA_SIZE),
                "UART_RX",
                UART_THREAD_PRIORITY,        /* Initial priority.    */
                _uart_read_thread,             /* Thread function.     */
                this,
                &ch1);
    }
}

void Rp2040ChibiOS::UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    if (rxS == 0) {
        rxS = RP2040_UART_RX_FIFO_SIZE;
    }
    if (txS == 0) {
        txS = RP2040_UART_TX_FIFO_SIZE;
    }
    if (is_initialized()) {
        // it is already initialized, reset it
        _end();
    }

    WITH_SEMAPHORE(_uartMutex);
    if (rxS > MAX_UART_RX_FIFO_SIZE) {
        rxS = MAX_UART_RX_FIFO_SIZE;
    }
    if (txS > MAX_UART_TX_FIFO_SIZE) {
        txS = MAX_UART_TX_FIFO_SIZE;
    }
    if (rxS != rxFIFO.get_size()) {
        rxFIFO.set_size(rxS);
    }
    if (txS != txFIFO.get_size()) {
        txFIFO.set_size(txS);
    }

    SIOConfig uart_config = {
        .baud         = b,
        .UARTLCR_H    = UART_UARTLCR_H_WLEN_8BITS | UART_UARTLCR_H_FEN,
        .UARTCR       = 0U,
        .UARTIFLS     = UART_UARTIFLS_RXIFLSEL_1_2F | UART_UARTIFLS_TXIFLSEL_1_2E,
        .UARTDMACR    = 0U
    };
    sioStart(uart_driver_inst, &uart_config);
    writeThread();
    readThread();
    initialized_flag = true;
}

bool Rp2040ChibiOS::UARTDriver::is_initialized() {
    WITH_SEMAPHORE(_uartMutex);
    return initialized_flag;
}
void Rp2040ChibiOS::UARTDriver::set_blocking_writes(bool blocking) {
    WITH_SEMAPHORE(_uartMutex);
    _blocking_writes = blocking;
}
bool Rp2040ChibiOS::UARTDriver::is_blocking_writes() {
    WITH_SEMAPHORE(_uartMutex);
    return _blocking_writes;
}
int8_t Rp2040ChibiOS::UARTDriver::driverSerialNr() {
    WITH_SEMAPHORE(_uartMutex);
    return _serial_num;
}

void Rp2040ChibiOS::UARTDriver::_end() {
    WITH_SEMAPHORE(_uartMutex);
    WITH_SEMAPHORE(_txUartMutex);
    WITH_SEMAPHORE(_rxUartMutex);

    rxFIFO.set_size(0);
    txFIFO.set_size(0);
    sioStop(uart_driver_inst);
    initialized_flag = false;
}

void Rp2040ChibiOS::UARTDriver::_flush(void) {
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

        /* We wrote less than we asked for, stop */
        if ((unsigned)ret != vec[i].len) {
            break;
        }
    }
}

void Rp2040ChibiOS::UARTDriver::async_read() {
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

bool Rp2040ChibiOS::UARTDriver::tx_pending() {
    if (!is_initialized()) return false;

    if (!_txUartMutex.take_nonblocking()){
        // the thread FIFO is locked.
        return true;
    }
    return txFIFO.available() > 0;
}

uint32_t Rp2040ChibiOS::UARTDriver::_available() {
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUartMutex);

    return rxFIFO.available();
}

uint32_t Rp2040ChibiOS::UARTDriver::txspace() {
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_txUartMutex);

    return txFIFO.space();
}

ssize_t Rp2040ChibiOS::UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUartMutex);

    return rxFIFO.read(buffer, count);
}

bool Rp2040ChibiOS::UARTDriver::_discard_input() {
    if (!is_initialized()) {
        return true;
    }

    WITH_SEMAPHORE(_rxUartMutex);

    rxFIFO.clear();
    return true;
}

void Rp2040ChibiOS::UARTDriver::clearTxFIFO() {
    if (!is_initialized()) {
        return;
    }

    WITH_SEMAPHORE(_txUartMutex);
    // clear the software fifo
    txFIFO.clear();
}

size_t Rp2040ChibiOS::UARTDriver::_write(const uint8_t *buffer, size_t size) {
    if (!is_initialized()) return 0;

    if (is_blocking_writes()) {
        size_t ret = 0;
        while (ret < size) {
            _txUartMutex.take_blocking();
            while (txFIFO.space() == 0) {
                _txUartMutex.give();
                hal.scheduler->delay(1);
                _txUartMutex.take_blocking();
            }
            txFIFO.write(buffer + ret, 1);
            _txUartMutex.give();
            ret++;
        }
        return ret;
    }

    WITH_SEMAPHORE(_txUartMutex);
    return txFIFO.write(buffer, size);
}

void Rp2040ChibiOS::UARTDriver::_uart_write_thread(void *arg)
{
    UARTDriver * uart = (UARTDriver *)arg;
    // add the number of uart interface to the name of thread
    char * thread_name;
    asprintf(&thread_name, "uart_write_thread_%d", uart->driverSerialNr());
    chRegSetThreadName(thread_name);

    while (true) {
        uart->flush();
        hal.scheduler->delay_microseconds(1000);
    }
}

void Rp2040ChibiOS::UARTDriver::_uart_read_thread(void *arg)
{
    UARTDriver * uart = (UARTDriver *)arg;
    // add the number of uart interface to the name of thread
    char * thread_name;
    asprintf(&thread_name, "uart_read_thread_%d", uart->driverSerialNr());
    chRegSetThreadName(thread_name);

    while (true) {
        uart->async_read();
        hal.scheduler->delay_microseconds(1000);
    }
}