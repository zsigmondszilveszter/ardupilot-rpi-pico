#include "UARTDriver.h"


extern const AP_HAL::HAL& hal;


#define UART_THREAD_PRIORITY  LOWPRIO

#ifndef UART_WRITE_THD_WA_SIZE
#define UART_WRITE_THD_WA_SIZE RP2040_UART_TX_FIFO_SIZE + 32
#endif

#ifndef UART_READ_THD_WA_SIZE
#define UART_READ_THD_WA_SIZE RP2040_UART_RX_FIFO_SIZE + 32
#endif

THD_WORKING_AREA(_uart_write_thread_wa_0, UART_WRITE_THD_WA_SIZE);
THD_WORKING_AREA(_uart_write_thread_wa_1, UART_WRITE_THD_WA_SIZE);
THD_WORKING_AREA(_uart_read_thread_wa_0, UART_READ_THD_WA_SIZE);
THD_WORKING_AREA(_uart_read_thread_wa_1, UART_READ_THD_WA_SIZE);

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
    switch (driverSerialNr()) {
        case 0: 
            _uart_write_thread_ctx = chThdCreateStatic(_uart_write_thread_wa_0,
                     sizeof(_uart_write_thread_wa_0),
                     UART_THREAD_PRIORITY,        /* Initial priority.    */
                     _uart_write_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
            break;
        case 1: 
            _uart_write_thread_ctx = chThdCreateStatic(_uart_write_thread_wa_1,
                     sizeof(_uart_write_thread_wa_1),
                     UART_THREAD_PRIORITY,        /* Initial priority.    */
                     _uart_write_thread,             /* Thread function.     */
                     this); 
            break;
        default: break;
    }
}

void Rp2040ChibiOS::UARTDriver::readThread() {
    // setup the uart worker thread to read the RX FIFO
    switch (driverSerialNr()) {
        case 0: 
            _uart_read_thread_ctx = chThdCreateStatic(_uart_read_thread_wa_0,
                     sizeof(_uart_read_thread_wa_0),
                     UART_THREAD_PRIORITY,        /* Initial priority.    */
                     _uart_read_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
            break;
        case 1: 
            _uart_read_thread_ctx = chThdCreateStatic(_uart_read_thread_wa_1,
                     sizeof(_uart_read_thread_wa_1),
                     UART_THREAD_PRIORITY,        /* Initial priority.    */
                     _uart_read_thread,             /* Thread function.     */
                     this); 
            break;
        default: break;
    }
}

void Rp2040ChibiOS::UARTDriver::begin(uint32_t b) {
    begin(b, RP2040_UART_RX_FIFO_SIZE, RP2040_UART_TX_FIFO_SIZE);
}
void Rp2040ChibiOS::UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    if (is_initialized()) {
        // it is already initialized, reset it
        end();
    }

    WITH_SEMAPHORE(_uartMutex);
    if (rxS > MAX_UART_RX_FIFO_SIZE) {
        rxS = MAX_UART_RX_FIFO_SIZE;
    }
    if (txS > MAX_UART_TX_FIFO_SIZE) {
        txS = MAX_UART_TX_FIFO_SIZE;
    }
    if (rxS != rxFIFO.get_size()) {
        // _rx_initialised = false;
        rxFIFO.set_size(rxS);
    }
    if (txS != txFIFO.get_size()) {
        // _tx_initialised = false;
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

void Rp2040ChibiOS::UARTDriver::end() {
    WITH_SEMAPHORE(_uartMutex);
    WITH_SEMAPHORE(_txUartMutex);
    WITH_SEMAPHORE(_rxUartMutex);

    rxFIFO.set_size(0);
    txFIFO.set_size(0);
    sioStop(uart_driver_inst);
    initialized_flag = false;
}

void Rp2040ChibiOS::UARTDriver::flush() { _flush(); }

uint8_t Rp2040ChibiOS::UARTDriver::_flush(void) {
    if (!is_initialized()) return 0;

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
    return txFIFO.available();
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

uint32_t Rp2040ChibiOS::UARTDriver::available() { 
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUartMutex);

    return rxFIFO.available();
}

uint32_t Rp2040ChibiOS::UARTDriver::txspace() { 
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_txUartMutex);

    return txFIFO.space();
}
int16_t Rp2040ChibiOS::UARTDriver::read() { 
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUartMutex);

    uint8_t ret = -1;
    if (!rxFIFO.read_byte(&ret)) {
        return -1;
    }
    return ret;
}

bool Rp2040ChibiOS::UARTDriver::discard_input() {
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

size_t Rp2040ChibiOS::UARTDriver::write(uint8_t c) { 
    _txUartMutex.take_blocking();
    if (!is_initialized()) {
        _txUartMutex.give();
        return 0;
    }

    while (txFIFO.space() == 0) {
        if (!is_blocking_writes()) {
            _txUartMutex.give();
            return 0;
        }
        // release the semaphore while sleeping
        _txUartMutex.give();
        hal.scheduler->delay(1);
        _txUartMutex.take_blocking();
    }
    size_t ret = txFIFO.write(&c, 1);
    _txUartMutex.give();
    return ret;
}

size_t Rp2040ChibiOS::UARTDriver::write(const uint8_t *buffer, size_t size) {
    if (!is_initialized()) {
		return 0;
	}

    if (is_blocking_writes()) {
        // use the per-byte delay loop in write() above for blocking writes
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    WITH_SEMAPHORE(_txUartMutex);

    size_t ret = txFIFO.write(buffer, size);
    return ret;
}

void Rp2040ChibiOS::UARTDriver::_uart_write_thread(void *arg)
{
    UARTDriver * uart = (UARTDriver *)arg;
    // add the number of uart interface to the name of thread
    char * thread_name;
    asprintf(&thread_name, "uart_write_thread_%d", uart->driverSerialNr());
    chRegSetThreadName(thread_name);

    uint16_t delay = 0;
    while (true) {
        // delay less if there are elements left in the TX FIFO
        delay =  uart->_flush() ? 10 : 1000;
        hal.scheduler->delay_microseconds(delay);
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