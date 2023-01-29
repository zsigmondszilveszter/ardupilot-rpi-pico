#include "UARTDriver.h"
#include <AP_Common/ExpandingString.h>

#include <AP_Math/AP_Math.h>


#define UART_THREAD_PRIORITY  LOWPRIO

#ifndef UART_THD_WA_SIZE
#define UART_THD_WA_SIZE 288
#endif

extern const AP_HAL::HAL& hal;

THD_WORKING_AREA(_uart_thread_wa_0, UART_THD_WA_SIZE);
THD_WORKING_AREA(_uart_thread_wa_1, UART_THD_WA_SIZE);

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
    chMtxObjectInit(&uartMutex);
}

void Rp2040ChibiOS::UARTDriver::startThreadOnCore1() {
    // setup the uart worker thread to flush the TX FIFO and read the RX FIFO
    switch (_serial_num) {
        case 0: 
            _uart_thread_ctx = chThdCreateStatic(_uart_thread_wa_0,
                     sizeof(_uart_thread_wa_0),
                     UART_THREAD_PRIORITY,        /* Initial priority.    */
                     _uart_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
            break;
        case 1: 
            _uart_thread_ctx = chThdCreateStatic(_uart_thread_wa_1,
                     sizeof(_uart_thread_wa_1),
                     UART_THREAD_PRIORITY,        /* Initial priority.    */
                     _uart_thread,             /* Thread function.     */
                     this); 
            break;
        default: break;
    }
}

void Rp2040ChibiOS::UARTDriver::begin(uint32_t b) {
    if (initialized_flag) {
        // it is already initialized, reset it
        end();
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
void Rp2040ChibiOS::UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    // max allowed fifo size is RPI_PICO_UART_MAX_ALLOWED_BUFFER_SIZE
    maxTxFIFO = txS <= RP2040_UART_MAX_ALLOWED_BUFFER_SIZE ? txS : RP2040_UART_MAX_ALLOWED_BUFFER_SIZE;
    maxRxFIFO = rxS <= RP2040_UART_MAX_ALLOWED_BUFFER_SIZE ? rxS : RP2040_UART_MAX_ALLOWED_BUFFER_SIZE;
    begin(b);
}

void Rp2040ChibiOS::UARTDriver::end() {
    discard_input();
    clearTxFIFO();
    sioStop(uart_driver_inst);
    initialized_flag = false;
}

void Rp2040ChibiOS::UARTDriver::flush() { _flush(); }

uint8_t Rp2040ChibiOS::UARTDriver::_flush(void) {
    if (!chMtxTryLock(&uartMutex)) {
        // the thread FIFO is locked.
        // the flush function is called periodically by the background thread
        // therefore we leave now and try in the next round
        return 0;
    }
    if (!is_initialized() || txFIFO.empty()) {
        chMtxUnlock(&uartMutex);
        return 0;
    }

    while (!txFIFO.empty()){
        if (sioIsTXFullX(uart_driver_inst)) {
            // don't force, try later
            break;
        }
        sioPutX(uart_driver_inst, (uint_fast16_t)txFIFO.front());
        txFIFO.pop();
    }
    uint8_t nrOfElementsInFIFO = txFIFO.size();
    chMtxUnlock(&uartMutex);
    return nrOfElementsInFIFO;
}

void Rp2040ChibiOS::UARTDriver::async_read() {
    if (!chMtxTryLock(&uartMutex)) {
        // the transfer FIFO is locked.
        // the async_read function is called periodically by the background thread
        // therefore we leave now and try in the next round
        return;
    }
    if (!is_initialized()) {
        chMtxUnlock(&uartMutex);
        return;
    }
    // we read all the 32 bytes (the size of PL011's RX buffer) at once
    uint8_t steps = 0;
    uint8_t spaceInFIFO = maxRxFIFO - rxFIFO.size();
    while(!sioIsRXEmptyX(uart_driver_inst) && steps < 32) {
        uint8_t c = (uint8_t)sioGetX(uart_driver_inst);
        if (spaceInFIFO == steps){
            // we start to delete the oldest characters from buffer, 
            // because we ran out of space
            // !!! this is questionable though 
            rxFIFO.pop();
        }
        rxFIFO.push(c);
        steps++;
    }
    chMtxUnlock(&uartMutex);
}


bool Rp2040ChibiOS::UARTDriver::tx_pending() {
    if (!chMtxTryLock(&uartMutex)){
        // the thread FIFO is locked.
        return true;
    }
    if (!is_initialized()) {
        chMtxUnlock(&uartMutex);
        return false;
    }

    bool ret = !txFIFO.empty();
    chMtxUnlock(&uartMutex);
    return ret; 
}

/* rp2040 implementations of Stream virtual methods */
uint32_t Rp2040ChibiOS::UARTDriver::available() { 
    if (!lockMutexIfInitialized()) return 0;
    uint32_t ret_val = rxFIFO.size();
    chMtxUnlock(&uartMutex);
    return ret_val;
}

uint32_t Rp2040ChibiOS::UARTDriver::txspace() { 
    if (!lockMutexIfInitialized()) return 0;

    uint32_t space = 0;
    space = maxTxFIFO - txFIFO.size();
    chMtxUnlock(&uartMutex);
    return space;
}
int16_t Rp2040ChibiOS::UARTDriver::read() { 
    if (!lockMutexIfInitialized()) return 0;

    int16_t ret = -1;
    if (!rxFIFO.empty()) {
        ret = (int16_t) rxFIFO.front();
        rxFIFO.pop();
    }
    chMtxUnlock(&uartMutex);
    return ret;
}

bool Rp2040ChibiOS::UARTDriver::discard_input() {
    if (!lockMutexIfInitialized()) return false;

    std::queue<uint8_t>().swap(rxFIFO);
    chMtxUnlock(&uartMutex);
    return true; 
}

void Rp2040ChibiOS::UARTDriver::clearTxFIFO() {
    if (!lockMutexIfInitialized()) return;

    // clear the software fifo
    std::queue<uint8_t>().swap(txFIFO);
    chMtxUnlock(&uartMutex);
}

/* rp2040 implementations of Print virtual methods */
size_t Rp2040ChibiOS::UARTDriver::write(uint8_t c) { 
    if (!lockMutexIfInitialized()) return 0;

    size_t ret = 0;
    if (txFIFO.size() < maxTxFIFO) {
        txFIFO.push(c);
        ret++;
    }
    chMtxUnlock(&uartMutex);
    return ret;
}

size_t Rp2040ChibiOS::UARTDriver::write(const uint8_t *buffer, size_t size) {
    if (!lockMutexIfInitialized()) return 0;
    
    size_t ret = 0;
    for (uint32_t i=0; i<MIN(size, maxTxFIFO - txFIFO.size()); i++) {
        txFIFO.push(buffer[i]);
        ret++;
    }
    chMtxUnlock(&uartMutex);
    return ret;
}

bool Rp2040ChibiOS::UARTDriver::lockMutexIfInitialized() {
    chMtxLock(&uartMutex);
    if (!is_initialized()) {
        chMtxUnlock(&uartMutex);
        return false;
    }
    return true;
}

void Rp2040ChibiOS::UARTDriver::_uart_thread(void *arg)
{
    UARTDriver * uart = (UARTDriver *)arg;
    // add the number of uart interface to the name of thread
    char * thread_name;
    asprintf(&thread_name, "uart_thread_%d", uart->driverSerialNr());
    chRegSetThreadName(thread_name);

    uint16_t delay = 0;
    while (true) {
        // delay less if there are elements left in the TX FIFO
        delay =  uart->_flush() ? 10 : 100;
        uart->async_read();
        hal.scheduler->delay_microseconds(delay);
    }
}

#if HAL_UART_STATS_ENABLED
void Rp2040ChibiOS::UARTDriver::uart_info(ExpandingString &str)
{
    str.printf("EMPTY\n");
}
#endif