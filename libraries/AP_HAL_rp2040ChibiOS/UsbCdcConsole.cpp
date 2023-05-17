#include "hal.h"
#include "UsbCdcConsole.h"
#include "usbcfg.h"
#include "Scheduler.h"
#include "rp2040_util.h"

extern const AP_HAL::HAL& hal;

/*
  initialise the USB bus, called from both UARTDriver and stdio for startup debug
  This can be called before the hal is initialised so must not call any hal functions
 */
void usb_initialise(void)
{
    static bool initialised;
    if (initialised) {
        return;
    }
    initialised = true;

    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleep(chTimeUS2I(1000));
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

}

#ifndef USB_CDC_WRITE_THD_WA_SIZE
#define USB_CDC_WRITE_THD_WA_SIZE RP2040_USB_CDC_TX_FIFO_SIZE + 32
#endif

#ifndef USB_CDC_READ_THD_WA_SIZE
#define USB_CDC_READ_THD_WA_SIZE RP2040_USB_CDC_RX_FIFO_SIZE + 32
#endif

Rp2040ChibiOS::UsbCdcConsole::UsbCdcConsole() {}

void Rp2040ChibiOS::UsbCdcConsole::writeThread() {
    // setup the uart worker thread to flush the TX FIFO
    if (_usb_cdc_write_thread_ctx == nullptr) {
        _usb_cdc_write_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(USB_CDC_WRITE_THD_WA_SIZE),
                                              "UART_TX",
                                              USB_CDC_THREAD_PRIORITY,
                                              _usb_cdc_write_thread,
                                              this, &ch1);
        if (_usb_cdc_write_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create USB CDC CONSOLE TX thread\n");
        }
    }
}

void Rp2040ChibiOS::UsbCdcConsole::readThread() {
    // setup the uart worker thread to read the RX FIFO
    if (_usb_cdc_read_thread_ctx == nullptr) {
        _usb_cdc_read_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(USB_CDC_READ_THD_WA_SIZE),
                                              "UART_RX",
                                              USB_CDC_THREAD_PRIORITY,
                                              _usb_cdc_read_thread,
                                              this, &ch1);
        if (_usb_cdc_read_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create USB CDC CONSOLE RX thread\n");
        }
    }
}

void Rp2040ChibiOS::UsbCdcConsole::begin(uint32_t b) {
    begin(b, RP2040_USB_CDC_RX_FIFO_SIZE, RP2040_USB_CDC_TX_FIFO_SIZE);
}
void Rp2040ChibiOS::UsbCdcConsole::begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    WITH_SEMAPHORE(_usbMutex);
    initialized_flag = false;

    if (rxS > MAX_USB_CDC_RX_FIFO_SIZE) {
        rxS = MAX_USB_CDC_RX_FIFO_SIZE;
    }
    if (txS > MAX_USB_CDC_TX_FIFO_SIZE) {
        txS = MAX_USB_CDC_TX_FIFO_SIZE;
    }
    if (rxS != rxFIFO.get_size()) {
        rxFIFO.set_size(rxS);
    }
    if (txS != txFIFO.get_size()) {
        txFIFO.set_size(txS);
    }
    writeThread();
    readThread();
    initialized_flag = true;
}

bool Rp2040ChibiOS::UsbCdcConsole::is_initialized() {
    WITH_SEMAPHORE(_usbMutex);
    return initialized_flag;
}
void Rp2040ChibiOS::UsbCdcConsole::set_blocking_writes(bool blocking) {
    WITH_SEMAPHORE(_usbMutex);
    _blocking_writes = blocking;
}
bool Rp2040ChibiOS::UsbCdcConsole::is_blocking_writes() {
    WITH_SEMAPHORE(_usbMutex);
    return _blocking_writes;
}

void Rp2040ChibiOS::UsbCdcConsole::end() {
    WITH_SEMAPHORE(_usbMutex);
    WITH_SEMAPHORE(_txUsbMutex);
    WITH_SEMAPHORE(_rxUsbMutex);

    // clear the software fifo
    rxFIFO.clear();
    txFIFO.clear();
    initialized_flag = false;
}


void Rp2040ChibiOS::UsbCdcConsole::flush(void) {
    if (!is_initialized()) return;
    if ((&SDU1)->state != SDU_READY) return;

    WITH_SEMAPHORE(_txUsbMutex);

    ByteBuffer::IoVec vec[2];
    const auto n_vec = txFIFO.peekiovec(vec, txFIFO.available());

    for (int i = 0; i < n_vec; i++) {
        size_t ret = chnWriteTimeout(&SDU1, vec[i].data, vec[i].len, TIME_IMMEDIATE);

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

void Rp2040ChibiOS::UsbCdcConsole::async_read() {
    if (!is_initialized()) return;
    if ((&SDU1)->state != SDU_READY) return;

    WITH_SEMAPHORE(_rxUsbMutex);

    // try to fill the read buffer
    ByteBuffer::IoVec vec[2];
    const auto n_vec = rxFIFO.reserve(vec, rxFIFO.space());

    for (uint32_t i = 0; i < n_vec; i++) {
        size_t ret = chnReadTimeout(&SDU1, vec[i].data, vec[i].len, TIME_IMMEDIATE);

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

bool Rp2040ChibiOS::UsbCdcConsole::tx_pending() {
    if (!is_initialized()) return false;

    if (!_txUsbMutex.take_nonblocking()){
        // the thread FIFO is locked.
        return true;
    }
    return txFIFO.available() > 0; 
}

/* rp2040 implementations of Stream virtual methods */
uint32_t Rp2040ChibiOS::UsbCdcConsole::available() { 
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUsbMutex);

    return rxFIFO.available();
}

uint32_t Rp2040ChibiOS::UsbCdcConsole::txspace() { 
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_txUsbMutex);

    return txFIFO.space();
}

bool Rp2040ChibiOS::UsbCdcConsole::read(uint8_t &b) { 
    if (!is_initialized()) return false;

    WITH_SEMAPHORE(_rxUsbMutex);

    return !rxFIFO.read_byte(&b);
}

ssize_t Rp2040ChibiOS::UsbCdcConsole::read(uint8_t *buffer, uint16_t count)
{
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUsbMutex);

    return rxFIFO.read(buffer, count);
}

bool Rp2040ChibiOS::UsbCdcConsole::discard_input() {
    if (!is_initialized()) {
        return true;
    }

    WITH_SEMAPHORE(_rxUsbMutex);

    rxFIFO.clear();
    return true; 
}

void Rp2040ChibiOS::UsbCdcConsole::clearTxFIFO() {
    if (!is_initialized()) {
        return;
    }

    WITH_SEMAPHORE(_txUsbMutex);
    // clear the software fifo
    txFIFO.clear();
}

/* rp2040 implementations of Print virtual methods */
size_t Rp2040ChibiOS::UsbCdcConsole::write(uint8_t c) { 
    _txUsbMutex.take_blocking();
    if (!is_initialized()) {
        _txUsbMutex.give();
        return 0;
    }

    while (txFIFO.space() == 0) {
        if (!is_blocking_writes()) {
            _txUsbMutex.give();
            return 0;
        }
        // release the semaphore while sleeping
        _txUsbMutex.give();
        hal.scheduler->delay(1);
        _txUsbMutex.take_blocking();
    }
    size_t ret = txFIFO.write(&c, 1);
    _txUsbMutex.give();
    return ret;
}

size_t Rp2040ChibiOS::UsbCdcConsole::write(const uint8_t *buffer, size_t size) {
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

    WITH_SEMAPHORE(_txUsbMutex);

    size_t ret = txFIFO.write(buffer, size);
    return ret;
}

void Rp2040ChibiOS::UsbCdcConsole::_usb_cdc_write_thread(void *arg)
{
    Rp2040ChibiOS::UsbCdcConsole * console = (Rp2040ChibiOS::UsbCdcConsole *)arg;
    chRegSetThreadName("usb_cdc_write_thread");

    while (true) {
        console->flush();
        hal.scheduler->delay_microseconds(1000);
    }
}

void Rp2040ChibiOS::UsbCdcConsole::_usb_cdc_read_thread(void *arg)
{
    Rp2040ChibiOS::UsbCdcConsole * console = (Rp2040ChibiOS::UsbCdcConsole *)arg;
    chRegSetThreadName("usb_cdc_read_thread");

    while (true) {
        console->async_read();
        hal.scheduler->delay_microseconds(1000);
    }
}