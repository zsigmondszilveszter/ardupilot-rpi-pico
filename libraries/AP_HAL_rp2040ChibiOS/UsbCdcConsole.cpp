#include "hal.h"
#include "UsbCdcConsole.h"
#include "usbcfg.h"
#include "Scheduler.h"
#include "rp2xxx_util.h"

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
    chThdSleep(chTimeUS2I(2500));
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
}

#ifndef USB_WRITE_THD_WA_SIZE
#define USB_WRITE_THD_WA_SIZE 256
#endif

#ifndef USB_READ_THD_WA_SIZE
#define USB_READ_THD_WA_SIZE 256
#endif

Rp2040ChibiOS::UsbCdcConsole::UsbCdcConsole() {}

void Rp2040ChibiOS::UsbCdcConsole::writeThread() {
    // setup the uart worker thread to flush the TX FIFO
    if (_usb_cdc_write_thread_ctx == nullptr) {
        _usb_cdc_write_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(USB_WRITE_THD_WA_SIZE),
                                              "UART_TX",
                                              USB_CDC_THREAD_PRIORITY,
                                              _usb_cdc_write_thread,
                                              this,
                                              &ch1);
        if (_usb_cdc_write_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create USB CDC CONSOLE TX thread\n");
        }
    }
}

void Rp2040ChibiOS::UsbCdcConsole::readThread() {
    // setup the uart worker thread to read the RX FIFO
    if (_usb_cdc_read_thread_ctx == nullptr) {
        _usb_cdc_read_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(USB_READ_THD_WA_SIZE),
                                              "UART_RX",
                                              USB_CDC_THREAD_PRIORITY,
                                              _usb_cdc_read_thread,
                                              this,
                                              &ch1);
        if (_usb_cdc_read_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create USB CDC CONSOLE RX thread\n");
        }
    }
}

void Rp2040ChibiOS::UsbCdcConsole::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    if (rxS == 0) {
        rxS = RP2040_USB_RX_FIFO_SIZE;
    }
    if (txS == 0) {
        txS = RP2040_USB_TX_FIFO_SIZE;
    }
    WITH_SEMAPHORE(_usbMutex);
    initialized_flag = false;

    if (rxS > MAX_USB_RX_FIFO_SIZE) {
        rxS = MAX_USB_RX_FIFO_SIZE;
    }
    if (txS > MAX_USB_TX_FIFO_SIZE) {
        txS = MAX_USB_TX_FIFO_SIZE;
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
void Rp2040ChibiOS::UsbCdcConsole::_end() {
    WITH_SEMAPHORE(_usbMutex);
    WITH_SEMAPHORE(_txUsbMutex);
    WITH_SEMAPHORE(_rxUsbMutex);

    // clear the software fifo
    rxFIFO.clear();
    txFIFO.clear();
    initialized_flag = false;
}


void Rp2040ChibiOS::UsbCdcConsole::_flush(void) {
    if (!is_initialized()) return;
    if ((&SDU1)->state != SDU_READY) return;
    if (!cdc_dtr_active) return;

    WITH_SEMAPHORE(_txUsbMutex);

    ByteBuffer::IoVec vec[2];
    const auto n_vec = txFIFO.peekiovec(vec, txFIFO.available());

    for (int i = 0; i < n_vec; i++) {
        size_t ret = chnWriteTimeout(&SDU1, vec[i].data, vec[i].len, TIME_IMMEDIATE);

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

    if (!_txUsbMutex.take_nonblocking()) {
        // the thread FIFO is locked, assume pending
        return true;
    }
    bool pending = txFIFO.available() > 0;
    _txUsbMutex.give();
    return pending;
}

/* rp2040 implementations of Stream virtual methods */
uint32_t Rp2040ChibiOS::UsbCdcConsole::_available() {
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUsbMutex);

    return rxFIFO.available();
}

uint32_t Rp2040ChibiOS::UsbCdcConsole::txspace() {
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_txUsbMutex);

    return txFIFO.space();
}

ssize_t Rp2040ChibiOS::UsbCdcConsole::_read(uint8_t *buffer, uint16_t count)
{
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUsbMutex);

    return rxFIFO.read(buffer, count);
}

bool Rp2040ChibiOS::UsbCdcConsole::_discard_input() {
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
size_t Rp2040ChibiOS::UsbCdcConsole::_write(const uint8_t *buffer, size_t size) {
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_txUsbMutex);
    return txFIFO.write(buffer, size);
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
