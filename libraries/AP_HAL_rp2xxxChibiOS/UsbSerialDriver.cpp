#include "hal.h"
#include "UsbSerialDriver.h"
#include "usbcfg.h"
#include "Scheduler.h"
#include "rp2xxx_util.h"
#include "GPIO.h"

extern const AP_HAL::HAL& hal;

#define MAX_USB_SERIAL_INSTANCES 2
static Rp2xxxChibiOS::UsbSerialDriver* _usb_instances[MAX_USB_SERIAL_INSTANCES];
static uint8_t _usb_instance_count = 0;

extern "C" void usb_sof_notify(void) {
    for (uint8_t i = 0; i < _usb_instance_count; i++) {
        _usb_instances[i]->sof_notify();
    }
}

void Rp2xxxChibiOS::UsbSerialDriver::sof_notify(void) {
    _rxWakeSem.signal_ISR();
}

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
    sduStart(&SDU1, &serusbcfg1);
    sduObjectInit(&SDU2);
    sduStart(&SDU2, &serusbcfg2);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg1.usbp);
    chThdSleep(chTimeUS2I(2500));
    usbStart(serusbcfg1.usbp, &usbcfg);
    usbConnectBus(serusbcfg1.usbp);
}

#ifndef USB_WRITE_THD_WA_SIZE
#define USB_WRITE_THD_WA_SIZE 256
#endif

#ifndef USB_READ_THD_WA_SIZE
#define USB_READ_THD_WA_SIZE 256
#endif

Rp2xxxChibiOS::UsbSerialDriver::UsbSerialDriver(SerialUSBDriver* sdu)
    : _sdu(sdu) {}

void Rp2xxxChibiOS::UsbSerialDriver::writeThread() {
    // setup the uart worker thread to flush the TX FIFO
    if (_usb_cdc_write_thread_ctx == nullptr) {
        _usb_cdc_write_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(USB_WRITE_THD_WA_SIZE),
                                              "UART_TX",
                                              APM_UART_PRIORITY,
                                              _usb_cdc_write_thread,
                                              this,
                                              &ch1);
        if (_usb_cdc_write_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create USB CDC CONSOLE TX thread\n");
        }
    }
}

void Rp2xxxChibiOS::UsbSerialDriver::readThread() {
    // setup the uart worker thread to read the RX FIFO
    if (_usb_cdc_read_thread_ctx == nullptr) {
        _usb_cdc_read_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(USB_READ_THD_WA_SIZE),
                                              "UART_RX",
                                              APM_UART_PRIORITY,
                                              _usb_cdc_read_thread,
                                              this,
                                              &ch1);
        if (_usb_cdc_read_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create USB CDC CONSOLE RX thread\n");
        }
    }
}

void Rp2xxxChibiOS::UsbSerialDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {
    if (rxS == 0) {
        rxS = RP2xxx_USB_RX_FIFO_SIZE;
    }
    if (txS == 0) {
        txS = RP2xxx_USB_TX_FIFO_SIZE;
    }
    WITH_SEMAPHORE(_usbMutex);
    initialized_flag = false;

    if (rxS > RP2xxx_USB_MAX_ALLOWED_BUFFER_SIZE ) {
        rxS = RP2xxx_USB_MAX_ALLOWED_BUFFER_SIZE ;
    }
    if (txS > RP2xxx_USB_MAX_ALLOWED_BUFFER_SIZE ) {
        txS = RP2xxx_USB_MAX_ALLOWED_BUFFER_SIZE ;
    }
    if (rxS != rxFIFO.get_size()) {
        rxFIFO.set_size(rxS);
    }
    if (txS != txFIFO.get_size()) {
        txFIFO.set_size(txS);
    }
    writeThread();
    readThread();
    // register for SOF notifications (only once per instance)
    bool already_registered = false;
    for (uint8_t i = 0; i < _usb_instance_count; i++) {
        if (_usb_instances[i] == this) {
            already_registered = true;
            break;
        }
    }
    if (!already_registered && _usb_instance_count < MAX_USB_SERIAL_INSTANCES) {
        _usb_instances[_usb_instance_count++] = this;
    }
    initialized_flag = true;
}

bool Rp2xxxChibiOS::UsbSerialDriver::is_initialized() {
    WITH_SEMAPHORE(_usbMutex);
    return initialized_flag;
}
void Rp2xxxChibiOS::UsbSerialDriver::_end() {
    WITH_SEMAPHORE(_usbMutex);
    WITH_SEMAPHORE(_txUsbMutex);
    WITH_SEMAPHORE(_rxUsbMutex);

    // clear the software fifo
    rxFIFO.clear();
    txFIFO.clear();
    initialized_flag = false;
}


void Rp2xxxChibiOS::UsbSerialDriver::_flush(void) {
    if (!is_initialized()) return;
    if (_sdu->state != SDU_READY) return;

    WITH_SEMAPHORE(_txUsbMutex);

    ByteBuffer::IoVec vec[2];
    const auto n_vec = txFIFO.peekiovec(vec, txFIFO.available());

    for (int i = 0; i < n_vec; i++) {
        size_t ret = chnWriteTimeout(_sdu, vec[i].data, vec[i].len, TIME_IMMEDIATE);

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

void Rp2xxxChibiOS::UsbSerialDriver::async_read() {
    if (!is_initialized()) return;
    if (_sdu->state != SDU_READY) return;

    ((GPIO *)hal.gpio)->set_usb_connected();

    WITH_SEMAPHORE(_rxUsbMutex);

    // try to fill the read buffer
    ByteBuffer::IoVec vec[2];
    const auto n_vec = rxFIFO.reserve(vec, rxFIFO.space());

    for (uint32_t i = 0; i < n_vec; i++) {
        size_t ret = chnReadTimeout(_sdu, vec[i].data, vec[i].len, TIME_IMMEDIATE);

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

bool Rp2xxxChibiOS::UsbSerialDriver::tx_pending() {
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
uint32_t Rp2xxxChibiOS::UsbSerialDriver::_available() {
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUsbMutex);

    return rxFIFO.available();
}

uint32_t Rp2xxxChibiOS::UsbSerialDriver::txspace() {
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_txUsbMutex);

    return txFIFO.space();
}

ssize_t Rp2xxxChibiOS::UsbSerialDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!is_initialized()) return 0;

    WITH_SEMAPHORE(_rxUsbMutex);

    return rxFIFO.read(buffer, count);
}

bool Rp2xxxChibiOS::UsbSerialDriver::_discard_input() {
    if (!is_initialized()) {
        return true;
    }

    WITH_SEMAPHORE(_rxUsbMutex);

    rxFIFO.clear();
    return true;
}

void Rp2xxxChibiOS::UsbSerialDriver::clearTxFIFO() {
    if (!is_initialized()) {
        return;
    }

    WITH_SEMAPHORE(_txUsbMutex);
    // clear the software fifo
    txFIFO.clear();
}

/* rp2040 implementations of Print virtual methods */
size_t Rp2xxxChibiOS::UsbSerialDriver::_write(const uint8_t *buffer, size_t size) {
    if (!is_initialized()) return 0;

    size_t written;
    {
        WITH_SEMAPHORE(_txUsbMutex);
        written = txFIFO.write(buffer, size);
    }
    if (written > 0) {
        _txWakeSem.signal();
    }
    return written;
}

void Rp2xxxChibiOS::UsbSerialDriver::_usb_cdc_write_thread(void *arg)
{
    Rp2xxxChibiOS::UsbSerialDriver * driver = (Rp2xxxChibiOS::UsbSerialDriver *)arg;
    chRegSetThreadName("usb_cdc_write_thread");

    while (true) {
        driver->_txWakeSem.wait(5000);
        driver->_flush();
    }
}

void Rp2xxxChibiOS::UsbSerialDriver::_usb_cdc_read_thread(void *arg)
{
    Rp2xxxChibiOS::UsbSerialDriver * driver = (Rp2xxxChibiOS::UsbSerialDriver *)arg;
    chRegSetThreadName("usb_cdc_read_thread");

    while (true) {
        driver->_rxWakeSem.wait(5000);
        driver->async_read();
    }
}
