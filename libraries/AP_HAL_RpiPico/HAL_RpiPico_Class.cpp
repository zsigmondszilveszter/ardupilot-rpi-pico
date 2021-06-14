
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_RPIPICO

#include <assert.h>

#include "HAL_RpiPico_Class.h"
#include "AP_HAL_RpiPico_Private.h"

#include "pico/multicore.h"


using namespace RpiPico;

static BgThread bgthread;
static Console console_over_USB;
static UARTDriver uartBDriver = UARTDriver(0); // UART 0
static UARTDriver uartFDriver = UARTDriver(1); // UART 1
static I2CDeviceManager i2cDeviceManager;
static SPIDeviceManager spiDeviceManager;
static AnalogIn analogIn;
static Storage storageDriver;
static GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;
static Scheduler schedulerInstance;
static Util utilInstance;
// static OpticalFlow opticalFlowDriver;
// static Flash flashDriver;

HAL_RpiPico::HAL_RpiPico() :
    AP_HAL::HAL(
        &console_over_USB,
        &uartBDriver,
        nullptr, //&uartCDriver,
        nullptr,            /* no uartD */
        nullptr,            /* no uartE */
        &uartFDriver, //rcin                
        nullptr,            /* no uartG */
        nullptr,            /* no uartH */
        nullptr,            /* no uartI */
        &i2cDeviceManager,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &console_over_USB, //nullptr,// &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        nullptr,// &opticalFlowDriver,
        nullptr,// &flashDriver,
        nullptr, /* no DSP */
        nullptr)    // no CAN        
{}

void HAL_RpiPico::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     */
    sleep_ms(1);

    // launch the background thread on second core (core1)
    multicore_launch_core1(RpiPico::BgThreadEntryPoint);
    // wait for it to start up
    multicore_fifo_pop_blocking();

    // register background workers
    ((RpiPico::Console*)console)->registerBackgroundWorkers();
    ((RpiPico::UARTDriver*)serial(3))->registerBackgroundWorkers();
    ((RpiPico::UARTDriver*)serial(5))->registerBackgroundWorkers();
    ((RpiPico::RCInput*)rcin)->registerBackgroundWorkers();

    ((RpiPico::Console*)console)->init();
    scheduler->init();
    // wait a couple of msec
    scheduler->delay(100);
    rcin->init();
    callbacks->setup();
    scheduler->set_system_initialized();

    for (;;) {
        callbacks->loop();
    }
}

/**
 * The instance of the background thread running on core1
 */
RpiPico::BgThread& RpiPico::getBgThread() {
    return bgthread;
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_RpiPico hal;
    return hal;
}

#endif
