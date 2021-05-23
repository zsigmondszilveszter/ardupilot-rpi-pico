
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_RPIPICO

#include <assert.h>

#include "HAL_RpiPico_Class.h"
#include "AP_HAL_RpiPico_Private.h"

#include "pico/multicore.h"


using namespace RpiPico;

static Console console_over_USB;
static UARTDriver uartBDriver = UARTDriver(0); // UART 0
static UARTDriver uartCDriver = UARTDriver(1); // UART 1
// static SPIDeviceManager spiDeviceManager;
// static AnalogIn analogIn;
// static Storage storageDriver;
static GPIO gpioDriver;
// static RCInput rcinDriver;
// static RCOutput rcoutDriver;
static Scheduler schedulerInstance;
static Util utilInstance;
// static OpticalFlow opticalFlowDriver;
// static Flash flashDriver;
static BgThread bgthread;

HAL_RpiPico::HAL_RpiPico() :
    AP_HAL::HAL(
        &console_over_USB,
        &uartBDriver,
        &uartCDriver,
        nullptr,            /* no uartD */
        nullptr,            /* no uartE */
        nullptr,            /* no uartF */
        nullptr,            /* no uartG */
        nullptr,            /* no uartH */
        nullptr,            /* no uartI */
        nullptr,// &I2CDeviceManager,
        nullptr,// &spiDeviceManager,
        nullptr,// &analogIn,
        nullptr,// &storageDriver,
        &console_over_USB, //nullptr,// &uartADriver,
        &gpioDriver,
        nullptr,// &rcinDriver,
        nullptr,// &rcoutDriver,
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
     * Scheduler should likely come first. */
    scheduler->init();

    scheduler->delay(1);
    // launch the background thread on second core (core1)
    multicore_launch_core1(RpiPico::BgThreadEntryPoint);
    // wait for it to start up
    multicore_fifo_pop_blocking();

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
