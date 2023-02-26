#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_RP2040CHIBIOS

#include <hal.h>
#include <ch.h>

#include <assert.h>

#include "HAL_rp2040ChibiOS_Class.h"
#include "AP_HAL_rp2040ChibiOS_Private.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InternalError/AP_InternalError.h>
#ifndef HAL_BOOTLOADER_BUILD
#include <AP_Logger/AP_Logger.h>
#endif
#include <AP_Vehicle/AP_Vehicle_Type.h>


using namespace Rp2040ChibiOS;

static Rp2040ChibiOS::UsbCdcConsole console_over_USB;
static Rp2040ChibiOS::UARTDriver uartBDriver(0); // UART 0
static Rp2040ChibiOS::UARTDriver uartFDriver(1); // UART 1
// static I2CDeviceManager i2cDeviceManager;
// static SPIDeviceManager spiDeviceManager;
// static AnalogIn analogIn;
// static Storage storageDriver;
// static GPIO gpioDriver;
// static RCInput rcinDriver;
// static RCOutput rcoutDriver;
static Rp2040ChibiOS::Scheduler schedulerInstance;
static Rp2040ChibiOS::Util utilInstance;
// static OpticalFlow opticalFlowDriver;
// static Flash flashDriver;

HAL_Rp2040ChibiOS::HAL_Rp2040ChibiOS() :
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
        nullptr,            /* no uartJ*/
        nullptr,// &i2cDeviceManager,
        nullptr,// &spiDeviceManager,
        nullptr,// $qspiDeviceManager,
        nullptr,// &analogIn,
        nullptr,// &storageDriver,
        &console_over_USB, //nullptr,// &uartADriver,
        nullptr,// &gpioDriver,
        nullptr,// &rcinDriver,
        nullptr,// &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        nullptr,// &opticalFlowDriver,
        nullptr,// &flashDriver,
        nullptr, /* no DSP */
        nullptr)    // no CAN       
{}

static bool start_core_1 = false;
static bool core_1_started = false;
static bool thread_running = false;        /**< Daemon status flag */
static thread_t* daemon_task;              /**< Handle of daemon task / thread */
extern const AP_HAL::HAL& hal;

/*
  set the priority of the main APM task
 */
void hal_chibios_set_priority(uint8_t priority)
{
    chSysLock();
#if CH_CFG_USE_MUTEXES == TRUE
    if ((daemon_task->hdr.pqueue.prio == daemon_task->realprio) || (priority > daemon_task->hdr.pqueue.prio)) {
      daemon_task->hdr.pqueue.prio = priority;
    }
    daemon_task->realprio = priority;
#endif
    chSchRescheduleS();
    chSysUnlock();
}

thread_t* get_main_thread()
{
    return daemon_task;
}

void HAL_Rp2040ChibiOS::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    daemon_task = chThdGetSelfX();

    /*
      switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY);

    start_core_1 = true;
    // Wait until core1 and the USB console is initialized 
    while (!core_1_started) {
        chThdSleepMicroseconds(10);
    }

    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    hal.scheduler->init();

     /*
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
    hal_chibios_set_priority(APM_STARTUP_PRIORITY);

    schedulerInstance.hal_initialized();
    callbacks->setup();
    


#if !defined(DISABLE_WATCHDOG)
    // setup watchdog to reset if main loop stops
    if (AP_BoardConfig::watchdog_enabled()) {
        wdgStart(&WDGD1, &wdgcfg);
    }

    if (hal.util->was_watchdog_reset()) {
        INTERNAL_ERROR(AP_InternalError::error_t::watchdog_reset);
    }
#endif // DISABLE_WATCHDOG

    schedulerInstance.watchdog_pat();

    hal.scheduler->set_system_initialized();

    thread_running = true;

    /*
      switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY);

    for (;;) {
        callbacks->loop();
        /*
          give up 50 microseconds of time if the INS loop hasn't
          called delay_microseconds_boost(), to ensure low priority
          drivers get a chance to run. Calling
          delay_microseconds_boost() means we have already given up
          time from the main loop, so we don't need to do it again
          here
         */
#if !defined(HAL_DISABLE_LOOP_DELAY) && !APM_BUILD_TYPE(APM_BUILD_Replay)
        if (!schedulerInstance.check_called_boost()) {
            hal.scheduler->delay_microseconds(50);
        }
#endif
        schedulerInstance.watchdog_pat();
    }
    thread_running = false;
    start_core_1 = false;
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_Rp2040ChibiOS halRp2040ChibiOS;
    return halRp2040ChibiOS;
}

// main function for core 1 (the second rp core)
extern "C" {
    int c1_main(void);
    int c1_main(void) {
        chSysWaitSystemState(ch_sys_running);
        chInstanceObjectInit(&ch1, &ch_core1_cfg);

        /* It is alive now. */
        chSysUnlock();

        usb_initialise();

        // Wait until the scheduler is initialized on core0
        while (!start_core_1) {
            chThdSleepMicroseconds(10);
        }
        hal.serial(0)->begin(115200);
        core_1_started = true;

        Rp2040ChibiOS::Scheduler * scheduler = (Rp2040ChibiOS::Scheduler *) hal.scheduler;
        scheduler->init_core1();

        Rp2040ChibiOS::UsbCdcConsole * usbCdcConsole = (Rp2040ChibiOS::UsbCdcConsole *) hal.serial(0);
        usbCdcConsole->writeThread();
        usbCdcConsole->readThread();

        Rp2040ChibiOS::UARTDriver * uart0 = (Rp2040ChibiOS::UARTDriver *) hal.serial(3);
        uart0->writeThread();
        uart0->readThread();
        Rp2040ChibiOS::UARTDriver * uart1 = (Rp2040ChibiOS::UARTDriver *) hal.serial(5);
        uart1->writeThread();
        uart1->readThread();

        while(true) {
            chThdSleepMilliseconds(1000);
        }

        core_1_started = false;
        return 0;
    }
}

#endif
