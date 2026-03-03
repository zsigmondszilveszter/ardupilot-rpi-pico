#include <AP_HAL/AP_HAL.h>
#include "hal.h"
#include <AP_HAL_rp2040ChibiOS/hwdef/common/rp2xxx_util.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  setup one UART at 115200
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(115200);
}

static void blinkerThread1(void *arg)
{
    while (true) {
        hal.scheduler->delay(300);
        hal.gpio->toggle(25U);
    }
}
static void blinkerThread2(void *arg)
{
    while (true) {
        hal.scheduler->delay(300);
        hal.gpio->toggle(6U);
    }
}

void setup(void)
{
    hal.gpio->pinMode(25U, 1);
    hal.gpio->pinMode(6U, 1);
    hal.gpio->write(25U, PAL_HIGH);
    hal.scheduler->delay(1000);
    hal.gpio->write(25U, PAL_LOW);

    setup_uart(hal.serial(0), "SERIAL0");  // console
    
    thread_create_alloc(THD_WORKING_AREA_SIZE(128),
                                          "BLINK1",
                                          50,
                                          blinkerThread1,
                                          nullptr,
                                          &ch0);
    thread_create_alloc(THD_WORKING_AREA_SIZE(128),
                                          "BLINK2",
                                          50,
                                          blinkerThread2,
                                          nullptr,
                                          &ch1);
}


static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->printf("Hello on UART %s at %.3f seconds. Let it be a long long text to test the software FIFO buffers from Szilveszter\r\n",
                 name, (double)(AP_HAL::millis() * 0.001f));
}

void loop(void)
{
    test_uart(hal.serial(0), "SERIAL0");

    hal.scheduler->delay(1);
}

AP_HAL_MAIN();
