#include <AP_HAL/AP_HAL.h>
#include "hal.h"

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

void setup(void)
{
    hal.gpio->pinMode(25U, 1);
    hal.gpio->write(25U, PAL_HIGH);
    hal.scheduler->delay(1000);
    hal.gpio->write(25U, PAL_LOW);

    /*
      start all UARTs at 115200 with default buffer sizes
    */

    setup_uart(hal.serial(0), "SERIAL0");  // console
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

static uint8_t counter = 0;
void loop(void)
{
    test_uart(hal.serial(0), "SERIAL0");

    hal.scheduler->delay(1);
    counter++;
    if(!counter) {
        hal.gpio->toggle(25U);
    }
}

AP_HAL_MAIN();
