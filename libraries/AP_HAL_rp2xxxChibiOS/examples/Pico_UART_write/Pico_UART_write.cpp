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
    hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
    hal.scheduler->delay(1000);
    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);

    /*
      start all UARTs at 115200 with default buffer sizes
    */

    setup_uart(hal.serial(0), "SERIAL0");  // console
    setup_uart(hal.serial(1), "SERIAL1");  // telemetry 1
    setup_uart(hal.serial(2), "SERIAL2");  // telemetry 2
    setup_uart(hal.serial(3), "SERIAL3");  // 1st GPS
    setup_uart(hal.serial(4), "SERIAL4");  // 2nd GPS
    setup_uart(hal.serial(5), "SERIAL5");  // RCin
    setup_uart(hal.console, "Debug Console");
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
    test_uart(hal.serial(1), "SERIAL1");
    test_uart(hal.serial(2), "SERIAL2");
    test_uart(hal.serial(3), "SERIAL3");
    test_uart(hal.serial(4), "SERIAL4");
    test_uart(hal.serial(5), "SERIAL5");
    test_uart(hal.console, "Debug Console");

    hal.gpio->toggle(HAL_GPIO_A_LED_PIN);
    hal.scheduler->delay(300);
    hal.gpio->toggle(HAL_GPIO_A_LED_PIN);
    hal.scheduler->delay(700);
}

AP_HAL_MAIN();
