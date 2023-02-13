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
    palSetLineMode(25U, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_DRIVE12);

    palSetLine(25U);
    hal.scheduler->delay(1000);
    palClearLine(25U);

    /*
      start all UARTs at 115200 with default buffer sizes
    */

    setup_uart(hal.serial(0), "SERIAL0");  // console
    setup_uart(hal.serial(1), "SERIAL1");  // telemetry 1
    setup_uart(hal.serial(2), "SERIAL2");  // telemetry 2
    setup_uart(hal.serial(3), "SERIAL3");  // 1st GPS
    setup_uart(hal.serial(4), "SERIAL4");  // 2nd GPS
    setup_uart(hal.serial(5), "SERIAL5");  // RCin
}


static void read_uart(AP_HAL::UARTDriver *uart, const char *name) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uint8_t buffer[130];
    uint16_t available = (uint16_t) uart->available();
    if (available > 0) {
        uart->read(buffer, available);
        buffer[available] = '\r';
        buffer[available] = '\n';
        buffer[available] = 0;
        uart->write((const char *)buffer);
    }
}

void loop(void)
{
    read_uart(hal.serial(0), "SERIAL0");
    read_uart(hal.serial(1), "SERIAL1");
    read_uart(hal.serial(2), "SERIAL2");
    read_uart(hal.serial(3), "SERIAL3");
    read_uart(hal.serial(4), "SERIAL4");
    read_uart(hal.serial(5), "SERIAL5");

    palToggleLine(25U);
}

AP_HAL_MAIN();
