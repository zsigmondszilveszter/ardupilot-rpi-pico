#include <AP_HAL/AP_HAL.h>
#include "hal.h"

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


void setup(void)
{
    palSetLineMode(25U, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_DRIVE12);
    palSetLine(25U);
    hal.scheduler->delay(1000);
    palClearLine(25U);

    hal.console->printf("Test Pico HAL's console\n");
}


static uint32_t counter = 0;
void loop(void)
{
    hal.scheduler->delay(400);
    palToggleLine(25U);

    counter++;
    hal.console->printf("Test Pico HAL's console %lu\n", counter);
}

AP_HAL_MAIN();
