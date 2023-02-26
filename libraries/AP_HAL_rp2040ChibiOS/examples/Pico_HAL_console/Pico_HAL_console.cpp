#include <AP_HAL/AP_HAL.h>
#include "hal.h"

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


void setup(void)
{
    hal.gpio->pinMode(25U, 1);
    hal.gpio->write(25U, PAL_HIGH);
    hal.scheduler->delay(1000);
    hal.gpio->write(25U, PAL_LOW);
}


static uint32_t counter = 0;
void loop(void)
{
    hal.scheduler->delay(400);
    hal.gpio->toggle(25U);

    counter++;
    hal.console->printf("Test rp2040's HAL console %lu\n", counter);
}

AP_HAL_MAIN();
