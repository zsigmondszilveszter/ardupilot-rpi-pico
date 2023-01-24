#include <AP_HAL/AP_HAL.h>
#include "hal.h"

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void)
{
    palSetLineMode(25U, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_DRIVE12);
    palSetLine(25U);
    hal.scheduler->delay(1500);
    palClearLine(25U);
}

void loop(void)
{
    hal.scheduler->delay(300);
    palToggleLine(25U);
}

AP_HAL_MAIN();
