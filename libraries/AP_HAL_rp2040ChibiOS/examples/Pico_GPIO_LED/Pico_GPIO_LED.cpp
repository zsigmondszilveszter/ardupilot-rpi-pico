#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void)
{
    hal.gpio->pinMode(25U, 1);
    hal.gpio->pinMode(6U, 1);
    hal.gpio->pinMode(9U, 1);
    hal.scheduler->delay(1500);
}

void loop(void)
{
    hal.gpio->toggle(25U);
    hal.scheduler->delay(300);
    hal.gpio->toggle(6U);
    hal.scheduler->delay(300);
    hal.gpio->toggle(9U);
    hal.scheduler->delay(300);
}

AP_HAL_MAIN();
