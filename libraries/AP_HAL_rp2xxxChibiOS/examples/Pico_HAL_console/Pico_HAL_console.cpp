#include <AP_HAL/AP_HAL.h>
#include "hal.h"

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


void setup(void)
{
    hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
    hal.scheduler->delay(1000);
    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
}


static uint32_t counter = 0;
void loop(void)
{
    hal.scheduler->delay(400);
    hal.gpio->toggle(HAL_GPIO_A_LED_PIN);

    counter++;
    hal.console->printf("Test rp2xxx's HAL console %lu\n", counter);
}

AP_HAL_MAIN();
