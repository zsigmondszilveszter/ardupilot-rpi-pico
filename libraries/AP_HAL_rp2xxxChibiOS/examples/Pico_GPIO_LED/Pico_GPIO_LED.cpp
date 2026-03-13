#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void)
{
    hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_B_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_C_LED_PIN, HAL_GPIO_OUTPUT);
    hal.scheduler->delay(1500);
}

void loop(void)
{
    hal.gpio->toggle(HAL_GPIO_A_LED_PIN);
    hal.scheduler->delay(300);
    hal.gpio->toggle(HAL_GPIO_B_LED_PIN);
    hal.scheduler->delay(300);
    hal.gpio->toggle(HAL_GPIO_C_LED_PIN);
    hal.scheduler->delay(300);
}

AP_HAL_MAIN();
