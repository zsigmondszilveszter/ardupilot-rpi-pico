
#include "Scheduler.h"

#include <stdarg.h>

#include "pico/time.h"

using namespace RpiPico;

extern const AP_HAL::HAL& hal;

Scheduler::Scheduler()
{}

void Scheduler::init()
{}

void Scheduler::delay(uint16_t ms)
{
    sleep_ms(ms);
}

void Scheduler::delay_microseconds(uint16_t us)
{
    sleep_us(us);
}

void Scheduler::register_timer_process(AP_HAL::MemberProc k)
{}

void Scheduler::register_io_process(AP_HAL::MemberProc k)
{}

void Scheduler::register_timer_failsafe(AP_HAL::Proc, uint32_t period_us)
{}

void Scheduler::set_system_initialized()
{}

void Scheduler::reboot(bool hold_in_bootloader) {
    for(;;);
}

bool Scheduler::in_main_thread() const {
    return true;
}

void Scheduler::call_delay_cb() {
    
}
