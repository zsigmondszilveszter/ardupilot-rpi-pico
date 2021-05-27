
#include "Scheduler.h"

#include <stdarg.h>

#include "pico/time.h"
#include "pico/sync.h"

using namespace RpiPico;

extern const AP_HAL::HAL& hal;

static repeating_timer_t KHz1_rt;
static bool repeating_1KHz_timer_callback(struct repeating_timer *t) {
    ((RpiPico::Scheduler*)hal.scheduler)->run_timers();
    return true;
}

Scheduler::Scheduler()
{}

void Scheduler::init()
{
    // 1khz timer for timer processes, they run on the main thread (core0)
    add_repeating_timer_us(-1000, repeating_1KHz_timer_callback, NULL, &KHz1_rt);
}

void Scheduler::delay(uint16_t ms)
{
    sleep_ms(ms);
}

void Scheduler::delay_microseconds(uint16_t us)
{
    sleep_us(us);
}

void Scheduler::register_timer_process(AP_HAL::MemberProc k)
{
    for (uint8_t i = 0; i < timerProcessPointer; i++) {
        if (timerProcesses[i] == k) {
            return;
        }
    }

    if (timerProcessPointer < RPI_PICO_MAX_TIMER_PROC) {
        timerProcesses[timerProcessPointer] = k;
        timerProcessPointer++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void Scheduler::register_io_process(AP_HAL::MemberProc k)
{
    for (uint8_t i = 0; i < ioProcessPointer; i++) {
        if (ioProcesses[i] == k) {
            return;
        }
    }

    if (ioProcessPointer < RPI_PICO_MAX_TIMER_PROC) {
        ioProcesses[ioProcessPointer] = k;
        ioProcessPointer++;
    } else {
        hal.console->printf("Out of io processes\n");
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::reboot(bool hold_in_bootloader) {
    for(;;);
}

bool Scheduler::in_main_thread() const {
    // core0 is the main thread
    return get_core_num() == 0;
}

void Scheduler::run_timers()
{
    if (inTimerProcesses) {
        return;
    }
    inTimerProcesses = true;

    // now call the timer based drivers
    for (int i = 0; i < timerProcessPointer; i++) {
        if (timerProcesses[i]) {
            timerProcesses[i]();
        }
    }
    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }
    inTimerProcesses = false;
}

void Scheduler::run_io()
{
    if (inIoProcesses) {
        return;
    }
    inIoProcesses = true;

    // now call the timer based drivers
    for (int i = 0; i < ioProcessPointer; i++) {
        if (ioProcesses[i]) {
            ioProcesses[i]();
        }
    }
    inIoProcesses = false;
}