
#include "Scheduler.h"
#include "BgThread.h"

#include <stdarg.h>

#include "pico/time.h"
#include "pico/sync.h"
#include "hardware/watchdog.h"


using namespace RpiPico;

extern const AP_HAL::HAL& hal;
RpiPico::BgThread& bgthread_pointer_scheduler = RpiPico::getBgThread();

static repeating_timer_t KHz1_rt;
static bool repeating_1KHz_timer_callback(struct repeating_timer *t) {
    ((RpiPico::Scheduler*)hal.scheduler)->run_timers();
    return true;
}

Scheduler::Scheduler()
{}

void Scheduler::init()
{
#if RPI_PICO_WATCHDOG_ENABLED
    // enable watchdog
    watchdog_enable(RPI_PICO_WATCHDOG_TIMEOUT, 1);
#endif

    // 1khz timer for timer processes, they run on the main thread (core0)
    add_repeating_timer_us(-1000, repeating_1KHz_timer_callback, NULL, &KHz1_rt);

    // 1000hz io tasks
    bgthread_pointer_scheduler.add_periodic_background_task_us(1000, FUNCTOR_BIND_MEMBER(&Scheduler::run_io, void), PR1);
}

void Scheduler::delay(uint16_t ms)
{
    sleep_ms(ms);
}

void Scheduler::delay_microseconds(uint16_t us)
{
    busy_wait_us_32(us);
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
    reboot_requested = true;
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

/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t stack_size, priority_base base, int8_t priority)
{
    // basically we only create a new background task scheduled in every 1ms 
    // bgthread_pointer_scheduler.add_periodic_background_task_us(1000, proc, PR2);
    return true;
}