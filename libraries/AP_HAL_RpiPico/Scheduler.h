#pragma once

#include "AP_HAL_RpiPico.h"

class RpiPico::Scheduler : public AP_HAL::Scheduler {
public:
    Scheduler();
    void     init() override;
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;

    void     set_system_initialized() override { _system_initialized = true; }
    bool     is_system_initialized() override { return _system_initialized; }

    void     reboot(bool hold_in_bootloader) override;

    bool     in_main_thread() const override;

    void     run_timers();
    void     run_io();

    bool     thread_create(AP_HAL::MemberProc, const char *name, uint32_t stack_size, priority_base base, int8_t priority) override;

private:
    bool _system_initialized = false;
    uint8_t timerProcessPointer = 0;
    AP_HAL::MemberProc timerProcesses[RPI_PICO_MAX_TIMER_PROC];
    uint8_t ioProcessPointer = 0;
    AP_HAL::MemberProc ioProcesses[RPI_PICO_MAX_IO_PROC];
    AP_HAL::Proc _failsafe;

    bool inTimerProcesses = false;
    bool inIoProcesses = false;
    bool reboot_requested = false;
};