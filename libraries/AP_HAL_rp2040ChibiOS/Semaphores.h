#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "AP_HAL_rp2040ChibiOS_Namespace.h"

class Rp2040ChibiOS::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;

    // methods within HAL_ChibiOS only
    bool check_owner(void);
    void assert_owner(void);
protected:
    // to avoid polluting the global namespace with the 'ch' variable,
    // we declare the lock as a uint32_t array, and cast inside the cpp file
    uint32_t _lock[5];
};
