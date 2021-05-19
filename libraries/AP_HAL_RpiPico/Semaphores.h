#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "AP_HAL_RpiPico_Namespace.h"

class RpiPico::Semaphore : public AP_HAL::Semaphore {
public:

    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;
private:
    bool _taken;
};
