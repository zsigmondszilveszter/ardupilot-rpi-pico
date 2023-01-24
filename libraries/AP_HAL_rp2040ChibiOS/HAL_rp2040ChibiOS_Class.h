#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_rp2040ChibiOS_Namespace.h"

#include "ch.h"
#include <stdio.h>

class HAL_Rp2040ChibiOS : public AP_HAL::HAL {
public:
    HAL_Rp2040ChibiOS();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};

void hal_chibios_set_priority(uint8_t priority);
thread_t* get_main_thread(void);