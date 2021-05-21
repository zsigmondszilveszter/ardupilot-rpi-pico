#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_RpiPico_Namespace.h"

typedef void BgCallable(void);

class HAL_RpiPico : public AP_HAL::HAL {
public:
    HAL_RpiPico();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};