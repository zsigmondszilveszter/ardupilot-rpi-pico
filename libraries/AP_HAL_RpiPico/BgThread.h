/**
 * Ardupilot port to Raspberry Pi Pico
 * Copyright (C) 2021  Szilveszter Zsigmond
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Szilveszter Zsigmond <email@zsigmondszilveszter.website>, May 2021
 */

#pragma once

#include "AP_HAL_RpiPico.h"
#include <vector>
#include "pico/time.h"

#define CORE1_ALARM_POOL_HARDWARE_ALARM_NUM 2
// do not change, unless you know what you are doing 
// bigger than 255, requires changing some uint8_t members
#define RPI_PICO_MAX_BG_TASKS 64

typedef void (*BgCallable)(void);

void RpiPico::BgThreadEntryPoint();

// The background thread running on the Pico's second core
class RpiPico::BgThread {
public:
    BgThread();
    void init_alarm_pool(alarm_pool_t * bg_thread_alarm_pool);

    bool add_periodic_background_task_us(int64_t period_usec, BgCallable callback);

    void runBgCallable(uint8_t index);
private:
    alarm_pool_t * _alarm_pool = NULL;
    uint8_t _indicies[RPI_PICO_MAX_BG_TASKS];
    repeating_timer_t _timers[RPI_PICO_MAX_BG_TASKS];
    BgCallable _callbacks[RPI_PICO_MAX_BG_TASKS];
    uint8_t _nr_of_tasks = 0;
};
