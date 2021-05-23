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

#define CORE1_ALARM_POOL_HARDWARE_ALARM_NUM 2
// do not change, unless you know what you are doing 
// bigger than 255, requires changing some uint8_t members
#define RPI_PICO_MAX_BG_TASKS 32


void RpiPico::BgThreadEntryPoint();

// The background thread running on the Pico's second core
class RpiPico::BgThread {
public:
    BgThread();

    bool add_new_Hz1_task(BgCallable * task);
    void run_Hz1_tasks();
    bool add_new_KHz1_task(BgCallable * task);
    void run_KHz1_tasks();
    bool add_new_KHz10_task(BgCallable * task);
    void run_KHz10_tasks();

private:
    std::vector<BgCallable*> Hz1_tasks;
    std::vector<BgCallable*> KHz1_tasks;
    std::vector<BgCallable*> KHz10_tasks;

    uint8_t totalNumberOfTasks();
    bool checkIfThereIsAnySlotLeft();
};
