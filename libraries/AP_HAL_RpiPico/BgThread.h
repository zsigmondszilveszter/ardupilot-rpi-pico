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
#include "AP_HAL/utility/functor.h"
#include "pico/time.h"
#include "pico/sem.h"

FUNCTOR_TYPEDEF(BgCallable, void);
typedef uint8_t* BgThreadPeriodicHandler;

enum bgTaskPriority{
    PR0 = 0,
    PR1 = 1,
    PR2 = 2
};

typedef struct bgTask {
    uint8_t index;
    bgTaskPriority priority;
    BgCallable cb;
    uint64_t period;
};

void RpiPico::BgThreadEntryPoint();

// The background thread running on the Pico's second core
class RpiPico::BgThread {
public:
    BgThread();

    /**
     * must be called from core1, because core1 executes the background thread 
     * and the irq registrations bind the irq rutines to the executing core
     */
    void init_alarm_pools(alarm_pool_t * bg_thread_alarm_pool_2, alarm_pool_t * bg_thread_alarm_pool_1, alarm_pool_t * bg_thread_alarm_pool_0);

    BgThreadPeriodicHandler add_periodic_background_task_us(uint64_t period_usec, BgCallable callback, bgTaskPriority priority);
    bool adjust_periodic_background_task_us(BgThreadPeriodicHandler h, uint64_t period_usec);

    uint64_t runTask(uint8_t index);
    bool addTaskToQueue(bgTaskPriority toPriorityQueue, BgCallable task);
private:
    alarm_pool_t * _alarm_pool_2 = NULL;
    alarm_pool_t * _alarm_pool_1 = NULL;
    alarm_pool_t * _alarm_pool_0 = NULL;

    uint8_t _indicies[RPI_PICO_MAX_BG_TASKS];
    uint8_t _nr_of_tasks = 0;

    bgTask _tasks[RPI_PICO_MAX_BG_TASKS];
    semaphore_t _tasks_sem;
};
