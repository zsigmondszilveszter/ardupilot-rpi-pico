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

#include "BgThread.h"
#include "AP_HAL_RpiPico_Private.h"

#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

extern const AP_HAL::HAL& hal;

RpiPico::BgThread& bgthread = RpiPico::getBgThread();

alarm_pool_t * core1_alarm_pool_2;
alarm_pool_t * core1_alarm_pool_1;
alarm_pool_t * core1_alarm_pool_0;

// the core1 (second core) entry point
void RpiPico::BgThreadEntryPoint(){
    // the smaller the alarm pool number the bigger its priority
    
    // init alarm pool 2 for core1
    core1_alarm_pool_2 = alarm_pool_create(2, RPI_PICO_MAX_BG_TASKS);
    // init alarm pool 1 for core1
    core1_alarm_pool_1 = alarm_pool_create(1, RPI_PICO_MAX_BG_TASKS);
    // init alarm pool 0 for core1
    core1_alarm_pool_0 = alarm_pool_create(0, RPI_PICO_MAX_BG_TASKS);
    // set the core1's alarm pools in the bgThread
    bgthread.init_alarm_pools(core1_alarm_pool_2, core1_alarm_pool_1, core1_alarm_pool_0);

    // let the core0 know, the core1 and USB console is initialized
    multicore_fifo_push_blocking(true);
}


static int64_t bg_thread_timer_task(alarm_id_t id, void *user_data) {
    uint8_t index = *(uint8_t *)user_data;
    return bgthread.runTask(index);
}

RpiPico::BgThread::BgThread() 
{
    // 3 is the number of different task priorities
    sem_init(&_tasks_sem, 3, 3);
}

/**
 * must be called from core1, because core1 executes the background thread 
 * and the irq registrations bind the irq rutines to the executing core
 */
void RpiPico::BgThread::init_alarm_pools(alarm_pool_t * bg_thread_alarm_pool_2, alarm_pool_t * bg_thread_alarm_pool_1, alarm_pool_t * bg_thread_alarm_pool_0)
{
    _alarm_pool_2 = bg_thread_alarm_pool_2;
    _alarm_pool_1 = bg_thread_alarm_pool_1;
    _alarm_pool_0 = bg_thread_alarm_pool_0;
}


// priority should be between 26 - 31 (inclusiv)
BgThreadPeriodicHandler RpiPico::BgThread::add_periodic_background_task_us(uint64_t period_usec, BgCallable callback, bgTaskPriority priority) 
{
    // acquire all the 3 permits
    // sem_acquire_blocking(&_tasks_sem);
    // sem_acquire_blocking(&_tasks_sem);
    // sem_acquire_blocking(&_tasks_sem);

    BgThreadPeriodicHandler pointer_to_timer_index = nullptr;
    if (_nr_of_tasks < RPI_PICO_MAX_BG_TASKS) {
        _indicies[_nr_of_tasks] = _nr_of_tasks;
        
        _tasks[_nr_of_tasks].index = _nr_of_tasks;
        _tasks[_nr_of_tasks].priority = priority;
        _tasks[_nr_of_tasks].cb = callback;
        _tasks[_nr_of_tasks].period = period_usec;

        // the smaller the alarm pool number the bigger its priority
        alarm_pool_t * alarm_pool = _alarm_pool_2; // default is 2
        switch (priority) {
            case PR0: alarm_pool = _alarm_pool_0; break;
            case PR1: alarm_pool = _alarm_pool_1; break;
            case PR2: alarm_pool = _alarm_pool_2; break;
        }
        alarm_id_t alarm_id = alarm_pool_add_alarm_in_us(alarm_pool, period_usec, bg_thread_timer_task, (void*)&_indicies[_nr_of_tasks], true);
        if (alarm_id > 0) {
            pointer_to_timer_index = (BgThreadPeriodicHandler) &_indicies[_nr_of_tasks];
            _nr_of_tasks++;
        }
    }

    // sem_release(&_tasks_sem);
    // sem_release(&_tasks_sem);
    // sem_release(&_tasks_sem);
    return pointer_to_timer_index;
}

bool RpiPico::BgThread::adjust_periodic_background_task_us(BgThreadPeriodicHandler h, uint64_t period_usec) 
{
    // acquire all the 3 permits
    // sem_acquire_blocking(&_tasks_sem);
    // sem_acquire_blocking(&_tasks_sem);
    // sem_acquire_blocking(&_tasks_sem);

    uint8_t index = *(uint8_t *)h;
    _tasks[index].period = period_usec;

    // sem_release(&_tasks_sem);
    // sem_release(&_tasks_sem);
    // sem_release(&_tasks_sem);

    return true;
}

uint64_t RpiPico::BgThread::runTask(uint8_t index) 
{
    sem_acquire_blocking(&_tasks_sem);

    uint64_t reschedule = _tasks[index].period;
    _tasks[index].cb();

    sem_release(&_tasks_sem);
    return reschedule;
}