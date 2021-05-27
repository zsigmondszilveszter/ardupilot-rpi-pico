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

extern const AP_HAL::HAL& hal;

RpiPico::BgThread& bgthread = RpiPico::getBgThread();

// register background tasks, don't forget to add them manually to the list of background tasks
void usbConsoleTask0(void) { ((RpiPico::Console*)(hal.console))->tusb_task(); }
void usbConsoleTask1(void) { ((RpiPico::Console*)(hal.console))->flush(); }
void schedulerIOTasks(void) { ((RpiPico::Scheduler*)(hal.scheduler))->run_io(); }
void uartTask0(void) { ((RpiPico::UARTDriver*)(hal.serial(3)))->flush(); }
void uartTask1(void) { ((RpiPico::UARTDriver*)(hal.serial(3)))->async_read(); }
void uartTask2(void) { ((RpiPico::UARTDriver*)(hal.serial(1)))->flush(); }
void uartTask3(void) { ((RpiPico::UARTDriver*)(hal.serial(1)))->async_read(); }


alarm_pool_t * core1_alarm_pool;

// the core1 (second core) entry point
void RpiPico::BgThreadEntryPoint(){
    // init alarm pool for core1
    core1_alarm_pool = alarm_pool_create(CORE1_ALARM_POOL_HARDWARE_ALARM_NUM, RPI_PICO_MAX_BG_TASKS);
    // set the core1's alarm pool to the bgThread
    bgthread.init_alarm_pool(core1_alarm_pool);

    // init USB console
    ((RpiPico::Console*)(hal.console))->init();

    // let the core0 know, the core1 and USB console is initialized
    multicore_fifo_push_blocking(true);

    // add background tasks
    bgthread.add_periodic_background_task_us(-1000, (BgCallable) &usbConsoleTask0);
    bgthread.add_periodic_background_task_us(-1000, (BgCallable) &usbConsoleTask1);
    bgthread.add_periodic_background_task_us(-1000, (BgCallable) &uartTask0);
    bgthread.add_periodic_background_task_us(-1000, (BgCallable) &uartTask1);
    bgthread.add_periodic_background_task_us(-1000, (BgCallable) &uartTask2);
    bgthread.add_periodic_background_task_us(-1000, (BgCallable) &uartTask3);
}



static bool general_repeating_timer_callback(struct repeating_timer *t) {
    uint8_t index = *(uint8_t *)t->user_data;
    bgthread.runBgCallable(index);
    return true;
}

RpiPico::BgThread::BgThread() {}

void RpiPico::BgThread::init_alarm_pool(alarm_pool_t * bg_thread_alarm_pool)
{
    _alarm_pool = bg_thread_alarm_pool;
}

bool RpiPico::BgThread::add_periodic_background_task_us(int64_t period_usec, BgCallable callback) {
    _indicies[_nr_of_tasks] = _nr_of_tasks;
    _callbacks[_nr_of_tasks] = callback;
    bool retval = alarm_pool_add_repeating_timer_us(_alarm_pool, period_usec, general_repeating_timer_callback, (void*)&_indicies[_nr_of_tasks], &_timers[_nr_of_tasks]);
    if (retval) {
        _nr_of_tasks++;
    }
    return retval;
}

void RpiPico::BgThread::runBgCallable(uint8_t index) {
    _callbacks[index]();
}
