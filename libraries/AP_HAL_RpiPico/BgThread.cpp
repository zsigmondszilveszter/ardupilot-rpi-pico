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
void usbConsoleTask0(void) { ((RpiPico::Console*)(hal.console))->flush(); }
void usbConsoleTask1(void) { ((RpiPico::Console*)(hal.console))->tusb_task(); }
void uartTask0(void) { ((RpiPico::UARTDriver*)(hal.serial(3)))->flush(); }
void uartTask1(void) { ((RpiPico::UARTDriver*)(hal.serial(3)))->async_read(); }
void uartTask2(void) { ((RpiPico::UARTDriver*)(hal.serial(1)))->flush(); }
void uartTask3(void) { ((RpiPico::UARTDriver*)(hal.serial(1)))->async_read(); }


static alarm_pool_t * core1_alarm_pool;
static repeating_timer_t Hz1_rt, KHz10_rt;

// 1 Hz interval timer callback
static bool repeating_1Hz_timer_callback(struct repeating_timer *t) {
    // TODO, maybe a LED blinker
    return true;
}
// 10 KHz interval timer
static bool repeating_10KHz_timer_callback(struct repeating_timer *t) {
    bgthread.run_KHz10_tasks();
    return true;
}

// the core1 (second core) entry point
void RpiPico::BgThreadEntryPoint(){
    // init alarm pool for core1
    core1_alarm_pool = alarm_pool_create(CORE1_ALARM_POOL_HARDWARE_ALARM_NUM, RPI_PICO_MAX_BG_TASKS);

    // 1Hz timer
    alarm_pool_add_repeating_timer_ms(core1_alarm_pool, -1000, repeating_1Hz_timer_callback, NULL, &Hz1_rt);
    // 10KHz timer
    alarm_pool_add_repeating_timer_us(core1_alarm_pool, -100, repeating_10KHz_timer_callback, NULL, &KHz10_rt);

    // init USB console
    ((RpiPico::Console*)(hal.console))->init();
    // let the core0 know, the core1 and USB console is initialized
    multicore_fifo_push_blocking(true);

    // add background tasks
    bgthread.add_new_KHz10_task((BgCallable*) &usbConsoleTask0);
    bgthread.add_new_KHz10_task((BgCallable*) &usbConsoleTask1);
    bgthread.add_new_KHz10_task((BgCallable*) &uartTask0);
    bgthread.add_new_KHz10_task((BgCallable*) &uartTask1);
    bgthread.add_new_KHz10_task((BgCallable*) &uartTask2);
    bgthread.add_new_KHz10_task((BgCallable*) &uartTask3);
}



RpiPico::BgThread::BgThread(){}

uint8_t RpiPico::BgThread::totalNumberOfTasks(){
    return KHz10_tasks.size() + Hz1_tasks.size();
}

bool RpiPico::BgThread::checkIfThereIsAnySlotLeft(){
    if (totalNumberOfTasks() >= RPI_PICO_MAX_BG_TASKS){
        // hal.console->printf("Warning! Tried to add new background task, but there is no more bg task slot left.\n");
        return false;
    }
    return true;
}

bool RpiPico::BgThread::add_new_Hz1_task(BgCallable * task){
    if (checkIfThereIsAnySlotLeft()) {
        Hz1_tasks.push_back(task);
        return true;
    }
    return false;
}

void RpiPico::BgThread::run_Hz1_tasks() {
    for (uint8_t i=0; i<Hz1_tasks.size(); i++) {
        Hz1_tasks[i]();
    }
}

bool RpiPico::BgThread::add_new_KHz10_task(BgCallable * task){
    if (checkIfThereIsAnySlotLeft()) {
        KHz10_tasks.push_back(task);
        return true;
    }
    return false;
}

void RpiPico::BgThread::run_KHz10_tasks() {
    for (uint8_t i=0; i<KHz10_tasks.size(); i++) {
        KHz10_tasks[i]();
    }
}
