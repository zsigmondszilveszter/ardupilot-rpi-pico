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
#include "HAL_RpiPico_Class.h"

#include "pico/time.h"

extern const AP_HAL::HAL& hal;

RpiPico::BgThread& bgthread = RpiPico::getBgThread();

static alarm_pool_t * core1_alarm_pool;
static repeating_timer_t Hz1_rt, MHz1_rt;

// 1 Hz interval timer
static bool repeating_1Hz_timer_callback(struct repeating_timer *t) {
    return true;
}
// 1 MHz interval timer
static bool repeating_1MHz_timer_callback(struct repeating_timer *t) {
    bgthread.run_MHz1_tasks();
    return true;
}

// the core1 (second core) entry point
void RpiPico::BgThreadEntryPoint(){
    // init alarm pool for core1
    core1_alarm_pool = alarm_pool_create(CORE1_ALARM_POOL_HARDWARE_ALARM_NUM, RPI_PICO_MAX_BG_TASKS);

    // 1Hz timer
    alarm_pool_add_repeating_timer_ms(core1_alarm_pool, -1000, repeating_1Hz_timer_callback, NULL, &Hz1_rt);
    // 1MHz timer
    alarm_pool_add_repeating_timer_us(core1_alarm_pool, -1, repeating_1MHz_timer_callback, NULL, &MHz1_rt);
}

RpiPico::BgThread::BgThread(){}

uint8_t RpiPico::BgThread::totalNumberOfTasks(){
    return MHz1_tasks.size() + Hz1_tasks.size();
}

bool RpiPico::BgThread::checkIfThereIsSlotLeft(){
    if (totalNumberOfTasks() >= RPI_PICO_MAX_BG_TASKS){
        hal.console->printf("Warning! Tried to add new background task, but there is no more bg task slot left.\n");
        return false;
    }
    return true;
}

bool RpiPico::BgThread::add_new_Hz1_task(BgCallable * task){
    if (checkIfThereIsSlotLeft()) {
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

bool RpiPico::BgThread::add_new_MHz1_task(BgCallable * task){
    if (checkIfThereIsSlotLeft()) {
        MHz1_tasks.push_back(task);
        return true;
    }
    return false;
}

void RpiPico::BgThread::run_MHz1_tasks() {
    for (uint8_t i=0; i<MHz1_tasks.size(); i++) {
        MHz1_tasks[i]();
    }
}
