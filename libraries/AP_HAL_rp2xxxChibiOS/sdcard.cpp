/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "sdcard.h"
#include <hal.h>

#if HAL_USE_MMC_SPI == TRUE
#include <ff.h>
#include "SPIDevice.h"
#include "hwdef/common/spi_hook.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/board/rp2xxxchibios.h>

extern const AP_HAL::HAL& hal;

static FATFS SDC_FS;
static HAL_Semaphore sem;
static bool sdcard_running;

MMCDriver MMCD1;
static AP_HAL::OwnPtr<AP_HAL::SPIDevice> device;
static MMCConfig mmcconfig;
static SPIConfig lowspeed;   // sentinel only — speed is set via set_speed()
static SPIConfig highspeed;  // sentinel only

bool sdcard_init()
{
    WITH_SEMAPHORE(sem);

    // RP2040 has no BRD_SD_SLOWDOWN parameter; slowdown is always 0
    const uint8_t sd_slowdown = 0;

    if (sdcard_running) {
        sdcard_stop();
    }

    device = AP_HAL::get_HAL().spi->get_device("sdcard");
    if (!device) {
        printf("No sdcard SPI device found\n");
        return false;
    }
    printf("sdcard_init: got SPI device for sdcard\n");
    device->set_slowdown(sd_slowdown);

    if (MMCD1.buffer == nullptr) {
        MMCD1.buffer = (uint8_t*)malloc(MMC_BUFFER_SIZE);
    }
    mmcObjectInit(&MMCD1, MMCD1.buffer);

    mmcconfig.spip  = static_cast<Rp2xxxChibiOS::SPIDevice*>(device.get())->get_driver();
    mmcconfig.lscfg = &lowspeed;
    mmcconfig.hscfg = &highspeed;

    // set before mmcStart so hooks can use device
    sdcard_running = true;

    const uint8_t tries = 3;
    for (uint8_t i = 0; i < tries; i++) {
        printf("sdcard_init: attempt %u/%u starting mmc\n",
               (unsigned)(i + 1), (unsigned)tries);
        mmcStart(&MMCD1, &mmcconfig);
        const bool connect_ok = (mmcConnect(&MMCD1) == HAL_SUCCESS);
        printf("sdcard_init: attempt %u/%u mmcConnect=%s\n",
               (unsigned)(i + 1), (unsigned)tries,
               connect_ok ? "OK" : "FAIL");
        if (!connect_ok) {
            mmcStop(&MMCD1);
            continue;
        }
        const FRESULT mount_res = f_mount(&SDC_FS, "/", 1);
        printf("sdcard_init: attempt %u/%u f_mount=%d\n",
               (unsigned)(i + 1), (unsigned)tries, (int)mount_res);
        if (mount_res != FR_OK) {
            mmcDisconnect(&MMCD1);
            mmcStop(&MMCD1);
            continue;
        }
        printf("Successfully mounted SDCard (slowdown=%u)\n", (unsigned)sd_slowdown);
        return true;
    }

    printf("sdcard_init: failed after %u attempts\n", (unsigned)tries);
    sdcard_running = false;
    return false;
}

void sdcard_stop()
{
    f_mount(nullptr, "/", 1);
    if (sdcard_running) {
        mmcDisconnect(&MMCD1);
        mmcStop(&MMCD1);
        sdcard_running = false;
    }
}

bool sdcard_retry()
{
    if (!sdcard_running) {
        if (sdcard_init()) {
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
            AP::FS().mkdir("/APM");
#endif
        }
    }
    return sdcard_running;
}

// ---- SPI Hooks ----
// Called by hal_mmc_spi.c which has spiXxx #define'd to spiXxxHook.
// In this file, spiXxx refers to the real ChibiOS functions (no redefinition here).
//
// IMPORTANT: mmcConnect() calls spiStart/spiIgnore/spiSelect/spiSend/spiReceive/
// spiUnselect WITHOUT a surrounding spiAcquireBus/spiReleaseBus pair.
// Only mmc_read()/mmc_write()/mmc_sync() use spiAcquireBus/ReleaseBus.
//
// Hook semaphore strategy:
//   spiAcquireBusHook – take semaphore; set sdcard_sem_held=true
//   spiReleaseBusHook – give semaphore; clear sdcard_sem_held
//   spiSelectHook     – if !sdcard_sem_held: take semaphore
//                       then acquire_bus(true,true) + CS low
//   spiUnselectHook   – CS high + acquire_bus(false,true)
//                       then if !sdcard_sem_held: give semaphore
// This way transfer() always finds check_owner()==true.

static bool sdcard_sem_held;  // true while spiAcquireBusHook holds the semaphore

void spiStartHook(SPIDriver *spip, const SPIConfig *config)
{
    if (!sdcard_running) {
        return;
    }
    // Update speed flag; the SPI will be (re)started at next acquire_bus call.
    device->set_speed(config == &lowspeed ?
        AP_HAL::Device::SPEED_LOW : AP_HAL::Device::SPEED_HIGH);
}

void spiStopHook(SPIDriver *spip)
{
}

// Called only from mmc_read/mmc_write/mmc_sync (NOT from mmcConnect).
void spiAcquireBusHook(SPIDriver *spip)
{
    if (!sdcard_running) {
        return;
    }
    device->get_semaphore()->take_blocking();
    sdcard_sem_held = true;
}

// Called only from mmc_read/mmc_write/mmc_sync (NOT from mmcConnect).
void spiReleaseBusHook(SPIDriver *spip)
{
    if (!sdcard_running) {
        return;
    }
    sdcard_sem_held = false;
    device->get_semaphore()->give();
}

// Called both from mmcConnect (no surrounding Acquire/Release) and from
// mmc_read/write (with surrounding Acquire/Release).
void spiSelectHook(SPIDriver *spip)
{
    if (!sdcard_running) {
        return;
    }
    if (!sdcard_sem_held) {
        // mmcConnect path: semaphore not yet taken — take it now.
        device->get_semaphore()->take_blocking();
    }
    // acquire_bus(true, true): takes ChibiOS bus lock, (re)starts SPI hw at
    // current freq_flag, sets cs_forced=true without asserting CS GPIO.
    static_cast<Rp2xxxChibiOS::SPIDevice*>(device.get())->acquire_bus(true, true);
    hal.gpio->write(RP2xxx_SPI_CS_FOR_SDCARD, 0);
}

void spiUnselectHook(SPIDriver *spip)
{
    if (!sdcard_running) {
        return;
    }
    hal.gpio->write(RP2xxx_SPI_CS_FOR_SDCARD, 1);
    // acquire_bus(false, true): releases ChibiOS bus lock, clears cs_forced.
    static_cast<Rp2xxxChibiOS::SPIDevice*>(device.get())->acquire_bus(false, true);
    if (!sdcard_sem_held) {
        // mmcConnect path: we took the semaphore in spiSelectHook — give it back.
        device->get_semaphore()->give();
    }
}

void spiIgnoreHook(SPIDriver *spip, size_t n)
{
    if (!sdcard_running) {
        return;
    }
    // clock_pulse() handles semaphore internally:
    //   cs_forced=false (before any Select): takes/gives semaphore around ignore.
    //   cs_forced=true  (between Select/Unselect): checks owner, runs ignore.
    device->clock_pulse(n);
}

void spiSendHook(SPIDriver *spip, size_t n, const void *txbuf)
{
    if (!sdcard_running) {
        return;
    }
    // semaphore held, cs_forced=true → acquire_bus is a no-op → DMA transfer
    device->transfer((const uint8_t*)txbuf, n, nullptr, 0);
}

void spiReceiveHook(SPIDriver *spip, size_t n, void *rxbuf)
{
    if (!sdcard_running) {
        return;
    }
    device->transfer(nullptr, 0, (uint8_t*)rxbuf, n);
}

#endif // HAL_USE_MMC_SPI
