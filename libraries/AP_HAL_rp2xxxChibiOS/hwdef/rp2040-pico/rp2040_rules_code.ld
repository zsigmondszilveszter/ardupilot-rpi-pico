/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

ENTRY(Reset_Handler)

__vectors_align__ = DEFINED(__vectors_align__) ? __vectors_align__ : 1024;

SECTIONS
{
    .vectors : ALIGN(__vectors_align__)
    {
        __textvectors_base__ = LOADADDR(.vectors);
        __vectors_base__ = .;
        KEEP(*(.vectors))
        __vectors_end__ = .;
    } > VECTORS_FLASH AT > VECTORS_FLASH_LMA

    .xtors : ALIGN(4)
    {
        __init_array_base__ = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end__ = .;
        __fini_array_base__ = .;
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        __fini_array_end__ = .;
    } > XTORS_FLASH AT > XTORS_FLASH_LMA

    .fastramfunc : ALIGN(4) SUBALIGN(4)
    {
        . = ALIGN(4);
        __ram0_init_text__ = LOADADDR(.fastramfunc);
        __ram0_init__ = .;
        EXCLUDE_FILE (*vectors.o *crt0*.o *crt1.o *board.o)
        /* Function-only hot set matched to the measured winners on rp2xxx. */
        *(.text._ZN7NavEKF312UpdateFilterEv)
        *(.rodata._ZN7NavEKF312UpdateFilterEv*)
        *(.text._ZN12NavEKF3_core12UpdateFilterEb)
        *(.rodata._ZN12NavEKF3_core12UpdateFilterEb*)
        *(.text._ZN12NavEKF3_core27UpdateStrapdownEquationsNEDEv)
        *(.text._ZN12NavEKF3_core16calcOutputStatesEv)
        *(.text._ZN12NavEKF3_core20CovariancePredictionEP7Vector3IfE)
        *(.text._ZN12NavEKF3_core15SelectMagFusionEv)
        *(.rodata._ZN12NavEKF3_core15SelectMagFusionEv*)
        *(.text._ZN12NavEKF3_core11readIMUDataEb)
        *(.text._ZN12NavEKF3_core18SelectVelPosFusionEv)
        *(.text._ZN24AC_AttitudeControl_Multi19rate_controller_runEv)
        *(.text._ZN20AP_MotorsMulticopter6outputEv)
        *(.text._ZN6Copter19run_rate_controllerEv)
        *(.text._ZN6Copter18update_flight_modeEv)
        *(.text._ZN6Copter13motors_outputEv)
        /* Small ChibiOS/HAL subset on the likely sensor-path hot route. */
        *libch.a:hrt.*(.text* .rodata*)
        *libch.a:hal_st.*(.text* .rodata*)
        *libch.a:hal_st_lld.*(.text* .rodata*)
        *libch.a:nvic.*(.text* .rodata*)
        *libch.a:chschd.*(.text* .rodata*)
        *libch.a:chsem.*(.text* .rodata*)
        *libch.a:chsys.*(.text* .rodata*)
        *(.fastramfunc)
        *(.fastramfunc.*)
        *(.ramfunc)
        *(.ramfunc.*)
        KEEP(*(.ram0_init))
        KEEP(*(.ram0_init.*))
        . = ALIGN(4);
        __ram0_clear__ = .;
        __ram0_noinit__ = .;
        __ram0_free__ = .;
    } > ram0 AT > RAM_INIT_FLASH_LMA

    .text : ALIGN(4) SUBALIGN(4)
    {
        KEEP(*(.apsec_data));
        KEEP(*(.app_descriptor));
        __text_base__ = .;
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
        __text_end__ = .;
    } > TEXT_FLASH AT > TEXT_FLASH_LMA

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .ARM.exidx : {
        __exidx_base__ = .;
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end__ = .;
        __exidx_end = .;
     } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA
}
