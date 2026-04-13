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

/*
 * RP2350 memory setup for XIP
 */

/*
 * Crash log region: 576 KB reserved at the end of XIP flash for the
 * CrashCatcher binary dump (RP2350 has 520 KB of RAM to dump + overhead).
 * 576 KB = 9 x 64 KB erase blocks, keeping erase time reasonable (~2 s).
 */
CRASH_LOG_LEN = 0x90000;

MEMORY
{
    flash0 (rx) : org = 0x00000000, len = 16k   /* ROM                  */
    flash1 (rx) : org = 0x10000000, len = (DEFINED(FLASH_LEN) ? FLASH_LEN : 4096k) - CRASH_LOG_LEN /* XIP */
    flash2 (rx) : org = 0x00000000, len = 0
    flash3 (rx) : org = 0x00000000, len = 0
    flash4 (rx) : org = 0x00000000, len = 0
    flash5 (rx) : org = 0x00000000, len = 0
    flash6 (rx) : org = 0x00000000, len = 0
    flash7 (rx) : org = 0x00000000, len = 0
    crash_log (r) : org = 0x10000000 + (DEFINED(FLASH_LEN) ? FLASH_LEN : 4096k) - CRASH_LOG_LEN, len = CRASH_LOG_LEN
    ram0   (wx) : org = 0x20000000, len = 512k  /* SRAM                 */
    ram1   (wx) : org = 0x00000000, len = 0
    ram2   (wx) : org = 0x00000000, len = 0
    ram3   (wx) : org = 0x00000000, len = 0
    ram4   (wx) : org = 0x20080000, len = 4k    /* SCRATCH_X            */
    ram5   (wx) : org = 0x20081000, len = 4k    /* SCRATCH_Y            */
    ram6   (wx) : org = 0x00000000, len = 0
    ram7   (wx) : org = 0x00000000, len = 0
}

REGION_ALIAS("VECTORS_FLASH", flash1);
REGION_ALIAS("VECTORS_FLASH_LMA", flash1);
REGION_ALIAS("XTORS_FLASH", flash1);
REGION_ALIAS("XTORS_FLASH_LMA", flash1);
REGION_ALIAS("TEXT_FLASH", flash1);
REGION_ALIAS("TEXT_FLASH_LMA", flash1);
REGION_ALIAS("RODATA_FLASH", flash1);
REGION_ALIAS("RODATA_FLASH_LMA", flash1);
REGION_ALIAS("VARIOUS_FLASH", flash1);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash1);
REGION_ALIAS("RAM_INIT_FLASH_LMA", flash1);
REGION_ALIAS("MAIN_STACK_RAM", ram4);
REGION_ALIAS("PROCESS_STACK_RAM", ram4);
REGION_ALIAS("C1_MAIN_STACK_RAM", ram5);
REGION_ALIAS("C1_PROCESS_STACK_RAM", ram5);
REGION_ALIAS("DATA_RAM", ram0);
REGION_ALIAS("DATA_RAM_LMA", flash1);
REGION_ALIAS("BSS_RAM", ram0);
REGION_ALIAS("HEAP_RAM", ram0);

SECTIONS
{
   .flash_begin : {
      __flash_binary_start = .;
   } > flash1

   /* Must be within first 4KB for bootrom detection */
   .embedded_block : ALIGN(4)
   {
      KEEP(*(.embedded_block))
   } > flash1
}

INCLUDE rules_stacks.ld
INCLUDE rules_stacks_c1.ld
INCLUDE rp2350_rules_code.ld
INCLUDE rules_data.ld
INCLUDE ../common/rp2xxx_rules_memory.ld

SECTIONS
{
   .flash_end : {
      __flash_binary_end = .;
   } > flash1
}
