/*
 * CrashCatcher backend for RP2xxx (RP2040 + RP2350).
 *
 * Stores a binary CrashCatcher dump in the crash_log flash region carved
 * out of the end of XIP flash.  The dump is binary (not hex), so it can
 * be fed directly to the CrashDebug GDB stub:
 *
 *   Tools/debug/gdb_crashdump.sh <elf> <crash_dump.bin>
 *
 * Design constraints
 * ------------------
 * This file runs in a HardFault context: the RTOS is dead, DMA is
 * unreliable, and — critically — we must NOT execute code from XIP flash
 * while the SSI peripheral is in raw-SPI mode.  Every function that
 * touches the flash hardware is tagged __FASTRAMFUNC__ so the linker places
 * it in .fastramfunc, which is copied to SRAM at startup.
 *
 * The ROM flash API (via rpRomGetFlashApiX) is used for all erase and
 * program operations; ROM lives at 0x00000000 and is always accessible.
 *
 * Erase strategy: one 64-KB block erase per block, re-entering XIP
 * between blocks so this code can keep executing from SRAM-cached literals.
 * Page programming (256 B) follows the same enter/exit pattern.
 *
 * Dump layout in flash
 * --------------------
 *   [0 .. size-1]          CrashCatcher binary dump
 *   [crash_log_end - 4]    uint32_t: total bytes written (size sentinel)
 *
 * The sentinel page (last 256 B of the region) is reserved; dump data
 * stops one page before the end to leave it available for the sentinel.
 */

/*
 * This file is only compiled when CRASHCATCHER is defined by the build
 * system (i.e. when AP_CRASHDUMP_ENABLED is active).  No preprocessor
 * guard is needed here — the LIBCC_CSRC makefile variable already
 * ensures crashdump.c is excluded from non-crashdump builds.
 */

#include <CrashCatcher.h>
#include <stdint.h>
#include <stddef.h>

#include "rp_bootrom.h"

/* board.h defines RP2040 or RP2350 and lives in $(BOARDDIR) which is on
 * the compiler's include search path (via ALLINC → IINCDIR).  We include
 * it explicitly here because this plain-C file has no other transitive
 * path to that define. */
#include <board.h>

#if defined(RP2350)
#include "rp2350.h"
#elif defined(RP2040)
#include "rp2040.h"
#endif

#if !defined(RP2040) && !defined(RP2350)
#error "Neither RP2040 nor RP2350 is defined — check your board.h include path"
#endif

/* __FASTRAMFUNC__ places code in .fastramfunc which is copied to SRAM.
 * Defined in AP_HAL/board/rp2xxxchibios.h for C++ files; we define it
 * here for this plain-C translation unit if not already available. */
#ifndef __FASTRAMFUNC__
#define __FASTRAMFUNC__ __attribute__((section(".fastramfunc")))
#endif

/* XIP flash is memory-mapped from this base on both RP2040 and RP2350. */
#define XIP_BASE 0x10000000U

/* Flash erase/program constants (W25Qxx QSPI flash used on Pico boards). */
#define FLASH_PAGE_SIZE       256U
#define FLASH_BLOCK_SIZE      0x10000U   /* 64 KB */
#define FLASH_BLOCK_ERASE_CMD 0xD8U

/* Linker-defined symbols for the crash log region.
 * Their *addresses* are the XIP addresses of the region boundaries. */
extern uint32_t __crash_log_base__;
extern uint32_t __crash_log_end__;

/* ------------------------------------------------------------------ */
/* State that must survive between CrashCatcher callbacks.            */
/* All in .bss so they live in SRAM.                                  */
/* ------------------------------------------------------------------ */

static rp_rom_flash_api_t s_flash_api;
static bool               s_flash_api_ready;   /* set in DumpStart after re-resolution */
static bool               s_is_bkpt_fault;

static uint8_t  s_page_buf[FLASH_PAGE_SIZE];
static uint32_t s_page_buf_used;    /* bytes currently buffered          */
static uint32_t s_write_off;        /* byte offset from crash_log base   */
static uint32_t s_total_written;    /* total bytes given to DumpMemory   */

#if defined(RP2350) || defined(RP2040)
__FASTRAMFUNC__
static void park_other_core_for_crashdump(void)
{
    /* Hold core1 in reset so it cannot execute from XIP while flash is in
     * direct mode for erase/program operations. */
    PSM->SET.FRCE_OFF = PSM_ANY_PROC1;
    while (PSM->DONE & PSM_ANY_PROC1) {
    }
}
#else
__FASTRAMFUNC__
static void park_other_core_for_crashdump(void)
{
}
#endif

/* ------------------------------------------------------------------ */
/* Public init — call once from normal context.                        */
/* Pre-resolves flash API pointers as a fast path for the crash        */
/* handler.  The handler always re-resolves anyway (see below), so     */
/* this call is optional but good practice.                            */
/* ------------------------------------------------------------------ */

void rp2xxx_crashdump_init(void)
{
    /*
     * Pre-resolve ROM flash API pointers into s_flash_api.
     * This is an optimisation only — the fault handler always re-resolves
     * at crash time via rpRomGetFlashApiX, so this call is not required
     * for correctness.  We do NOT touch the flash hardware here: calling
     * flash_exit_xip() before chSysInit() risks a LOCKUP if any HAL
     * interrupt handler (in XIP flash) fires during the erase/program
     * window.  Flash write correctness was verified separately.
     */
    rpRomGetFlashApiX(&s_flash_api);
}

/* ------------------------------------------------------------------ */
/* Accessors used by Util::last_crash_dump_size() / _ptr().           */
/* ------------------------------------------------------------------ */

uint32_t rp2xxx_crash_dump_addr(void)
{
    return (uint32_t)&__crash_log_base__;
}

uint32_t rp2xxx_crash_dump_max_size(void)
{
    return (uint32_t)&__crash_log_end__ - (uint32_t)&__crash_log_base__;
}

/* Reads the size sentinel stored in the last 4 bytes of the region. */
uint32_t rp2xxx_crash_dump_size(void)
{
    /* Use uintptr_t arithmetic to avoid the compiler treating the subtraction
     * as a negative subscript into the linker-defined symbol object. */
    uintptr_t addr = (uintptr_t)&__crash_log_end__ - sizeof(uint32_t);
    return *(const uint32_t *)addr;
}

/* ------------------------------------------------------------------ */
/* RAM-resident helpers — all tagged __FASTRAMFUNC__                   */
/* ------------------------------------------------------------------ */

/* Inline byte copy — avoids calling libc memcpy which lives in flash. */
__FASTRAMFUNC__
static void crash_memcpy(void *dst, const void *src, uint32_t n)
{
    uint8_t       *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    while (n--) {
        *d++ = *s++;
    }
}

/* Inline byte fill — avoids calling libc memset which lives in flash. */
__FASTRAMFUNC__
static void crash_memset(void *dst, uint8_t val, uint32_t n)
{
    uint8_t *d = (uint8_t *)dst;
    while (n--) {
        *d++ = val;
    }
}

/*
 * Program one 256-byte page at byte offset log_off from the start of the
 * crash log region.  Disables XIP for the duration of the write.
 */
__FASTRAMFUNC__
static void program_page(uint32_t log_off, const uint8_t *data)
{
    uint32_t flash_off = ((uint32_t)&__crash_log_base__ - XIP_BASE) + log_off;
    s_flash_api.connect_internal_flash();
    s_flash_api.flash_exit_xip();
    s_flash_api.flash_range_program(flash_off, data, FLASH_PAGE_SIZE);
    s_flash_api.flash_flush_cache();
    s_flash_api.flash_enter_cmd_xip();
}

/*
 * Erase the entire crash log region using 64-KB block erases.
 * XIP is re-enabled between blocks so this SRAM function can still
 * reach its own stack frame and local literals.
 */
__FASTRAMFUNC__
static void erase_crash_region(void)
{
    uint32_t base_off = (uint32_t)&__crash_log_base__ - XIP_BASE;
    uint32_t total    = (uint32_t)&__crash_log_end__ - (uint32_t)&__crash_log_base__;

    for (uint32_t off = 0; off < total; off += FLASH_BLOCK_SIZE) {
        s_flash_api.connect_internal_flash();
        s_flash_api.flash_exit_xip();
        s_flash_api.flash_range_erase(base_off + off,
                                      FLASH_BLOCK_SIZE,
                                      FLASH_BLOCK_SIZE,
                                      FLASH_BLOCK_ERASE_CMD);
        s_flash_api.flash_flush_cache();
        s_flash_api.flash_enter_cmd_xip();
    }
}

/*
 * Flush s_page_buf to flash if it is full.
 * If pad==true, pad the remainder with 0xFF and flush regardless.
 *
 * The last page of the region is reserved for the size sentinel, so we
 * stop writing data before that page.
 */
__FASTRAMFUNC__
static void flush_page(int pad)
{
    if (pad && s_page_buf_used > 0) {
        crash_memset(s_page_buf + s_page_buf_used,
                     0xFF,
                     FLASH_PAGE_SIZE - s_page_buf_used);
        s_page_buf_used = FLASH_PAGE_SIZE;
    }

    if (s_page_buf_used < FLASH_PAGE_SIZE) {
        return;
    }

    uint32_t region_size =
        (uint32_t)&__crash_log_end__ - (uint32_t)&__crash_log_base__;

    /* Don't write into the sentinel page (last FLASH_PAGE_SIZE bytes). */
    if (s_write_off + FLASH_PAGE_SIZE <= region_size - FLASH_PAGE_SIZE) {
        program_page(s_write_off, s_page_buf);
    }
    s_write_off    += FLASH_PAGE_SIZE;
    s_page_buf_used = 0;
}

/* ------------------------------------------------------------------ */
/* CrashCatcher required callbacks                                     */
/* ------------------------------------------------------------------ */

__FASTRAMFUNC__
void CrashCatcher_DumpStart(const CrashCatcherInfo *pInfo)
{
    s_page_buf_used = 0;
    s_write_off     = 0;
    s_total_written = 0;
    s_is_bkpt_fault = pInfo != NULL && pInfo->isBKPT;
    /*
     * Always re-resolve the ROM flash API here, even if rp2xxx_crashdump_init()
     * ran at boot. A stack overflow can corrupt the cached s_flash_api struct
     * in .bss before we reach this handler. At this point XIP is still active
     * (we have not called flash_exit_xip yet), so calling rpRomGetFlashApiX —
     * which lives in flash — is safe.
     */
    s_flash_api_ready = rpRomGetFlashApiX(&s_flash_api);
    if (!s_flash_api_ready) {
        return;
    }
    park_other_core_for_crashdump();
    erase_crash_region();
}

__FASTRAMFUNC__
const CrashCatcherMemoryRegion *CrashCatcher_GetMemoryRegions(void)
{
    /* All three RAM banks on each chip.
     * Addresses must match the MEMORY{} regions in the linker script. */
    static const CrashCatcherMemoryRegion regions[] = {
#if defined(RP2040)
        { 0x20000000U, 0x20040000U, CRASH_CATCHER_BYTE }, /* SRAM0, 256 KB */
        { 0x20040000U, 0x20041000U, CRASH_CATCHER_BYTE }, /* SRAM4,   4 KB */
        { 0x20041000U, 0x20042000U, CRASH_CATCHER_BYTE }, /* SRAM5,   4 KB */
#elif defined(RP2350)
        { 0x20000000U, 0x20080000U, CRASH_CATCHER_BYTE }, /* SRAM,  512 KB */
        { 0x20080000U, 0x20081000U, CRASH_CATCHER_BYTE }, /* SCRATCH_X 4KB */
        { 0x20081000U, 0x20082000U, CRASH_CATCHER_BYTE }, /* SCRATCH_Y 4KB */
#endif
        { 0xFFFFFFFFU, 0xFFFFFFFFU, CRASH_CATCHER_BYTE }  /* sentinel      */
    };
    return regions;
}

__FASTRAMFUNC__
void CrashCatcher_DumpMemory(const void *pvMemory,
                              CrashCatcherElementSizes elementSize,
                              size_t elementCount)
{
    if (!s_flash_api_ready) {
        return;
    }

    uint32_t byte_count = (uint32_t)elementCount;
    if (elementSize == CRASH_CATCHER_HALFWORD) {
        byte_count *= 2U;
    } else if (elementSize == CRASH_CATCHER_WORD) {
        byte_count *= 4U;
    }

    s_total_written += byte_count;

    const uint8_t *src = (const uint8_t *)pvMemory;
    while (byte_count > 0U) {
        uint32_t space = FLASH_PAGE_SIZE - s_page_buf_used;
        uint32_t chunk = (byte_count < space) ? byte_count : space;
        crash_memcpy(s_page_buf + s_page_buf_used, src, chunk);
        s_page_buf_used += chunk;
        src             += chunk;
        byte_count      -= chunk;
        flush_page(0);
    }
}

__FASTRAMFUNC__
CrashCatcherReturnCodes CrashCatcher_DumpEnd(void)
{
    if (!s_flash_api_ready) {
        return CRASH_CATCHER_EXIT;
    }
    /* Flush any partial page, padding with 0xFF. */
    flush_page(1);

    /*
     * Write the size sentinel into the last page of the crash log region.
     * That page was not touched by flush_page (reserved above).
     * Fill it with 0xFF and place the 4-byte size at the very end.
     */
    uint8_t sentinel[FLASH_PAGE_SIZE];
    crash_memset(sentinel, 0xFF, FLASH_PAGE_SIZE);
    crash_memcpy(sentinel, "CCEND___", 8);
    crash_memcpy(sentinel + FLASH_PAGE_SIZE - sizeof(uint32_t),
                 &s_total_written,
                 sizeof(uint32_t));

    uint32_t region_size =
        (uint32_t)&__crash_log_end__ - (uint32_t)&__crash_log_base__;
    program_page(region_size - FLASH_PAGE_SIZE, sentinel);

    if (s_is_bkpt_fault) {
        return CRASH_CATCHER_EXIT;
    }

    /* For a real crash, don't return to the faulting instruction after the
     * dump has been committed to flash. Stay here until the board is reset
     * or forced into BOOTSEL for extraction. */
    for (;;) {
    }
}

/*
 * Serial callbacks — not used in the flash-only path, but CrashCatcher.c
 * requires them to be linked.
 */
void CrashCatcher_putc(int c) { (void)c; }
int  CrashCatcher_getc(void)  { return 0; }
