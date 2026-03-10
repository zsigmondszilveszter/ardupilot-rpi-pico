/*
 * RP2040 ROM floating-point table initialisation.
 * SPDX-License-Identifier: BSD-3-Clause (matching original pico-sdk)
 *
 * Derived from pico-sdk: src/rp2_common/pico_float/float_init_rom_rp2040.c
 * Rewritten to have no pico-sdk header dependencies.
 *
 * Call __aeabi_float_init() early in board startup (from boardInit() in board.c)
 * before any floating-point operations are executed.
 */

#include <string.h>
#include <stdint.h>

/* ROM addresses are intentional fixed-address reads — suppress bounds checker */
#pragma GCC diagnostic ignored "-Warray-bounds"

/*
 * The software-float function pointer table used by float_aeabi_rp2040.S.
 * Size = SF_TABLE_V2_SIZE / sizeof(uint32_t) = 0x80 / 4 = 32 entries.
 */
uint32_t sf_table[32];

/* CLZ32 ROM function pointer, used by the int-to-float path in float_aeabi_rp2040.S */
void __attribute__((weak)) *sf_clz_func;

/*
 * Minimal RP2040 boot ROM access (RP2040 datasheet §2.8).
 *
 * ROM memory map (fixed addresses):
 *   0x13        : ROM version byte (1 = B0 silicon, 2 = B1+)
 *   0x14        : 16-bit pointer to ROM function table
 *   0x16        : 16-bit pointer to ROM data table
 *   0x18        : 16-bit pointer to ROM table lookup helper function
 *
 * The lookup helper signature: uint16_t lookup(uint16_t *table, uint32_t code)
 * Returns the 16-bit (thumb) address of the matching entry, or 0 if not found.
 */
typedef uint16_t (*_rom_lookup_fn)(uint16_t *table, uint32_t code);

#define _rom_hword_ptr(addr)  ((void *)(uintptr_t)(*(volatile uint16_t *)(uintptr_t)(addr)))

static inline void *_rom_func_lookup(uint32_t code)
{
    _rom_lookup_fn lookup = (_rom_lookup_fn)_rom_hword_ptr(0x18);
    uint16_t *func_table  = (uint16_t *)_rom_hword_ptr(0x14);
    return (void *)(uintptr_t)lookup(func_table, code);
}

static inline void *_rom_data_lookup(uint32_t code)
{
    _rom_lookup_fn lookup = (_rom_lookup_fn)_rom_hword_ptr(0x18);
    uint16_t *data_table  = (uint16_t *)_rom_hword_ptr(0x16);
    return (void *)(uintptr_t)lookup(data_table, code);
}

/* ROM table entry codes: two ASCII chars packed as (c1 | c2<<8) */
#define ROM_CODE(c1, c2)  ((uint32_t)(c1) | ((uint32_t)(c2) << 8))
#define ROM_CODE_SF        ROM_CODE('S', 'F')   /* single-float table    */
#define ROM_CODE_SD        ROM_CODE('S', 'D')   /* double-float table    */
#define ROM_CODE_CLZ32     ROM_CODE('L', '3')   /* count-leading-zeros32 */

#define SF_TABLE_V2_BYTES  0x80

/*
 * ROM version byte at 0x13 (per RP2040 datasheet §2.8 and pico-sdk):
 *   1 = B0 silicon (ROM v1: float V1 table only, no double table)
 *   2 = B1 silicon (ROM v2: full float + double V2 tables)
 */
#define ROM_VERSION_ADDR  0x13u
#define SF_TABLE_V1_BYTES 0x54  /* SF_TABLE_V1_SIZE */

/* Forward declarations of the on-use shim helpers defined in the shim .S files.
 * On B0, sf_table / sd_table slots that have no ROM backing are filled with
 * these helpers.  On first call the helper patches the slot with the real
 * software shim and tail-calls it, so subsequent calls go direct. */
extern void float_table_shim_on_use_helper(void);
extern void double_table_shim_on_use_helper(void);

void __aeabi_float_init(void)
{
    uint8_t rom_version = *(volatile uint8_t *)ROM_VERSION_ADDR;
    void *rom_sf_table = _rom_data_lookup(ROM_CODE_SF);

    if (rom_version == 1) {
        /* B0: ROM only has the V1 float table (SF_TABLE_V1_SIZE = 0x54 bytes).
         * Copy V1 entries from ROM; fill V2-only slots (0x54–0x7C) with the
         * on-use helper so the first call patches each slot with a software
         * fallback (defined in float_v1_rom_shim_rp2040.S). */
        memcpy(sf_table, rom_sf_table, SF_TABLE_V1_BYTES);
        for (int i = SF_TABLE_V1_BYTES / 4; i < SF_TABLE_V2_BYTES / 4; i++) {
            sf_table[i] = (uint32_t)(uintptr_t)float_table_shim_on_use_helper;
        }
    } else {
        /* B1+: ROM has the full V2 float table. */
        memcpy(sf_table, rom_sf_table, SF_TABLE_V2_BYTES);
    }

    sf_clz_func = _rom_func_lookup(ROM_CODE_CLZ32);
}

/*
 * Double-precision ROM function pointer table used by double_aeabi_rp2040.S.
 * The SD (software-double) table has the same index layout as the SF table.
 * Size: SF_TABLE_V2_BYTES / sizeof(uint32_t) = 32 entries.
 */
uint32_t sd_table[32];

void __aeabi_double_init(void)
{
    uint8_t rom_version = *(volatile uint8_t *)ROM_VERSION_ADDR;

    if (rom_version == 1) {
        /* B0: ROM has NO double support at all.
         * Fill every sd_table slot with the on-use helper; each slot is patched
         * with a full-software fallback (double_v1_rom_shim_rp2040.S) on first
         * call.  sf_clz_func is already set by __aeabi_float_init(). */
        for (int i = 0; i < SF_TABLE_V2_BYTES / 4; i++) {
            sd_table[i] = (uint32_t)(uintptr_t)double_table_shim_on_use_helper;
        }
    } else {
        /* B1+: copy full V2 SD table from ROM.  sf_clz_func is already set by
         * __aeabi_float_init() (called first in __late_init) and is shared. */
        void *rom_sd_table = _rom_data_lookup(ROM_CODE_SD);
        memcpy(sd_table, rom_sd_table, SF_TABLE_V2_BYTES);
    }
}
