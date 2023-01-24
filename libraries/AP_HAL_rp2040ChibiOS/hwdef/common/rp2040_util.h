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
#pragma once

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// allocation functions in malloc.c    
size_t mem_available(void);
void *malloc_dma(size_t size);
void *malloc_axi_sram(size_t size);
void *malloc_fastmem(size_t size);
thread_t *thread_create_alloc(size_t size, const char *name, tprio_t prio, tfunc_t pf, void *arg);

struct memory_region {
    void *address;
    uint32_t size;
    uint32_t flags;
};
#if CH_CFG_USE_HEAP == TRUE
uint8_t malloc_get_heaps(memory_heap_t **_heaps, const struct memory_region **regions);
#endif

// UTC system clock handling    
void rp2040_set_utc_usec(uint64_t time_utc_usec);
uint64_t rp2040_get_utc_usec(void);


#ifdef __cplusplus
}
#endif
