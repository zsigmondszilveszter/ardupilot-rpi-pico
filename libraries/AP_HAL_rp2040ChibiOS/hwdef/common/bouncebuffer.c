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
/*
  bouncebuffer code for DMA safe memory operations
 */
#include "rp2040_util.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "bouncebuffer.h"

// this checks an address is in main memory and 16 bit aligned
#define IS_DMA_SAFE(addr) ((((uint32_t)(addr)) & 0xF0000001) == 0x20000000)

// Enable when trying to check if you are not just listening yourself
#define ENABLE_ECHO_SAFE 0

/*
  initialise a bouncebuffer
 */
void bouncebuffer_init(struct bouncebuffer_t **bouncebuffer, uint32_t prealloc_bytes)
{
    (*bouncebuffer) = calloc(1, sizeof(struct bouncebuffer_t));
    osalDbgAssert(((*bouncebuffer) != NULL), "bouncebuffer init");
    (*bouncebuffer)->on_axi_sram = false;
    if (prealloc_bytes) {
        (*bouncebuffer)->dma_buf = malloc_dma(prealloc_bytes);
        if ((*bouncebuffer)->dma_buf) {
            (*bouncebuffer)->size = prealloc_bytes;
        }
    }
}

/*
  setup for reading from a device into memory, allocating a bouncebuffer if needed
  Note that *buf can be NULL, in which case we allocate DMA capable memory, but don't
  copy to it in bouncebuffer_finish_read(). This avoids DMA failures in dummyrx in the SPI LLD
 */
bool bouncebuffer_setup_read(struct bouncebuffer_t *bouncebuffer, uint8_t **buf, uint32_t size)
{
    if (!bouncebuffer || IS_DMA_SAFE(*buf)) {
        // nothing needs to be done
        return true;
    }
    osalDbgAssert((bouncebuffer->busy == false), "bouncebuffer read");        
    bouncebuffer->orig_buf = *buf;
    if (bouncebuffer->size < size) {
        if (bouncebuffer->size > 0) {
            free(bouncebuffer->dma_buf);
        }
        bouncebuffer->dma_buf = malloc_dma(size);
        if (!bouncebuffer->dma_buf) {
            bouncebuffer->size = 0;
            return false;
        }
        bouncebuffer->size = size;
    }
    *buf = bouncebuffer->dma_buf;
#if ENABLE_ECHO_SAFE
    memset(bouncebuffer->dma_buf, 0xBB, bouncebuffer->size);
#endif
    bouncebuffer->busy = true;
    return true;
}

/*
  finish a read operation
 */
void bouncebuffer_finish_read(struct bouncebuffer_t *bouncebuffer, const uint8_t *buf, uint32_t size)
{
    if (bouncebuffer && buf == bouncebuffer->dma_buf) {
        osalDbgAssert((bouncebuffer->busy == true), "bouncebuffer finish_read");
        if (bouncebuffer->orig_buf) {
            memcpy(bouncebuffer->orig_buf, buf, size);
        }
        bouncebuffer->busy = false;
    }
}


/*
  setup for reading from memory to a device, allocating a bouncebuffer if needed
 */
bool bouncebuffer_setup_write(struct bouncebuffer_t *bouncebuffer, const uint8_t **buf, uint32_t size)
{
    if (!bouncebuffer || IS_DMA_SAFE(*buf)) {
        // nothing needs to be done
        return true;
    }
    osalDbgAssert((bouncebuffer->busy == false), "bouncebuffer write");        
    if (bouncebuffer->size < size) {
        if (bouncebuffer->size > 0) {
            free(bouncebuffer->dma_buf);
        }
        bouncebuffer->dma_buf = malloc_dma(size);
        if (!bouncebuffer->dma_buf) {
            bouncebuffer->size = 0;
            return false;
        }
        bouncebuffer->size = size;
    }
    if (*buf) {
        memcpy(bouncebuffer->dma_buf, *buf, size);
    }
    *buf = bouncebuffer->dma_buf;
    bouncebuffer->busy = true;
    return true;
}


/*
  finish a write operation
 */
void bouncebuffer_finish_write(struct bouncebuffer_t *bouncebuffer, const uint8_t *buf)
{
    if (bouncebuffer && buf == bouncebuffer->dma_buf) {
        osalDbgAssert((bouncebuffer->busy == true), "bouncebuffer finish_wite");        
        bouncebuffer->busy = false;
    }
}

/*
  abort an operation
 */
void bouncebuffer_abort(struct bouncebuffer_t *bouncebuffer)
{
    if (bouncebuffer) {
        bouncebuffer->busy = false;
    }
}
