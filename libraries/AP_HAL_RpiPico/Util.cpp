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


#include "Util.h"

RpiPico::Util::Util() {}

#ifdef ENABLE_HEAP
void* RpiPico::Util::allocate_heap_memory(size_t size){
    // not implemented yet
    return NULL;
}
void* RpiPico::Util::heap_realloc(void *heap, void *ptr, size_t new_size){
    // not implemented yet
    return NULL;
}
#if !USE_LIBC_REALLOC
void* RpiPico::Util::std_realloc(void *ptr, size_t new_size){
    return std::realloc(ptr, new_size);
}
#endif // USE_LIBC_REALLOC
#endif // ENABLE_HEAP
