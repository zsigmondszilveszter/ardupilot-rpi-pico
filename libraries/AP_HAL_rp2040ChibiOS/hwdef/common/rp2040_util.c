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

#include "rp2040_util.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <hrt.h>

static int64_t utc_time_offset;


/*
  set the utc time
 */
void rp2040_set_utc_usec(uint64_t time_utc_usec)
{
    uint64_t now = hrt_micros64();
    if (now <= time_utc_usec) {
        utc_time_offset = time_utc_usec - now;
    }
}

/*
  get system clock in UTC microseconds
*/
uint64_t rp2040_get_utc_usec()
{
    return hrt_micros64() + utc_time_offset;
}