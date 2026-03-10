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
 * 
 * Code by Szilveszter Zsigmond
 */
#pragma once
#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Board identifier.
 */
#define BOARD_RP_PICO_RP2040
#define BOARD_NAME                  "Raspberry Pi Pico"

/*
 * Board oscillators-related settings.
 */
#if !defined(RP_XOSCCLK)
#define RP_XOSCCLK                  12000000U
#endif

/*
 * MCU type.
 */
#define RP2040

#include "../common/board.h"

#endif /* BOARD_H */
