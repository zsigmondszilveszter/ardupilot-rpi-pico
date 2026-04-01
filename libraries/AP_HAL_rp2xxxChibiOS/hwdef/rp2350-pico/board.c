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

#include "hal.h"

#define SCB_CPACR_REG (*(volatile uint32_t *)0xE000ED88U)
#define RP2350_CPACR_CP10_CP11_FULL_ACCESS 0x00F00000U
#define RP2350_CPACR_CP4_FULL_ACCESS       0x0000C300U

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Early initialization code.
 * @details GPIO ports and system clocks are initialized before everything
 *          else.
 */
void __early_init(void) {
//  rp_gpio_init();
    /* Disable the bootrom watchdog. */
    WATCHDOG->CLR.CTRL = WATCHDOG_CTRL_ENABLE;
}

void __late_init(void) {
  /*
   * Match the RP2040 shim pattern: perform coprocessor-specific init after
   * crt0 has completed core/FPU setup.
   */
  SCB_CPACR_REG |= RP2350_CPACR_CP10_CP11_FULL_ACCESS |
                   RP2350_CPACR_CP4_FULL_ACCESS;
  __asm volatile ("dsb");
  __asm volatile ("isb");
  __asm volatile ("mrc p4,#0,r0,c0,c0,#1" : : : "r0");
  halInit();
  chSysInit();
}

#if HAL_USE_SDC || defined(__DOXYGEN__)
/**
 * @brief   SDC card detection.
 */
bool sdc_lld_is_card_inserted(SDCDriver *sdcp) {

  (void)sdcp;
  /* CHTODO: Fill the implementation.*/
  return true;
}

/**
 * @brief   SDC card write protection detection.
 */
bool sdc_lld_is_write_protected(SDCDriver *sdcp) {

  (void)sdcp;
  /* CHTODO: Fill the implementation.*/
  return false;
}
#endif /* HAL_USE_SDC */

#if HAL_USE_MMC_SPI || defined(__DOXYGEN__)
/**
 * @brief   MMC_SPI card detection.
 */
bool mmc_lld_is_card_inserted(MMCDriver *mmcp) {

  (void)mmcp;
  /* CHTODO: Fill the implementation.*/
  return true;
}

/**
 * @brief   MMC_SPI card write protection detection.
 */
bool mmc_lld_is_write_protected(MMCDriver *mmcp) {

  (void)mmcp;
  /* CHTODO: Fill the implementation.*/
  return false;
}
#endif

/**
 * @brief   Board-specific initialization code.
 * @note    You can add your board-specific code here.
 */
void boardInit(void) {

}
