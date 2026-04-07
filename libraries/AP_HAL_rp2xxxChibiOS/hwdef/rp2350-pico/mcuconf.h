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

#ifndef MCUCONF_H
#define MCUCONF_H

/*
 * RP2350_MCUCONF drivers configuration.
 *
 * IRQ priorities:
 * 15...0       Lowest...Highest (4 bits on Cortex-M33).
 *
 * DMA priorities:
 * 0...1        Lowest...Highest.
 */

#define RP2350_MCUCONF

/*
 * HAL driver system settings.
 */
#define RP_NO_INIT                          FALSE
#define RP_CORE1_START                      TRUE
#define RP_CORE1_VECTORS_TABLE              _vectors
#define RP_CORE1_ENTRY_POINT                _crt0_c1_entry
#define RP_CORE1_STACK_END                  __c1_main_stack_end__

/*
 * Set the clock to 276MHz
 *
 * PLL_SYS configuration: 12 MHz XOSC * 115 / 5 / 1 = 276 MHz
 * VCO = 1380 MHz (within 750-1600 MHz limit)
 * POSTDIV1=5, POSTDIV2=1 -> 276 MHz output
 */
#define RP_PLL_SYS_REFDIV               1U
#define RP_PLL_SYS_VCO_FREQ             1380000000U
#define RP_PLL_SYS_POSTDIV1             5U
#define RP_PLL_SYS_POSTDIV2             1U

/*
 * IRQ system settings.
 */
#define RP_IRQ_SYSTICK_PRIORITY             2
#define RP_IRQ_TIMER0_ALARM0_PRIORITY       2
#define RP_IRQ_TIMER0_ALARM1_PRIORITY       2
#define RP_IRQ_TIMER0_ALARM2_PRIORITY       2
#define RP_IRQ_TIMER0_ALARM3_PRIORITY       2
#define RP_IRQ_UART0_PRIORITY               3
#define RP_IRQ_UART1_PRIORITY               3
#define RP_IRQ_SPI0_PRIORITY                2
#define RP_IRQ_SPI1_PRIORITY                2
#define RP_IRQ_USB0_PRIORITY                2
#define RP_IRQ_I2C0_PRIORITY                2
#define RP_IRQ_I2C1_PRIORITY                2

/*
 * SIO driver system settings.
 */
#define RP_ADC_USE_ADC1                     TRUE
#define RP_SIO_USE_UART0                    TRUE
#define RP_SIO_USE_UART1                    TRUE

/*
 * SPI driver system settings.
 */
#define RP_SPI_USE_SPI0                     TRUE
#define RP_SPI_USE_SPI1                     TRUE
#define RP_SPI_SPI0_RX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#define RP_SPI_SPI0_TX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#define RP_SPI_SPI1_RX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#define RP_SPI_SPI1_TX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#define RP_SPI_SPI0_DMA_PRIORITY            1
#define RP_SPI_SPI1_DMA_PRIORITY            1
#define RP_SPI_DMA_ERROR_HOOK(spip)

/*
 * I2C driver system settings.
 */
#define RP_I2C_USE_I2C0                     TRUE
#define RP_I2C_USE_I2C1                     TRUE
#define RP_I2C_I2C0_RX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#define RP_I2C_I2C0_TX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#define RP_I2C_I2C1_RX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#define RP_I2C_I2C1_TX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#define RP_I2C_I2C0_DMA_PRIORITY            1
#define RP_I2C_I2C1_DMA_PRIORITY            1
#define RP_I2C_DMA_ERROR_HOOK(spip)
#define RP_I2C_ADDRESS_MODE_10BIT           FALSE

/*
 * USB driver system settings.
 */
#define RP_USB_USE_USB1                     TRUE
#define RP_USB_FORCE_VBUS_DETECT            TRUE
#define RP_USE_EXTERNAL_VBUS_DETECT         FALSE
#define RP_USB_USE_ERROR_DATA_SEQ_INTR      TRUE

/**
 * PWM driver system settings
 * Slices chosen to match RC output pins:
 *   GPIO 8    → PWM slice 4 (PWMD4)
 *   GPIO 10/11 → PWM slice 5 (PWMD5)
 *   GPIO 20/21 → PWM slice 2 (PWMD2)
 *   GPIO 22   → PWM slice 3 (PWMD3)
 */
#define RP_PWM_USE_PWM4                     TRUE
#define RP_PWM_USE_PWM5                     TRUE
#define RP_PWM_USE_PWM2                     TRUE
#define RP_PWM_USE_PWM3                     TRUE


#endif /* MCUCONF_H */
