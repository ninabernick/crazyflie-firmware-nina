/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __SCREEN_CFAL12864G_H__
#define __SCREEN_CFAL12864G_H__

#define SSD1309_00_SET_LOWER_COLUMN_ADDRESS_BIT       (0x00)
#define SSD1309_10_SET_UPPER_COLUMN_ADDRESS_BIT       (0x10)
#define SSD1309_20_MEMORY_ADDRESSING_MODE_PREFIX      (0x20)
#define SSD1309_00_ADRESSING_HORIZONTAL_PARAMETER     (0x00)
#define SSD1309_01_ADRESSING_VERTICAL_PARAMETER       (0x01)
#define SSD1309_02_ADRESSING_PAGE_PARAMETER           (0x02)
#define SSD1309_21_SET_COLUMN_ADDRESS_PREFIX          (0x21)
#define SSD1309_22_SET_PAGE_ADDRESS_PREFIX            (0x22)
#define SSD1309_26_SCROLL_SET_CONT_HORIZ_RIGHT_PREFIX (0x26)
#define SSD1309_27_SCROLL_SET_CONT_HORIZ_LEFT_PREFIX  (0x27)
#define SSD1309_29_SCROLL_SET_VERT_HORIZ_RIGHT_PREFIX (0x29)
#define SSD1309_2A_SCROLL_SET_VERT_HORIZ_LEFT_PREFIX  (0x2A)
#define SSD1309_2C_SCROLL_SET_ONE_HORIZ_RIGHT_PREFIX  (0x2C)
#define SSD1309_2D_SCROLL_SET_ONE_HORIZ_LEFT_PREFIX   (0x2D)
#define SSD1309_2E_SCROLL_DEACTIVATE                  (0x2E)
#define SSD1309_2F_SCROLL_ACTIVATE                    (0x2F)
#define SSD1309_40_SET_DISPLAY_START_LINE_BIT         (0x40)
#define SSD1309_81_CONTRAST_PREFIX                    (0x81)
#define SSD1309_A0_SEGMENT_REMAP_NORMAL               (0xA0)
#define SSD1309_A1_SEGMENT_REMAP_REVERSE              (0xA1)
#define SSD1309_A3_SCROLL_VERT_AREA_PREFIX            (0xA3)
#define SSD1309_A4_ENTIRE_DISPLAY_NORMAL              (0xA4)
#define SSD1309_A5_ENTIRE_DISPLAY_FORCE_ON            (0xA5)
#define SSD1309_A6_INVERSION_NORMAL                   (0xA6)
#define SSD1309_A7_INVERSION_INVERTED                 (0xA7)
#define SSD1309_A8_MULTIPLEX_RATIO_PREFIX             (0xA8)
#define SSD1309_AE_DISPLAY_OFF_SLEEP_YES              (0xAE)
#define SSD1309_AF_DISPLAY_ON_SLEEP_NO                (0xAF)
#define SSD1309_B0_SET_PAGE_START_ADDRESS_BIT         (0xB0)
#define SSD1309_C0_COM_DIRECTION_NORMAL               (0xC0)
#define SSD1309_C8_COM_DIRECTION_REVERSE              (0xC8)
#define SSD1309_D3_DISPLAY_VERT_OFFSET_PREFIX         (0xD3)
#define SSD1309_D5_CLOCK_DIVIDE_PREFIX                (0xD5)
#define SSD1309_D9_PRECHARGE_PERIOD_PREFIX            (0xD9)
#define SSD1309_DA_COM_PINS_CONFIGURATION_PREFIX      (0xDA)
#define SSD1309_DB_VCOMH_DESELECT_PREFIX              (0xDB)
#define SSD1309_DC_GPIO_CONFIGURATION_PREFIX          (0xDC)
#define SSD1309_E3_NOP                                (0xE3)
#define SSD1309_FD_LOCK_UNLOCK_PREFIX                 (0xFD)
#define SSD1309_16_LOCK_PARAMETER                     (0x16)
#define SSD1309_12_UNLOCK_PARAMETER                   (0x12)

/* Defines for the SPI and GPIO pins used to drive the SPI Flash */
// use transmit-only master mode
#define CFAL12864G_SPI                     SPI3
#define CFAL12864G_SPI_AF                  GPIO_AF_SPI3
#define CFAL12864G_SPI_CLK                 RCC_APB1Periph_SPI3
#define CFAL12864G_GPIO_SPI_PORT           GPIOC
#define CFAL12864G_GPIO_SPI_CLK            RCC_AHB1Periph_GPIOC
#define CFAL12864G_GPIO_SPI_SCK            GPIO_Pin_10
#define CFAL12864G_GPIO_SPI_SCK_SRC        GPIO_PinSource10
// #define CFAL12864G_GPIO_SPI_MISO           GPIO_Pin_11
// #define CFAL12864G_GPIO_SPI_MISO_SRC       GPIO_PinSource11
#define CFAL12864G_GPIO_SPI_MOSI           GPIO_Pin_12
#define CFAL12864G_GPIO_SPI_MOSI_SRC       GPIO_PinSource12

/* Defines for RS and RESET pins */
#define CFAL12864G_GPIO_RS_PORT            GPIOC
#define CFAL12864G_GPIO_RS                 GPIO_Pin_11
#define CFAL12864G_GPIO_RESET_PORT         GPIOB
#define CFAL12864G_GPIO_RESET              GPIO_Pin_8

/* Defines for SPI DMA */
#define CFAL12864G_SPI_DMA_IRQ_PRIO        NVIC_HIGH_PRI
#define CFAL12864G_SPI_DMA                 DMA1
#define CFAL12864G_SPI_DMA_CLK             RCC_AHB1Periph_DMA1
#define CFAL12864G_SPI_DMA_CLK_INIT        RCC_AHB1PeriphClockCmd

#define CFAL12864G_SPI_TX_DMA_STREAM       DMA1_Stream7
#define CFAL12864G_SPI_TX_DMA_IRQ          DMA1_Stream7_IRQn
#define CFAL12864G_SPI_TX_DMA_IRQHandler   DMA1_Stream7_IRQHandler
#define CFAL12864G_SPI_TX_DMA_CHANNEL      DMA_Channel_0
#define CFAL12864G_SPI_TX_DMA_FLAG_TCIF    DMA_FLAG_TCIF7

#define SCREEN_FRESH_RATE_HZ            30

#define MAX_CONTENT_NUMBER				31
struct textContent_t {
  uint8_t num;
  char ct[MAX_CONTENT_NUMBER];
};

void screenCFAL12864GInit(void);
void screenTextSet(textContent_t *ct);

#endif // __SCREEN_CFAL12864G_H__