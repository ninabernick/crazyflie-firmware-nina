/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 *
 * cfal12864g.c: OLED screen
 */

#define DEBUG_MODULE "SCN"

#include <math.h>
#include <string.h>

#include "stm32fxxx.h"

#include "cfal12864g.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "system.h"
#include "configblock.h"
#include "nvicconf.h"
#include "param.h"
#include "log.h"
#include "debug.h"

#define CLR_RS    GPIO_ResetBits(CFAL12864G_GPIO_RS_PORT, CFAL12864G_GPIO_RS)
#define SET_RS    GPIO_SetBits(CFAL12864G_GPIO_RS_PORT, CFAL12864G_GPIO_RS)
#define CLR_RESET GPIO_ResetBits(CFAL12864G_GPIO_RESET_PORT, CFAL12864G_GPIO_RESET)
#define SET_RESET GPIO_SetBits(CFAL12864G_GPIO_RESET_PORT, CFAL12864G_GPIO_RESET)
#define CLR_MOSI  GPIO_ResetBits(CFAL12864G_GPIO_SPI_PORT, CFAL12864G_GPIO_SPI_MOSI)
#define SET_MOSI  GPIO_SetBits(CFAL12864G_GPIO_SPI_PORT, CFAL12864G_GPIO_SPI_MOSI)
#define CLR_SCK   GPIO_ResetBits(CFAL12864G_GPIO_SPI_PORT, CFAL12864G_GPIO_SPI_SCK)
#define SET_SCK   GPIO_SetBits(CFAL12864G_GPIO_SPI_PORT, CFAL12864G_GPIO_SPI_SCK)

/* Defines and buffers for full duplex SPI DMA transactions */
#define SCREEN_DATA_H 8
#define SCRREN_DATA_W 128
static uint8_t SPITxBuffer[SCREEN_DATA_H][SCRREN_DATA_W];
static xSemaphoreHandle SPITxDMAComplete;

static bool isInit = false;

static void CFAL12864G_ms_delay(uint32_t period) {
  /**< Delay code comes */
  vTaskDelay(M2T(period));
}

/***********************
 * SPI private methods *
 ***********************/
void SPISendCommand(uint8_t command) {
  CLR_RS;
  SPI_I2S_SendData(CFAL12864G_SPI, command);
}

void SPISendData(uint8_t data) {
  SET_RS;
  SPI_I2S_SendData(CFAL12864G_SPI, data);
}

void SetAddress(uint8_t column, uint8_t page) {
  DEBUG_PRINT("add:%d\n", page);
  // set column-lower nibble
  SPISendCommand(SSD1309_00_SET_LOWER_COLUMN_ADDRESS_BIT | (column & 0x0F));
  // set column-upper nibble
  SPISendCommand(SSD1309_10_SET_UPPER_COLUMN_ADDRESS_BIT | ((column >> 4) & 0x0F));
  // set page address, limiting from 0 to 7
  SPISendCommand(SSD1309_B0_SET_PAGE_START_ADDRESS_BIT | (page & 0x07));
}

void SetBrightness(uint8_t brightness) {
  // set the "contrast" (brightness, max determined by IREF current)
  SPISendCommand(SSD1309_81_CONTRAST_PREFIX);
  SPISendCommand(brightness);
}

void CFAL12864GInit(uint8_t brightness) {
  CLR_RESET;
  CLR_RS;
  CLR_MOSI;
  CLR_SCK;
  // thump the hardware reset line.
  CFAL12864G_ms_delay(10);
  CLR_RESET;
  CFAL12864G_ms_delay(10);
  SET_RESET;
  CFAL12864G_ms_delay(10);

  // Start with the display off (sleeping)
  SPISendCommand(SSD1309_AE_DISPLAY_OFF_SLEEP_YES);

  // Set the memory addressing mode to PAGE (increment column, no wrap)
  SPISendCommand(SSD1309_20_MEMORY_ADDRESSING_MODE_PREFIX);
  SPISendCommand(SSD1309_02_ADRESSING_PAGE_PARAMETER);

  // Point to the upper-left
  SetAddress(0, 0);

  // Set the "contrast" (brightness, max determined by IREF current)
  SPISendCommand(SSD1309_81_CONTRAST_PREFIX);
  SPISendCommand(brightness);

  // Set start line to 0
  SPISendCommand(SSD1309_40_SET_DISPLAY_START_LINE_BIT); //Set Display Start Line

  // Set Segment remap for the CFAL12864G, we want the lower-left to be 0,0
  SPISendCommand(SSD1309_A1_SEGMENT_REMAP_REVERSE);

  // Ensure tha the "entire display force on", test mide is disabled. Read
  // the data from the RAM and display it as normal.
  SPISendCommand(SSD1309_A4_ENTIRE_DISPLAY_NORMAL);

  // Make sure that inversion is disabled.
  SPISendCommand(SSD1309_A6_INVERSION_NORMAL);

  // Set the multiplex ratio. 64 lines so 1/64
  SPISendCommand(SSD1309_A8_MULTIPLEX_RATIO_PREFIX);
  SPISendCommand(63);

  // Set COM directiom CFAL12864G, we want the lower-left to be 0,0
  SPISendCommand(SSD1309_C0_COM_DIRECTION_NORMAL);

  // No display vertical offset
  SPISendCommand(SSD1309_D3_DISPLAY_VERT_OFFSET_PREFIX);
  SPISendCommand(0);

  // Set clock frequency and division ratio
  SPISendCommand(SSD1309_D5_CLOCK_DIVIDE_PREFIX);
  // Fastest clock, smallest divisor.
  SPISendCommand(0xF0);
  // FFFF DDDD
  // |||| ||||-- DDDD = division, 0=>1, 1=>2 etc
  // ||||------- FFFF = frequency, 0 = slow 15 = fast

  //Set Pre-charge Period
  SPISendCommand(SSD1309_D9_PRECHARGE_PERIOD_PREFIX);
  SPISendCommand(0xF1);
  // 2222 1111
  // |||| ||||-- Phase 1 DCLK periods
  // ||||------- Phase 2 DCLK periods

  // Set COM Pins Hardware Configuration
  SPISendCommand(SSD1309_DA_COM_PINS_CONFIGURATION_PREFIX);
  // Alternate, Disable remap
  SPISendCommand(0x12);

  // Set Vcomh Deselect Level
  SPISendCommand(SSD1309_DB_VCOMH_DESELECT_PREFIX);
  // not in 1309 datasheet says it should be one of 00, 34 or 3C
  SPISendCommand(0x40);

  // Make sure the scroll is deactivated.
  SPISendCommand(SSD1309_2E_SCROLL_DEACTIVATE);

  // Turn the display on (wake)
  SPISendCommand(SSD1309_AF_DISPLAY_ON_SLEEP_NO);
}
// static char spiSendByte(char byte) {
//   /* Loop while DR register in not emplty */
//   while (SPI_I2S_GetFlagStatus(CFAL12864G_SPI, SPI_I2S_FLAG_TXE) == RESET);

//   /* Send byte through the SPI peripheral */
//   SPI_I2S_SendData(CFAL12864G_SPI, byte);

//   /* Wait to receive a byte */
//   while (SPI_I2S_GetFlagStatus(CFAL12864G_SPI, SPI_I2S_FLAG_RXNE) == RESET);

//   /* Return the byte read from the SPI bus */
//   return SPI_I2S_ReceiveData(CFAL12864G_SPI);
// }

/*
static void spiDMATransaction(uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  ASSERT(len < SPI_MAX_DMA_TRANSACTION_SIZE);

  // Disable peripheral before setting up for duplex DMA
  SPI_Cmd(CFAL12864G_SPI, DISABLE);

  // DMA already configured, just need to set memory addresses and read command byte
  spiTxBuffer[0] = reg_addr;
  CFAL12864G_SPI_TX_DMA_STREAM->M0AR = (uint32_t)&spiTxBuffer[0];
  CFAL12864G_SPI_TX_DMA_STREAM->NDTR = len + 1;

  CFAL12864G_SPI_RX_DMA_STREAM->M0AR = (uint32_t)&spiRxBuffer[0];
  CFAL12864G_SPI_RX_DMA_STREAM->NDTR = len + 1;

  // Enable SPI DMA Interrupts
  DMA_ITConfig(CFAL12864G_SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_ITConfig(CFAL12864G_SPI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

  // Clear DMA Flags
  DMA_ClearFlag(CFAL12864G_SPI_TX_DMA_STREAM, DMA_FLAG_FEIF4|DMA_FLAG_DMEIF4|
                DMA_FLAG_TEIF4|DMA_FLAG_HTIF4|DMA_FLAG_TCIF4);
  DMA_ClearFlag(CFAL12864G_SPI_RX_DMA_STREAM, DMA_FLAG_FEIF3|DMA_FLAG_DMEIF3|
                DMA_FLAG_TEIF3|DMA_FLAG_HTIF3|DMA_FLAG_TCIF3);

  // Enable DMA Streams
  DMA_Cmd(CFAL12864G_SPI_TX_DMA_STREAM,ENABLE);
  DMA_Cmd(CFAL12864G_SPI_RX_DMA_STREAM,ENABLE);

  // Enable SPI DMA requests
  SPI_I2S_DMACmd(CFAL12864G_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(CFAL12864G_SPI, SPI_I2S_DMAReq_Rx, ENABLE);

  // Enable peripheral to begin the transaction
  SPI_Cmd(CFAL12864G_SPI, ENABLE);

  // Wait for completion
  // TODO: Better error handling rather than passing up invalid data
  xSemaphoreTake(spiTxDMAComplete, portMAX_DELAY);

  // Copy the data (discarding the dummy byte) into the buffer
  // TODO: Avoid this memcpy either by figuring out how to configure the STM SPI to discard the byte or handle it higher up
  memcpy(reg_data, &spiRxBuffer[1], len);
}
*/
/* Initialisation */
void SPIInit(void) {
  if (isInit)
    return;

  GPIO_InitTypeDef GPIO_InitStructure;

  /* clock configure */
  // Enable SPI and GPIO clocks
  RCC_AHB1PeriphClockCmd(CFAL12864G_GPIO_SPI_CLK, ENABLE);
  // Enable SPI and GPIO clocks
  RCC_APB1PeriphClockCmd(CFAL12864G_SPI_CLK, ENABLE);

  /* GPIO configure */
  // configure SPI pins: SCK, MOSI
  GPIO_InitStructure.GPIO_Pin = CFAL12864G_GPIO_SPI_SCK |  CFAL12864G_GPIO_SPI_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; // SPI should be pull-down
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CFAL12864G_GPIO_SPI_PORT, &GPIO_InitStructure);

  // configure RS pin
  GPIO_InitStructure.GPIO_Pin = CFAL12864G_GPIO_RS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(CFAL12864G_GPIO_RS_PORT, &GPIO_InitStructure);
  // configure RESET pin
  GPIO_InitStructure.GPIO_Pin = CFAL12864G_GPIO_RESET;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(CFAL12864G_GPIO_RESET_PORT, &GPIO_InitStructure);
  //* Configure MISO */
  // GPIO_InitStructure.GPIO_Pin = CFAL12864G_GPIO_SPI_MISO;
  // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  // GPIO_Init(CFAL12864G_GPIO_SPI_PORT, &GPIO_InitStructure);

  // !< Connect SPI pins to AF5
  GPIO_PinAFConfig(CFAL12864G_GPIO_SPI_PORT, CFAL12864G_GPIO_SPI_SCK_SRC, CFAL12864G_SPI_AF);
  GPIO_PinAFConfig(CFAL12864G_GPIO_SPI_PORT, CFAL12864G_GPIO_SPI_MOSI_SRC, CFAL12864G_SPI_AF);

  /* SPI configure */
  SPI_InitTypeDef  SPI_InitStructure;

  SPI_Cmd(CFAL12864G_SPI, DISABLE);
  SPI_I2S_DeInit(CFAL12864G_SPI);
  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //~4: 10.5 MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used

  SPI_Init(CFAL12864G_SPI, &SPI_InitStructure);
  /* Enable the SPI  */
  SPI_Cmd(CFAL12864G_SPI, ENABLE);
}

void SPIDMAInit(void) {
  /* Init structure */
  DMA_InitTypeDef  DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /*!< Enable DMA Clocks */
  CFAL12864G_SPI_DMA_CLK_INIT(CFAL12864G_SPI_DMA_CLK, ENABLE);

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_Channel = CFAL12864G_SPI_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(CFAL12864G_SPI->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;                  // set later
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = SCRREN_DATA_W;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  

  DMA_Init(CFAL12864G_SPI_TX_DMA_STREAM, &DMA_InitStructure);
  DMA_Cmd(CFAL12864G_SPI_TX_DMA_STREAM, DISABLE);
  // DMA_Cmd(CFAL12864G_SPI_TX_DMA_STREAM,ENABLE);
  // Enable SPI DMA requests
  SPI_I2S_DMACmd(CFAL12864G_SPI, SPI_I2S_DMAReq_Tx, DISABLE);
  // Enable peripheral to begin the transaction
  SPI_Cmd(CFAL12864G_SPI, ENABLE);

  // Configure interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannel = CFAL12864G_SPI_TX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  SPITxDMAComplete = xSemaphoreCreateBinary();
}

QueueHandle_t textContentQueue;
static textContent_t nullTextContent;

void screenTextInit(void) {
  textContentQueue = xQueueCreate(1, sizeof(textContent_t));
  ASSERT(textContentQueue);
  xQueueSend(textContentQueue, &nullTextContent, 0);
}

void screenTextSet(textContent_t *ct) {
  xQueueOverwrite(textContentQueue, ct);
}

bool screenTextGet(textContent_t *ct) {
  return (pdTRUE == xQueueReceive(textContentQueue, ct, 0));
}

static int page;

void refreshContent() {
  // textContent_t ct;
  // if (screenTextGet(&ct)) {
  //   memset(SPITxBuffer, 0, sizeof(SPITxBuffer));
  //   // update content
  // }
  // int ct;
  // uint8_t page;
  for (page = 0; page < 8; page++) {
    //Set the LCD to the left of this line.
    SPI_Cmd(CFAL12864G_SPI, ENABLE);
    SetAddress(0, page);
    // Move across the columns, alternating the two data
    // Select the LCD's data register
    SET_RS;

    // SET_MOSI;
    // SET_SCK;
    SPI_Cmd(CFAL12864G_SPI, DISABLE);

    CFAL12864G_SPI_TX_DMA_STREAM->M0AR = (uint32_t) SPITxBuffer[page];
    
    DMA_ClearFlag(CFAL12864G_SPI_TX_DMA_STREAM, DMA_FLAG_FEIF7|DMA_FLAG_DMEIF7|
                DMA_FLAG_TEIF7|DMA_FLAG_HTIF7|DMA_FLAG_TCIF7);
    DMA_ITConfig(CFAL12864G_SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);

    DMA_Cmd(CFAL12864G_SPI_TX_DMA_STREAM, ENABLE);
    SPI_I2S_DMACmd(CFAL12864G_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

    SPI_Cmd(CFAL12864G_SPI, ENABLE);
    xSemaphoreTake(SPITxDMAComplete, portMAX_DELAY);

  }
}

void clearBuffer(uint8_t v) {
  uint8_t *tmp = SPITxBuffer[0];
  for (int x = 0; x < 1024; x++) tmp[x] = v;
}

void horizontal_line(uint8_t x1, uint8_t y, uint8_t x2) {
  uint8_t *LCD_Memory;
  uint8_t column;
  uint8_t Set_Mask;

  //Bail for bogus parametrers.
  if ((x2 < x1) || (127 < x1) || (127 < x2) || (63 < y))
    return;

  //Calculate the address of the first uint8_t in display in LCD_Memory
  LCD_Memory = &SPITxBuffer[y >> 3][x1];

  //Calculate Set_Mask, the vertical mask that we will or with
  //LCD_Memory to clear the space before we or in the data from the
  //font. It is 9 pixels.
  Set_Mask = 0x01 << (y & 0x07);

  //Move across memory, oring the mask in.
  for (column = x1; column <= x2; column++) {
    LCD_Memory[0] |= Set_Mask;
    LCD_Memory++;
  }
}

static void screenTask(void *param) {
  systemWaitStart();
  static portTickType lastWakeTime;
  lastWakeTime = xTaskGetTickCount();

  static int cnt = 0;
  while (1) {
    vTaskDelayUntil(&lastWakeTime, M2T(100));
    // vTaskDelay(M2T(2000));
    clearBuffer(0x00);
    horizontal_line(110, cnt, 127);
    refreshContent();

    cnt++;
    if (cnt % 1 == 0)
      DEBUG_PRINT("1round\n");
    if (cnt > 60) cnt = 0;
  }
}

void screenCFAL12864GInit(void) {
  if (isInit) {
    return;
  }

  SPIInit();
  SPIDMAInit();
  CFAL12864GInit(255);
  // test
  clearBuffer(0x00);
  // horizontal_line(110, 10, 127);
  // refreshContent();
  // screenTextInit();
  xTaskCreate(screenTask, SCREEN_TASK_NAME, SCREEN_TASK_STACKSIZE, NULL, SCREEN_TASK_PRI, NULL);
  isInit = true;
}

void __attribute__((used)) CFAL12864G_SPI_TX_DMA_IRQHandler(void) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  // Stop and cleanup DMA stream
  DMA_ITConfig(CFAL12864G_SPI_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(CFAL12864G_SPI_TX_DMA_STREAM, CFAL12864G_SPI_TX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(CFAL12864G_SPI_TX_DMA_STREAM, CFAL12864G_SPI_TX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(CFAL12864G_SPI, SPI_I2S_DMAReq_Tx, DISABLE);

  // Disable streams
  DMA_Cmd(CFAL12864G_SPI_TX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(SPITxDMAComplete, &xHigherPriorityTaskWoken);
  // xSemaphoreGive(SPITxDMAComplete);

}