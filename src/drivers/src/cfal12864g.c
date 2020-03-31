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
#define SPI_MAX_DMA_TRANSACTION_SIZE    15
static uint8_t spiTxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];

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

void CFAL12864GInit(uint8_t brightness = 255) {
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
  SetAddress(0,0);

  // Set the "contrast" (brightness, max determined by IREF current)
  SPISendCommand(SSD1309_81_CONTRAST_PREFIX);
  SPISendCommand(brightness);

  // Set start line to 0
  SPISendCommand(SSD1309_40_SET_DISPLAY_START_LINE_BIT);//Set Display Start Line

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
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used

  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //~10.5 MHz
  SPI_Init(CFAL12864G_SPI, &SPI_InitStructure);
  /* Enable the SPI  */
  SPI_Cmd(CFAL12864G_SPI, ENABLE);
}

void SPIDMAInit(void) {
  DMA_InitTypeDef  DMA_InitStructure;

  /*!< Enable DMA Clocks */
  CFAL12864G_SPI_DMA_CLK_INIT(CFAL12864G_SPI_DMA_CLK, ENABLE);

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_Channel = CFAL12864G_SPI_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(CFAL12864G_SPI->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr = 0; // set later
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = 0; // set later
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  
  DMA_Cmd(CFAL12864G_SPI_TX_DMA_STREAM, DISABLE);
  DMA_Init(CFAL12864G_SPI_TX_DMA_STREAM, &DMA_InitStructure);

  // Configure interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannel = CFAL12864G_SPI_TX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  spiTxDMAComplete = xSemaphoreCreateBinary();
}

static void screenTask(void *param) {
  systemWaitStart();

  /* wait an additional second the keep bus free
   * this is only required by the z-ranger, since the
   * configuration will be done after system start-up */
  //vTaskDelayUntil(&lastWakeTime, M2T(1500));
  while (1) {
    if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {
      sensorData.interruptTimestamp = imuIntTimestamp;

      /* get data from chosen sensors */
      sensorsGyroGet(&gyroRaw);
      sensorsAccelGet(&accelRaw);

      if (gyroBiasFound) {
         processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
      }
      /* Gyro */
      sensorData.gyro.x =  (gyroRaw.x - gyroBias.x) * SENSORS_CFAL12864G_DEG_PER_LSB_CFG;
      sensorData.gyro.y =  (gyroRaw.y - gyroBias.y) * SENSORS_CFAL12864G_DEG_PER_LSB_CFG;
      sensorData.gyro.z =  (gyroRaw.z - gyroBias.z) * SENSORS_CFAL12864G_DEG_PER_LSB_CFG;
      applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensorData.gyro);

      /* Acelerometer */
      accScaled.x = accelRaw.x * SENSORS_CFAL12864G_G_PER_LSB_CFG / accScale;
      accScaled.y = accelRaw.y * SENSORS_CFAL12864G_G_PER_LSB_CFG / accScale;
      accScaled.z = accelRaw.z * SENSORS_CFAL12864G_G_PER_LSB_CFG / accScale;
      sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
      applyAxis3fLpf((lpf2pData*)(&accLpf), &sensorData.acc);
    }

    if (isBarometerPresent) {
      static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
      if (--baroMeasDelay == 0) {
        uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;
        struct bmp3_data data;
        baro_t* baro388 = &sensorData.baro;
        /* Temperature and Pressure data are read and stored in the bmp3_data instance */
        bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);
        sensorsScaleBaro(baro388, data.pressure, data.temperature);
        baroMeasDelay = baroMeasDelayMin;
      }
    }
    xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
    xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
    if (isBarometerPresent) {
      xQueueOverwrite(barometerDataQueue, &sensorData.baro);
    }

    xSemaphoreGive(dataReady);
  }
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

void screenCFAL12864GInit(void) {
  if (isInit) {
    return;
  }

  SPIInit();
  SPIDMAInit();
  CFAL12864GInit(255);
  screenTextInit();
  xTaskCreate(screenTask, SCREEN_TASK_NAME, SCREEN_TASK_STACKSIZE, NULL, SCREEN_TASK_PRI, NULL);
}
