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
static uint8_t spiRxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];
static xSemaphoreHandle spiTxDMAComplete;

/* initialize necessary variables */
static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;
static xSemaphoreHandle sensorsDataReady;
static xSemaphoreHandle dataReady;

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

void Initialize_CFAL12864G(uint8_t brightness) {
  // thump the hardware reset line.
  CFAL12864G_ms_delay(10);
  CLR_RESET;
  CFAL12864G_ms_delay(10);
  SET_RESET;
  CFAL12864G_ms_delay(10);

  //Start with the display off (sleeping)
  SPISendCommand(SSD1309_AE_DISPLAY_OFF_SLEEP_YES);

  //Set the memory addressing mode to PAGE (increment column, no wrap)
  SPISendCommand(SSD1309_20_MEMORY_ADDRESSING_MODE_PREFIX);
  SPISendCommand(SSD1309_02_ADRESSING_PAGE_PARAMETER);

  //Point to the upper-left
  SetAddress(0,0);

  //Set the "contrast" (brightness, max determined by IREF current)
  SPISendCommand(SSD1309_81_CONTRAST_PREFIX);
  SPISendCommand(brightness);

  //Set start line to 0
  SPISendCommand(SSD1309_40_SET_DISPLAY_START_LINE_BIT);//Set Display Start Line

  //Set Segment remap for the CFAL12864G, we want the lower-left to be 0,0
  SPISendCommand(SSD1309_A1_SEGMENT_REMAP_REVERSE);

  //Ensure tha the "entire display force on", test mide is disabled. Read
  //the data from the RAM and display it as normal.
  SPISendCommand(SSD1309_A4_ENTIRE_DISPLAY_NORMAL);

  //Make sure that inversion is disabled.
  SPISendCommand(SSD1309_A6_INVERSION_NORMAL);

  //Set the multiplex ratio. 64 lines so 1/64
  SPISendCommand(SSD1309_A8_MULTIPLEX_RATIO_PREFIX);
  SPISendCommand(63);

  //Set COM directiom CFAL12864G, we want the lower-left to be 0,0
  SPISendCommand(SSD1309_C0_COM_DIRECTION_NORMAL);

  //No display vertical offset
  SPISendCommand(SSD1309_D3_DISPLAY_VERT_OFFSET_PREFIX);
  SPISendCommand(0);

  //Set clock frequency and division ratio
  SPISendCommand(SSD1309_D5_CLOCK_DIVIDE_PREFIX);
  //Fastest clock, smallest divisor.
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

  //Set COM Pins Hardware Configuration
  SPISendCommand(SSD1309_DA_COM_PINS_CONFIGURATION_PREFIX);
  // Alternate, Disable remap
  SPISendCommand(0x12);

  //Set Vcomh Deselect Level
  SPISendCommand(SSD1309_DB_VCOMH_DESELECT_PREFIX);
//// not in 1309 datasheet says it should be one of 00, 34 or 3C
  SPISendCommand(0x40);

  //Make sure the scroll is deactivated.
  SPISendCommand(SSD1309_2E_SCROLL_DEACTIVATE);

  //Turn the display on (wake)
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
  xSemaphoreTake(spiRxDMAComplete, portMAX_DELAY);

  // Copy the data (discarding the dummy byte) into the buffer
  // TODO: Avoid this memcpy either by figuring out how to configure the STM SPI to discard the byte or handle it higher up
  memcpy(reg_data, &spiRxBuffer[1], len);
}

static void spiConfigure(void) {
  SPI_InitTypeDef  SPI_InitStructure;

  SPI_Cmd(CFAL12864G_SPI, DISABLE);
  SPI_I2S_DeInit(CFAL12864G_SPI);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
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

/* Initialisation */
static void spiInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  if (isInit)
    return;

  /* Enable SPI and GPIO clocks */
  RCC_AHB1PeriphClockCmd(CFAL12864G_GPIO_SPI_CLK | CFAL12864G_ACC_GPIO_CS_PERIF, ENABLE);
  /* Enable SPI and GPIO clocks */
  RCC_APB1PeriphClockCmd(CFAL12864G_SPI_CLK, ENABLE);

  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = CFAL12864G_GPIO_SPI_SCK |  CFAL12864G_GPIO_SPI_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CFAL12864G_GPIO_SPI_PORT, &GPIO_InitStructure);

  //* Configure MISO */
  GPIO_InitStructure.GPIO_Pin = CFAL12864G_GPIO_SPI_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(CFAL12864G_GPIO_SPI_PORT, &GPIO_InitStructure);

  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = CFAL12864G_ACC_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(CFAL12864G_ACC_GPIO_CS_PORT, &GPIO_InitStructure);
  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = CFAL12864G_GYR_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(CFAL12864G_GYR_GPIO_CS_PORT, &GPIO_InitStructure);


  /*!< Connect SPI pins to AF5 */
  GPIO_PinAFConfig(CFAL12864G_GPIO_SPI_PORT, CFAL12864G_GPIO_SPI_SCK_SRC, CFAL12864G_SPI_AF);
  GPIO_PinAFConfig(CFAL12864G_GPIO_SPI_PORT, CFAL12864G_GPIO_SPI_MISO_SRC, CFAL12864G_SPI_AF);
  GPIO_PinAFConfig(CFAL12864G_GPIO_SPI_PORT, CFAL12864G_GPIO_SPI_MOSI_SRC, CFAL12864G_SPI_AF);

  /* disable the chip select */
  ACC_DIS_CS();

  spiConfigure();
}

static void spiDMAInit(void) {
  DMA_InitTypeDef  DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /*!< Enable DMA Clocks */
  CFAL12864G_SPI_DMA_CLK_INIT(CFAL12864G_SPI_DMA_CLK, ENABLE);

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(CFAL12864G_SPI->DR));
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_BufferSize = 0; // set later
  DMA_InitStructure.DMA_Memory0BaseAddr = 0; // set later

  // Configure TX DMA
  DMA_InitStructure.DMA_Channel = CFAL12864G_SPI_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_Cmd(CFAL12864G_SPI_TX_DMA_STREAM,DISABLE);
  DMA_Init(CFAL12864G_SPI_TX_DMA_STREAM, &DMA_InitStructure);

  // Configure RX DMA
  DMA_InitStructure.DMA_Channel = CFAL12864G_SPI_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_Cmd(CFAL12864G_SPI_RX_DMA_STREAM, DISABLE);
  DMA_Init(CFAL12864G_SPI_RX_DMA_STREAM, &DMA_InitStructure);

  // Configure interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannel = CFAL12864G_SPI_TX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CFAL12864G_SPI_RX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  spiTxDMAComplete = xSemaphoreCreateBinary();
  spiRxDMAComplete = xSemaphoreCreateBinary();
}

static void sensorsTask(void *param) {
  systemWaitStart();

  Axis3f accScaled;
  /* wait an additional second the keep bus free
   * this is only required by the z-ranger, since the
   * configuration will be done after system start-up */
  //vTaskDelayUntil(&lastWakeTime, M2T(1500));
  while (1) {
    if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
    {
      sensorData.interruptTimestamp = imuIntTimestamp;

      /* get data from chosen sensors */
      sensorsGyroGet(&gyroRaw);
      sensorsAccelGet(&accelRaw);

      /* calibrate if necessary */
#ifdef GYRO_BIAS_LIGHT_WEIGHT
      gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
      gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif
      if (gyroBiasFound)
      {
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

    if (isBarometerPresent)
    {
      static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
      if (--baroMeasDelay == 0)
      {
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
    if (isBarometerPresent)
    {
      xQueueOverwrite(barometerDataQueue, &sensorData.baro);
    }

    xSemaphoreGive(dataReady);
  }
}

void sensorsCFAL12864GSpiBmp388WaitDataReady(void) {
  xSemaphoreTake(dataReady, portMAX_DELAY);
}

static void sensorsDeviceInit(void) {
  if (isInit)
    return;

  bstdr_ret_t rslt;
  isBarometerPresent = false;

  // Wait for sensors to startup
  vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS));

  /* CFAL12864G */
  CFAL12864GDev.accel_id = CFAL12864G_ACCEL_I2C_ADDR_PRIMARY;
  CFAL12864GDev.gyro_id = CFAL12864G_GYRO_I2C_ADDR_PRIMARY;
  CFAL12864GDev.interface = CFAL12864G_SPI_INTF;
  CFAL12864GDev.read = spi_burst_read;
  CFAL12864GDev.write = spi_burst_write;
  CFAL12864GDev.delay_ms = CFAL12864G_ms_delay;

  /* CFAL12864G GYRO */
  rslt = CFAL12864G_gyro_init(&CFAL12864GDev); // initialize the device
  if (rslt == BSTDR_OK)
  {
    struct CFAL12864G_int_cfg intConfig;

    DEBUG_PRINT("CFAL12864G Gyro SPI connection [OK].\n");
    /* set power mode of gyro */
    CFAL12864GDev.gyro_cfg.power = CFAL12864G_GYRO_PM_NORMAL;
    rslt |= CFAL12864G_set_gyro_power_mode(&CFAL12864GDev);
    /* set bandwidth and range of gyro */
    CFAL12864GDev.gyro_cfg.bw = CFAL12864G_GYRO_BW_116_ODR_1000_HZ;
    CFAL12864GDev.gyro_cfg.range = SENSORS_CFAL12864G_GYRO_FS_CFG;
    CFAL12864GDev.gyro_cfg.odr = CFAL12864G_GYRO_BW_116_ODR_1000_HZ;
    rslt |= CFAL12864G_set_gyro_meas_conf(&CFAL12864GDev);

    intConfig.gyro_int_channel = CFAL12864G_INT_CHANNEL_3;
    intConfig.gyro_int_type = CFAL12864G_GYRO_DATA_RDY_INT;
    intConfig.gyro_int_pin_3_cfg.enable_int_pin = 1;
    intConfig.gyro_int_pin_3_cfg.lvl = 1;
    intConfig.gyro_int_pin_3_cfg.output_mode = 0;
    /* Setting the interrupt configuration */
    rslt = CFAL12864G_set_gyro_int_config(&intConfig, &CFAL12864GDev);

    CFAL12864GDev.delay_ms(50);
    struct CFAL12864G_sensor_data gyr;
    rslt |= CFAL12864G_get_gyro_data(&gyr, &CFAL12864GDev);

  }
  else
  {
#ifndef SENSORS_IGNORE_IMU_FAIL
    DEBUG_PRINT("CFAL12864G Gyro SPI connection [FAIL]\n");
    isInit = false;
#endif
  }

  /* CFAL12864G ACCEL */
  rslt |= CFAL12864G_accel_switch_control(&CFAL12864GDev, CFAL12864G_ACCEL_POWER_ENABLE);
  CFAL12864GDev.delay_ms(5);

  rslt = CFAL12864G_accel_init(&CFAL12864GDev); // initialize the device
  if (rslt == BSTDR_OK)
  {
    DEBUG_PRINT("CFAL12864G Accel SPI connection [OK]\n");
    /* set power mode of accel */
    CFAL12864GDev.accel_cfg.power = CFAL12864G_ACCEL_PM_ACTIVE;
    rslt |= CFAL12864G_set_accel_power_mode(&CFAL12864GDev);
    CFAL12864GDev.delay_ms(10);

    /* set bandwidth and range of accel */
    CFAL12864GDev.accel_cfg.bw = CFAL12864G_ACCEL_BW_OSR4;
    CFAL12864GDev.accel_cfg.range = SENSORS_CFAL12864G_ACCEL_FS_CFG;
    CFAL12864GDev.accel_cfg.odr = CFAL12864G_ACCEL_ODR_1600_HZ;
    rslt |= CFAL12864G_set_accel_meas_conf(&CFAL12864GDev);

    struct CFAL12864G_sensor_data acc;
    rslt |= CFAL12864G_get_accel_data(&acc, &CFAL12864GDev);
  }
  else
  {
#ifndef SENSORS_IGNORE_IMU_FAIL
    DEBUG_PRINT("CFAL12864G Accel SPI connection [FAIL]\n");
    isInit = false;
#endif
  }

  /* BMP388 */
  bmp388Dev.dev_id = BMP3_I2C_ADDR_SEC;
  bmp388Dev.intf = BMP3_I2C_INTF;
  bmp388Dev.read = i2c_burst_read;
  bmp388Dev.write = i2c_burst_write;
  bmp388Dev.delay_ms = CFAL12864G_ms_delay;

  int i = 3;
  do {
    bmp388Dev.delay_ms(1);
    // For some reason it often doesn't work first time
    rslt = bmp3_init(&bmp388Dev);
  } while (rslt != BMP3_OK && i-- > 0);

  if (rslt == BMP3_OK)
  {
    isBarometerPresent = true;
    DEBUG_PRINT("BMP388 I2C connection [OK]\n");
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;
    /* Select the pressure and temperature sensor to be enabled */
    bmp388Dev.settings.press_en = BMP3_ENABLE;
    bmp388Dev.settings.temp_en = BMP3_ENABLE;
    /* Select the output data rate and oversampling settings for pressure and temperature */
    bmp388Dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    bmp388Dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    bmp388Dev.settings.odr_filter.odr = BMP3_ODR_50_HZ;
    bmp388Dev.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL | BMP3_IIR_FILTER_SEL;
    rslt = bmp3_set_sensor_settings(settings_sel, &bmp388Dev);

    /* Set the power mode to normal mode */
    bmp388Dev.settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp3_set_op_mode(&bmp388Dev);


    bmp388Dev.delay_ms(20); // wait before first read out
    // read out data
    /* Variable used to select the sensor component */
    uint8_t sensor_comp;
    /* Variable used to store the compensated data */
    struct bmp3_data data;

    /* Sensor component selection */
    sensor_comp = BMP3_PRESS | BMP3_TEMP;
    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
    rslt = bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);

    /* Print the temperature and pressure data */
//    DEBUG_PRINT("BMP388 T:%0.2f  P:%0.2f\n",data.temperature, data.pressure/100.0f);
    baroMeasDelayMin = SENSORS_DELAY_BARO;
  } else {
#ifndef SENSORS_IGNORE_BAROMETER_FAIL
    DEBUG_PRINT("BMP388 I2C connection [FAIL]\n");
    isInit = false;
    return;
#endif
  }

  // Init second order filer for accelerometer and gyro
  for (uint8_t i = 0; i < 3; i++)
  {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
  }

  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI / 180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI / 180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI / 180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI / 180);

  isInit = true;
}

static void sensorsTaskInit(void) {
  accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
  gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
  magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
  barometerDataQueue = xQueueCreate(1, sizeof(baro_t));

  xTaskCreate(sensorsTask, SENSORS_TASK_NAME, SENSORS_TASK_STACKSIZE, NULL, SENSORS_TASK_PRI, NULL);
}

static void sensorsInterruptInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  sensorsDataReady = xSemaphoreCreateBinary();
  dataReady = xSemaphoreCreateBinary();

  // Enable the interrupt on PC14
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14);

  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  portDISABLE_INTERRUPTS();
  EXTI_Init(&EXTI_InitStructure);
  EXTI_ClearITPendingBit(EXTI_Line14);
  portENABLE_INTERRUPTS();
}

void sensorsCFAL12864GSpiBmp388Init(void) {
  if (isInit) {
    return;
  }

  i2cdevInit(I2C3_DEV);
  spiInit();
  spiDMAInit();

  sensorsBiasObjInit(&gyroBiasRunning);
  sensorsDeviceInit();
  sensorsInterruptInit();
  sensorsTaskInit();
}

bool sensorsCFAL12864GSpiBmp388Test(void) {
  bool testStatus = true;

  if (!isInit) {
    DEBUG_PRINT("Uninitialized\n");
    testStatus = false;
  }

  return testStatus;
}

void sensorsCFAL12864GSpiBmp388DataAvailableCallback(void) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  imuIntTimestamp = usecTimestamp();
  xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

