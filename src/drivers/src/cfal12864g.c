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
#include "stabilizer_types.h"

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
static uint8_t positionY;
extern state_t state;

const unsigned char Font_08x08[96][8] = {
   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, //  32 = 0x20 = " "
   {0x00,0x00,0x60,0xFA,0xFA,0x60,0x00,0x00}, //  33 = 0x21 = "!"
   {0x00,0xE0,0xE0,0x00,0xE0,0xE0,0x00,0x00}, //  34 = 0x22 = """
   {0x28,0xFE,0xFE,0x28,0xFE,0xFE,0x28,0x00}, //  35 = 0x23 = "#"
   {0x24,0x74,0x54,0xD6,0xD6,0x5C,0x48,0x00}, //  36 = 0x24 = "$"
   {0x42,0x46,0x0C,0x18,0x30,0x62,0x42,0x00}, //  37 = 0x25 = "%"
   {0x0C,0x5E,0xF2,0xB2,0xEC,0x5E,0x12,0x00}, //  38 = 0x26 = "&"
   {0x00,0x00,0x20,0xE0,0xC0,0x00,0x00,0x00}, //  39 = 0x27 = "'"
   {0x00,0x38,0x7C,0xC6,0x82,0x00,0x00,0x00}, //  40 = 0x28 = "("
   {0x00,0x82,0xC6,0x7C,0x38,0x00,0x00,0x00}, //  41 = 0x29 = ")"
   {0x54,0x7C,0x7C,0x38,0x7C,0x7C,0x54,0x00}, //  42 = 0x2A = "*"
   {0x00,0x10,0x10,0x7C,0x7C,0x10,0x10,0x00}, //  43 = 0x2B = "+"
   {0x00,0x00,0x02,0x0E,0x0C,0x00,0x00,0x00}, //  44 = 0x2C = ","
   {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00}, //  45 = 0x2D = "-"
   {0x00,0x00,0x06,0x06,0x06,0x00,0x00,0x00}, //  46 = 0x2E = "."
   {0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00}, //  47 = 0x2F = "/"
   {0x7C,0xFE,0x8E,0x9A,0xB2,0xFE,0x7C,0x00}, //  48 = 0x30 = "0"
   {0x00,0x42,0x42,0xFE,0xFE,0x02,0x02,0x00}, //  49 = 0x31 = "1"
   {0x42,0xC6,0x8E,0x9A,0xB2,0xE6,0x46,0x00}, //  50 = 0x32 = "2"
   {0x44,0xC6,0x92,0x92,0x92,0xFE,0x6C,0x00}, //  51 = 0x33 = "3"
   {0x08,0x18,0x38,0x68,0xFE,0xFE,0x08,0x00}, //  52 = 0x34 = "4"
   {0xE4,0xE6,0xA2,0xA2,0xA2,0xBE,0x9C,0x00}, //  53 = 0x35 = "5"
   {0x7C,0xFE,0x92,0x92,0x92,0xDE,0x4C,0x00}, //  54 = 0x36 = "6"
   {0xC0,0xC0,0x80,0x8E,0x9E,0xF0,0xE0,0x00}, //  55 = 0x37 = "7"
   {0x6C,0xFE,0x92,0x92,0x92,0xFE,0x6C,0x00}, //  56 = 0x38 = "8"
   {0x64,0xF6,0x92,0x92,0x92,0xFE,0x7C,0x00}, //  57 = 0x39 = "9"
   {0x00,0x00,0x00,0x66,0x66,0x66,0x00,0x00}, //  58 = 0x3A = ":"
   {0x00,0x00,0x01,0x67,0x66,0x00,0x00,0x00}, //  59 = 0x3B = ";"
   {0x00,0x10,0x38,0x6C,0xC6,0x82,0x00,0x00}, //  60 = 0x3C = "<"
   {0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x00}, //  61 = 0x3D = "="
   {0x00,0x82,0xC6,0x6C,0x38,0x10,0x00,0x00}, //  62 = 0x3E = ">"
   {0x40,0xC0,0x80,0x8A,0x9A,0xF0,0x60,0x00}, //  63 = 0x3F = "?"
   {0x7C,0xFE,0x82,0x9A,0x9A,0xFA,0x72,0x00}, //  64 = 0x40 = "@"
   {0x3E,0x7E,0xC8,0x88,0xC8,0x7E,0x3E,0x00}, //  65 = 0x41 = "A"
   {0x82,0xFE,0xFE,0x92,0x92,0xFE,0x6C,0x00}, //  66 = 0x42 = "B"
   {0x38,0x7C,0xC6,0x82,0x82,0xC6,0x44,0x00}, //  67 = 0x43 = "C"
   {0x82,0xFE,0xFE,0x82,0xC6,0x7C,0x38,0x00}, //  68 = 0x44 = "D"
   {0xFE,0xFE,0x92,0x92,0x92,0x82,0xC6,0x00}, //  69 = 0x45 = "E"
   {0x82,0xFE,0xFE,0x92,0x90,0x90,0xC0,0x00}, //  70 = 0x46 = "F"
   {0x7C,0xFE,0x82,0x8A,0x8A,0xCE,0x4C,0x00}, //  71 = 0x47 = "G"
   {0xFE,0xFE,0x10,0x10,0x10,0xFE,0xFE,0x00}, //  72 = 0x48 = "H"
   {0x00,0x00,0x82,0xFE,0xFE,0x82,0x00,0x00}, //  73 = 0x49 = "I"
   {0x0C,0x0E,0x82,0xFE,0xFC,0x80,0x00,0x00}, //  74 = 0x4A = "J"
   {0xFE,0xFE,0x10,0x38,0x6C,0xC6,0x82,0x00}, //  75 = 0x4B = "K"
   {0x82,0xFE,0xFE,0x82,0x02,0x02,0x06,0x00}, //  76 = 0x4C = "L"
   {0xFE,0xFE,0x60,0x38,0x60,0xFE,0xFE,0x00}, //  77 = 0x4D = "M"
   {0xFE,0xFE,0x70,0x18,0x0C,0xFE,0xFE,0x00}, //  78 = 0x4E = "N"
   {0x7C,0xFE,0x82,0x82,0x82,0xFE,0x7C,0x00}, //  79 = 0x4F = "O"
   {0x82,0xFE,0xFE,0x92,0x90,0xF0,0x60,0x00}, //  80 = 0x50 = "P"
   {0x7C,0xFE,0x82,0x86,0x82,0xFF,0x7D,0x00}, //  81 = 0x51 = "Q"
   {0xFE,0xFE,0x90,0x98,0x9C,0xF6,0x62,0x00}, //  82 = 0x52 = "R"
   {0x64,0xF6,0x92,0x92,0x92,0xDE,0x4C,0x00}, //  83 = 0x53 = "S"
   {0x00,0xC0,0x82,0xFE,0xFE,0x82,0xC0,0x00}, //  84 = 0x54 = "T"
   {0xFC,0xFE,0x02,0x02,0x02,0xFE,0xFC,0x00}, //  85 = 0x55 = "U"
   {0xF0,0xF8,0x0C,0x06,0x0C,0xF8,0xF0,0x00}, //  86 = 0x56 = "V"
   {0xFE,0xFE,0x0C,0x38,0x0C,0xFE,0xFE,0x00}, //  87 = 0x57 = "W"
   {0x82,0xC6,0x7C,0x38,0x7C,0xC6,0x82,0x00}, //  88 = 0x58 = "X"
   {0x00,0xE0,0xF2,0x1E,0x1E,0xF2,0xE0,0x00}, //  89 = 0x59 = "Y"
   {0xC2,0x86,0x8E,0x9A,0xB2,0xE2,0xC6,0x00}, //  90 = 0x5A = "Z"
   {0x00,0xFE,0xFE,0x82,0x82,0x82,0x00,0x00}, //  91 = 0x5B = "["
   {0x80,0xC0,0x60,0x30,0x18,0x0C,0x06,0x00}, //  92 = 0x5C = "\"
   {0x00,0x82,0x82,0x82,0xFE,0xFE,0x00,0x00}, //  93 = 0x5D = "]"
   {0x10,0x30,0x60,0xC0,0x60,0x30,0x10,0x00}, //  94 = 0x5E = "^"
   {0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01}, //  95 = 0x5F = "_"
   {0x00,0x00,0xC0,0xE0,0x20,0x00,0x00,0x00}, //  96 = 0x60 = "`"
   {0x04,0x2E,0x2A,0x2A,0x3E,0x1E,0x02,0x00}, //  97 = 0x61 = "a"
   {0x82,0xFE,0xFE,0x22,0x22,0x3E,0x1C,0x00}, //  98 = 0x62 = "b"
   {0x1C,0x3E,0x22,0x22,0x22,0x36,0x14,0x00}, //  99 = 0x63 = "c"
   {0x1C,0x3E,0x22,0xA2,0xFE,0xFE,0x02,0x00}, // 100 = 0x64 = "d"
   {0x1C,0x3E,0x2A,0x2A,0x2A,0x3A,0x18,0x00}, // 101 = 0x65 = "e"
   {0x10,0x12,0x7E,0xFE,0x92,0xD0,0x40,0x00}, // 102 = 0x66 = "f"
   {0x18,0x3D,0x25,0x25,0x15,0x3F,0x3E,0x00}, // 103 = 0x67 = "g"
   {0x82,0xFE,0xFE,0x20,0x20,0x3E,0x1E,0x00}, // 104 = 0x68 = "h"
   {0x00,0x00,0x22,0xBE,0xBE,0x02,0x00,0x00}, // 105 = 0x69 = "i"
   {0x02,0x03,0x01,0x21,0xBF,0xBE,0x00,0x00}, // 106 = 0x6A = "j"
   {0x82,0xFE,0xFE,0x08,0x1C,0x36,0x22,0x00}, // 107 = 0x6B = "k"
   {0x00,0x00,0x00,0xFE,0xFE,0x02,0x00,0x00}, // 108 = 0x6C = "l"
   {0x1E,0x3E,0x30,0x1C,0x30,0x3E,0x1E,0x00}, // 109 = 0x6D = "m"
   {0x20,0x3E,0x1E,0x20,0x20,0x3E,0x1E,0x00}, // 110 = 0x6E = "n"
   {0x1C,0x3E,0x22,0x22,0x22,0x3E,0x1C,0x00}, // 111 = 0x6F = "o"
   {0x21,0x3F,0x1F,0x25,0x24,0x3C,0x18,0x00}, // 112 = 0x70 = "p"
   {0x18,0x3C,0x24,0x25,0x1F,0x3F,0x21,0x00}, // 113 = 0x71 = "q"
   {0x22,0x3E,0x1E,0x22,0x20,0x30,0x10,0x00}, // 114 = 0x72 = "r"
   {0x10,0x3A,0x2A,0x2A,0x2A,0x2E,0x04,0x00}, // 115 = 0x73 = "s"
   {0x20,0x20,0xFC,0xFE,0x22,0x26,0x04,0x00}, // 116 = 0x74 = "t"
   {0x3C,0x3E,0x02,0x02,0x3C,0x3E,0x02,0x00}, // 117 = 0x75 = "u"
   {0x30,0x38,0x0C,0x06,0x0C,0x38,0x30,0x00}, // 118 = 0x76 = "v"
   {0x3C,0x3E,0x06,0x0C,0x06,0x3E,0x3C,0x00}, // 119 = 0x77 = "w"
   {0x22,0x36,0x1C,0x08,0x1C,0x36,0x22,0x00}, // 120 = 0x78 = "x"
   {0x38,0x3D,0x05,0x05,0x09,0x3F,0x3E,0x00}, // 121 = 0x79 = "y"
   {0x32,0x26,0x2E,0x3A,0x32,0x26,0x00,0x00}, // 122 = 0x7A = "z"
   {0x00,0x10,0x10,0x7C,0xEE,0x82,0x82,0x00}, // 123 = 0x7B = "{"
   {0x00,0x00,0x00,0xEE,0xEE,0x00,0x00,0x00}, // 124 = 0x7C = "|"
   {0x00,0x82,0x82,0xEE,0x7C,0x10,0x10,0x00}, // 125 = 0x7D = "}"
   {0x40,0xC0,0x80,0xC0,0x40,0xC0,0x80,0x00}, // 126 = 0x7E = "~"
   {0x02,0x0E,0x3E,0x72,0x3E,0x0E,0x02,0x00} // 127 = 0x7F = ""
 };

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
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
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

void refreshContent() {
  // textContent_t ct;
  // if (screenTextGet(&ct)) {
  //   memset(SPITxBuffer, 0, sizeof(SPITxBuffer));
  //   // update content
  // }
  // int ct;
  // uint8_t page;
  for (int page = 0; page < 8; page++) {
    //Set the LCD to the left of this line.
    SPI_Cmd(CFAL12864G_SPI, ENABLE);
    SetAddress(0, page);
    // Move across the columns, alternating the two data
    // Select the LCD's data register
    SPI_Cmd(CFAL12864G_SPI, DISABLE);

    CFAL12864G_SPI_TX_DMA_STREAM->M0AR = (uint32_t) SPITxBuffer[page];
    
    DMA_ClearFlag(CFAL12864G_SPI_TX_DMA_STREAM, DMA_FLAG_FEIF7|DMA_FLAG_DMEIF7|
                DMA_FLAG_TEIF7|DMA_FLAG_HTIF7|DMA_FLAG_TCIF7);
    DMA_ITConfig(CFAL12864G_SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);

    DMA_Cmd(CFAL12864G_SPI_TX_DMA_STREAM, ENABLE);
    SPI_I2S_DMACmd(CFAL12864G_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

    SET_RS; // key for corrent settings, put SET_RS here !!!
    SPI_Cmd(CFAL12864G_SPI, ENABLE);
    xSemaphoreTake(SPITxDMAComplete, portMAX_DELAY);

  }
}

void clearBuffer(uint8_t v) {
  uint8_t *tmp = SPITxBuffer[0];
  for (int x = 0; x < 1024; x++) tmp[x] = v;
}

void put_string(uint8_t x, uint8_t y, uint8_t Field_Width, const char *input) {
  uint8_t Terminator_Found;
  uint8_t Characters_Placed;
  uint8_t this_character;
  uint8_t *LCD_Memory;
  uint8_t column;
  uint8_t row;
  word_union_t Clearing_Mask;
  word_union_t Pixel_Data;

  //Get the first row of the display character.
  row = y >> 3;
  //Calculate the address of the first uint8_t in display in LCD_Memory
  LCD_Memory = &SPITxBuffer[row][x];

  //Calculate Clearing_Mask, the vertical mask that we will and with
  //LCD_Memory to clear the space before we or in the data from the
  //font. It is 9 pixels.
  Clearing_Mask.as_word =~ (0x01FF << (y & 0x07));

  //Clear the first col to the left of the string.
  LCD_Memory[0] &= Clearing_Mask.as_bytes[0];
  if (row < 7)
    LCD_Memory[128] &= Clearing_Mask.as_bytes[1];
  LCD_Memory++;

  //Initialize Terminator_Found.
  Terminator_Found = 0;
  //Move across the field. We will either put the character or a blank
  //in every position of Field_Width.
  for (Characters_Placed = 0; Characters_Placed < Field_Width; Characters_Placed++) {
    //If we have not passed the terminator, then get the next
    //character in the string. If we find the terminator,
    //remember that we are out of characters.
    if (!Terminator_Found) {
      this_character = *input++;
      if (!this_character) {
        Terminator_Found = 1;
        this_character = ' ';
      }
    } else this_character = ' ';
    //Get a pointer into the font information for this
    //character.
//    Font_Pointer=&Font_08x08[(this_character-FONT_08X08_BASE)][0];

    //Write the eight columns of this character.
    for (column = 0; column <= 7; column++) {
      //Clear the correct bits in this row and the next row down.
      LCD_Memory[0] &= Clearing_Mask.as_bytes[0];
      if (row < 7)
        LCD_Memory[128] &= Clearing_Mask.as_bytes[1];
      // Get the font data, convert it to a word and shift it down. Leave
      // one blank row of pixels above as a spacer.
      // Pixel_Data.as_word=((uword)(pgm_read_byte_near(Font_Pointer++)))<<((y&0x07)+1);
      Pixel_Data.as_word = ((uint16_t)(Font_08x08[this_character - FONT_08X08_BASE][column])) << ((y & 0x07) + 1);


      
      //Set the correct bits in this row and the next row down.
      LCD_Memory[0] |= Pixel_Data.as_bytes[0];
      if (row < 7)
        LCD_Memory[128] |= Pixel_Data.as_bytes[1];
      LCD_Memory++;
    }
  }
}

void horizontal_line(uint8_t x1, uint8_t y, uint8_t x2) {
  uint8_t *LCD_Memory;
  uint8_t column;
  uint8_t Set_Mask;

  if ((x2 < x1) || (127 < x1) || (127 < x2) || (63 < y))
    return;

  // Calculate the address of the first uint8_t in display in LCD_Memory
  LCD_Memory = &SPITxBuffer[y >> 3][x1];

  // Calculate Set_Mask, the vertical mask that we will or with
  // LCD_Memory to clear the space before we or in the data from the
  // font. It is 9 pixels.
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
    positionY = state.position.z * 150 - 30;
    positionY = positionY > 0 ? positionY : 0;
    // horizontal_line(110, cnt, 127);
    put_string(0, positionY, 12, "A MESSAGE");
    // DEBUG_PRINT("p: %d\n", (int)(state.position.z * 100));
    refreshContent();

    cnt++;
    // if (cnt % 1 == 0)
    //   DEBUG_PRINT("1round\n");
    if (cnt > 20) cnt = 0;
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