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
 * motors.c - Motor driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 */
#define DEBUG_MODULE "MTR-DRV"

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"

#include "motors.h"
#include "pm.h"
#include "debug.h"

//FreeRTOS includes
#include "task.h"

//Logging includes
#include "log.h"

static uint16_t motorsBLConvBitsTo16(uint16_t bits);
static uint16_t motorsBLConv16ToBits(uint16_t bits);
static uint16_t motorsConvBitsTo16(uint16_t bits);
static uint16_t motorsConv16ToBits(uint16_t bits);

uint32_t motor_ratios[] = {0, 0, 0, 0};

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

#include "motors_def_cf2.c"

const MotorPerifDef** motorMap;  /* Current map configuration */

const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5 };

static bool isInit = false;

uint32_t DMA_GPIO_DATA[2][MOTORS_DMA_BUFFER_SIZE];

/* Private functions */

static uint16_t motorsBLConvBitsTo16(uint16_t bits) {
  return (0xFFFF * (bits - MOTORS_BL_PWM_CNT_FOR_HIGH) / MOTORS_BL_PWM_CNT_FOR_HIGH);
}

static uint16_t motorsBLConv16ToBits(uint16_t bits) {
  return (MOTORS_BL_PWM_CNT_FOR_HIGH + ((bits * MOTORS_BL_PWM_CNT_FOR_HIGH) / 0xFFFF));
}

static uint16_t motorsConvBitsTo16(uint16_t bits) {
  return ((bits) << (16 - MOTORS_PWM_BITS));
}

static uint16_t motorsConv16ToBits(uint16_t bits) {
  return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

/* Public functions */

// Initialization. Will set all motors ratio to 0%
// The iflight version of motor init
void motorsInitIFlight(const MotorPerifDef** motorMapSelect) {
  int i;
  // Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  // NVIC_InitTypeDef NVIC_InitStructure;

  if (isInit) {
    return;
  }

  motorMap = motorMapSelect;

  // Timer configuration
  // Configure TIM8 base
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  TIM_TimeBaseStructure.TIM_Period = MOTORS_SIG_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_SIG_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);
  for (int k = 0; k < MOTORS_DMA_BUFFER_SIZE; k++) DMA_GPIO_DATA[0][k] = DMA_GPIO_DATA[1][k] = 0;

  for (i = 0; i < 4; i++) {
    // Configure the GPIO for the timer output
    MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPerif, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = motorMap[i]->gpioOType;
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);
    
    // Init dma buffer data
    for (int k = 0; k < MOTORS_DMA_DATA_SIZE; k++) {
      DMA_GPIO_DATA[i % 2][k] |= (k % 3 == 0 ? motorMap[i]->gpioPin : motorMap[i]->gpioPin << 16);
    }
    for (int k = MOTORS_DMA_DATA_SIZE; k < MOTORS_DMA_BUFFER_SIZE; k++) {
      DMA_GPIO_DATA[i % 2][k] |= motorMap[i]->gpioPin << 16;
    }

    // only two DMA streams are used, so avoid duplicated configuration
    if (i > 1) continue;

    // DMA configuration
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_7;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&motorMap[i]->gpioPort->BSRRL);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_GPIO_DATA[i % 2];
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = MOTORS_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(motorMap[i]->dmaXStreamY, &DMA_InitStructure);

    // do not use interrupt, it will block the mcu
    // NVIC_InitStructure.NVIC_IRQChannel = motorMap[i]->dmaNVICIRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);
    // DMA_ITConfig(motorMap[i]->dmaXStreamY, DMA_IT_TC, ENABLE);

    // channel configuration
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = motorMap[i]->timPolarity;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    // Configure Output Compare for PWM
    motorMap[i]->ocInit(motorMap[i]->tim, &TIM_OCInitStructure);
    motorMap[i]->preloadConfig(motorMap[i]->tim, TIM_OCPreload_Enable);
    TIM_DMACmd(motorMap[i]->tim, motorMap[i]->dmaSource, ENABLE);
    //MOTORS_TIM_DBG_CFG(motorMap[i]->timDbgStop, ENABLE);
    DMA_Cmd(motorMap[i]->dmaXStreamY, ENABLE);
  }

  TIM_Cmd(TIM8, ENABLE);
  motorsDrive = motorsSetValue;
  isInit = true;
}

// Contorl gpio using dma, test version
/*
void motorsInitTest(const MotorPerifDef** motorMapSelect) {
  // int i;
  // Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  if (isInit) {
    return;
  }

  // DMA2_Stream2, PD2
  MOTORS_RCC_GPIO_CMD(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DMA_InitStructure.DMA_Channel = DMA_Channel_7;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&GPIOD->BSRRL);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_GPIO_DATA;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = MOTORS_DMA_BUFFER_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);

  // DMA2_Stream3, PA1
  MOTORS_RCC_GPIO_CMD(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DMA_InitStructure.DMA_Channel = DMA_Channel_7;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&GPIOA->BSRRL);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_GPIO_DATA;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = MOTORS_DMA_BUFFER_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, ENABLE);

  // motorMap = motorMapSelect;
  //leo: try the PA1

  //Clock the gpio and the timers
  // MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPerif, ENABLE);
  // MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPowerswitchPerif, ENABLE);
  // MOTORS_RCC_TIM_CMD(motorMap[i]->timPerif, ENABLE);

  // If there is a power switch, as on Bolt, enable power to ESC by
  // switching on mosfet.
  // if (motorMap[i]->gpioPowerswitchPin != 0)
  // {
  //   GPIO_StructInit(&GPIO_InitStructure);
  //   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  //   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //   GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPowerswitchPin;
  //   GPIO_Init(motorMap[i]->gpioPowerswitchPort, &GPIO_InitStructure);
  //   GPIO_WriteBit(motorMap[i]->gpioPowerswitchPort, motorMap[i]->gpioPowerswitchPin, 1);
  // }

  // Configure the GPIO for the timer output
  // GPIO_StructInit(&GPIO_InitStructure);
  // GPIO_InitStructure.GPIO_Mode = MOTORS_GPIO_MODE;
  // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  // GPIO_InitStructure.GPIO_OType = motorMap[i]->gpioOType;
  // GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
  // GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);


  // Map timers to alternate functions
  // MOTORS_GPIO_AF_CFG(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, motorMap[i]->gpioAF);

  // Timer configuration
  // Configure TIM8 base
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);

  // configure TIM8 channals
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  // CC1
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
  TIM_DMACmd(TIM8, TIM_DMA_CC1, ENABLE);
  // CC2
  TIM_OC2Init(TIM8, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
  TIM_DMACmd(TIM8, TIM_DMA_CC2, ENABLE);

  TIM_Cmd(TIM8, ENABLE);
  DMA_Cmd(DMA2_Stream2, ENABLE);
  DMA_Cmd(DMA2_Stream3, ENABLE);

  isInit = true;
}
*/

// Initialization. Will set all motors ratio to 0%
// The standard version of motor init
void motorsInit(const MotorPerifDef** motorMapSelect)
{
  int i;
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  if (isInit)
  {
    // First to init will configure it
    return;
  }

  motorMap = motorMapSelect;

  DEBUG_PRINT("Using %s motor driver\n", motorMap[0]->drvType == BRUSHED ? "brushed" : "brushless");

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    //Clock the gpio and the timers
    MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPerif, ENABLE);
    MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPowerswitchPerif, ENABLE);
    MOTORS_RCC_TIM_CMD(motorMap[i]->timPerif, ENABLE);

    // If there is a power switch, as on Bolt, enable power to ESC by
    // switching on mosfet.
    if (motorMap[i]->gpioPowerswitchPin != 0)
    {
      GPIO_StructInit(&GPIO_InitStructure);
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPowerswitchPin;
      GPIO_Init(motorMap[i]->gpioPowerswitchPort, &GPIO_InitStructure);
      GPIO_WriteBit(motorMap[i]->gpioPowerswitchPort, motorMap[i]->gpioPowerswitchPin, 1);
    }

    // Configure the GPIO for the timer output
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = MOTORS_GPIO_MODE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = motorMap[i]->gpioOType;
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    MOTORS_GPIO_AF_CFG(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, motorMap[i]->gpioAF);

    //Timer configuration
    TIM_TimeBaseStructure.TIM_Period = motorMap[i]->timPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = motorMap[i]->timPrescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(motorMap[i]->tim, &TIM_TimeBaseStructure);

    // PWM channels configuration (All identical!)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    // leo: test the pulse value
    TIM_OCInitStructure.TIM_Pulse = 0; // TIM_Pulse / TIM_Period = duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = motorMap[i]->timPolarity;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    // Configure Output Compare for PWM
    motorMap[i]->ocInit(motorMap[i]->tim, &TIM_OCInitStructure);
    motorMap[i]->preloadConfig(motorMap[i]->tim, TIM_OCPreload_Enable);

    MOTORS_TIM_DBG_CFG(motorMap[i]->timDbgStop, ENABLE);
    //Enable the timer PWM outputs
    TIM_CtrlPWMOutputs(motorMap[i]->tim, ENABLE);
  }

  // Start the timers
  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    TIM_Cmd(motorMap[i]->tim, ENABLE);
  }
  motorsDrive = motorsSetRatio;
  isInit = true;
}

// we don't need this function
void motorsDeInit() {
  int i;
  GPIO_InitTypeDef GPIO_InitStructure;

  for (i = 0; i < NBR_OF_MOTORS; i++) {
    // Configure default
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    GPIO_PinAFConfig(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, 0x00);

    //Deinit timer
    TIM_DeInit(motorMap[i]->tim);
  }
}

bool motorsTest(void) {
  int i;

  for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++) {
    if (motorMap[i]->drvType == BRUSHED) {
#ifdef ACTIVATE_STARTUP_SOUND
      motorsBeep(MOTORS[i], true, testsound[i], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
      vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
      motorsBeep(MOTORS[i], false, 0, 0);
      vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#else
      (*motorsDrive)(MOTORS[i], MOTORS_TEST_RATIO);
      vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
      (*motorsDrive)(MOTORS[i], 0);
      vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#endif
    }
  }

  return isInit;
}

/**
  * @brief  set/clear the DMA data bit
  * @param  id: the id of motor, can be 0, 1, 2, 3
  * @param  bit: the index of data bit
  * @retval None
  */
static inline void setDMADataBit(uint16_t id, uint16_t bit) {
  DMA_GPIO_DATA[id % 2][bit * 3 + 1] &= ~motorMap[id]->gpioPin << 16;
  DMA_GPIO_DATA[id % 2][bit * 3 + 1] |= motorMap[id]->gpioPin;
}

static inline void clearDMADataBit(uint16_t id, uint16_t bit) {
  DMA_GPIO_DATA[id % 2][bit * 3 + 1] |= motorMap[id]->gpioPin << 16;
  DMA_GPIO_DATA[id % 2][bit * 3 + 1] &= ~motorMap[id]->gpioPin;
}

/**
  * @brief  set the output for esc board
  *         | 0 0 0 0, 0 0 0 0, 0 0 | 0 0 | 0 0 0 0  |
  *         | motor power value     | 0 0 | checksum |
  * @param  id: the id of motor, can be 0, 1, 2, 3
  * @param  value: value of motor output, between 0-1024
  * @retval None
  */
void motorsSetValue(uint32_t id, uint16_t value) {
  if (isInit) {
    ASSERT(id < NBR_OF_MOTORS);
    
    value = value / (float)0xFFFF * (float)MOTORS_MAX_OUTPUT;
    motor_ratios[id] = value;

    uint16_t u10 = 1 << 9;
    uint16_t check_sum = ((value >> 6) & 0xf) ^ ((value >> 2) & 0xf) ^ ((value << 2) & 0xf);

    // set bits 0-9, the output value
    for (int i = 0; i < 10; i++) {
      if (value & (u10 >> i))
        setDMADataBit(id, i);
      else clearDMADataBit(id, i);
    }

    // set bits 12-15, the checksum for the output value
    u10 = 1 << 15;
    for (int i = 12; i < 16; i++) {
      if (check_sum & (u10 >> i))
        setDMADataBit(id, i);
      else clearDMADataBit(id, i);
    }
  }
}

// Ithrust is thrust mapped for 65536 <==> 60 grams
void motorsSetRatio(uint32_t id, uint16_t ithrust) {
  if (isInit) {
    uint16_t ratio;

    ASSERT(id < NBR_OF_MOTORS);

    ratio = ithrust;

  #ifdef ENABLE_THRUST_BAT_COMPENSATED
    if (motorMap[id]->drvType == BRUSHED) {
      float thrust = ((float)ithrust / 65536.0f) * 60;
      float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
      float supply_voltage = pmGetBatteryVoltage();
      float percentage = volts / supply_voltage;
      percentage = percentage > 1.0f ? 1.0f : percentage;
      ratio = percentage * UINT16_MAX;
      motor_ratios[id] = ratio;

    }
  #endif
    if (motorMap[id]->drvType == BRUSHLESS) {
      motorMap[id]->setCompare(motorMap[id]->tim, motorsBLConv16ToBits(ratio));
    }
    else {
      motorMap[id]->setCompare(motorMap[id]->tim, motorsConv16ToBits(ratio));
    }
  }
}

int motorsGetRatio(uint32_t id) {
  int ratio;

  ASSERT(id < NBR_OF_MOTORS);
  if (motorMap[id]->drvType == BRUSHLESS) {
    ratio = motorsBLConvBitsTo16(motorMap[id]->getCompare(motorMap[id]->tim));
  }
  else {
    ratio = motorsConvBitsTo16(motorMap[id]->getCompare(motorMap[id]->tim));
  }

  return ratio;
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio) {
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  ASSERT(id < NBR_OF_MOTORS);

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  if (enable) {
    TIM_TimeBaseStructure.TIM_Prescaler = (5 - 1);
    TIM_TimeBaseStructure.TIM_Period = (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency);
  }
  else {
    TIM_TimeBaseStructure.TIM_Period = motorMap[id]->timPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = motorMap[id]->timPrescaler;
  }

  // Timer configuration
  TIM_TimeBaseInit(motorMap[id]->tim, &TIM_TimeBaseStructure);
  motorMap[id]->setCompare(motorMap[id]->tim, ratio);
}


// Play a tone with a given frequency and a specific duration in milliseconds (ms)
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec) {
  motorsBeep(MOTOR_M1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M3, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M4, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  vTaskDelay(M2T(duration_msec));
  motorsBeep(MOTOR_M1, false, frequency, 0);
  motorsBeep(MOTOR_M2, false, frequency, 0);
  motorsBeep(MOTOR_M3, false, frequency, 0);
  motorsBeep(MOTOR_M4, false, frequency, 0);
}

// Plays a melody from a note array
void motorsPlayMelody(uint16_t *notes) {
  int i = 0;
  uint16_t note;      // Note in hz
  uint16_t duration;  // Duration in ms

  do {
    note = notes[i++];
    duration = notes[i++];
    motorsPlayTone(note, duration);
  } while (duration != 0);
}

LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT32, m1_pwm, &motor_ratios[0])
LOG_ADD(LOG_UINT32, m2_pwm, &motor_ratios[1])
LOG_ADD(LOG_UINT32, m3_pwm, &motor_ratios[2])
LOG_ADD(LOG_UINT32, m4_pwm, &motor_ratios[3])
LOG_GROUP_STOP(pwm)
