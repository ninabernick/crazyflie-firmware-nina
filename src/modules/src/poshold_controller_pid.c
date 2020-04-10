/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 * poshold_estimator_pid.c: PID-based implementation of the position hold controller
 */

#define DEBUG_MODULE "PPID"

#include <math.h>
#include "num.h"

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"
#include "debug.h"

static const float thrustScale = 1000.0f;

#define POSHOLD_LPF_CUTOFF_FREQ 20.0f
#define POSHOLD_LPF_ENABLE true

/* CF2 */
// #define thrustBase  36000
// #define thrustMin   20000

/* iFlight with 2S */
// #define thrustBase  30000
// #define thrustMin   20000
#define thrustBase  32000
#define thrustMin   20000

/* iFlight with 3S */
//#define thrustBase  20000
//#define thrustMin   15000

// PID coefficiencies
#define PID_X_RATE_KP  25.0
#define PID_X_RATE_KI  2.0
#define PID_X_RATE_KD  0.0
#define PID_X_RATE_INTEGRATION_LIMIT   25.0

#define PID_Y_RATE_KP  25.0
#define PID_Y_RATE_KI  2.0
#define PID_Y_RATE_KD  0.0
#define PID_Y_RATE_INTEGRATION_LIMIT   25.0

// #define PID_Z_RATE_KP  25.0
// #define PID_Z_RATE_KI  15.0
// #define PID_Z_RATE_KD  1.0
#define PID_Z_RATE_KP  7.0
#define PID_Z_RATE_KI  2.0
#define PID_Z_RATE_KD  0.3
#define PID_Z_RATE_INTEGRATION_LIMIT   30.0

#define PID_X_KP  2.0
#define PID_X_KI  0.5
#define PID_X_KD  0.35
#define PID_X_INTEGRATION_LIMIT   2.0

#define PID_Y_KP  2.0
#define PID_Y_KI  0.5
#define PID_Y_KD  0.35
#define PID_Y_INTEGRATION_LIMIT   2.0

#define PID_Z_KP  10.0
#define PID_Z_KI  2.0
#define PID_Z_KD  1.0
#define PID_Z_INTEGRATION_LIMIT   2.0

PidObject pidXRate;
PidObject pidYRate;
PidObject pidZRate;
PidObject pidX;
PidObject pidY;
PidObject pidZ;

static bool isInit;

bool posHoldControllerTest() {
  return isInit;
}

void posHoldControllerInit(const float updateDt) {
  if (isInit)
    return;
  pidInit(&pidXRate, 0, PID_X_RATE_KP, PID_X_RATE_KI, PID_X_RATE_KD,
      updateDt, POSHOLD_RATE, POSHOLD_LPF_CUTOFF_FREQ, POSHOLD_LPF_ENABLE);
  pidInit(&pidYRate, 0, PID_Y_RATE_KP, PID_Y_RATE_KI, PID_Y_RATE_KD,
      updateDt, POSHOLD_RATE, POSHOLD_LPF_CUTOFF_FREQ, POSHOLD_LPF_ENABLE);
  pidInit(&pidZRate, 0, PID_Z_RATE_KP, PID_Z_RATE_KI, PID_Z_RATE_KD,
      updateDt, POSHOLD_RATE, POSHOLD_LPF_CUTOFF_FREQ, POSHOLD_LPF_ENABLE);

  pidSetIntegralLimit(&pidXRate,  PID_X_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYRate,  PID_Y_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidZRate,  PID_Z_RATE_INTEGRATION_LIMIT);

  pidInit(&pidX, 0, PID_X_KP, PID_X_KI, PID_X_KD,
      updateDt, POSHOLD_RATE, POSHOLD_LPF_CUTOFF_FREQ, POSHOLD_LPF_ENABLE);
  pidInit(&pidY, 0, PID_Y_KP, PID_Y_KI, PID_Y_KD,
      updateDt, POSHOLD_RATE, POSHOLD_LPF_CUTOFF_FREQ, POSHOLD_LPF_ENABLE);
  pidInit(&pidZ, 0, PID_Z_KP, PID_Z_KI, PID_Z_KD,
      updateDt, POSHOLD_RATE, POSHOLD_LPF_CUTOFF_FREQ, POSHOLD_LPF_ENABLE);

  pidSetIntegralLimit(&pidX,  PID_X_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidY,  PID_Y_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidZ,  PID_Z_INTEGRATION_LIMIT);

  isInit = true;
}

void posHoldController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state) {
  float cos_yaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sin_yaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  float body_vx = setpoint->velocity.x;
  float body_vy = setpoint->velocity.y;


  // leonana: debug the height
  // static int cnt = 0;
  // if (cnt++ % 5 == 0) {
  //   int tmp = 100 * state->position.x;
  //   DEBUG_PRINT("%c%c\n", (tmp / 10) + 48, (tmp % 10) + 48);
  // }
  

  // X, Y
  if (setpoint->mode.x == modeAbs) {
    // set world axis absolute x velocity
    pidSetDesired(&pidX, setpoint->position.x);
    setpoint->velocity.x = pidUpdate(&pidX, state->position.x, true);
  } else if (setpoint->velocity_body) {
    // convert body axis to world axis
    setpoint->velocity.x = body_vx * cos_yaw - body_vy * sin_yaw;
  }

  if (setpoint->mode.y == modeAbs) {
    // set world axis absolute y velocity
    pidSetDesired(&pidY, setpoint->position.y);
    setpoint->velocity.y = pidUpdate(&pidY, state->position.y, true);
  } else if (setpoint->velocity_body) {
    // convert body axis to world axis
    setpoint->velocity.y = body_vy * cos_yaw + body_vx * sin_yaw;
  }

  if (setpoint->mode.z == modeAbs) {
    // keep height stable
    pidSetDesired(&pidZ, setpoint->position.z);
    setpoint->velocity.z = pidUpdate(&pidZ, state->position.z, true);
  }

  // Roll and Pitch (X and Y)
  pidSetDesired(&pidXRate, setpoint->velocity.x);
  float XRaw = pidUpdate(&pidXRate, state->velocity.x, true);
  pidSetDesired(&pidYRate, setpoint->velocity.y);
  float YRaw = pidUpdate(&pidYRate, state->velocity.y, true);

  attitude->pitch = -(XRaw  * cos_yaw) - (YRaw * sin_yaw);
  attitude->roll  = -(YRaw * cos_yaw) + (XRaw  * sin_yaw);

  // Thrust
  pidSetDesired(&pidZRate, setpoint->velocity.z);
  float thrustRaw = pidUpdate(&pidZRate, state->velocity.z, true);
  *thrust = thrustRaw * thrustScale + thrustBase;
  // Check for minimum thrust
  if (*thrust < thrustMin) {
    *thrust = thrustMin;
  }
}

void posHoldControllerResetAllPID() {
  pidReset(&pidXRate);
  pidReset(&pidYRate);
  pidReset(&pidZRate);
  pidReset(&pidX);
  pidReset(&pidY);
  pidReset(&pidZ);
}
