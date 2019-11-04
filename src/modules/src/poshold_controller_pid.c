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

#include <math.h>
#include "num.h"

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"

static const float thrustScale = 1000.0f;

#define POSHOLD_LPF_CUTOFF_FREQ 20.0f
#define POSHOLD_LPF_ENABLE true

#define thrustBase  36000
#define thrustMin   20000

#define PID_X_RATE_KP  25.0
#define PID_X_RATE_KI  1.0
#define PID_X_RATE_KD  0.0
#define PID_X_RATE_INTEGRATION_LIMIT    25.0

#define PID_Y_RATE_KP  25.0
#define PID_Y_RATE_KI  1.0
#define PID_Y_RATE_KD  0.0
#define PID_Y_RATE_INTEGRATION_LIMIT   25.0

#define PID_Z_RATE_KP  25.0
#define PID_Z_RATE_KI  15.0
#define PID_Z_RATE_KD  0.0
#define PID_Z_RATE_INTEGRATION_LIMIT     (UINT16_MAX / 2000)

#define PID_X_KP  2.0
#define PID_X_KI  1.0
#define PID_X_KD  0.0
#define PID_X_INTEGRATION_LIMIT    2.0

#define PID_Y_KP  2.0
#define PID_Y_KI  1.0
#define PID_Y_KD  0.0
#define PID_Y_INTEGRATION_LIMIT   2.0

#define PID_Z_KP  2.0
#define PID_Z_KI  0.5
#define PID_Z_KD  0.35
#define PID_Z_INTEGRATION_LIMIT     2.0

PidObject pidXRate;
PidObject pidYRate;
PidObject pidZRate;
PidObject pidX;
PidObject pidY;
PidObject pidZ;

static bool isInit;


void posHoldControllerInit(const float updateDt)
{
  if(isInit)
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
}

void posHoldController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state)
{
  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  float bodyvx = setpoint->velocity.x;
  float bodyvy = setpoint->velocity.y;

  // X, Y
  if (setpoint->mode.x == modeAbs) {
    pidSetDesired(&pidX, setpoint->position.x);
    setpoint->velocity.x = pidUpdate(&pidX, state->position.x, true);
  } else if (setpoint->velocity_body) {
    setpoint->velocity.x = bodyvx * cosyaw - bodyvy * sinyaw;
  }
  if (setpoint->mode.y == modeAbs) {
    pidSetDesired(&pidY, setpoint->position.y);
    setpoint->velocity.y = pidUpdate(&pidY, state->position.y, true);
  } else if (setpoint->velocity_body) {
    setpoint->velocity.y = bodyvy * cosyaw + bodyvx * sinyaw;
  }
  if (setpoint->mode.z == modeAbs) {
    pidSetDesired(&pidZ, setpoint->position.z);
    setpoint->velocity.z = pidUpdate(&pidZ, state->position.z, true);
  }

  velocityController(thrust, attitude, setpoint, state);
}

void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state)
{
  // Roll and Pitch (X and Y)
  pidSetDesired(&pidXRate, setpoint->velocity.x);
  pidSetDesired(&pidYRate, setpoint->velocity.y);
  float rollRaw  = pidUpdate(&pidXRate, state->velocity.x, true);
  float pitchRaw = pidUpdate(&pidYRate, state->velocity.y, true);

  float yawRad = state->attitude.yaw * (float)M_PI / 180;
  attitude->pitch = -(rollRaw  * cosf(yawRad)) - (pitchRaw * sinf(yawRad));
  attitude->roll  = -(pitchRaw * cosf(yawRad)) + (rollRaw  * sinf(yawRad));

  // Thrust
  pidSetDesired(&pidZRate, setpoint->velocity.z);
  float thrustRaw = pidUpdate(&pidZRate, state->velocity.z, true);
  *thrust = thrustRaw * thrustScale + thrustBase;
  // Check for minimum thrust
  if (*thrust < thrustMin) {
    *thrust = thrustMin;
  }
}

void posHoldControllerResetAllPID()
{
  pidReset(&pidXRate);
  pidReset(&pidYRate);
  pidReset(&pidZRate);
  pidReset(&pidX);
  pidReset(&pidY);
  pidReset(&pidZ);
}
