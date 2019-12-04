/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
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
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "crtp_commander.h"

#include "commander.h"
#include "param.h"
#include "crtp.h"
#include "num.h"
#include "quatcompress.h"
#include "FreeRTOS.h"

/*
 * Add by Guojun Chen
 */

/* The generic commander format contains a packet type and data that has to be
 * decoded into a setpoint_t structure. The aim is to make it future-proof
 * by easily allowing        # while thrust < 10000:
        #     self._cf.commander.send_position(0, 0, start_height + (target_height - start_height) * (thrust / 20000.0))
        #     thrust += thrust_step * thrust_mult
        #     time.sleep(0.05) the addition of new packets for future use cases.
 *
 * The packet format is:
 * +------+==========================+
 * | TYPE |     DATA                 |
 * +------+==========================+
 *
 * The type is defined bellow together with a decoder function that should take
 * the data buffer in and fill up a setpoint_t structure.
 * The maximum data size is 29 bytes.
 */


/* 
 * position hold mode Decoder
 * Set the Crazyflie absolute height and relative distance in the world coordinate system
 */
struct posHoldPackets {
  float xDistance;    // relative distance
  float yDistance;    // relative distance
  float yawrate;      // deg/s
  float zDistance;    // m in the world frame of reference
} __attribute__((packed));

void crtpCommanderPosHoldDecodeSetpoint(setpoint_t *setpoint, CRTPPacket *pk) {
  memset(setpoint, 0, sizeof(setpoint_t));
  struct posHoldPackets *values = (struct posHoldPackets *)(((char*)pk->data) + 1);
  ASSERT(pk->size - 1 == sizeof(struct posHoldPackets));
  // set absolute height
  setpoint->mode.z = modeAbs;
  setpoint->position.z = values->zDistance;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = -values->yawrate;
  
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->position.x = values->xDistance;
  setpoint->position.y = values->yDistance;

  setpoint->velocity_body = false;
}

/* 
 * height hold mode Decoder
 * Set the Crazyflie absolute height and velocity in the body coordinate system
 */

struct heightHoldPackets {
  float xVelocity;    // relative velocity
  float yVelocity;    // relative velocity
  float yawrate;      // deg/s
  float zDistance;    // m in the world frame of reference
} __attribute__((packed));

void crtpCommanderHeightHoldDecodeSetpoint(setpoint_t *setpoint, CRTPPacket *pk) {
  memset(setpoint, 0, sizeof(setpoint_t));
  struct heightHoldPackets *values = (struct heightHoldPackets *)(((char*)pk->data) + 1);
  ASSERT(pk->size - 1 == sizeof(struct heightHoldPackets));
  // set absolute height
  setpoint->mode.z = modeAbs;
  setpoint->position.z = values->zDistance;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = -values->yawrate;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = values->xVelocity;
  setpoint->velocity.y = values->yVelocity;

  setpoint->velocity_body = true;
}