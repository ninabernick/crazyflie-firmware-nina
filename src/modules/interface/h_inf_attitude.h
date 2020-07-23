#ifndef H_INF_ATTITUDE_H
#define H_INF_ATTITUDE_H

#include "stabilizer_types.h"

void hInfAttitudeInit(const float updateDt);
void hInfAttitude(float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float rollRateActual, float pitchRateActual, float yawRateActual);

void hInfAttGetActuatorOutput(float* roll, float* pitch, float* yaw);

#endif
