#ifndef H_INF_POSITION_CONTROLLER_H
#define H_INF_POSITION_CONTROLLER_H

#include "stabilizer_types.h"

void hinfPositionControllerInit(const float updateDt);
void hinfPositionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state);

#endif
