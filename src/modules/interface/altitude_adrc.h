#ifndef ALTITUDE_ADRC_H_
#define ALTITUDE_ADRC_H_

#include "stabilizer_types.h"
#include "pid.h"

double fal(double eps, double alpha, double delta);
double fhan(double v1, double v2, double r, double h);
void altitudeADRCInit(const float updateDt);
bool altitudeADRCTest();
void altitudeADRC(float* thrust, float zDesired, float zActual, float zRateActual);
float altitudeADRCSimplified(float* thrust, float zDesired, float zActual, float zRateActual);
float altitudeADRCPID(PidObject* pidZ, PidObject* pidZRate, float* thrust, float zDesired, float zActual, float zRateActual);


#endif