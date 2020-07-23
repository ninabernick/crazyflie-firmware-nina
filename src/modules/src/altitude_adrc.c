/* An active disturbance rejection controller for altitude. 
Based off of "From PID to Active Disturbance Rejection Control" by J. Han and
"A Novel Robust Attitude Control for Quadrotor Aircraft Subject to Actuator
 Faults and Wind Gusts" by Guo et. al.*/

#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "FreeRTOS.h"


#include "pid.h"
#include "param.h"
#include "log.h"
#include "stabilizer_types.h"
#include "debug.h"
#include "pid.h"

#define TIMESTEP .01f
#define thrustBase  40000
#define thrustMin   20000
static const float thrustScale = 1000.0;


static float r = 2;
static float beta1;
static float beta2;
static float beta3;
static float b0 = 100;
static float c = 1;
static float h1 = 4*TIMESTEP;
static float thrustOld;
static float B = 2.0;
static float wc = 5.0;

static bool isInit;
static float v1old = 0.0;
static float v2old = 0.0;

static float zSetpoint;
static float stateZ;
static float z1old = 0.0;
static float z2old = 0.0;
static float z3old = 0.0;
static float v2inc;
static float u0;
static float thrustScaled;

static inline float signum(float x) {
    if (fabs(x) < .00001) {
        return 0;
    }
    return x/(float)fabs(x);
}

//from Han
static inline float fhan(float v1, float v2, float r, float h){
    float d = h*powf(r, 2);
    float a = h*v2;
    float y = v1 + a;
    float a1 = powf(d*(d + 8*(float)fabs(y)), .5);
    float a2 = a + signum(y)*(a1 - d)/2;
    float sy = (signum(y + d) - signum(y -d))/2;
    float a_final = (a + y - a2)*sy + a2;
    float sa = (signum(a_final + d) -signum(a_final-d))/2;
    return -r*((a_final/d) - signum(a_final))*sa - r*signum(a_final);

}

static inline float fal(float eps, float alpha, float delta) {
    if ((float)fabs(eps) > delta) {
        return -(float)(powf(fabs(eps), alpha))*signum(eps);
    }
    else {
        return (float)(eps/((float)pow(delta, 1-alpha)));
    }
}

void altitudeADRCInit(const float updateDt){
    if (isInit) {
        return;
    }
    // Betas initialized for nonlinear feedback gain
    beta1 = 1;
    beta2 = 1/(2*powf(TIMESTEP, .5));
    beta3 = 2/(25*powf(TIMESTEP, 1.2));
    z1old = 0.0;
    z2old = 0.0;
    z3old = 0.0;
    thrustOld = 0;


    isInit = true;
}

bool altitudeADRCTest() {
  return isInit;
}

//this has very inaccurate z1 tracking of the actual height
void altitudeADRC(float* thrust, float zDesired, float zActual, float zRateActual) {
    zSetpoint = zDesired;
    stateZ = zActual;

    //tracking differentiator -- get desired trajectory.
    float v1new = v1old + TIMESTEP*v2old;
    v2inc = fhan(v1old - zDesired, v2old, r, TIMESTEP);
    float v2new = v2old + TIMESTEP*v2inc;
    DEBUG_PRINT("V1 new: %f, v2 new: %f\n", (double)v1new, (double)v2new);

    //Extended state observer
    float err = z1old - zActual;
    float fe = fal(err, .5, TIMESTEP);
    float fe1 = fal(err, .25, TIMESTEP);
    float z1new = z1old + TIMESTEP*z2old- beta1*err;
    float z2new = z2old + TIMESTEP*(z3old + b0*thrustOld) - beta2*fe;
    float z3new = z3old - beta3*fe1;

    //NLSEF
    float eps1 = v1new - z1new; // position error
    float eps2 = v2new - z2new; //velocity error
    //float u = -(fhan(eps1, c*eps2, r, h1) + z3new)/b0;
    float u = (10*eps1 + 1*eps2 - z3new)/b0;
    thrustScaled = (u*thrustScale + thrustBase);


    //update variables
    thrustOld = u;
    z1old = z1new;
    z2old = z2new;
    z3old = z3new;
    v1old = v1new;
    v2old = v2new;




}
//only ESO, linear error feedback with disturbance. This performs best, but has a constant error when tracking height.
float altitudeADRCSimplified(float* thrust, float zDesired, float zActual, float zRateActual) {
    zSetpoint = zDesired;
    stateZ = zActual;
    beta1 = (float)12.0*wc;
    beta2 = (float)48.0*powf(wc, 2.0);
    beta3 = (float)192.0*powf(wc, 2.0);
    float kp = powf(wc, 2);
    float kd = (float)2.0*wc;
    //tracking differentiator -- get desired trajectory. This doesn not work well and blows up.
    float v1new = v1old + TIMESTEP*v2old;
    v2inc = fhan(v1old - zDesired, v2old, r, TIMESTEP);
    float v2new = v2old + TIMESTEP*v2inc;
    DEBUG_PRINT("V1 new: %f, v2 new: %f\n", (double)v1new, (double)v2new);


    float eps = zActual - z1old;
    float z1new = z1old + TIMESTEP*(z2old + beta1*eps);
    float z2new = z2old + TIMESTEP*(z3old + beta2*eps + B*thrustOld);
    float z3new = beta3*eps;

    u0 = kp*(zDesired - z1new) - kd*(zRateActual); //linear error feedback of position and velocity
    float u = u0 - z3new/B; //incorporate z3, a measure of the disturbance

    //update variables for next iteration
    v1old = v1new;
    v2old = v2new;
    thrustOld = u;
    z1old = z1new;
    z2old = z2new;
    z3old = z3new;

    return u; // this is unscaled. Make sure to scale before using.


}
//have not tested this one yet
float altitudeADRCPID(PidObject* pidZ, PidObject* pidZRate, float* thrust, float zDesired, float zActual, float zRateActual) {
    zSetpoint = zDesired;
    stateZ = zActual;
    //tracking differentiator -- get desired trajectory
    float v1new = v1old + TIMESTEP*v2old;
    v2inc = fhan(v1old - zDesired, v2old, r, TIMESTEP);
    float v2new = v2old + TIMESTEP*v2inc;
    DEBUG_PRINT("V1 new: %f, v2 new: %f\n", (double)v1new, (double)v2new);

    float eps1 = v1new - zActual;
    float fe1 = fal(eps1, .25, TIMESTEP);
    float z3new = z3old - beta3*fe1;

    pidSetDesired(pidZ, v1new);
    float velRaw = pidUpdate(pidZ, zActual, true);

    pidSetDesired(pidZRate, velRaw);
    float thrustRaw = pidUpdate(pidZRate, zRateActual, true);
    z3old = z3new;
    v1old = v1new;
    v2old = v2new;
    
    return (thrustRaw - z3old/b0);
}

PARAM_GROUP_START(altitude)
PARAM_ADD(PARAM_FLOAT, c, &c)
PARAM_ADD(PARAM_FLOAT, h1, &h1)
PARAM_ADD(PARAM_FLOAT, b0, &b0)
PARAM_ADD(PARAM_FLOAT, wc, &wc)
PARAM_ADD(PARAM_FLOAT, B, &B)
PARAM_GROUP_STOP(altitude)