/* A H-infinity based position controller that determines output thrust, roll, and pitch. 
Based on "Nonlinear H∞ Measurement Feedback Control Algorithm for Quadrotor
 Position Tracking" by Rekabi et al.*/
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include "num.h"


#include "log.h"
#include "param.h"


#include "h_inf_position_controller.h"

#define thrustBase  36000
#define thrustMin   20000
#define cf_mass_newtons 0.323f
#define drag_const 0.01f

static const float thrustScale = 1000.0f;



static float errInt[3]; //accumulated x, y, z error
static float dt;
static float err[3]; //x, y, z error
float w1;
float w2;
float w3;
float r;
float pz;
float iz;
float dz;
float kp;
float ki;
float kd;

static bool isInit;





void hinfPositionControllerInit(const float updateDt) {
    if (isInit) {
        return;
    }
    memset(&errInt, 0, sizeof(errInt));
    memset(&err, 0, sizeof(err));
    dt = updateDt;
    w1 = 1.2;
    w2 = 8;
    w3 = 2;
    r = 1;
    kp = powf(powf(w2, 2) + 2*w1*w3, 0.5)/(cf_mass_newtons*r*r*w1) + (w3/w1) + powf(powf(w2, 2) + 2*w1*w3, 0.5)*drag_const/(cf_mass_newtons*w1);
    ki = w3/(cf_mass_newtons*r*r*w1) + w3*drag_const/(cf_mass_newtons*w1);
    kd = powf(cf_mass_newtons*r*r, -1) + powf(powf(w2, 2) + 2*w1*w3, 0.5)/w1 + drag_const/cf_mass_newtons;
    isInit = true;
}

void hinfPositionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state) {
    float errz = state->position.z - setpoint->position.z;
    float errx = state->position.x - setpoint->position.x;
    float erry = state->position.y - setpoint->position.y;

    float zderiv = (errz - err[2])/dt;
    float xderiv = (errx - err[0])/dt;
    float yderiv = (erry - err[1])/dt;

    err[0] = errx;
    err[1] = erry;
    err[2] = errz;

    errInt[0] += errx*dt;
    errInt[1] += erry*dt;
    errInt[2] += errz*dt;

    //calculate gains
    pz = kp*err[2];
    iz = ki*errInt[2];
    dz = kd*zderiv;

    float fx = drag_const*state->velocity.x - cf_mass_newtons*(kp*err[0] + kd*xderiv + ki*errInt[0]);
    float fy = drag_const*state->velocity.y - cf_mass_newtons*(kp*err[1] + kd*yderiv + ki*errInt[1]);
    float fz = drag_const*state->velocity.z - cf_mass_newtons*(kp*err[2] + kd*zderiv + ki*errInt[2]);

    float u = powf(fx*fx + fy*fy + fz*fz, 0.5);
    float roll = asin((double)(-fy/u));
    float pitch = asin((double)(fx/(powf(fz*fz + fx*fx, 0.5))));

    *thrust = u*thrustScale + thrustBase;
    attitude->roll = roll;
    attitude->pitch = pitch;
}

LOG_GROUP_START(h_inf_position)
LOG_ADD(LOG_FLOAT, errz, &err[2])
LOG_ADD(LOG_FLOAT, errzI, &errInt[2])
LOG_ADD(LOG_FLOAT, pz, &pz)
LOG_ADD(LOG_FLOAT, iz, &iz)
LOG_ADD(LOG_FLOAT, dz, &dz)
LOG_GROUP_STOP(h_inf_position)

PARAM_GROUP_START(h_inf_position)
PARAM_ADD(PARAM_FLOAT, w1, &w1)
PARAM_ADD(PARAM_FLOAT, w2, &w2)
PARAM_ADD(PARAM_FLOAT, w3, &w3)
PARAM_ADD(PARAM_FLOAT, r, &r)
PARAM_GROUP_STOP(h_inf_position)