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
 * h_inf_attitude.c: a nonlinear h-infinity attitude controller based on 'An integral predictive/nonlinear  control structure for a quadrotor helicopter'
 */
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "filter.h"

#include "FreeRTOS.h"

#include "param.h"
#include "log.h"
#include "stabilizer_types.h"
#include "debug.h"
#include "cf_math.h"
#include "math3d.h"
#include "cfassert.h"
#include "h_inf_attitude.h"

#define ATTITUDE_LPF_CUTOFF_FREQ      15.0f
#define ATTITUDE_LPF_ENABLE false

static bool isInit;
static float dt;
static float w1 = 1;
static float w2 = 1;
static float w3 = 1;
static float wu = 1;
static float M[3][3];
static float Minv[3][3];
static float C[3][3];
static float MC[3][3];
static float err[3];
static float errRate[3];
static float errInt[3];
static float D[3];
static float P[3];
static float I[3];
static float crates[3];
static lpf2pData dfilter;

static float rollOutput;
static float pitchOutput;
static float yawOutput;

// moments of inertia as calculated by MIT paper
static float Ixx = 0.000023951;
static float Iyy = 0.000023951;
static float Izz = 0.000032347;



static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }



static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

void hInfAttitudeInit(const float updateDt) {
    if (isInit) {
        return;
    }
    //M uses small angle assumption
    M[0][0] = Ixx;
    M[1][1] = Iyy;
    M[2][2] = Izz;
    Minv[0][0] = 1/Ixx;
    Minv[1][1] = 1/Iyy;
    Minv[2][2] = 1/Izz;
    memset(&errInt, 0, sizeof(errInt));
    memset(&err, 0, sizeof(err));
    lpf2pInit(&dfilter, ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ);

    dt = updateDt;

    isInit = true;

    
}

void hInfAttitude(float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float rollRateActual, float pitchRateActual, float yawRateActual) {
    // calculate C matrix using small angle assumption
    C[0][0] = 0.0;
    C[0][1] = (Izz - Iyy - Ixx)*yawRateActual;
    C[0][2] = 0;
    C[1][0] = (Iyy + Ixx - Izz)*yawRateActual;
    C[1][1] = 0.0;
    C[1][2] = 0;
    C[2][0] = -Ixx*pitchRateActual;
    C[2][1] = (Iyy - Izz)*rollRateActual;
    C[2][2] = 0.0;

    float yawError = eulerYawDesired - eulerYawActual;
    if (yawError > 180.0f)
        yawError -= 360.0f;
    else if (yawError < -180.0f)
        yawError += 360.0f;

    errRate[0] = (eulerRollDesired - eulerRollActual - err[0])/dt;
    errRate[1] = (eulerPitchDesired - eulerPitchActual - err[1])/dt;
    errRate[2] = (yawError - err[2])/dt;

    errRate[0] = lpf2pApply(&dfilter, errRate[0]);

    err[0] = eulerRollDesired - eulerRollActual;
    err[1] = eulerPitchDesired - eulerPitchActual;
    err[2] = yawError;



    errInt[0] += err[0]*dt;
    errInt[1] += err[1]*dt;
    errInt[2] += err[2]*dt;

    float rates[3] = {rollRateActual, pitchRateActual, yawRateActual};
    arm_matrix_instance_f32 ratesm = {3, 1, (float*)rates};
    arm_matrix_instance_f32 cratesm = {3, 1, (float*)crates};
    arm_matrix_instance_f32 Cm = {3, 3, (float *)C};
    mat_mult(&Cm, &ratesm, &cratesm);

    //calculate gains
    float Kd[3][3];
    float Kp[3][3];
    float Ki[3][3];
    C[0][0] = powf(wu, -2);
    C[1][1] = powf(wu, -2);
    C[2][2] = powf(wu, -2);
    arm_matrix_instance_f32 Minvm = {3, 3, (float *)Minv};
    arm_matrix_instance_f32 resm = {3, 3, (float *)MC};
    mat_mult(&Minvm, &Cm, &resm);




    float kd = powf(powf(w2, 2.0) + (float)2.0*w1*w3, 0.5)/w1;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Kd[i][j] = C[i][j];
            Kp[i][j] = kd*C[i][j];
            Ki[i][j] = (w3/w1)*C[i][j];
        }
        Kd[i][i] += kd;
        Kp[i][i] += (w3/w1);
    }


    arm_matrix_instance_f32 Dm = {3, 1, (float*)D};
    arm_matrix_instance_f32 Pm = {3, 1, (float*)P};
    arm_matrix_instance_f32 Im = {3, 1, (float*)I};
    arm_matrix_instance_f32 Kdm = {3, 3, (float*)Kd};
    arm_matrix_instance_f32 Kpm = {3, 3, (float*)Kp};
    arm_matrix_instance_f32 Kim = {3, 3, (float*)Ki};
    arm_matrix_instance_f32 errm = {3, 1, (float*)err};
    arm_matrix_instance_f32 errRatem = {3, 1, (float*)errRate};
    arm_matrix_instance_f32 errIntm = {3, 1, (float*)errInt};
    mat_mult(&Kdm, &errRatem, &Dm);
    mat_mult(&Kpm, &errm, &Pm);
    mat_mult(&Kim, &errIntm, &Im);
    float rollCmd = crates[0] + Ixx*(D[0] + P[0] + I[0]);
    float pitchCmd = crates[1] + Iyy*(D[1] + P[1] + I[1]);
    float yawCmd = crates[2] + Izz*(D[2] + P[2] + I[2]);

    rollOutput = rollCmd*powf(10.0, -34);
    pitchOutput = pitchCmd*powf(10, -31);
    yawOutput = yawCmd*powf(10, -31);
}

void hInfAttGetActuatorOutput(float* roll, float* pitch, float* yaw) {
    *roll = rollOutput;
    *pitch = pitchOutput;
    *yaw = -yawOutput;
}

LOG_GROUP_START(h_inf_attitude)
LOG_ADD(LOG_FLOAT, d_roll, &D[0])
LOG_ADD(LOG_FLOAT, i_roll, &I[0])
LOG_ADD(LOG_FLOAT, p_roll, &P[0])
LOG_ADD(LOG_FLOAT, rollErr, &err[0])
LOG_ADD(LOG_FLOAT, rollDrag, &crates[0])
LOG_GROUP_STOP(h_inf_attitude)

PARAM_GROUP_START(h_inf_attitude)
PARAM_ADD(PARAM_FLOAT, w1, &w1)
PARAM_ADD(PARAM_FLOAT, w2, &w2)
PARAM_ADD(PARAM_FLOAT, w3, &w3)
PARAM_ADD(PARAM_FLOAT, wu, &wu)
PARAM_GROUP_STOP(h_inf_attitude)