/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\flight\imu.h                                               #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sun, 20th Apr 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif 

extern int16_t throttleAngleCorrection;
extern uint32_t accTimeSum;
extern int accSumCount,accSumCountXYZ;
extern float accVelScale;
extern t_fp_vector EstG;
extern int16_t accSmooth[XYZ_AXIS_COUNT];
extern int32_t accSum[XYZ_AXIS_COUNT];
extern int32_t accSumXYZ[XYZ_AXIS_COUNT];
extern int16_t smallAngle;

typedef struct rollAndPitchInclination_s {
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        int16_t rollDeciDegrees;
        int16_t pitchDeciDegrees;
} rollAndPitchInclination_t_def;

typedef union {
        int16_t raw[ANGLE_INDEX_COUNT];
        rollAndPitchInclination_t_def values;
} rollAndPitchInclination_t;

extern rollAndPitchInclination_t inclination;
extern rollAndPitchInclination_t inclination_generalised;

typedef struct accDeadband_s {
        uint8_t xy;                 // set the acc deadband for xy-Axis
        uint8_t z;                  // set the acc deadband for z-Axis, this ignores small accelerations
} accDeadband_t;

typedef struct imuRuntimeConfig_s {
        uint8_t acc_lpf_factor;
        uint8_t gyro_lpf_factor;
        uint8_t acc_unarmedcal;
        float gyro_cmpf_factor;
        float gyro_cmpfm_factor;
        uint8_t small_angle;
} imuRuntimeConfig_t;

void imuConfigure(imuRuntimeConfig_t *initialImuRuntimeConfig, pidProfile_t *initialPidProfile, accDeadband_t *initialAccDeadband, float accz_lpf_cutoff, uint16_t throttle_correction_angle);

void calculateEstimatedAltitude(uint32_t currentTime);
void imuUpdate(rollAndPitchTrims_t *accelerometerTrims);
float calculateThrottleAngleScale(uint16_t throttle_correction_angle);
int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value);
float calculateAccZLowPassFilterRCTimeConstant(float accz_lpf_cutoff);
float * dcmBodyToEarth3D(float* vector);
float * earthToBody2D(float* vector);



int16_t imuCalculateHeading(t_fp_vector *vec);
void imuResetAccelerationSum(int i);
void imuInit(void);

extern int32_t netAccMagnitude;
extern int32_t accZoffsetCalCycle;



extern float tempMat[3][3];
extern float tempMat1[3][3];
extern float anglerad[ANGLE_INDEX_COUNT];


//extern int32_t imuDebug;
//extern int32_t imuDebug1;



#ifdef __cplusplus
}
#endif 
