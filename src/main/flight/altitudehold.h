/*
 * This file is part of Cleanflight and Magis.
 *
 * Cleanflight and Magis are free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight and Magis are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif 

#include "io/escservo.h"
#include "io/rc_controls.h"
#include "flight/pid.h"

#include "sensors/barometer.h"



extern int16_t accalttemp;
extern int16_t althold_throttle;
extern int16_t max_altitude;
extern int32_t AltHold;
extern int32_t vario;
extern int32_t baroAlt_offset_print;

extern float ToF_Height;

extern barometerConfig_t *barometerConfig_tmp;

void configureAltitudeHold(pidProfile_t *initialPidProfile,barometerConfig_t *intialBarometerConfig, rcControlsConfig_t *initialRcControlsConfig, escAndServoConfig_t *initialEscAndServoConfig);
void applyAltHold(airplaneConfig_t *airplaneConfig);
void updateAltHoldState(void);
void updateSonarAltHoldState(void);

int32_t altitudeHoldGetEstimatedAltitude(void);
int32_t getSetVelocity(void);
int32_t getSetAltitude(void);
void AltRst(void);
// Leave the ARM-switch ground idle now, so the next controller cycle flies AltHold instead of resetting it.
void altHoldReleaseGroundIdle(void);
// Throttle stick inside the alt-hold deadband ( no climb or descent request ).
bool altHoldStickCentred(void);




void correctedWithTof(float tofHeightCm);
void correctedWithBaro(float baroAlt, float dt);
void checkReading();
// Altitude estimator source: 1 laser, 0 baro, 2 holding on the baro after an object step.
uint8_t altHoldSource(void);
// Altitude-laser fusion: on with XRanging.initAlt ( ) ( VL53L0X ) or X1Ranging.initAlt ( ) ( VL53L1X ).
// altLaserFusionLatch ( ) fixes it once at boot, after plutoInit ( ); it never changes afterwards. vl53l1x
// records the family, which picks the fusion copy checkReading ( ) runs ( VL53L1X or VL53L0X ).
bool altLaserFusion(void);
void altLaserFusionLatch(bool on, bool vl53l1x);
void checkBaro();

void updateGains();
void updateGains1();
void updateTimeConstantandGains(uint8_t timeConstant);
float getTimeConstant();
void apmCalculateEstimatedAltitude(uint32_t currtime);
void setAltitude(int32_t altitude);
void setRelativeAltitude(int32_t altitude);
int32_t getEstAltitude();
int32_t getEstVelocity(void);
int32_t getEstAltitude1();
int32_t getEstVelocity1(void);


bool limitAltitude();

// The alt-hold cascade's sole integrator - carries the whole hover trim.
extern int32_t errorVelocityI;
extern int32_t altholdDebug;
extern int32_t altholdDebug1;
extern int32_t altholdDebug2;
extern int32_t altholdDebug3;
extern int32_t altholdDebug4;
extern int32_t altholdDebug5;
extern int32_t altholdDebug6;
extern int32_t altholdDebug7;
extern int32_t altholdDebug8;
extern int32_t altholdDebug9;
extern int32_t velControlDebug[3];


#ifdef __cplusplus
}
#endif
