/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\flight\altitudehold.cpp                                    #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 31st Mar 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "drivers/gpio.h"
#include "drivers/ranging_vl53l0x.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/escservo.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "config/runtime_config.h"

#include "command/command.h"
#include "altitudehold.h"
// #include "API/Debugging.h"

// Kalman Filter Structure
typedef struct {
  float Q;    // Process noise covariance
  float R;    // Measurement noise covariance
  float X;    // Estimated value
  float P;    // Estimation error covariance
  float K;    // Kalman gain
} KalmanFilter;

// Global Kalman filter instances
static KalmanFilter altHoldFilter;
static KalmanFilter velHoldFilter;

uint8_t velocityControl           = 1;
int16_t max_altitude              = -1;
int16_t althold_throttle          = 0;
int32_t errorVelocityI            = 0;
int32_t errorAltitudeI            = 0;
int32_t altHoldThrottleAdjustment = 0;
int32_t AltHold;
int32_t vario           = 0;    // variometer in cm/s
int32_t setVelocity     = 0;
int32_t calculatedError = 10;
int32_t VelocityZ;
int32_t baroAlt_offset_print = 0;
int32_t PositionZ;
uint32_t baro_last_update;
bool AltRstRequired = 1;

// int32_t altest;

static barometerConfig_t *barometerConfig;
static pidProfile_t *pidProfile;
static rcControlsConfig_t *rcControlsConfig;
static escAndServoConfig_t *escAndServoConfig;
barometerConfig_t *barometerConfig_tmp;

int QUEUE_MAX_LENGTH = 15;

static float buff [ 15 ];
static int16_t head      = 0;
static int16_t rear      = -1;
static int16_t itemCount = 0;
#ifdef LASER_ALT
float _time_constant_z = 1.5f;
#else
// Reduced from 2.0 → 1.5 to tighten the baro correction loop.
// Lower TC means higher k1/k2/k3 gains → EstAlt tracks baro faster →
// less phase lag → breaks the 3-4 second oscillation cycle.
float _time_constant_z = 1.5f;
#endif
float accZ_tmp;
static float accZ_old = 0.0f;

int16_t first_reads               = 0;
int16_t first_velocity_reads      = 0;
int16_t ctr                       = 0;
static int32_t last_hist_position = 0;

void setAltitude ( float new_altitude );

// Kalman Filter Functions
void kalmanFilterInit ( KalmanFilter *filter, float Q, float R, float initialValue ) {
  filter->Q = Q;
  filter->R = R;
  filter->X = initialValue;
  filter->P = 1.0;    // Initial estimation error covariance
  filter->K = 0.0;    // Initial Kalman gain
}

float kalmanFilterUpdate ( KalmanFilter *filter, float measurement ) {
  // Prediction update
  filter->P = filter->P + filter->Q;

  // Measurement update
  filter->K = filter->P / ( filter->P + filter->R );
  filter->X = filter->X + filter->K * ( measurement - filter->X );
  filter->P = ( 1 - filter->K ) * filter->P;

  return filter->X;
}

float _k1_z;    // gain for vertical position correction
float _k2_z;    // gain for vertical velocity correction
float _k3_z;    // gain for vertical accelerometer offset correction

// general variables
float _position_base_z;          // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float _position_correction_z;    // sum of corrections to _position_base from delayed 1st order samples in cm
float _accel_correction_hbf_z;
float _velocity_z;          // latest velocity estimate (integrated from accelerometer values) in cm/s
float _position_error_z;    // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
float _position_z;          // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float accel_ef_z;

// temp vr

float _time_constant_z1 = 2.0f;

float _k1_z1;    // gain for vertical position correction
float _k2_z1;    // gain for vertical velocity correction
float _k3_z1;    // gain for vertical accelerometer offset correction

int32_t VelocityZ1 = 0;
int32_t EstAlt1    = 0;

uint32_t baro_last_update1;

int16_t first_reads1               = 0;
static int32_t last_hist_position1 = 0;

float _position_base_z1;          // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float _position_correction_z1;    // sum of corrections to _position_base from delayed 1st order samples in cm
float _accel_correction_hbf_z1;
float _velocity_z1;          // latest velocity estimate (integrated from accelerometer values) in cm/s
float _position_error_z1;    // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
float _position_z1;          // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)

float ToF_Height  = 0.0f;
float Baro_Height = 0.0f;
float fused       = 0.0f;
float filtered    = 0.0f;

int32_t altholdDebug          = 0;
int32_t altholdDebug1         = 0;
int32_t altholdDebug2         = 0;
int32_t altholdDebug3         = 0;
int32_t altholdDebug4         = 0;
int32_t altholdDebug5         = 0;
int32_t altholdDebug6         = 0;
int32_t altholdDebug7         = 0;
int32_t altholdDebug8         = 0;
int32_t altholdDebug9         = 0;
int32_t velControlDebug [ 3 ] = { 0 };

void configureAltitudeHold ( pidProfile_t *initialPidProfile, barometerConfig_t *intialBarometerConfig, rcControlsConfig_t *initialRcControlsConfig, escAndServoConfig_t *initialEscAndServoConfig ) {
  pidProfile          = initialPidProfile;
  barometerConfig     = intialBarometerConfig;
  rcControlsConfig    = initialRcControlsConfig;
  barometerConfig_tmp = barometerConfig;
  escAndServoConfig   = initialEscAndServoConfig;

  // Initialize Kalman filters
  kalmanFilterInit ( &altHoldFilter, 0.01, 0.5, 0.0 );    // Lower Q, higher R for altitude
  kalmanFilterInit ( &velHoldFilter, 0.1, 1.0, 0.0 );     // Higher Q, higher R for velocity
}

#if defined( BARO ) || defined( SONAR ) || defined( LASER_ALT )

int16_t initialThrottleHold_test;
int16_t debug_e1;

static int16_t initialThrottleHold;
static int32_t EstAlt = 0;    // in cm

  #define BARO_UPDATE_FREQUENCY_40HZ ( 1000 * 25 )
  #define UPDATE_FREQUENCY           ( 1000 * 10 )    // 100Hz

  #define DEGREES_80_IN_DECIDEGREES  800

static void applyMultirotorAltHold ( void ) {
  static uint8_t isAltHoldChanged = 0;
  static int16_t throttle_history = 0;
  static int16_t sensitivity_inv  = 6;

  if ( rcControlsConfig->alt_hold_fast_change ) {
    if ( ABS ( rcData [ THROTTLE ] - initialThrottleHold ) > rcControlsConfig->alt_hold_deadband ) {
      isAltHoldChanged       = 1;
      rcCommand [ THROTTLE ] = throttle_history + constrain ( ( rcData [ THROTTLE ] - initialThrottleHold ) / sensitivity_inv, -50, 80 );
    } else {
      if ( isAltHoldChanged ) {
        AltHold          = EstAlt;
        isAltHoldChanged = 0;
        errorVelocityI   = 0;
      }
      rcCommand [ THROTTLE ] = constrain ( initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle );
    }
    throttle_history = rcCommand [ THROTTLE ];
  } else {
    if ( ABS ( rcData [ THROTTLE ] - 1500 ) > rcControlsConfig->alt_hold_deadband ) {
      setVelocity      = ( rcData [ THROTTLE ] - 1500 ) / 4;
      setVelocity      = constrain ( setVelocity, -100, 120 );
      velocityControl  = 1;
      isAltHoldChanged = 1;
    } else {
      velocityControl = 0;
      setVelocity     = 0;
      if ( isAltHoldChanged ) {
        AltHold          = EstAlt;
        isAltHoldChanged = 0;
        errorVelocityI   = 0;
      }
    }
    rcCommand [ THROTTLE ] = constrain ( initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle );
  }

  if ( isThrottleStickArmed && rcData [ THROTTLE ] <= 1500 ) {
    rcCommand [ THROTTLE ] = 1000;
  } else {
    isThrottleStickArmed = false;
  }

  // static int debug_counter = 0;
  // if ( ++debug_counter >= 10 ) {
  //   debug_counter = 0;
  //   Monitor_Println ( "Alt:", EstAlt );
  //   Monitor_Println ( "Set:", AltHold );
  //   Monitor_Println ( "Thr:", rcCommand [ THROTTLE ] );
  // }

  altholdDebug  = AltHold;
  altholdDebug1 = altHoldThrottleAdjustment;
  altholdDebug2 = initialThrottleHold;
  altholdDebug3 = isAltHoldChanged;
  altholdDebug4 = rcCommand [ THROTTLE ];
}

static void applyFixedWingAltHold ( airplaneConfig_t *airplaneConfig ) {
  rcCommand [ PITCH ] += altHoldThrottleAdjustment * airplaneConfig->fixedwing_althold_dir;
}

void applyAltHold ( airplaneConfig_t *airplaneConfig ) {
  if ( STATE ( FIXED_WING ) ) {
    applyFixedWingAltHold ( airplaneConfig );
  } else {
    applyMultirotorAltHold ( );
  }
}

void updateAltHoldState ( void ) {
  if ( ! IS_RC_MODE_ACTIVE ( BOXBARO ) ) {
    DISABLE_FLIGHT_MODE ( BARO_MODE );
    return;
  }

  if ( ! FLIGHT_MODE ( BARO_MODE ) ) {
    ENABLE_FLIGHT_MODE ( BARO_MODE );
    // baroResetGroundLevel(); // Reset ground level for barometer
    AltHold                   = EstAlt;
    // NOTE: Using fixed 1500 as the hover baseline. Capturing rcData[THROTTLE]
    // at activation was tried but is unsafe: if the stick is at 1204 when BARO
    // activates, the drone crash-dives 50+ cm while the I-term winds up +481
    // units. 1500 gives a consistent, predictable starting point. The velocity
    // I-term will converge to the true hover offset within a few seconds.
    initialThrottleHold       = 1500;
    errorVelocityI            = 0;
    altHoldThrottleAdjustment = 0;
  }
  initialThrottleHold_test = initialThrottleHold;
  debug_e1                 = rcCommand [ THROTTLE ];
}

void updateSonarAltHoldState ( void ) {
  if ( ! IS_RC_MODE_ACTIVE ( BOXSONAR ) ) {
    DISABLE_FLIGHT_MODE ( SONAR_MODE );
    return;
  }

  if ( ! FLIGHT_MODE ( SONAR_MODE ) ) {
    ENABLE_FLIGHT_MODE ( SONAR_MODE );
    AltHold                   = EstAlt;
    initialThrottleHold       = rcData [ THROTTLE ];
    errorVelocityI            = 0;
    altHoldThrottleAdjustment = 0;
  }
}

bool isThrustFacingDownwards ( rollAndPitchInclination_t *inclination ) {
  return ABS ( inclination->values.rollDeciDegrees ) < DEGREES_80_IN_DECIDEGREES && ABS ( inclination->values.pitchDeciDegrees ) < DEGREES_80_IN_DECIDEGREES;
}

int16_t calculateTiltAngle ( rollAndPitchInclination_t *inclination ) {
  return MAX ( ABS ( inclination->values.rollDeciDegrees ), ABS ( inclination->values.pitchDeciDegrees ) );
}

int32_t calculateAltHoldThrottleAdjustment ( int32_t velocity_z, float accZ_tmp, float accZ_old ) {
  int32_t result = 0;
  int32_t error;
  int32_t setVel;

  if ( ! isThrustFacingDownwards ( &inclination ) ) {
    return result;
  }

  if ( ! ARMING_FLAG ( ARMED ) ) {
    AltHold = EstAlt;
  }

  if ( ! velocityControl ) {
    error           = constrain ( AltHold - EstAlt, -500, 500 );
    // Deadband prevents the motors from micro-twitching continuously when the drone 
    // is parked perfectly within ±1cm of the target setpoint.
    // error           = applyDeadband ( error, 1 );
    calculatedError = error;
    altholdDebug8   = error;

    if ( ARMING_FLAG ( ARMED ) ) {
      errorAltitudeI += ( pidProfile->I8 [ PIDALT ] * error );
      // Severely cap positional I-term windup to max ±15 cm/s drift
      errorAltitudeI = constrain ( errorAltitudeI, -( 8192 * 15 ), ( 8192 * 15 ) );
    } else {
      errorAltitudeI = 0;
    }

    // Add I-term to position correction (scaled similar to velocity I-term)
    setVel          = constrain ( ( pidProfile->P8 [ PIDALT ] * error / 128 ) + ( errorAltitudeI / 8192 ), -300, +300 );
  } else {
    setVel = setVelocity;
    errorAltitudeI = 0;
  }

  error         = setVel - velocity_z;
  altholdDebug9 = error;
  result        = constrain ( ( pidProfile->P8 [ PIDVEL ] * error / 32 ), -300, +300 );

  velControlDebug [ 0 ] = result;

  if ( ARMING_FLAG ( ARMED ) ) {
    errorVelocityI += ( pidProfile->I8 [ PIDVEL ] * error );
  } else {
    errorVelocityI = 0;
  }

  // Severely restrict the inner-loop velocity I-term windup limits.
  // Previously set to ±300, which allowed massive +300 throttle integration during 
  // slow large climbs, completely cancelling out P-term braking and causing 15-second
  // limit-cycle pendulum overshoots. Clamping strictly to ±100 still easily covers
  // a 1500 -> 1580 hover shift but forces immediate P-braking at the setpoint.
  errorVelocityI = constrain ( errorVelocityI, -( 8192 * 150 ), ( 8192 * 150 ) );
  result += errorVelocityI / 8192;

  velControlDebug [ 1 ] = errorVelocityI / 8192;

  result -= constrain ( pidProfile->D8 [ PIDVEL ] * ( accZ_tmp + accZ_old ) / 512, -150, 150 );
  velControlDebug [ 2 ] = constrain ( pidProfile->D8 [ PIDVEL ] * ( accZ_tmp + accZ_old ) / 512, -150, 150 );

  return result;
}

int16_t accalttemp;
float Temp;

void calculateEstimatedAltitude ( uint32_t currentTime ) {
  static uint32_t previousTime;
  uint32_t dTime;
  int32_t baroVel;
  float dt;
  float vel_acc;
  int32_t vel_tmp;
  float accZ_tmp;
  int32_t sonarAlt      = -1;
  static float accZ_old = 0.0f;
  static float vel      = 0.0f;
  static float accAlt   = 0.0f;
  static int32_t lastBaroAlt;
  static int32_t baroAlt_offset = 0;
  float sonarTransition;

  #ifdef SONAR
  int16_t tiltAngle;
  #endif

  dTime = currentTime - previousTime;
  if ( dTime < BARO_UPDATE_FREQUENCY_40HZ )
    return;

  previousTime = currentTime;

  #ifdef BARO
  if ( ! isBaroCalibrationComplete ( ) ) {
    performBaroCalibrationCycle ( );
    vel    = 0;
    accAlt = 0;
  }
  #else
  BaroAlt = 0;
  #endif

  #ifdef SONAR
  tiltAngle = calculateTiltAngle ( &inclination );
  sonarAlt  = sonarRead ( );
  sonarAlt  = sonarCalculateAltitude ( sonarAlt, tiltAngle );
  #endif

  if ( sonarAlt > 0 && sonarAlt < 200 ) {
    baroAlt_offset = BaroAlt - sonarAlt;
    BaroAlt        = sonarAlt;
  } else {
    BaroAlt -= baroAlt_offset;
    if ( sonarAlt > 0 && sonarAlt <= 300 ) {
      sonarTransition = ( 300 - sonarAlt ) / 100.0f;
      BaroAlt         = sonarAlt * sonarTransition + BaroAlt * ( 1.0f - sonarTransition );
    }
  }

  dt = accTimeSum * 1e-6f;

  if ( accSumCount ) {
    accZ_tmp = ( float ) accSum [ 2 ] / ( float ) accSumCount;
  } else {
    accZ_tmp = 0;
  }
  vel_acc = accZ_tmp * accVelScale * ( float ) accTimeSum;

  accAlt += ( vel_acc * 0.5f ) * dt + vel * dt;
  accalttemp = lrintf ( 100 * accAlt );
  accAlt     = accAlt * barometerConfig->baro_cf_alt + ( float ) BaroAlt * ( 1.0f - barometerConfig->baro_cf_alt );
  vel += vel_acc;

  #ifdef DEBUG_ALT_HOLD
  debug [ 1 ] = accSum [ 2 ] / accSumCount;
  debug [ 2 ] = vel;
  debug [ 3 ] = accAlt;
  #endif

  imuResetAccelerationSum ( 1 );

  #ifdef BARO
  if ( ! isBaroCalibrationComplete ( ) ) {
    return;
  }
  #endif

  if ( sonarAlt > 0 && sonarAlt < 200 ) {
    EstAlt = BaroAlt;
  } else {
    EstAlt = accAlt;
  }

  baroVel     = ( BaroAlt - lastBaroAlt ) * 1000000.0f / dTime;
  lastBaroAlt = BaroAlt;

  baroVel = constrain ( baroVel, -1500, 1500 );
  baroVel = applyDeadband ( baroVel, 10 );

  vel     = vel * barometerConfig->baro_cf_vel + baroVel * ( 1.0f - barometerConfig->baro_cf_vel );
  vel_tmp = lrintf ( vel );

  vario = applyDeadband ( vel_tmp, 5 );

  // Update Kalman filters with new sensor readings
  float filteredVel = kalmanFilterUpdate ( &velHoldFilter, vel_tmp );
  float filteredAlt = kalmanFilterUpdate ( &altHoldFilter, EstAlt );

  altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment ( vel_tmp, accZ_tmp, accZ_old );

  // NOTE: baro_cf_alt override removed — was silently overwriting the EEPROM
  //       config value with a PID-derived value on every cycle.
  accZ_old = accZ_tmp;

  // Update global variables with filtered values
  EstAlt = filteredAlt;
  vel    = filteredVel;
  // altest=EstAlt;
}

/* queue implementation */

void addHistPositionBaseEstZ ( float position ) {
  if ( itemCount < QUEUE_MAX_LENGTH ) {
    rear++;
    if ( rear >= QUEUE_MAX_LENGTH ) {
      rear = 0;
    }
    buff [ rear ] = position;
    itemCount++;
  } else {
    if ( ++rear == QUEUE_MAX_LENGTH ) {
      rear          = 0;
      buff [ rear ] = position;
      head++;
    } else {
      buff [ rear ] = position;
      head++;
      if ( head == QUEUE_MAX_LENGTH ) {
        head = 0;
      }
    }
  }
}

float getFrontHistPositionBaseEstZ ( ) {
  float return_value = buff [ head ];
  head++;
  if ( head == QUEUE_MAX_LENGTH ) {
    head = 0;
  }
  itemCount--;
  return return_value;
}

bool isPositionBaseQueueIsFull ( ) {
  return itemCount == QUEUE_MAX_LENGTH;
}

/* using ArduPilots Third Order Compilmentary filter */

void apmCalculateEstimatedAltitude ( uint32_t currentTime ) {
  static uint32_t previousTime;
  float dt       = ( currentTime - previousTime ) / 1000000.0f;
  uint32_t dTime = currentTime - previousTime;

  if ( dTime < UPDATE_FREQUENCY )
    return;

  previousTime = currentTime;

  if ( dTime > 2 * UPDATE_FREQUENCY ) {
    imuResetAccelerationSum ( 1 );
  }

  if ( AltRstRequired && ! ARMING_FLAG ( ARMED ) )
    AltRst ( );

  #if defined( BARO ) && ! ( defined( LASER_ALT ) )
  checkBaro ( );
  #else
  checkReading ( );
  #endif

  if ( accSumCount > 0 ) {
    accel_ef_z = ( float ) accSum [ 2 ] / ( float ) accSumCount;
  } else {
    accel_ef_z = 0;
  }

  accZ_tmp   = accel_ef_z;
  accel_ef_z = constrainf ( accel_ef_z * accVelScale, -800, 800 );

  imuResetAccelerationSum ( 1 );

  if ( first_velocity_reads <= 5 ) {
    first_velocity_reads++;
    return;
  }

  _accel_correction_hbf_z += _position_error_z * _k3_z * dt;
  _velocity_z += _position_error_z * _k2_z * dt;
  _position_correction_z += _position_error_z * _k1_z * dt;

  float velocity_increase_z = ( accel_ef_z + _accel_correction_hbf_z ) * dt;
  _position_base_z += ( _velocity_z + velocity_increase_z * 0.5f ) * dt;
  _position_z = _position_base_z + _position_correction_z;
  _velocity_z += velocity_increase_z;

  VelocityZ = lrintf ( _velocity_z );
  EstAlt    = lrintf ( _position_z );

  addHistPositionBaseEstZ ( _position_base_z );

  // Apply Kalman filtering to APM estimates
  // The Complementary filter calculates the most accurate estimates natively.
  // Running these outputs through a 1D Kalman filter introduces ~1 second of 
  // phase-delay lag, causing large slow "wave-like" pendulum limit cycles.
  float filteredVelocityZ = VelocityZ; // kalmanFilterUpdate ( &velHoldFilter, VelocityZ );
  float filteredEstAlt    = EstAlt;    // kalmanFilterUpdate ( &altHoldFilter, EstAlt );

  // Update global variables with filtered values
  VelocityZ = lrintf ( filteredVelocityZ );
  EstAlt    = lrintf ( filteredEstAlt );

  altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment ( VelocityZ, accZ_tmp, accZ_old );
  accZ_old                  = accZ_tmp;
  vario                     = applyDeadband ( VelocityZ, 5 );
  if ( abs ( VelocityZ ) > 200 )
    AltRstRequired = 1;
}

  #ifdef LASER_ALT
void checkReading ( ) {
  uint32_t baro_update_time;
  float dt = 0.0f;
  float tilt                 = 0;
  static int32_t baro_offset = 0;
  bool got_new_baro = false;

  baro_update_time = getBaroLastUpdate ( );
  if ( baro_update_time != baro_last_update ) {
    dt               = ( float ) ( baro_update_time - baro_last_update ) * 0.001f;
    Baro_Height      = baroCalculateAltitude ( );
    filtered         = ( 0.75f * filtered ) + ( ( 1 - 0.75f ) * Baro_Height );
    baro_last_update = baro_update_time;
    got_new_baro     = true;
  }
    #ifdef LASER_TOF
  bool got_new_tof = false;
  if ( isTofDataNew ( ) && ( ! isOutofRange ( ) ) ) {
    ToF_Height       = ( float ) NewSensorRange / 10.0f;
    isTofDataNewflag = false;
    tilt             = degreesToRadians ( calculateTiltAngle ( &inclination ) / 10 );
    if ( tilt < 25 )
      ToF_Height *= cos_approx ( tilt );
    got_new_tof = true;
  }

  if ( ( ToF_Height > 0 && ToF_Height < 200 ) && ( ! isOutofRange ( ) ) ) {
    if ( got_new_baro ) {
      baro_offset = filtered - ToF_Height;
    }
    if ( got_new_tof ) {
      correctedWithTof ( ToF_Height );
    }
  } else {
    if ( got_new_baro ) {
      correctedWithBaro ( Baro_Height - baro_offset, dt );
    }
  }
    #endif

    #ifdef LASER_TOF_L1x
  bool got_new_tof_L1 = false;
  if ( isTofDataNew_L1 ( ) && ( ! isOutofRange_L1 ( ) ) ) {

    ToF_Height       = ( float ) NewSensorRange_L1 / 10.0f;
    isTofDataNewflag = false;

    tilt = degreesToRadians ( calculateTiltAngle ( &inclination ) / 10 );
    if ( tilt < 25 )
      ToF_Height *= cos_approx ( tilt );
    got_new_tof_L1 = true;
  }
  // Fusion
  if ( ToF_Height > 0 && ToF_Height < 350 ) {
    if ( got_new_baro ) {
      baro_offset = Baro_filtered - EstAlt;
    }
    if ( got_new_tof_L1 ) {
      correctedWithTof ( ToF_Height );
    }
  } else {
    if ( got_new_baro ) {
      correctedWithBaro ( Baro_Height - baro_offset, dt );
    }
  }

    #endif
}
  #endif

void checkBaro ( ) {
  uint32_t baro_update_time;
  baro_update_time = getBaroLastUpdate ( );
  if ( baro_update_time != baro_last_update ) {
    const float dt = ( float ) ( baro_update_time - baro_last_update ) * 0.001f;
    correctedWithBaro ( baroCalculateAltitude ( ), dt );
    baro_last_update = baro_update_time;
  }
}

void correctedWithBaro ( float baroAlt, float dt ) {
  static bool firstBaroRead = true;
  altholdDebug5 = baroAlt;

  if ( dt > 0.5f ) {
    return;
  }

  // On the very first valid baro reading, seed the position estimator so that
  // _position_base_z starts at actual altitude (not 0). Without this, the queue
  // is empty and _position_base_z = 0, causing a large initial _position_error_z
  // spike equal to the full baro altitude, which takes many cycles to converge.
  if ( firstBaroRead ) {
    setAltitude ( baroAlt );
    firstBaroRead = false;
  }

  float hist_position_base_z;
  if ( isPositionBaseQueueIsFull ( ) ) {
    hist_position_base_z = getFrontHistPositionBaseEstZ ( );
  } else {
    hist_position_base_z = _position_base_z;
  }
  _position_error_z = baroAlt - ( hist_position_base_z + _position_correction_z );

  if ( ABS ( inclination_generalised.values.rollDeciDegrees ) > 30 || ABS ( inclination_generalised.values.pitchDeciDegrees ) > 30 ) {
    _time_constant_z = 5;
    updateGains ( );
  } else {
    _time_constant_z = 2;
    updateGains ( );
  }
}

  #ifdef LASER_ALT
void correctedWithTof ( float ToF_Height ) {
  if ( first_reads == 0 ) {
    setAltitude ( ToF_Height );
    first_reads++;
  }
  _position_error_z = ToF_Height - ( _position_base_z + _position_correction_z );
  // Reduce the TOF time-constant to 0.25s. 
  // 1.5s forces the drone to slowly phase the laser in over 1.5 seconds.
  // 0.25s allows the CF to instantly trust and lock onto the true laser distance.
  if ( _time_constant_z != 0.25f ) {
    _time_constant_z = 0.25f;
    updateGains ( );
  }
}
  #endif

void updateGains ( ) {
  if ( _time_constant_z == 0.0f ) {
    _k1_z = _k2_z = _k3_z = 0.0f;
  } else {
    _k1_z = 3.0f / _time_constant_z;
    _k2_z = 3.0f / ( _time_constant_z * _time_constant_z );
    _k3_z = 1.0f / ( _time_constant_z * _time_constant_z * _time_constant_z );
  }
}

void updateGains1 ( ) {
  if ( _time_constant_z1 == 0.0f ) {
    _k1_z1 = _k2_z1 = _k3_z1 = 0.0f;
  } else {
    _k1_z1 = 3.0f / _time_constant_z1;
    _k2_z1 = 3.0f / ( _time_constant_z1 * _time_constant_z1 );
    _k3_z1 = 1.0f / ( _time_constant_z1 * _time_constant_z1 * _time_constant_z1 );
  }
}

void setAltitude ( float new_altitude ) {
  _position_base_z       = new_altitude;
  _position_correction_z = 0;
  _position_z            = new_altitude;
  last_hist_position     = 0;
  imuResetAccelerationSum ( 1 );
}

int32_t altitudeHoldGetEstimatedAltitude ( void ) {
  return EstAlt;
}

int32_t getSetVelocity ( void ) {
  return setVelocity;
}

int32_t getSetAltitude ( void ) {
  return AltHold;
}

void AltRst ( void ) {
  _velocity_z = 0;
  imuResetAccelerationSum ( 1 );
  AltRstRequired            = 0;
  initialThrottleHold       = 1500;
  errorVelocityI            = 0;
  altHoldThrottleAdjustment = 0;

  // Reset Kalman filters with same tuning as configureAltitudeHold()
  kalmanFilterInit ( &altHoldFilter, 0.01, 0.5, 0.0 );    // Low Q, higher R: trust baro more
  kalmanFilterInit ( &velHoldFilter, 0.1, 1.0, 0.0 );     // Higher Q, higher R for velocity
}

float getTimeConstant ( ) {
  return _time_constant_z;
}

#endif

void setAltitude ( int32_t altitude ) {
  AltHold = altitude;
}

void setRelativeAltitude ( int32_t altitude ) {
  AltHold = EstAlt + altitude;
}

int32_t getEstAltitude ( ) {
  return EstAlt;
  // altest=EstAlt;
}

int32_t getEstVelocity ( ) {
  return VelocityZ;
}

int32_t getEstAltitude1 ( ) {
  return EstAlt1;
}

int32_t getEstVelocity1 ( ) {
  return VelocityZ1;
}

bool limitAltitude ( ) {
  if ( max_altitude != -1 && IS_RC_MODE_ACTIVE ( BOXBARO ) ) {
    if ( EstAlt >= ( max_altitude - 50 ) ) {
      return true;
    }
  }
  return false;
}