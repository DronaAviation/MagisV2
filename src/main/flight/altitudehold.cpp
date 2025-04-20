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
 #  Last Modified: Sun, 20th Apr 2025                                          #
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
// #include "config/config.h"
#include "config/runtime_config.h"

#include "command/command.h"
#include "altitudehold.h"

uint8_t velocityControl           = 1;
int16_t max_altitude              = -1;
int16_t althold_throttle          = 0;
int32_t errorVelocityI            = 0;
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
float _time_constant_z = 2.0f;
#endif
float accZ_tmp;
static float accZ_old = 0.0f;

int16_t first_reads               = 0;
int16_t first_velocity_reads      = 0;
int16_t ctr                       = 0;
static int32_t last_hist_position = 0;

void setAltitude ( float new_altitude );

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
}

#if defined( BARO ) || defined( SONAR ) || defined( LASER_ALT )

int16_t initialThrottleHold_test;
int16_t debug_e1;

static int16_t initialThrottleHold;
static int32_t EstAlt = 0;    // in cm

  // 40hz update rate (20hz LPF on acc)
  #define BARO_UPDATE_FREQUENCY_40HZ ( 1000 * 25 )
  #define UPDATE_FREQUENCY           ( 1000 * 10 )    // 100Hz

  #define DEGREES_80_IN_DECIDEGREES  800

// float baroHighPassFilter ( float newAltitude ) {
//   static float filteredAltitude = 0;
//   static float lastAltitude     = 0;

//   float alpha      = 0.98f;    // Adjust filtering strength
//   filteredAltitude = alpha * ( filteredAltitude + ( newAltitude - lastAltitude ) );
//   lastAltitude     = newAltitude;
//   return filteredAltitude;
// }

/** Drone should maintian its altitude which it attins at the throttle of '1500'*/
// static void applyMultirotorAltHold ( void ) {
//   static uint8_t isAltHoldChanged = 0;
//   static int16_t throttle_history = 0;
//   static int16_t sensitivity_inv  = 6;
//   // multirotor alt hold
//   if ( rcControlsConfig->alt_hold_fast_change ) {
//     // rapid alt changes
//     if ( ABS ( rcData [ THROTTLE ] - initialThrottleHold ) > rcControlsConfig->alt_hold_deadband ) {

//       isAltHoldChanged = 1;
//       // rcCommand[THROTTLE] += (rcData[THROTTLE] > initialThrottleHold) ? -rcControlsConfig->alt_hold_deadband : rcControlsConfig->alt_hold_deadband; //drona
//       rcCommand [ THROTTLE ] = throttle_history + constrain ( ( rcData [ THROTTLE ] - initialThrottleHold ) / sensitivity_inv, -50, 80 );    // Magis
//       //            if (rcData[THROTTLE] < 1100){
//       //                rcCommand[THROTTLE] = 1150;
//       //            }

//     } else {
//       if ( isAltHoldChanged ) {
//         AltHold = EstAlt;
//         //                AltHold = 120;
//         isAltHoldChanged = 0;
//         if ( ARMING_FLAG ( ARMED ) ) {
//           // altHoldThrottleAdjustment = throttle_history;
//         }
//       }

//       rcCommand [ THROTTLE ] = constrain ( initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle );
//       throttle_history       = rcCommand [ THROTTLE ];    // Magis
//     }
//   } else {
//     // slow alt changes, mostly used for aerial photography
//     if ( ABS ( rcData [ THROTTLE ] - 1500 ) > rcControlsConfig->alt_hold_deadband ) {
//       // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
//       // setVelocity = (rcData[THROTTLE] - masterConfig.rxConfig.midrc) / 6;

//       setVelocity = ( rcData [ THROTTLE ] - 1500 ) / 4;
//       setVelocity = constrain ( setVelocity, -100, 120 );    // to descend smoothly

//       /* if (isAltHoldChanged)
//        velocityControl = 0;
//        else
//        velocityControl = 1;*/

//       velocityControl  = 1;    // Switch on velocity hold
//       isAltHoldChanged = 1;

//     } else {

//       velocityControl = 0;    // Switch on altitude hold
//       setVelocity     = 0;
//       if ( isAltHoldChanged ) {
//         AltHold          = EstAlt;    // Record the height to be maintained
//         isAltHoldChanged = 0;
//       }
//     }

//     rcCommand [ THROTTLE ] = constrain ( initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle );

//     altholdDebug1 = altHoldThrottleAdjustment;

//     altholdDebug2 = initialThrottleHold;

//     // #ifdef LASER_TOF
//     //         if(rcData[THROTTLE]<1150 && isOutofRange())
//     // #else
//     //     if (rcData[THROTTLE] < 1150)
//     // #endif
//     //         if (rcData[THROTTLE] < 1150)
//     //         {
//     //             rcCommand[THROTTLE] = 1350;
//     //         }

//     //        if(!isTakeOffHeightSet){
//     //
//     //        	  rcCommand[THROTTLE] = 1000;
//     //        }

//     if ( isThrottleStickArmed && rcData [ THROTTLE ] <= 1500 ) {
//       // rcData[THROTTLE] = 1000;
//       rcCommand [ THROTTLE ] = 1000;
//     } else {
//       isThrottleStickArmed = false;
//     }

//     althold_throttle = rcCommand [ THROTTLE ];

//     altholdDebug = AltHold;
//     // altholdDebug2 = velocityControl;
//     altholdDebug3 = isAltHoldChanged;
//     altholdDebug4 = rcCommand [ THROTTLE ];
//   }
// }

static void applyMultirotorAltHold ( void ) {
  static uint8_t isAltHoldChanged = 0;
  static int16_t throttle_history = 0;
  static int16_t sensitivity_inv  = 6;

  // Fast Altitude Changes
  if ( rcControlsConfig->alt_hold_fast_change ) {
    // rapid alt changes
    if ( ABS ( rcData [ THROTTLE ] - initialThrottleHold ) > rcControlsConfig->alt_hold_deadband ) {

      isAltHoldChanged       = 1;
      rcCommand [ THROTTLE ] = throttle_history + constrain ( ( rcData [ THROTTLE ] - initialThrottleHold ) / sensitivity_inv, -50, 80 );    // Magis

    } else {
      if ( isAltHoldChanged ) {

        AltHold          = EstAlt;
        isAltHoldChanged = 0;
      }

      rcCommand [ THROTTLE ] = constrain ( initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle );
    }
    throttle_history = rcCommand [ THROTTLE ];    // Magis
  } else {
    // slow alt changes, mostly used for aerial photography
    if ( ABS ( rcData [ THROTTLE ] - 1500 ) > rcControlsConfig->alt_hold_deadband ) {

      setVelocity = ( rcData [ THROTTLE ] - 1500 ) / 4;
      setVelocity = constrain ( setVelocity, -100, 120 );    // to descend smoothly

      velocityControl  = 1;    // Switch on velocity hold
      isAltHoldChanged = 1;

    } else {
      velocityControl = 0;    // Switch on altitude hold
      setVelocity     = 0;

      if ( isAltHoldChanged ) {
        AltHold          = EstAlt;    // Record the height to be maintained
        isAltHoldChanged = 0;
      }
    }

    rcCommand [ THROTTLE ] = constrain ( initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle );
  }
  // Reset AltHold when drone is stationary
  // if ( ABS ( VelocityZ ) < 5 && ABS ( accel_ef_z ) < 0.05f ) {
  //   AltHold = EstAlt;    // Store the current altitude to maintain baseline
  // }

  if ( isThrottleStickArmed && rcData [ THROTTLE ] <= 1500 ) {
    // rcData[THROTTLE] = 1000;
    rcCommand [ THROTTLE ] = 1000;
  } else {
    isThrottleStickArmed = false;
  }

  // If the drone is landed and disarmed, reset altitude to zero
  if ( ! ARMING_FLAG ( ARMED ) ) {    // Check if the drone is disarmed
    if ( ABS ( VelocityZ ) < 5 && ABS ( accel_ef_z ) < 0.05f ) {
      EstAlt           = 0;
      AltHold          = 0;
      _position_base_z = 0;
    }
  }

  // Debugging Outputs
  altholdDebug  = AltHold;
  altholdDebug1 = altHoldThrottleAdjustment;
  altholdDebug2 = initialThrottleHold;
  altholdDebug3 = isAltHoldChanged;
  altholdDebug4 = rcCommand [ THROTTLE ];
}

static void applyFixedWingAltHold ( airplaneConfig_t *airplaneConfig ) {
  // handle fixedwing-related althold. UNTESTED! and probably wrong
  // most likely need to check changes on pitch channel and 'reset' althold similar to
  // how throttle does it on multirotor

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
  // Baro alt hold activate
  if ( ! IS_RC_MODE_ACTIVE ( BOXBARO ) ) {
    DISABLE_FLIGHT_MODE ( BARO_MODE );
    return;
  }

  if ( ! FLIGHT_MODE ( BARO_MODE ) ) {

    ENABLE_FLIGHT_MODE ( BARO_MODE );
    AltHold = EstAlt;    //+100;
    // initialThrottleHold = rcData[THROTTLE];
    initialThrottleHold       = 1500;
    errorVelocityI            = 0;
    altHoldThrottleAdjustment = 0;
  }
  initialThrottleHold_test = initialThrottleHold;

  // debug_d0 = pidProfile->D8[PIDALT];
  debug_e1 = rcCommand [ THROTTLE ];
}

void updateSonarAltHoldState ( void ) {
  // Sonar alt hold activate
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

/*
 * This (poorly named) function merely returns whichever is higher, roll inclination or pitch inclination.
 * //TODO: Fix this up. We could either actually return the angle between 'down' and the normal of the craft
 * (my best interpretation of scalar 'tiltAngle') or rename the function.
 */
int16_t calculateTiltAngle ( rollAndPitchInclination_t *inclination ) {
  return MAX ( ABS ( inclination->values.rollDeciDegrees ), ABS ( inclination->values.pitchDeciDegrees ) );
}
// int32_t calculateAltHoldThrottleAdjustment ( int32_t velocity_z, float accZ_tmp, float accZ_old ) {
//   int32_t result = 0;
//   int32_t error;
//   int32_t setVel;

//   if ( ! isThrustFacingDownwards ( &inclination ) ) {
//     return result;
//   }

//   // Altitude P-Controller
//   if ( ! ARMING_FLAG ( ARMED ) ) {
//     AltHold = EstAlt;
//     // initialThrottleHold=1500;
//   }

//   if ( ! velocityControl ) {
//     // error = constrain(AltHold - EstAlt, -100, 100);
//     error = constrain ( AltHold - EstAlt, -500, 500 );
//     error = applyDeadband ( error, 5 );    // remove small P parameter to reduce noise near zero position

//     calculatedError = error;
//     altholdDebug8   = error;

//     // setVel = constrain((pidProfile->P8[PIDALT] * error / 128), -80, +150); // limit velocity to +/- 3 m/s
//     setVel = constrain ( ( pidProfile->P8 [ PIDALT ] * error / 128 ), -300, +300 );    // limit velocity to +/- 3 m/s

//   } else {

//     setVel = setVelocity;
//   }

//   //  altholdDebug=setVel;

//   //  altholdDebug1 = setVel;

//   // Velocity PID-Controller
//   // P
//   error = setVel - velocity_z;

//   altholdDebug9 = error;

//   // result = constrain((pidProfile->P8[PIDVEL] * error / 32), -100, +100);
//   result = constrain ( ( pidProfile->P8 [ PIDVEL ] * error / 32 ), -300, +300 );

//   velControlDebug [ 0 ] = result;

//   // I
//   if ( ARMING_FLAG ( ARMED ) ) {
//     errorVelocityI += ( pidProfile->I8 [ PIDVEL ] * error );
//   } else {
//     errorVelocityI = 0;
//   }

//   //  errorVelocityI = constrain(errorVelocityI, -(8192 * 150), (8192 * 150));
//   errorVelocityI = constrain ( errorVelocityI, -( 8192 * 300 ), ( 8192 * 300 ) );

//   result += errorVelocityI / 8192;    // I in range +/-200

//   velControlDebug [ 1 ] = errorVelocityI / 8192;

//   // D
//   result -= constrain ( pidProfile->D8 [ PIDVEL ] * ( accZ_tmp + accZ_old ) / 512, -150, 150 );

//   velControlDebug [ 2 ] = constrain ( pidProfile->D8 [ PIDVEL ] * ( accZ_tmp + accZ_old ) / 512, -150, 150 );

//   return result;
// }
int32_t calculateAltHoldThrottleAdjustment ( int32_t velocity_z, float accZ_tmp, float accZ_old ) {
  int32_t result = 0;
  int32_t error;
  int32_t setVel;

  // Ensure thrust is facing downwards before adjusting altitude hold
  if ( ! isThrustFacingDownwards ( &inclination ) ) {
    return result;
  }

  // Initialize AltHold on arming
  if ( ! ARMING_FLAG ( ARMED ) ) {
    AltHold = EstAlt;
  }

  // Altitude P-Controller
  if ( ! velocityControl ) {
    error = constrain ( AltHold - EstAlt, -500, 500 );
    error = applyDeadband ( error, 5 );    // remove small P parameter to reduce noise near zero position

    // Increase the error gain for positive altitude errors
    if ( error > 0 ) {
      error *= 1.2f;    // Increase responsiveness when altitude is decreasing
    }

    calculatedError = error;
    altholdDebug8   = error;

    // setVel = constrain((pidProfile->P8[PIDALT] * error / 128), -80, +150); // limit velocity to +/- 3 m/s
    setVel = constrain ( ( pidProfile->P8 [ PIDALT ] * error / 128 ), -300, +300 );    // limit velocity to +/- 3 m/s

  } else {
    setVel = setVelocity;
  }

  // Velocity PID Control
  // P-Term
  error         = setVel - velocity_z;
  altholdDebug9 = error;
  // result = constrain ( ( pidProfile->P8 [ PIDVEL ] * error / 32 ), -300, +300 ); //default
  // result = constrain((pidProfile->P8[PIDVEL] * error / 32), -100, +100);
  // result = constrain ( ( pidProfile->P8 [ PIDVEL ] * error / 40 ), -200, +200 );
  result = constrain ( ( pidProfile->P8 [ PIDVEL ] * error / 30 ), -250, +250 );

  // I-Term: Only integrate if armed to prevent windup
  if ( ARMING_FLAG ( ARMED ) ) {
    // errorVelocityI += ( pidProfile->I8 [ PIDVEL ] * error );
    errorVelocityI += ( pidProfile->I8 [ PIDVEL ] * error /* * 1.2f */ );
  } else {
    errorVelocityI = 0;
  }

  // I-Term: Apply Integral Windup Protection with Slow Decay
  // if ( ARMING_FLAG ( ARMED ) ) {
  //   if ( abs ( error ) > 10 ) {
  //     errorVelocityI += ( pidProfile->I8 [ PIDVEL ] * error /* * 20.0f */ );
  //   }
  //   // Instead of full reset, apply gradual decay to prevent altitude loss
  //   else if ( rcData [ THROTTLE ] > 1450 && rcData [ THROTTLE ] < 1550 ) {
  //     errorVelocityI *= 0.995f;    // Slowly reduce integral effect instead of full reset
  //   }
  // } else {
  //   errorVelocityI = 0;
  // }

  // errorVelocityI = constrain ( errorVelocityI, -( 8192 * 300 ), ( 8192 * 300 ) ); //default
  //  errorVelocityI = constrain(errorVelocityI, -(8192 * 150), (8192 * 150));
  errorVelocityI = constrain ( errorVelocityI, -( 8192 * 400 ), ( 8192 * 400 ) );
  // errorVelocityI = constrain ( errorVelocityI, -( 8192 * 250 ), ( 8192 * 250 ) );
  result += errorVelocityI / 8192.0f;

  // D-Term: Use filtered acceleration to reduce noise
  // result -= constrain ( pidProfile->D8 [ PIDVEL ] * ( accZ_tmp + accZ_old ) / 512, -150, 150 ); //old

  // float filteredAccZ = 0.8f * accZ_old + 0.2f * accZ_tmp;    // TODO: TEST
  float filteredAccZ = 0.85f * accZ_old + 0.15f * accZ_tmp;    // TODO: TEST
  // float filteredAccZ = 0.9f * accZ_old + 0.1f * accZ_tmp;    // TODO: TEST
  // float filteredAccZ = 0.95f * accZ_old + 0.05f * accZ_tmp;    // TODO: TEST
  // result -= constrain ( pidProfile->D8 [ PIDVEL ] * filteredAccZ / 512, -150, 150 );    // TODO: TEST

  // result -= constrain ( pidProfile->D8 [ PIDVEL ] * filteredAccZ / 550, -100, 100 );    // TODO: TEST
  // result -= constrain ( pidProfile->D8 [ PIDVEL ] * filteredAccZ / 550, -120, 120 );    // TODO: TEST
  result -= constrain ( pidProfile->D8 [ PIDVEL ] * filteredAccZ / 512, -150, 150 );    // default
  // result -= constrain ( pidProfile->D8 [ PIDVEL ] * filteredAccZ / 500, -140, 140 );    // TODO: TEST

  // Debugging values
  // velControlDebug [ 0 ] = result;
  // velControlDebug [ 1 ] = errorVelocityI / 8192;
  // velControlDebug [ 2 ] = constrain ( pidProfile->D8 [ PIDVEL ] * filteredAccZ / 512, -150, 150 );

  // Increase Throttle Response to Improve Stability
  // result = constrain ( result, -140, 140 );

  // Apply Stronger Low-Pass Filter for Smoother Response
  // static float smoothedThrottleAdjustment = 0;
  // smoothedThrottleAdjustment              = 0.90f * smoothedThrottleAdjustment + 0.10f * result;
  // result                                  = smoothedThrottleAdjustment;

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

    // BaroAlt = baroCalculateAltitude();
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

  dt = accTimeSum * 1e-6f;    // delta acc reading time in seconds

  // Integrator - velocity, cm/sec
  if ( accSumCount ) {
    accZ_tmp = ( float ) accSum [ 2 ] / ( float ) accSumCount;
  } else {
    accZ_tmp = 0;
  }
  vel_acc = accZ_tmp * accVelScale * ( float ) accTimeSum;

  // Integrator - Altitude in cm
  accAlt += ( vel_acc * 0.5f ) * dt + vel * dt;
  accalttemp = lrintf ( 100 * accAlt );                                                                                // Checking how acc measures height                                                                // integrate velocity to get distance (x= a/2 * t^2)
  accAlt     = accAlt * barometerConfig->baro_cf_alt + ( float ) BaroAlt * ( 1.0f - barometerConfig->baro_cf_alt );    // complementary filter for altitude estimation (baro & acc)
  vel += vel_acc;

  #ifdef DEBUG_ALT_HOLD
  debug [ 1 ] = accSum [ 2 ] / accSumCount;    // acceleration
  debug [ 2 ] = vel;                           // velocity
  debug [ 3 ] = accAlt;                        // height
  #endif

  imuResetAccelerationSum ( 1 );

  #ifdef BARO
  if ( ! isBaroCalibrationComplete ( ) ) {
    return;
  }
  #endif

  if ( sonarAlt > 0 && sonarAlt < 200 ) {
    // the sonar has the best range
    EstAlt = BaroAlt;
  } else {
    EstAlt = accAlt;
  }

  baroVel     = ( BaroAlt - lastBaroAlt ) * 1000000.0f / dTime;
  lastBaroAlt = BaroAlt;

  baroVel = constrain ( baroVel, -1500, 1500 );    // constrain baro velocity +/- 1500cm/s
  baroVel = applyDeadband ( baroVel, 10 );         // to reduce noise near zero

  // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
  // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
  vel     = vel * barometerConfig->baro_cf_vel + baroVel * ( 1.0f - barometerConfig->baro_cf_vel );
  vel_tmp = lrintf ( vel );

  // set vario
  vario = applyDeadband ( vel_tmp, 5 );
  if ( 1 )    //(ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig->alt_hold_deadband))
  {
    altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment ( vel_tmp, accZ_tmp, accZ_old );
  }    // dronadrona_1200am
  Temp                         = pidProfile->I8 [ PIDALT ];
  barometerConfig->baro_cf_alt = 1 - Temp / 1000;
  accZ_old                     = accZ_tmp;
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

  }

  else {

    if ( ++rear == QUEUE_MAX_LENGTH ) {

      rear = 0;

      buff [ rear ] = position;

      head++;

    }

    else {
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
  static uint32_t previousTime       = 0;
  static bool altitudeResetTriggered = false;    // Track if reset was triggered

  if ( previousTime == 0 ) {
    previousTime = currentTime;
    return;    // Skip first iteration to avoid errors
  }

  float dt       = ( currentTime - previousTime ) / 1000000.0f;
  uint32_t dTime = currentTime - previousTime;

  if ( dTime < UPDATE_FREQUENCY )
    return;

  previousTime = currentTime;

  // Handle long gaps in updates
  if ( dTime > 2 * UPDATE_FREQUENCY ) {
    imuResetAccelerationSum ( 1 );
    //! NEW : Reduce velocity drift instead of full reset
    _velocity_z *= 0.5f;    // TODO: TEST
  }

  // // 🚀 **Trigger altitude reset ONCE when arming**
  // if ( ARMING_FLAG ( ARMED ) && ! altitudeResetTriggered ) {
  //   AltRst ( );
  //   altitudeResetTriggered = true;    // Ensure it triggers only once per arming
  // }

  // if ( ! ARMING_FLAG ( ARMED ) ) {
  //   altitudeResetTriggered = false;    // Allow reset next time it arms
  // }

  if ( AltRstRequired && ! ARMING_FLAG ( ARMED ) )    // Velocity out of bounds reset variables
    AltRst ( );

  #if defined( BARO ) && ! ( defined( LASER_ALT ) )
  checkBaro ( );    // check if new baro readings have arrived and use them to correct vertical accelerometer offsets.
  #else
  checkReading ( );
  #endif

  // Process acceleration
  if ( accSumCount > 0 ) {
    accel_ef_z = ( float ) accSum [ 2 ] / ( float ) accSumCount;
  } else {
    accel_ef_z = 0;
  }

  accZ_tmp   = accel_ef_z;
  accel_ef_z = constrainf ( accel_ef_z * accVelScale, -800, 800 );

  // altholdDebug6 = accZ_tmp;

  imuResetAccelerationSum ( 1 );    // Check position of this

  //!
  //! NEW : Test the new integration
  //!
  // Apply velocity decay to prevent drift
  _velocity_z *= 0.99f;
  if ( abs ( accel_ef_z ) < 0.05f ) {
    _velocity_z = 0;    // Reset velocity if no movement detected
  }

  // Handle initial sensor instability
  if ( first_velocity_reads <= 5 ) {
    first_velocity_reads++;

    //! NEW : Force-set Altitude depending on the condition
    if ( first_velocity_reads == 6 )
      setAltitude ( baroCalculateAltitude ( ) );    // Force-set altitude
    return;
  }

  _accel_correction_hbf_z += _position_error_z * _k3_z * dt;
  _velocity_z += _position_error_z * _k2_z * dt;
  _position_correction_z += _position_error_z * _k1_z * dt;

  // Calculate velocity increase from acceleration
  float velocity_increase_z = ( accel_ef_z + _accel_correction_hbf_z ) * dt;

  // Estimate position
  _position_base_z += ( _velocity_z + velocity_increase_z * 0.5f ) * dt;
  _position_z = _position_base_z + _position_correction_z;
  _velocity_z += velocity_increase_z;

  // Apply high-pass filter to baro correction
  // _position_error_z = 0.95f * _position_error_z + 0.05f * baroHighPassFilter ( baroCalculateAltitude ( ) - ( getFrontHistPositionBaseEstZ ( ) + _position_correction_z ) );

  // Apply a decay filter to prevent EstAlt from snapping back to zero
  // if ( ABS ( _position_error_z ) < 5 ) {    // Threshold to ignore small errors
  //   _position_error_z *= 0.95f;             // Gradually reduce error instead of snapping to 0
  // }

  // Convert float velocity & altitude to integers
  VelocityZ = lrintf ( _velocity_z );
  EstAlt    = lrintf ( _position_z );

  // Reset altitude if it detects the drone is on the ground
  // if ( EstAlt < 3 && EstAlt > -3 ) {
  //   EstAlt = 0;
  // }

  // Store altitude history for smoothing
  addHistPositionBaseEstZ ( _position_base_z );

  // Calculate throttle adjustment
  altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment ( VelocityZ, accZ_tmp, accZ_old );
  accZ_old                  = accZ_tmp;

  // altholdDebug7 = altHoldThrottleAdjustment;

  // Apply deadband to velocity
  vario = applyDeadband ( VelocityZ, 5 );

  // Trigger altitude reset if velocity is too high
  if ( abs ( VelocityZ ) > 200 )
    AltRstRequired = 1;

  // If the drone is disarmed and on the ground, reset altitude estimation
  if ( ! ARMING_FLAG ( ARMED ) ) {
    if ( ABS ( VelocityZ ) < 5 && ABS ( accel_ef_z ) < 0.05f ) {
      _position_base_z       = 0;
      _position_correction_z = 0;
      _velocity_z            = 0;
      EstAlt                 = 0;    // Force altitude reset
    }
  }
}

  #ifdef LASER_ALT
void checkReading ( ) {
  uint32_t baro_update_time;
  float dt;
  float tilt = 0;
  // float tofTransition;
  static int32_t baro_offset = 0;

  // Baro update reading
  baro_update_time = getBaroLastUpdate ( );
  if ( baro_update_time != baro_last_update ) {
    dt          = ( float ) ( baro_update_time - baro_last_update ) * 0.001f;    // in seconds
    Baro_Height = baroCalculateAltitude ( );
    filtered    = ( 0.75f * filtered ) + ( ( 1 - 0.75f ) * Baro_Height );

    baro_last_update = baro_update_time;
  }

  // Laser sensor update reading
  if ( isTofDataNew ( ) && ( ! isOutofRange ( ) ) ) {

    ToF_Height       = ( float ) NewSensorRange / 10.0f;
    isTofDataNewflag = false;

    tilt = degreesToRadians ( calculateTiltAngle ( &inclination ) / 10 );
    if ( tilt < 25 )
      ToF_Height *= cos_approx ( tilt );
  }

  // Fusion
  if ( ( ToF_Height > 0 && ToF_Height < 200 ) && ( ! isOutofRange ( ) ) ) {
    //   	baro_offset = ToF_Height-filtered;
    baro_offset = filtered - ToF_Height;
    // baroAlt_offset_print = baro_offset;
    correctedWithTof ( ToF_Height );
  } /* else
   //{ Baro_Height -= baro_offset;
   if (ToF_Height >= 120  && ToF_Height <= 200) {
   //tofTransition = (200 - ToF_Height) / 100.0f;
   tofTransition = 0.5f;
   fused = ToF_Height * tofTransition + Baro_Height * (1.0f - tofTransition);

   correctedWithBaro( fused, dt);
   } */
  else {
    correctedWithBaro ( Baro_Height - baro_offset, dt );
  }
}
  #endif

void checkBaro ( ) {

  uint32_t baro_update_time;

  // calculate time since last baro reading (in ms)

  baro_update_time = getBaroLastUpdate ( );

  if ( baro_update_time != baro_last_update ) {

    const float dt = ( float ) ( baro_update_time - baro_last_update ) * 0.001f;    // in seconds
    // call correction method
    correctedWithBaro ( baroCalculateAltitude ( ), dt );

    baro_last_update = baro_update_time;
  }

  //    if (baro_update_time != baro_last_update1) {
  //
  //
  //        const float dt = (float) (baro_update_time - baro_last_update1) * 0.001f; // in seconds
  //        // call correction method
  //        correctedWithBaro(apmBaroCalculateAltitude(), dt);
  //
  //
  //        baro_last_update1 = baro_update_time;
  //    }
}

void correctedWithBaro ( float baroAlt, float dt ) {
  altholdDebug5 = baroAlt;

  if ( dt > 0.5f ) return;    // Ignore old data

  float hist_position_base_z = isPositionBaseQueueIsFull ( ) ? getFrontHistPositionBaseEstZ ( ) : _position_base_z;

  // Apply exponential moving average (EMA) to smooth altitude correction
  static float smoothed_position_error_z = 0;
  float position_error_z                 = baroAlt - ( hist_position_base_z + _position_correction_z );
  smoothed_position_error_z              = 0.95f * smoothed_position_error_z + 0.4f * position_error_z;

  _position_error_z = smoothed_position_error_z;

  // Adaptive gain adjustment based on inclination
  float inclination_roll  = ABS ( inclination_generalised.values.rollDeciDegrees );
  float inclination_pitch = ABS ( inclination_generalised.values.pitchDeciDegrees );

  float new_time_constant = ( inclination_roll > 30 || inclination_pitch > 30 ) ? 5.0f : 2.0f;
  _time_constant_z        = 0.85f * _time_constant_z + 0.15f * new_time_constant;    // Smooth transition

  updateGains ( );
}

  #ifdef LASER_ALT
void correctedWithTof ( float ToF_Height ) {
  if ( first_reads == 0 ) {
    setAltitude ( ToF_Height );
    first_reads++;
  }
  _position_error_z = ToF_Height - EstAlt;
  if ( _time_constant_z != 1.5f ) {
    _time_constant_z = 1.5;
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

// void setAltitude ( float new_altitude )

// {
//   _position_base_z       = new_altitude;
//   _position_correction_z = 0;
//   _position_z            = new_altitude;    // _position = _position_base + _position_correction
//   last_hist_position     = 0;               // reset z history to avoid fake z velocity at next baro calibration (next rearm)
//   imuResetAccelerationSum ( 1 );
// }

void setAltitude ( float new_altitude ) {
  static bool groundLevelSet     = false;
  static float groundLevelOffset = 0.0f;

  if ( ! groundLevelSet ) {
    groundLevelOffset = new_altitude;    // Store initial ground level
    groundLevelSet    = true;
  }

  _position_base_z       = new_altitude - groundLevelOffset;    // Normalize altitude
  _position_correction_z = 0;
  _position_z            = _position_base_z;
  last_hist_position     = 0;    // Reset history to avoid fake z velocity after landing
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
