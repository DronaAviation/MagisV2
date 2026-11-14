/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\FC-Config-Setpoints.cpp                            #
 #  Created Date: Sun, 24th Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sun, 15th Nov 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#include <stdint.h>

#include <stdint.h>

#include "platform.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
// #include "blackbox/blackbox.h"

#include "drivers/system.h"
#include "drivers/sensor.h"

#include "drivers/accgyro.h"
// #include "drivers/light_led.h"

// #include "drivers/gpio.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

// #include "rx/rx.h"

#include "io/gps.h"
#include "io/ledstrip.h"
// #include "io/beeper.h"
// #include "io/escservo.h"
#include "io/rc_controls.h"
// #include "io/rc_curves.h"

// #include "io/display.h"

#include "flight/mixer.h"
#include "flight/pid.h"
// #include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"

#include "telemetry/telemetry.h"
// #include "blackbox/blackbox.h"

// #include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

// #include "command/command.h"

#include "mw.h"

#include "API/API-Utils.h"
#include "API/FC-Config.h"
#include "flight/acrobats.h"    // flipState: setDesiredRate ( ) leaves a running flip's rates alone

int32_t getDesiredAngle ( angle_e ANGLE ) {
  // Return the desired angle value from the desiredAngle array based on the specified angle type
  return desiredAngle [ ANGLE ];
}

void setDesiredAngle ( angle_e ANGLE, int32_t angle ) {
  // Use a switch statement to determine which angle value should be set based on the specified angle type
  switch ( ANGLE ) {
    case AG_ROLL:
    case AG_PITCH:
      // Roll / pitch: hold the user's angle target ( deci-degrees ) until it is replaced. pidRewrite ( )
      // adds it to the ANGLE-mode stick target and limits the sum to max_angle_inclination. This used
      // to write the measured attitude ( inclination ), which the next imuUpdate ( ) overwrote, so the
      // value was read back once and the drone never tilted ( API test T11 / B4 ). The +/-180 deg bound
      // only keeps the later sum in range; the real limit is the controller's clamp.
      userDesiredAngle [ ANGLE ]   = constrain ( angle, -1800, 1800 );
      isUserDesiredAngle [ ANGLE ] = true;
      break;
    case AG_YAW: {
      // If the angle is yaw, set the magnetic hold and user heading, then mark the user heading as set.
      // Wrap into 0..359 first: updateMagHold ( ) folds the heading error by 360 only once, so an
      // unwrapped bearing of 720 or more ( e.g. 900 written as deci-degrees ), or -361 or less,
      // never settles and the craft keeps yawing.
      int32_t bearing = angle % 360;
      if ( bearing < 0 ) bearing += 360;

      magHold               = ( int16_t ) bearing;
      userHeading           = ( int16_t ) bearing;
      isUserHeadingSet      = true;
      isUserHeadingRelative = false;    // absolute: return to this bearing after a pilot yaw
      userHeadingStale      = false;
      break;
    }
    default:
      // Default case to handle any unspecified angles; no action needed
      break;
  }
}

void DesiredAngle_setRelative ( angle_e ANGLE, int32_t angle ) {
  // Only yaw takes a relative target: a turn measured from the nose. Roll and
  // pitch are absolute tilts from level ( setDesiredAngle ), so a relative call
  // on them is ignored.
  if ( ANGLE != AG_YAW ) return;

  // Compose against the setpoint user code has already latched so repeated
  // calls accumulate exactly ( +90 three times is a full turn, not three
  // partial ones chasing a heading that is still slewing ). The first call
  // after entering Developer Mode has nothing latched, so it falls back to the
  // measured heading - that is the "turn X degrees from where the nose is
  // pointing now" case. So does a call made after yaw was taken off heading
  // hold ( pilot stick ) and before updateMagHold ( ) re-based the target: the
  // latched target no longer describes where the pilot left the nose.
  int32_t base = ( isUserHeadingSet && ! userHeadingStale ) ? ( int32_t ) userHeading : ( int32_t ) heading;

  // Wrap into the 0..359 range magHold is compared against in updateMagHold ( ).
  // Reduce the turn first so base + turn cannot overflow for any int32_t input.
  int32_t target = ( base + angle % 360 ) % 360;
  if ( target < 0 ) target += 360;

  magHold               = ( int16_t ) target;
  userHeading           = ( int16_t ) target;
  isUserHeadingSet      = true;
  isUserHeadingRelative = true;    // re-based to the nose after a pilot yaw
  userHeadingStale      = false;
}

int32_t getDesiredRate ( angle_e ANGLE ) {
  // The rate setDesiredRate ( ) holds, or the rate in use when none was set ( roll / pitch 75 in practice ).
  // AG_ROLL / AG_PITCH / AG_YAW share their index with FD_ROLL / FD_PITCH / FD_YAW.
  if ( ANGLE > AG_YAW ) return 0;
  if ( isUserDesiredRate [ ANGLE ] ) return userDesiredRate [ ANGLE ];
  return currentControlRateProfile->rates [ ANGLE ];
}

void setDesiredRate ( angle_e ANGLE, int32_t rate ) {
  // Cleanflight rate ( 0-100 roll / pitch, 0-255 yaw ), clamped to the limits MSP_SET_RC_TUNING uses. Held in
  // userDesiredRate [ ] until reboot or the next call, so the flip code restores it instead of the stock 75
  // ( userRollPitchRate ( ) ), and written to the live profile so it acts at once. Never saved to EEPROM here.
  // Exception: roll / pitch during a flip are held but not applied. userCode ( ) runs after flip ( true ) and before
  // the PID, and the flip sets its spin rate only once, so a live write would replace it; a low rate could stall the
  // flip into its 2 s PITCHING timeout, which disarms in the air. The flip's exit ( restoreRestRates ( ) ) applies the
  // held value instead. Yaw is never touched by the flip and always applies at once.
  int maxRate;
  switch ( ANGLE ) {
    case AG_ROLL:
    case AG_PITCH:
      maxRate = CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX;
      break;
    case AG_YAW:
      maxRate = CONTROL_RATE_CONFIG_YAW_RATE_MAX;
      break;
    default:
      return;
  }

  int clamped                 = constrain ( ( int ) rate, 0, maxRate );
  userDesiredRate [ ANGLE ]   = clamped;
  isUserDesiredRate [ ANGLE ] = true;

  if ( ANGLE != AG_YAW && flipState != 0 ) return;    // held; the flip's exit applies it
  currentControlRateProfile->rates [ ANGLE ] = ( uint8_t ) clamped;
}

int32_t getDesiredPositions ( axis_e AXIS ) {
  // Check if the specified axis is Z
  if ( AXIS == Z ) {
    // If the axis is Z, return the current set altitude
    return getSetAltitude ( );
  }
  // If the axis is not Z, return 0 as a default value
  return 0;
}

void setDesiredPosition ( axis_e AXIS, int32_t position ) {
  // Check if the specified axis is Z
  if ( AXIS == Z ) {
    // If the axis is Z, set the altitude using the provided position
    setAltitude ( position );
  }
}

void DesiredPosition_setRelative ( axis_e AXIS, int32_t position ) {
  // Check if the specified axis is Z
  if ( AXIS == Z ) {
    // If the axis is Z, set the relative altitude using the provided position
    setRelativeAltitude ( position );
  }
}

void Failsafe_enable ( failsafe_e FAILSAFE ) {
  // Determine if all failsafes need to be enabled
  bool enableAll = ( FAILSAFE == ALL );

  // Only ever set a flag here, never clear one: the failsafes not named keep their state
  // ( API test T07: assigning each flag switched the other two off, and enabling CRASH
  // removed the low-battery landing ).

  // Set the low battery failsafe if 'enableAll' is true or if FAILSAFE is specifically LOW_BATTERY
  // ( no firmware path reads fsLowBattery: it is kept for API compatibility )
  if ( enableAll || ( FAILSAFE == LOW_BATTERY ) ) {
    fsLowBattery = true;
  }

  // Set the in-flight low battery failsafe if 'enableAll' is true or if FAILSAFE is specifically INFLIGHT_LOW_BATTERY
  if ( enableAll || ( FAILSAFE == INFLIGHT_LOW_BATTERY ) ) {
    fsInFlightLowBattery = true;
  }

  // Set the crash failsafe if 'enableAll' is true or if FAILSAFE is specifically CRASH
  if ( enableAll || ( FAILSAFE == CRASH ) ) {
    fsCrash = true;
  }
}

void Failsafe_disable ( failsafe_e FAILSAFE ) {
  // Determine if all failsafes need to be disabled
  bool disableAll = ( FAILSAFE == ALL );

  // Disable low battery failsafe if 'disableAll' is true or if FAILSAFE is specifically LOW_BATTERY;
  // otherwise, retain the current state of fsLowBattery
  fsLowBattery = disableAll || ( FAILSAFE == LOW_BATTERY ) ? false : fsLowBattery;

  // Disable in-flight low battery failsafe if 'disableAll' is true or if FAILSAFE is specifically INFLIGHT_LOW_BATTERY;
  // otherwise, retain the current state of fsInFlightLowBattery
  fsInFlightLowBattery = disableAll || ( FAILSAFE == INFLIGHT_LOW_BATTERY ) ? false : fsInFlightLowBattery;

  // Disable crash failsafe if 'disableAll' is true or if FAILSAFE is specifically CRASH;
  // otherwise, retain the current state of fsCrash
  fsCrash = disableAll || ( FAILSAFE == CRASH ) ? false : fsCrash;
}
