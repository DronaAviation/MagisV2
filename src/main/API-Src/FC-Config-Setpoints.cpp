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
 #  Last Modified: Thu, 15th Oct 2026                                          #
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

int32_t getDesiredAngle ( angle_e ANGLE ) {
  // Return the desired angle value from the desiredAngle array based on the specified angle type
  return desiredAngle [ ANGLE ];
}

void setDesiredAngle ( angle_e ANGLE, int32_t angle ) {
  // Use a switch statement to determine which angle value should be set based on the specified angle type
  switch ( ANGLE ) {
    case AG_ROLL:
      // If the angle is roll, set the roll inclination in deci-degrees
      inclination.values.rollDeciDegrees = angle;
      break;
    case AG_PITCH:
      // If the angle is pitch, set the pitch inclination in deci-degrees
      inclination.values.pitchDeciDegrees = angle;
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
  // Only yaw carries an absolute setpoint the firmware holds between loops.
  // Roll and pitch are re-derived from the sticks on every PID iteration, so a
  // relative offset on them would be overwritten before it could take effect.
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
  // Return the desired rate value from the desiredRate array based on the specified angle type
  return desiredRate [ ANGLE ];
}

void setDesiredRate ( angle_e ANGLE, int32_t rate ) {
  // Use a switch statement to determine which control rate should be set based on the specified angle
  switch ( ANGLE ) {
    case AG_ROLL:
      // If the angle is roll, set the roll rate in the current control rate profile
      currentControlRateProfile->rates [ FD_ROLL ] = rate;
      break;
    case AG_PITCH:
      // If the angle is pitch, set the pitch rate in the current control rate profile
      currentControlRateProfile->rates [ FD_PITCH ] = rate;
      break;
    case AG_YAW:
      // If the angle is yaw, set the yaw rate in the current control rate profile
      currentControlRateProfile->rates [ FD_YAW ] = rate;
      break;
    default:
      // Default case to handle any unspecified angles; no action needed
      break;
  }
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

  // Enable low battery failsafe if 'enableAll' is true or if FAILSAFE is specifically LOW_BATTERY
  fsLowBattery = enableAll || ( FAILSAFE == LOW_BATTERY );

  // Enable in-flight low battery failsafe if 'enableAll' is true or if FAILSAFE is specifically INFLIGHT_LOW_BATTERY
  fsInFlightLowBattery = enableAll || ( FAILSAFE == INFLIGHT_LOW_BATTERY );

  // Enable crash failsafe if 'enableAll' is true or if FAILSAFE is specifically CRASH
  fsCrash = enableAll || ( FAILSAFE == CRASH );
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
