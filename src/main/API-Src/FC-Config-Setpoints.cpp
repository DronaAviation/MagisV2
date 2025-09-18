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
 #  Last Modified: Sun, 24th Aug 2025                                          #
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
    case AG_YAW:
      // If the angle is yaw, set the magnetic hold and user heading, then mark the user heading as set
      magHold          = angle;
      userHeading      = angle;
      isUserHeadingSet = true;
      break;
    default:
      // Default case to handle any unspecified angles; no action needed
      break;
  }
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

void Failsafe_enable(failsafe_e FAILSAFE) {
  // Determine if all failsafes need to be enabled
  bool enableAll = (FAILSAFE == ALL);

  // Enable low battery failsafe if 'enableAll' is true or if FAILSAFE is specifically LOW_BATTERY
  fsLowBattery = enableAll || (FAILSAFE == LOW_BATTERY);

  // Enable in-flight low battery failsafe if 'enableAll' is true or if FAILSAFE is specifically INFLIGHT_LOW_BATTERY
  fsInFlightLowBattery = enableAll || (FAILSAFE == INFLIGHT_LOW_BATTERY);

  // Enable crash failsafe if 'enableAll' is true or if FAILSAFE is specifically CRASH
  fsCrash = enableAll || (FAILSAFE == CRASH);
}



void Failsafe_disable(failsafe_e FAILSAFE) {
  // Determine if all failsafes need to be disabled
  bool disableAll = (FAILSAFE == ALL);

  // Disable low battery failsafe if 'disableAll' is true or if FAILSAFE is specifically LOW_BATTERY;
  // otherwise, retain the current state of fsLowBattery
  fsLowBattery = disableAll || (FAILSAFE == LOW_BATTERY) ? false : fsLowBattery;

  // Disable in-flight low battery failsafe if 'disableAll' is true or if FAILSAFE is specifically INFLIGHT_LOW_BATTERY;
  // otherwise, retain the current state of fsInFlightLowBattery
  fsInFlightLowBattery = disableAll || (FAILSAFE == INFLIGHT_LOW_BATTERY) ? false : fsInFlightLowBattery;

  // Disable crash failsafe if 'disableAll' is true or if FAILSAFE is specifically CRASH;
  // otherwise, retain the current state of fsCrash
  fsCrash = disableAll || (FAILSAFE == CRASH) ? false : fsCrash;
}
