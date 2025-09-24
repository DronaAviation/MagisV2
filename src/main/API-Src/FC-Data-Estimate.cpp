/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 MechAsh (j.mechash@gmail.com)                 #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\FC-Data-Estimate.cpp                               #
 #  Created Date: Sat, 23rd Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 18th Sep 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include "platform.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "rx/rx.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"

#include "io/display.h"

#include "telemetry/telemetry.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/posEstimate.h"
#include "config/config.h"
#include "config/runtime_config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "API/API-Utils.h"
#include "API/FC-Data.h"


int16_t Estimate_Get ( FC_Estimate_e _estimateOf, axis_e _axis ) {
  // Use a switch statement to handle different types of estimates.
  switch ( _estimateOf ) {
    case Rate:    // Handle Rate estimates.
      // Return the roll rate if the axis is X.
      if ( _axis == X ) return currentControlRateProfile->rates [ FD_ROLL ];
      // Return the pitch rate if the axis is Y.
      else if ( _axis == Y )
        return currentControlRateProfile->rates [ FD_PITCH ];
      // Return the yaw rate if the axis is Z.
      else if ( _axis == Z )
        return currentControlRateProfile->rates [ FD_YAW ];
      break;

    case Position:    // Handle Position estimates.
      // Return the position along X-axis.
      if ( _axis == X ) return PositionX;
      // Return the position along Y-axis.
      else if ( _axis == Y )
        return PositionY;
      // Return the estimated altitude for Z-axis.
      else if ( _axis == Z )
        return getEstAltitude ( );
      break;

    case Velocity:    // Handle Velocity estimates.
      // Return the velocity along X-axis.
      if ( _axis == X ) return VelocityX;
      // Return the velocity along Y-axis.
      else if ( _axis == Y )
        return VelocityY;
      // Return the estimated velocity for Z-axis.
      else if ( _axis == Z )
        return getEstVelocity ( );
      break;

    default:
      // Return 0 for any unhandled estimate types.
      return 0;
  }
  // Default return to handle any unexpected cases outside of defined logic.
  return 0;
}

int16_t Estimate_Get(FC_Estimate_e _estimateOf, angle_e _angle) {
    // Check if the requested estimate is of type Angle.
    if (_estimateOf == Angle) {
        // Determine which angle value to return based on the specified angle type.
        switch (_angle) {
            case AG_ROLL:
                // Return the roll angle in deciDegrees.
                return inclination.values.rollDeciDegrees; // unit: deciDegree
            case AG_PITCH:
                // Return the pitch angle in deciDegrees.
                return inclination.values.pitchDeciDegrees; // unit: deciDegree
            case AG_YAW:
                // Return the yaw angle in degrees.
                return heading; // unit: degree
            default:
                // Handle any unanticipated angle types.
                break;
        }
    }
    // If the estimate type is not Angle or an unexpected case occurs, return 0.
    return 0;
}
