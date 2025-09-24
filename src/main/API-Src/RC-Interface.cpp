/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\RC-Interface.cpp                                   #
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

#include <stdint.h>

#include "platform.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "blackbox/blackbox.h"

#include "drivers/system.h"
#include "drivers/system.h"
#include "drivers/sensor.h"

#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"

#include "io/display.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "command/command.h"

#include "mw.h"

#include "API/API-Utils.h"
#include "API/RC-Interface.h"

// Function to get a copy of the rcData array
int16_t *RcData_Get ( void ) {
  int16_t rcDataArray [ 11 ];    // Temporary array to store copied data

  // Copy elements from global rcData to local rcDataArray
  for ( int i = 0; i < 11; i++ ) {
    rcDataArray [ i ] = rcData [ i ];
  }

  // Returning the address of a local array will lead to undefined behavior
  return rcDataArray;
}

// Function to get a specific channel value from rcData
int16_t RcData_Get ( rc_channel_e CHANNEL ) {
  // Return the rcData value at the specified channel index
  return rcData [ CHANNEL ];
}

// Function to get a copy of the rcCommand array
int16_t *RcCommand_get ( void ) {
  int16_t rcCommandArray [ 4 ];    // Temporary array to store copied command data

  // Copy elements from global rcCommand to local rcCommandArray
  for ( int i = 0; i < 4; i++ ) {
    rcCommandArray [ i ] = rcCommand [ i ];
  }

  // Returning the address of a local array will lead to undefined behavior
  return rcCommandArray;
}

// Function to get a specific channel value from rcCommand with an offset
int16_t RcCommand_Get ( rc_channel_e CHANNEL ) {
  // Check if the channel is within the valid range
  if ( CHANNEL <= RC_THROTTLE ) {
    // Return the rcCommand value at the specified channel, offset by 1500
    return 1500 + rcCommand [ CHANNEL ];
  }
  // Return 0 if the channel is not valid
  return 0;
}

void RcCommand_Set ( int16_t *rcValueArray ) {
  // Loop through the first 4 elements of rcValueArray
  for ( int i = 0; i < 4; i++ ) {
    // Constrain rcValue to be between 1000 and 2000
    int16_t rcValue = constrain ( rcValueArray [ i ], 1000, 2000 );

    if ( i < 3 ) {
      // Adjust rcValue and assign to rcCommand for the first three indices
      rcValue -= 1500;
      rcCommand [ i ] = rcValue;
    } else {
      // For the fourth index, directly assign rcValue to rcData
      rcData [ i ] = rcValue;
    }

    // Store the constrained value in RC_ARRAY
    RC_ARRAY [ i ] = rcValue;

    // Set userRCflag to true indicating that the command has been set
    userRCflag [ i ] = true;
  }
}

void RcCommand_Set ( rc_channel_e CHANNEL, int16_t rcValue ) {
  // Constrain the input value between 1000 and 2000
  int16_t setValue = constrain ( rcValue, 1000, 2000 );

  // Update values if channel is below RC_THROTTLE
  if ( CHANNEL < RC_THROTTLE ) {
    // Adjust setValue by subtracting 1500 for non-throttle channels
    setValue -= 1500;
    rcCommand [ CHANNEL ] = setValue;
  }

  // Common assignments for any channel
  // Store the constrained or adjusted value in RC_ARRAY
  RC_ARRAY [ CHANNEL ] = setValue;

  // Set the userRCflag to true indicating a command has been set for the channel
  userRCflag [ CHANNEL ] = true;

  // Assign rcData only if channel is RC_THROTTLE
  if ( CHANNEL == RC_THROTTLE ) {
    // Directly assign setValue to rcData for the throttle channel
    rcData [ CHANNEL ] = setValue;
  }
}

int16_t App_getAppHeading ( void ) {
  return appHeading;
}

bool App_isArmSwitchOn ( void ) {
  return IS_RC_MODE_ACTIVE ( BOXARM );
}
