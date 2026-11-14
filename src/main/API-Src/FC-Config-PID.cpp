/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\FC-Config-PID.cpp                                  #
 #  Created Date: Sun, 24th Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sat, 14th Nov 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  14-11-2026	AJ	setDefault_PIDProfile ( ): restore the five API profiles   #
 #              	from pidProfileDefaultGains ( ); the old loop read past    #
 #              	its tables and overwrote the hidden gains.                 #
*******************************************************************************/

#include <stddef.h>
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
#include "io/ledstrip.h"
#include "io/rc_controls.h"
// #include "io/rc_curves.h"

// #include "io/display.h"

#include "flight/mixer.h"
#include "flight/pid.h"
// #include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
// #include "flight/altitudehold.h"

#include "telemetry/telemetry.h"
// #include "blackbox/blackbox.h"

// #include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

// #include "command/command.h"

// #include "mw.h"

#include "API/API-Utils.h"
#include "API/FC-Config.h"

// Function to set the user-defined loop frequency
void setUserLoopFrequency ( float frequency ) {
  uint32_t resultFrequency;    // Variable to store the frequency in Hz
  // Constrain the input frequency to be within 3.5 Hz and 2000 Hz
  frequency = constrainf ( frequency, 3.5, 2000 );
  // Convert frequency from Hz to milliHz for higher precision
  resultFrequency = frequency * 1000;
  // Assign the calculated frequency to the global/desired variable
  userLoopFrequency = resultFrequency;
}

/**
 * @brief Sets PID gains for a specified profile into the provided PID structure.
 *
 * @param PROFILE The PID profile type to retrieve gains from.
 * @param pid Pointer to the PID structure to be updated.
 */
void get_PIDProfile(pid_profile_e PROFILE, PID *pid) {
    // Define an array mapping profile indices to specific axis/profile constants
    static const int profileIndex[] = {ROLL, PITCH, YAW, PIDALT, PIDUSER};

    // Check if the provided PROFILE is within the valid range
    if (PROFILE >= PID_ROLL && PROFILE <= PID_USER) {
        // Retrieve and set the proportional gain for the specified profile
        pid->p = currentProfile->pidProfile.P8[profileIndex[PROFILE]];
        
        // Retrieve and set the integral gain for the specified profile
        pid->i = currentProfile->pidProfile.I8[profileIndex[PROFILE]];
        
        // Retrieve and set the derivative gain for the specified profile
        pid->d = currentProfile->pidProfile.D8[profileIndex[PROFILE]];
    }
}




/**
 * @brief Updates the PID profile with gains from the provided PID structure.
 *
 * @param PROFILE The PID profile type to update.
 * @param pid Pointer to the PID structure containing new gain values.
 */
void set_PIDProfile(pid_profile_e PROFILE, PID *pid) {
    // Define an array mapping profile indices to specific axis/profile constants
    static const int profileIndex[] = {ROLL, PITCH, YAW, PIDALT, PIDUSER};

    // Check if the provided PROFILE is within the valid range
    if (PROFILE >= PID_ROLL && PROFILE <= PID_USER) {
        // Set the proportional gain for the specified profile
        currentProfile->pidProfile.P8[profileIndex[PROFILE]] = pid->p;
        
        // Set the integral gain for the specified profile
        currentProfile->pidProfile.I8[profileIndex[PROFILE]] = pid->i;
        
        // Set the derivative gain for the specified profile
        currentProfile->pidProfile.D8[profileIndex[PROFILE]] = pid->d;
    }
}



/**
 * @brief Restores the firmware default gains of the five API profiles.
 *
 * Only the slots get_PIDProfile ( ) / set_PIDProfile ( ) expose are written
 * ( PIDROLL, PIDPITCH, PIDYAW, PIDALT, PIDUSER ); the hidden gains ( PIDLEVEL,
 * PIDVEL, PIDPOS, PIDPOSR, PIDNAVR, PIDMAG ) are left as they are. The values
 * come from pidProfileDefaultGains ( ), the same source as a config reset, so
 * they cannot drift from the firmware defaults. RAM only: nothing is saved.
 */
void setDefault_PIDProfile ( void ) {
  // The API profiles, in pid_profile_e order.
  static const pidIndex_e apiSlot [] = { PIDROLL, PIDPITCH, PIDYAW, PIDALT, PIDUSER };

  pidProfile_t defaults = { };    // 88 B on the stack, called from user code, not the control path
  pidProfileDefaultGains ( &defaults );

  for ( size_t n = 0; n < sizeof ( apiSlot ) / sizeof ( apiSlot [ 0 ] ); n++ ) {
    const pidIndex_e slot = apiSlot [ n ];

    currentProfile->pidProfile.P8 [ slot ] = defaults.P8 [ slot ];
    currentProfile->pidProfile.I8 [ slot ] = defaults.I8 [ slot ];
    currentProfile->pidProfile.D8 [ slot ] = defaults.D8 [ slot ];
  }
}
