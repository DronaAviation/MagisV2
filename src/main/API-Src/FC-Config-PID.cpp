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
 * @brief Initializes the PID profiles with default gain values for all axes.
 */
void setDefault_PIDProfile(void) {
    // Define default PID proportional gain values for each axis/profile
    static const int defaultP8[] = {40, 40, 150, 100, 0};
    
    // Define default PID integral gain values for each axis/profile
    static const int defaultI8[] = {10, 10, 70, 0, 0};
    
    // Define default PID derivative gain values for each axis/profile
    static const int defaultD8[] = {30, 30, 50, 30, 0};

    // Iterate over each profile index from ROLL to PIDUSER
    for (int i = ROLL; i <= PIDUSER; ++i) {
        // Assign default proportional gain value to the current profile
        currentProfile->pidProfile.P8[i] = defaultP8[i];
        
        // Assign default integral gain value to the current profile
        currentProfile->pidProfile.I8[i] = defaultI8[i];
        
        // Assign default derivative gain value to the current profile
        currentProfile->pidProfile.D8[i] = defaultD8[i];
    }
}
