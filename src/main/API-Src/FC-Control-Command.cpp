/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\FC-Control-Command.cpp                             #
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

#include "common/axis.h"

#include "drivers/sensor.h"

#include "drivers/accgyro.h"

#include "sensors/barometer.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "io/gps.h"
#include "io/rc_controls.h"

#include "flight/imu.h"

#include "telemetry/telemetry.h"

#include "config/runtime_config.h"
#include "config/config_profile.h"

#include "command/command.h"

#include "mw.h"

#include "API/FC-Control.h"
#include "API/API-Utils.h"

bool FlightMode_Check ( flight_mode_e MODE ) {
  switch ( MODE ) {
    case ANGLE:
      // Check if the current mode is ANGLE_MODE
      return FLIGHT_MODE ( ANGLE_MODE );
    case RATE:
      // Check if the current mode is not ANGLE_MODE
      return ! FLIGHT_MODE ( ANGLE_MODE );
    case MAGHOLD:
      // Check if the current mode is MAG_MODE
      return FLIGHT_MODE ( MAG_MODE );
    case HEADFREE:
      // Check if the current mode is HEADFREE_MODE
      return FLIGHT_MODE ( HEADFREE_MODE );
    case ATLTITUDEHOLD:
      // Check if the current mode is BARO_MODE
      return FLIGHT_MODE ( BARO_MODE );
    case THROTTLE_MODE:
      // Check if the current mode is not BARO_MODE
      return ! FLIGHT_MODE ( BARO_MODE );
    default:
      break;
  }

  // Return false if no matching mode is found
  return false;
}

void FlightMode_Set ( flight_mode_e MODE ) {
  switch ( MODE ) {
    case ANGLE:
      // Enable ANGLE_MODE and update user flight mode states
      ENABLE_FLIGHT_MODE ( ANGLE_MODE );
      isUserFlightModeSet [ ANGLE ] = true;
      isUserFlightModeSet [ RATE ]  = false;
      break;
    case RATE:
      // Disable ANGLE_MODE and update user flight mode states
      DISABLE_FLIGHT_MODE ( ANGLE_MODE );
      isUserFlightModeSet [ RATE ]  = true;
      isUserFlightModeSet [ ANGLE ] = false;
      break;
    case MAGHOLD:
      // Enable MAG_MODE, disable HEADFREE_MODE, and update user flight mode states
      ENABLE_FLIGHT_MODE ( MAG_MODE );
      DISABLE_FLIGHT_MODE ( HEADFREE_MODE );
      isUserFlightModeSet [ MAGHOLD ]  = true;
      isUserFlightModeSet [ HEADFREE ] = false;
      break;
    case HEADFREE:
      // Enable HEADFREE_MODE, disable MAG_MODE, and update user flight mode states
      ENABLE_FLIGHT_MODE ( HEADFREE_MODE );
      DISABLE_FLIGHT_MODE ( MAG_MODE );
      isUserFlightModeSet [ HEADFREE ] = true;
      isUserFlightModeSet [ MAGHOLD ]  = false;
      break;
    case ATLTITUDEHOLD:
      // Enable BARO_MODE and update user flight mode states
      ENABLE_FLIGHT_MODE ( BARO_MODE );
      isUserFlightModeSet [ ATLTITUDEHOLD ] = true;
      isUserFlightModeSet [ THROTTLE_MODE ] = false;
      break;
    case THROTTLE_MODE:
      // Disable BARO_MODE and update user flight mode states
      DISABLE_FLIGHT_MODE ( BARO_MODE );
      isUserFlightModeSet [ THROTTLE_MODE ] = true;
      isUserFlightModeSet [ ATLTITUDEHOLD ] = false;
      break;
    default:
      // No action for unsupported modes
      break;
  }
}

void Command_TakeOff ( uint16_t height ) {
  // Check if the current command is not already TAKE_OFF
  if ( current_command != TAKE_OFF ) {
    // Set the current command to TAKE_OFF and update command status to RUNNING
    current_command = TAKE_OFF;
    command_status  = RUNNING;

    // Constrain the take-off height and set it
    height        = constrain ( height, 100, 250 );
    takeOffHeight = height;
  }
}

void Command_Land ( uint8_t landSpeed ) {
  // Check if the command status is FINISHED or ABORTED and if the system is armed
  if ( ( command_status == FINISHED || command_status == ABORT ) && ARMING_FLAG ( ARMED ) ) {
    // Set the current command to LAND and update the command status to RUNNING
    current_command = LAND;
    command_status  = RUNNING;

    // Adjust the throttle for landing
    landThrottle = 1305 - landSpeed;
  }
}

void Command_Flip ( flip_direction_e direction ) {
  // Check if the current command is not already B_FLIP
  if ( current_command != B_FLIP ) {
    // Set the current command to B_FLIP
    current_command = B_FLIP;
  }
}

bool Command_Arm ( void ) {
  // Check if RC mode BOXARM is active and it's OK to arm
  if ( IS_RC_MODE_ACTIVE ( BOXARM ) && ARMING_FLAG ( OK_TO_ARM ) ) {
    // Reset PID errors for angle and gyro, then arm the system
    pidResetErrorAngle ( );
    pidResetErrorGyro ( );
    mwArm ( );
  }
  // Return the arming status
  return ARMING_FLAG ( ARMED );
}

bool Command_DisArm ( void ) {
  // Disarm the system
  mwDisarm ( );
  // Return the negation of the arming status
  return ! ARMING_FLAG ( ARMED );
}

// Function to set the head-free mode heading
void setheadFreeModeHeading ( int16_t heading ) {
  // Assign the input heading to userHeadFreeHoldHeading variable
  userHeadFreeHoldHeading = heading;
  // Set the flag indicating that the user head-free hold heading is set
  isUserHeadFreeHoldSet = true;
}
