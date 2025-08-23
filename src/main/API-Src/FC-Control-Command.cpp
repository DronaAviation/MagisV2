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
 #  Last Modified: Sat, 23rd Aug 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include "platform.h"

#include "config/runtime_config.h"

#include "API/FC-Control.h"
#include "API/API-Utils.h"


bool FlightMode_check(flight_mode_e MODE) {
  switch (MODE) {
    case ANGLE:
      // Check if the current mode is ANGLE_MODE
      return FLIGHT_MODE(ANGLE_MODE);
    case RATE:
      // Check if the current mode is not ANGLE_MODE
      return !FLIGHT_MODE(ANGLE_MODE);
    case MAGHOLD:
      // Check if the current mode is MAG_MODE
      return FLIGHT_MODE(MAG_MODE);
    case HEADFREE:
      // Check if the current mode is HEADFREE_MODE
      return FLIGHT_MODE(HEADFREE_MODE);
    case ATLTITUDEHOLD:
      // Check if the current mode is BARO_MODE
      return FLIGHT_MODE(BARO_MODE);
    case THROTTLE_MODE:
      // Check if the current mode is not BARO_MODE
      return !FLIGHT_MODE(BARO_MODE);
    default:
      break;
  }

  // Return false if no matching mode is found
  return false;
}


void FlightMode_set(flight_mode_e MODE) {
  switch (MODE) {
    case ANGLE:
      // Enable ANGLE_MODE and update user flight mode states
      ENABLE_FLIGHT_MODE(ANGLE_MODE);
      isUserFlightModeSet[ANGLE] = true;
      isUserFlightModeSet[RATE] = false;
      break;
    case RATE:
      // Disable ANGLE_MODE and update user flight mode states
      DISABLE_FLIGHT_MODE(ANGLE_MODE);
      isUserFlightModeSet[RATE] = true;
      isUserFlightModeSet[ANGLE] = false;
      break;
    case MAGHOLD:
      // Enable MAG_MODE, disable HEADFREE_MODE, and update user flight mode states
      ENABLE_FLIGHT_MODE(MAG_MODE);
      DISABLE_FLIGHT_MODE(HEADFREE_MODE);
      isUserFlightModeSet[MAGHOLD] = true;
      isUserFlightModeSet[HEADFREE] = false;
      break;
    case HEADFREE:
      // Enable HEADFREE_MODE, disable MAG_MODE, and update user flight mode states
      ENABLE_FLIGHT_MODE(HEADFREE_MODE);
      DISABLE_FLIGHT_MODE(MAG_MODE);
      isUserFlightModeSet[HEADFREE] = true;
      isUserFlightModeSet[MAGHOLD] = false;
      break;
    case ATLTITUDEHOLD:
      // Enable BARO_MODE and update user flight mode states
      ENABLE_FLIGHT_MODE(BARO_MODE);
      isUserFlightModeSet[ATLTITUDEHOLD] = true;
      isUserFlightModeSet[THROTTLE_MODE] = false;
      break;
    case THROTTLE_MODE:
      // Disable BARO_MODE and update user flight mode states
      DISABLE_FLIGHT_MODE(BARO_MODE);
      isUserFlightModeSet[THROTTLE_MODE] = true;
      isUserFlightModeSet[ATLTITUDEHOLD] = false;
      break;
    default:
      // No action for unsupported modes
      break;
  }
}
