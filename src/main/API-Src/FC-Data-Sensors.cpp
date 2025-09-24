/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\FC-Data-Sensors.cpp                                #
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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/ranging_vl53l0x.h"

#include "sensors/sensors.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "API/FC-Data.h"

uint32_t Sensor_Get ( FC_Sensors_e _sensor, axis_e _axis ) {
  // Constants for scaling gyro and magnetometer values
  const float gyroScale = 1.64f;
  const float magScale  = 6.82f;

  // Determine sensor type and retrieve corresponding data based on the axis
  switch ( _sensor ) {
    case Accelerometer: {
      // Check if net acceleration magnitude is requested
      if ( _axis == Net_Acc ) return netAccMagnitude;
      // Return scaled accelerometer value for the specified axis
      return ( int32_t ) ( accSmooth [ _axis ] * accVelScale );
    }
    case Gyroscope: {
      // Return scaled gyroscope value for the specified axis
      if ( _axis != Net_Acc ) return ( int32_t ) ( gyroADC [ _axis ] / gyroScale );
      break;
    }
    case Magnetometer: {
      // Return scaled magnetometer value for the specified axis
      if ( _axis != Net_Acc ) return ( int32_t ) ( magADC [ _axis ] / magScale );
      break;
    }
    default:
      // Return 0 as a fallback for unsupported sensor types
      return 0;
  }
}

uint32_t Sensor_Get ( FC_Sensors_e _sensor, BARO_Data_e _data ) {
  // Ensure the sensor is a barometer before proceeding
  if ( _sensor != Barometer ) return 0;

  // Retrieve barometer data based on the request type
  switch ( _data ) {
    case Pressure:
      // Return barometric pressure in units of 100*millibar
      return getBaroPressure ( );
    case Temperature:
      // Return barometric temperature in units of 100*degreeCelsius
      return getBaroTemperature ( );
    default:
      // Return 0 as a fallback for unsupported data types
      return 0;
  }
}
