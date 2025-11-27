/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\ranging_vl53l1x.cpp                                #
 #  Created Date: Sat, 8th Nov 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 27th Nov 2025                                          #
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

#include "vl53l1_platform.h"
// #include "vl53l0x_i2c_platform.h"

#include "vl53l1_api_core.h"
#include "vl53l1_api_strings.h"
#include "vl53l1_def.h"
#include "vl53l1_api.h"
#include "vl53l1_types.h"

#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/bus_i2c.h"
#include "drivers/system.h"
#include "ranging_vl53l1x.h"
#include "API/Peripherals.h"
#include "API/Debugging.h"
#include "API/Scheduler-Timer.h"

#define LASER_LPS  0.75
#define RANGE_POLL 10

VL53L1_Dev_t MyDevice_L1;
// VL53L0X_Dev_t *pMyDevice = &MyDevice;

VL53L1_Error Global_Status_L1 = 0;
// VL53L1_Error _Global_Status_L1 = 0;
VL53L1_RangingMeasurementData_t RangingMeasurementData_L1;
VL53L1_RangingMeasurementData_t _RangingMeasurementData_L1x;

uint8_t Range_Status_L1    = 0;
uint16_t NewSensorRange_L1 = 0;
uint16_t debug_range_L1    = 0;
bool isTofDataNewflag_L1   = false;
bool out_of_range_L1       = false;
bool startRanging_L1       = false;    // Cleanup later
bool useRangingSensor_L1   = false;    // Cleanup later

Interval rangePoll_L1;

void update_status_L1 ( VL53L1_Error Status ) {
  Global_Status_L1 = Status;
}

LaserSensor_L1 XVision;

#ifdef LASER_TOF_L1x

void ranging_init_L1 ( void ) {
  VL53L1_Error Status = Global_Status_L1;

  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;

  MyDevice_L1.I2cDevAddr      = 0x29;
  MyDevice_L1.comms_type      = 1;
  MyDevice_L1.comms_speed_khz = 400;

  Status = VL53L1_WaitDeviceBooted ( &MyDevice_L1 );    // Wait till the device boots. Blocking.

  if ( Status == VL53L1_ERROR_NONE )
    Status = VL53L1_DataInit ( &MyDevice_L1 );    // Data initialization

  update_status_L1 ( Status );

  if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {
    Status = VL53L1_StaticInit ( &MyDevice_L1 );    // Device Initialization
    update_status_L1 ( Status );
  }

  if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {
    Status = VL53L1_SetDistanceMode ( &MyDevice_L1, VL53L1_DISTANCEMODE_MEDIUM );    // Device Initialization
    update_status_L1 ( Status );
  }
}

void getRange_L1 ( ) {
  VL53L1_Error Status     = Global_Status_L1;
  static uint8_t dataFlag = 0, SysRangeStatus = 0;
  static bool startNow = true;
  static uint8_t print_counter;

  if ( rangePoll_L1.set ( RANGE_POLL, true ) ) {    // Check for new data every 10ms

    if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {
      if ( startNow ) {
        Status = VL53L1_StartMeasurement ( &MyDevice_L1 );
        update_status_L1 ( Status );
        startNow = false;
      }
    }

    if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {    // Check if data is ready
      if ( ! startNow ) {
        Status = VL53L1_GetMeasurementDataReady ( &MyDevice_L1, &dataFlag );
        update_status_L1 ( Status );
      }
    }

    if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {
      if ( dataFlag ) {
        Status = VL53L1_GetRangingMeasurementData ( &MyDevice_L1, &RangingMeasurementData_L1 );
        update_status_L1 ( Status );

        // startNow            = true;
        isTofDataNewflag_L1 = true;
        Range_Status_L1     = RangingMeasurementData_L1.RangeStatus;

        if ( RangingMeasurementData_L1.RangeStatus == 0 ) {

          if ( RangingMeasurementData_L1.RangeMilliMeter < 4500 ) {
            // NewSensorRange_L1 = ( NewSensorRange_L1 * 0.25f ) + ( RangingMeasurementData_L1.RangeMilliMeter * 0.75f );    // low-pass smoothing
            NewSensorRange_L1 = RangingMeasurementData_L1.RangeMilliMeter;
            out_of_range_L1   = false;

          } else {
            out_of_range_L1 = true;
          }

        } else
          out_of_range_L1 = true;
        // dataFlag = 0;    // Reset the data flag
      }
    }
  }
}

bool isTofDataNew_L1 ( void ) {
  return isTofDataNewflag_L1;
}

bool isOutofRange_L1 ( void ) {
  return out_of_range_L1;
}

#endif

static bool _startNow = true;

bool LaserSensor_L1::init ( VL53L1_DistanceModes _DistanceMode ) {

  // Set the I2C device address for the sensor
  MyDevice_L1x.I2cDevAddr = 0x29;
  // Set the communication type (1 indicates I2C)
  MyDevice_L1x.comms_type = 1;
  // Set the communication speed in kHz
  MyDevice_L1x.comms_speed_khz = 400;

  // Wait until the device has completed its boot process.
  // This is a blocking call that updates _Global_Status_L1x with the result.
  _Global_Status_L1x = VL53L1_WaitDeviceBooted ( &MyDevice_L1x );

  // Check if there was no error during the boot process
  if ( _Global_Status_L1x == VL53L1_ERROR_NONE )
    // Initialize data structures related to the device
    _Global_Status_L1x = VL53L1_DataInit ( &MyDevice_L1x );

  // If data initialization was successful, proceed with static device initialization
  if ( _Global_Status_L1x == VL53L1_ERROR_NONE )
    _Global_Status_L1x = VL53L1_StaticInit ( &MyDevice_L1x );

  // If static initialization was successful, set the distance mode to medium
  if ( _Global_Status_L1x == VL53L1_ERROR_NONE )
    _Global_Status_L1x = VL53L1_SetDistanceMode ( &MyDevice_L1x, _DistanceMode );

  // Return true if all operations were successful, otherwise return false
  if ( _Global_Status_L1x == VL53L1_ERROR_NONE )
    return true;
  return false;
}

void LaserSensor_L1::setAddress ( uint8_t _address ) {
  // Call the VL53L1 API function to set the device address.
  // The address is multiplied by 2 due to the I2C protocol shift requirement.
  VL53L1_SetDeviceAddress ( &MyDevice_L1x, ( _address * 2 ) );
  // Update the internal record of the device's I2C address.
  MyDevice_L1x.I2cDevAddr = _address;
}

bool LaserSensor_L1::startRanging ( void ) {

  if ( _Global_Status_L1x == VL53L1_ERROR_NONE && _startNow ) {
    _Global_Status_L1x = VL53L1_StartMeasurement ( &MyDevice_L1x );
    _startNow          = false;
    Monitor_Println ( "check1", _Global_Status_L1x );
  }
  if ( _Global_Status_L1x == VL53L1_ERROR_NONE ) {
    _Global_Status_L1x = VL53L1_GetRangingMeasurementData ( &MyDevice_L1x, &_RangingMeasurementData_L1x );

    isTofDataNewflag_L1 = true;
    if ( ! _RangingMeasurementData_L1x.RangeStatus ) {

      if ( _RangingMeasurementData_L1x.RangeMilliMeter < 4500 ) {

        _range = _RangingMeasurementData_L1x.RangeMilliMeter;
        return true;
      }
    }
  }
  _range = -1;
  return false;
}

int16_t LaserSensor_L1::getLaserRange ( void ) {
  return _range;
}
