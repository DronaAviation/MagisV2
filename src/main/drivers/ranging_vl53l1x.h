/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\ranging_vl53l1x.h                                  #
 #  Created Date: Sat, 8th Nov 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Fri, 14th Nov 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "vl53l1_def.h"
#include "vl53l1_LL_device.h"
#include "vl53l1_platform.h"
#include "vl53l1_platform_user_data.h"
#include "API/Specifiers.h"

class LaserSensor_L1 {

  VL53L1_RangingMeasurementData_t _RangingMeasurementData_L1x;
  VL53L1_Dev_t MyDevice_L1x;
  int16_t _range;

 public:
  LaserSensor_L1 ( ) : _Global_Status_L1x ( VL53L1_ERROR_NONE ), Range_Status_L1x ( 0 ), _range ( 0 ) {}

  /**
   * @brief Initializes the LaserSensor_L1 device by setting up communication parameters,
   * waiting for the device to boot, and performing necessary initialization steps.
   * @return `true` if the device is successfully initialized, otherwise returns false.
   */
  bool init ( VL53L1_DistanceModes _DistanceMode );

  /**
   * Sets a new I2C address for the LaserSensor_L1 device.
   * The provided address is adjusted and applied to the sensor,
   * updating the internal device's I2C address accordingly.
   *
   * @param _address The new I2C address to set for the device.
   */
  void setAddress ( uint8_t address );

  /**
   * @brief Initiates the ranging process for the LaserSensor_L1 device and retrieves measurement data.
   *
   * This function starts the ranging process if the global status is error-free and a start flag is set.
   * It updates the ranging measurement data if no errors occur, and checks the range validity and limits.
   *
   * @return true if valid ranging data is acquired with a measurement less than 4500 mm; otherwise, false.
   */
  bool startRanging ( void );

  /**
   * @brief Retrieves the last measured range from the LaserSensor_L1 device.
   *
   * This function returns the stored range measurement obtained during the last ranging process.
   *
   * @return The last measured distance in millimeters as an int16_t value.
   */
  int16_t getLaserRange ( void );

 private:
  VL53L1_Error _Global_Status_L1x;
  uint8_t Range_Status_L1x;
};

extern LaserSensor_L1 XVision;

void ranging_init_L1 ( void );
void getRange_L1 ( void );
bool isTofDataNew_L1 ( void );
bool isOutofRange_L1 ( void );

extern VL53L1_Error Global_Status_L1;
extern VL53L1_RangingMeasurementData_t RangingMeasurementData_L1;

extern uint8_t Range_Status_L1;
extern uint16_t NewSensorRange_L1;
extern uint16_t debug_range_L1;
extern bool startRanging_L1;
extern bool isTofDataNewflag_L1;
extern bool useRangingSensor_L1;

#ifdef __cplusplus
}
#endif
