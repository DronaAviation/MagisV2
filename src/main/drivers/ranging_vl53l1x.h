/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2-3.0.0-beta-vl53l1x                                        #
 #  File: \src\main\drivers\ranging_vl53l1x.h                                  #
 #  Created Date: Sat, 8th Nov 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 11th Nov 2025                                          #
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
  bool init ( );
  void setAddress ( uint8_t address );
  bool startRanging ( );
  int16_t getLaserRange ( );

 private:
  VL53L1_Error _Global_Status_L1x;
  uint8_t Range_Status_L1x;
};

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
