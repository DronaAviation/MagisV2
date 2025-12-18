/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\BMS.cpp                                            #
 #  Created Date: Tue, 19th Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Fri, 14th Nov 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include "API/BMS.h"

#include "platform.h"
#include "sensors/battery.h"

uint16_t Bms_Get ( BMS_Option_e _bms_option ) {
  switch ( _bms_option ) {
    case Voltage:
      // Return the current battery voltage in mV
      return vbat*100;
    case Current:
      // Return the current amperage
      return amperage;
    case mAh_Consumed:
      // Return the milliamp hours consumed
      return mAhDrawn;
    case mAh_Remain:
      // Return the remaining milliamp hours
      return mAhRemain;
    case Battery_Capicity:
      // Return the total battery capacity in milliamp hours
      return battery_capacity_mAh;
    case Estimated_Capacity:
      // Return the estimated capacity of the battery
      return EstBatteryCapacity;
    default:
      // Return 0 for any undefined BMS option
      return 0;
  }
}

void Bms_Using_New_Battery_Capicity ( uint16_t _new_cap_mAh ) {
  // Assign the new capacity value to the battery capacity in mAh
  battery_capacity_mAh = _new_cap_mAh;
}
