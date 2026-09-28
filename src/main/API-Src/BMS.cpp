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
 #  Last Modified: Tue, 29th Sep 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-09-29	AJ	Voltage in mV, no gain; SoC / level / R                    #
*******************************************************************************/
#include "API/BMS.h"

#include "platform.h"
#include "sensors/battery.h"

uint16_t Bms_Get ( BMS_Option_e _bms_option ) {
  switch ( _bms_option ) {
    case Voltage:
      // Battery voltage in mV ( 50-sample average )
      return vBat_mV;
    case Current:
      // Battery current in mA ( 50-sample average, no gain )
      return mAmpRaw;
    case mAh_Consumed:
      // Return the milliamp hours consumed
      return mAhDrawn;
    case mAh_Remain:
      // Return the remaining milliamp hours
      return mAhRemain;
    case Battery_Capicity:
      // Return the total battery capacity in milliamp hours
      return batteryCapacity_mAh;
    case Estimated_Capacity:
      // Charge in the pack at plug-in ( mAh ), from the resting voltage on the LiPo curve
      return EstBatteryCapacity;
    case SoC:
      // State of charge in percent ( 0 .. 100 ), rounded
      return static_cast< uint16_t > ( soc_Fused + 0.5f );
    case Warning_Level:
      // 0 = OK, 1 = low battery, 2 = critical
      return BatteryWarningMode;
    case Resistance:
      // Pack resistance in milliohms, 0 until measured this power-up
      return batteryResistance_mOhm;
    default:
      // Return 0 for any undefined BMS option
      return 0;
  }
}