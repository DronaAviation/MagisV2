/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\BMS.h                                                  #
 #  Created Date: Tue, 19th Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 29th Sep 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-09-29	AJ	Bms_Get: SoC, Warning_Level, Resistance                    #
*******************************************************************************/

#ifndef BMS_H
#define BMS_H

#include "stdint.h"

/**
 * @enum BMS_Option_e
 * @brief Enumerates the various options for the battery management system (BMS) monitoring.
 *
 * This enumeration defines the different parameters that can be monitored or evaluated
 * within the battery management system. It includes options to track voltage, current,
 * consumed capacity, remaining capacity, total battery capacity, the charge at plug-in, the state of
 * charge, the low-battery level and the pack resistance. Units and validity: docs/API/BMS_API_WIKI.md.
 */
typedef enum BMS_Option {
  Voltage,              // Battery voltage in mV ( averaged ).
  Current,              // Battery current in mA ( averaged ).
  mAh_Consumed,         // The milliampere-hours consumed from the battery.
  mAh_Remain,           // The milliampere-hours remaining in the battery.
  Battery_Capicity,     // The total capacity of the battery in milliampere-hours.
  Estimated_Capacity,   // Charge in the pack at plug-in in milliampere-hours, from its resting voltage.
  SoC,                  // State of charge in percent ( 0 .. 100 ): remaining / capacity, rounded ( the app truncates ).
  Warning_Level,        // Low-battery level: 0 = OK, 1 = low battery, 2 = critical ( latched until power-off ).
  Resistance            // Resistance at the battery sensor ( pack, wiring, shunt ) in milliohms, 0 until measured.
} BMS_Option_e;

/**
 * @brief Retrieves various battery management system (BMS) parameters.
 *
 * This function takes a BMS option and returns the corresponding value
 * such as voltage, current, or capacity metrics from the battery management system.
 *
 * `_bms_option` An enumerator of type BMS_Option_e indicating which BMS parameter to retrieve.
 *        Possible values are:
 *        @param Voltage: To get the battery voltage in mV ( averaged ).
 *        @param Current: To get the battery current in mA ( averaged ).
 *        @param mAh_Consumed: To get the milliamp hours consumed.
 *        @param mAh_Remain: To get the remaining milliamp hours.
 *        @param Battery_Capicity: To get the total battery capacity in milliamp hours.
 *        @param Estimated_Capacity: To get the charge in the pack at plug-in ( mAh ), from its resting voltage.
 *        @param SoC: To get the state of charge in percent ( 0 .. 100 ).
 *        @param Warning_Level: To get the low-battery level ( 0 OK, 1 low battery, 2 critical ).
 *        @param Resistance: To get the pack resistance in milliohms ( 0 until measured in flight ).
 *
 * @return `uint16_t` The value of the requested BMS parameter. Returns 0 if an undefined option is passed.
 */
uint16_t Bms_Get ( BMS_Option_e _bms_option );

#endif
