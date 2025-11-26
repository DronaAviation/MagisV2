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
 #  Last Modified: Fri, 14th Nov 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
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
 * consumed capacity, remaining capacity, total battery capacity, and estimated capacity.
 */
typedef enum BMS_Option {
  Voltage,              // The voltage of the battery.
  Current,              // The current flowing through the battery.
  mAh_Consumed,         // The milliampere-hours consumed from the battery.
  mAh_Remain,           // The milliampere-hours remaining in the battery.
  Battery_Capicity,     // The total capacity of the battery in milliampere-hours.
  Estimated_Capacity    // The estimated capacity of the battery in milliampere-hours.
} BMS_Option_e;

/**
 * @brief Retrieves various battery management system (BMS) parameters.
 *
 * This function takes a BMS option and returns the corresponding value
 * such as voltage, current, or capacity metrics from the battery management system.
 *
 * `_bms_option` An enumerator of type BMS_Option_e indicating which BMS parameter to retrieve.
 *        Possible values are:
 *        @param Voltage: To get the current battery voltage.
 *        @param Current: To get the current amperage.
 *        @param mAh_Consumed: To get the milliamp hours consumed.
 *        @param mAh_Remain: To get the remaining milliamp hours.
 *        @param Battery_Capicity: To get the total battery capacity in milliamp hours.
 *        @param Estimated_Capacity: To get the estimated capacity of the battery.
 *
 * @return `uint16_t` The value of the requested BMS parameter. Returns 0 if an undefined option is passed.
 */
uint16_t Bms_Get ( BMS_Option_e _bms_option );

/**
 * @brief Updates the battery management system's capacity with a new value.
 *
 * This function assigns the provided new capacity, given in milliampere-hours (mAh),
 * to the existing battery capacity variable, effectively updating it to reflect
 * the new capacity.
 *
 * @param _new_cap_mAh The new battery capacity in milliampere-hours (mAh).
 */
void Bms_Using_New_Battery_Capicity ( uint16_t _new_cap_mAh );

#endif
