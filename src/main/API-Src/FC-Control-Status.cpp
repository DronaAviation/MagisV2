/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\FC-Control-Status.cpp                              #
 #  Created Date: Sat, 23rd Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 5th Nov 2026                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-11-05	AJ	Status mapped by name to flightstatus_e ( API 1.8.4 )      #
*******************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include "common/maths.h"
#include "common/utils.h"

#include "config/runtime_config.h"

#include "API/FC-Control.h"

// Internal status flag ( FlightStatus_e, a bit of flightIndicatorFlag; the bit order is the priority ) and the public
// status of the same meaning. Mapped by name: the two enums are numbered differently at 2-4 ( internal Crash 2,
// Low_battery 3, LowBattery_inFlight 4; public FS_LOW_BATTERY 2, FS_INFLIGHT_LOW_BATTERY 3, FS_CRASHED 4 ), so a cast
// reported a crash as FS_LOW_BATTERY. MSP_FLIGHT_STATUS ( serial_msp.cpp ) maps by name as well.
typedef struct {
  FlightStatus_e internal;
  flightstatus_e api;
} flightStatusMap_t;

static const flightStatusMap_t flightStatusMap [ ] = {
  { Accel_Gyro_Calibration, FS_ACCEL_GYRO_CALIBRATION },
  { Mag_Calibration, FS_MAG_CALIBARATION },
  { Crash, FS_CRASHED },
  { Low_battery, FS_LOW_BATTERY },
  { LowBattery_inFlight, FS_INFLIGHT_LOW_BATTERY },
  { Signal_loss, FS_SIGNAL_LOSS },
  { Not_ok_to_arm, FS_NOT_OK_TO_ARM },
  { Ok_to_arm, FS_OK_TO_ARM },
  { Armed, FS_ARMED },
};

flightstatus_e FlightStatus_Get ( void ) {
  const uint16_t bit = leastSignificantBit ( flightIndicatorFlag );    // highest-priority flag set

  for ( size_t i = 0; i < ARRAYLEN ( flightStatusMap ); i++ ) {
    if ( ( uint16_t ) flightStatusMap [ i ].internal == bit ) {
      return flightStatusMap [ i ].api;
    }
  }

  // No flag set ( leastSignificantBit returns 0xFFFF; one read right after a disarm with the ARM switch still on ):
  // the raw value, as before the map. Outside the enum's range, which GCC keeps as is ( no -fstrict-enums )
  return ( flightstatus_e ) bit;
}

bool FlightStatus_Check ( flightstatus_e status ) {
  for ( size_t i = 0; i < ARRAYLEN ( flightStatusMap ); i++ ) {
    if ( flightStatusMap [ i ].api == status ) {
      return status_FSI ( flightStatusMap [ i ].internal );
    }
  }

  return false;    // not a flightstatus_e value
}
