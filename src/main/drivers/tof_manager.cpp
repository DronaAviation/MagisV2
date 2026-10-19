/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                               #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\tof_manager.cpp                                    #
 #  Brief: Single owner of every VL53L0X on I2C1. Each table entry has its own #
 #         device struct, status and results ( no shared latch, no shared      #
 #         result globals ), so altitude hold only ever sees DOWN's samples.   #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 19th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-14	AJ	Created ( tof-single-pipeline task 2 )                     #
 #  2026-10-19	AJ	Device table shared with the other laser family            #
 #            	  	( vl53l1x-xranging-api t16 )                               #
*******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include "drivers/ranging_vl53l0x.h"
#include "drivers/tof_manager.h"
#include "drivers/tof_dev_area.h"    // tofDevArea.l0x: the device table, shared with the VL53L1X manager
#include "API/API-Utils.h"           // rangingFamily ( )
#include "API/Scheduler-Timer.h"

#define TOF_DOWN_PERIOD_MS 33       // DOWN start + read period ( getRange ( ) )
#define TOF_DOWN_MAX_MM    2000     // DOWN samples at or above this are out of range
#define TOF_DOWN_LPS       0.1f     // DOWN range IIR weight ( LASER_LPS )
#define TOF_SIDE_POLL_MS   10       // one side laser per 10 ms: 4 lasers -> each every ~40 ms ( 20 ms budget )

// DOWN keeps the former ranging_init ( ) configuration ( R6: altitude hold was tuned on it ).
static const vl53l0xConfig_t tofDownConfig = { VL53L0X_DEVICEMODE_SINGLE_RANGING, 33000, 18, 14, false };
// Side lasers keep the former LaserSensor::init ( ) configuration, plus the sample-ready interrupt for the paced poll.
static const vl53l0xConfig_t tofSideConfig = { VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, 20000, 18, 14, true };

// The device structs live in tofDevArea.l0x ( drivers/tof_dev_area.h ), shared with the VL53L1X manager: touched
// only by a bring-up in the VL53L0X family, and afterwards only for an entry whose status that bring-up set OK.
static tofEntry_t tofTable [ TOF_COUNT ];

static Interval downPoll;
static Interval sidePoll;
static float downFiltState     = 0.0f;    // IIR kept in float ( a uint16_t state sticks within 10 mm )
static uint16_t downFiltMm     = 0;
static bool downReseedPending  = true;

static inline bool tofIdValid ( tofId_e id ) {
  return ( unsigned ) id < ( unsigned ) TOF_COUNT;
}

void tofManagerReset ( void ) {
  memset ( tofTable, 0, sizeof ( tofTable ) );
  for ( uint8_t i = 0; i < ( uint8_t ) TOF_COUNT; i++ ) {
    tofTable [ i ].xshutGpio = TOF_XSHUT_NONE;
    tofTable [ i ].rangeMm   = TOF_RANGE_INVALID;
  }
  downFiltState     = 0.0f;
  downFiltMm        = 0;
  downReseedPending = true;
}

void tofManagerRequest ( tofId_e id, int8_t xshutGpio ) {
  if ( ! tofIdValid ( id ) ) return;
  tofTable [ id ].requested = true;
  tofTable [ id ].xshutGpio = xshutGpio;
}

const tofEntry_t *tofManagerEntry ( tofId_e id ) {
  return tofIdValid ( id ) ? &tofTable [ id ] : NULL;
}

bool tofManagerBringUp ( tofId_e id, uint8_t fromAddr7, uint8_t toAddr7 ) {
  if ( ! tofIdValid ( id ) ) return false;
  if ( rangingFamily ( ) != RANGING_VL53L0X ) return false;    // tofDevArea is the VL53L1X table in that family

  tofEntry_t *e     = &tofTable [ id ];
  VL53L0X_Dev_t *dev = &tofDevArea.l0x [ id ];
  uint8_t step      = VL53L0X_STEP_NONE;

  dev->I2cDevAddr      = fromAddr7;
  dev->comms_type      = 1;
  dev->comms_speed_khz = 400;

  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  if ( fromAddr7 != toAddr7 ) {
    status = vl53l0xSetAddress ( dev, toAddr7 );    // F2: checked, the host follows only on success
    if ( status != VL53L0X_ERROR_NONE ) step = VL53L0X_STEP_SET_ADDRESS;
  }
  if ( status == VL53L0X_ERROR_NONE ) {
    status = vl53l0xConfigure ( dev, toAddr7, ( id == TOF_DOWN ) ? &tofDownConfig : &tofSideConfig, &step );
  }

  e->outOfRange = false;
  e->newSample  = false;
  e->rawMm      = 0;
  e->rangeMm    = TOF_RANGE_INVALID;
  e->samples    = 0;
  if ( id == TOF_DOWN ) downReseedPending = true;

  if ( status != VL53L0X_ERROR_NONE ) {
    e->status     = TOF_STATUS_BOOT_FAILED;
    e->failedStep = step;
    e->lastError  = status;
    return false;
  }
  e->status     = TOF_STATUS_OK;
  e->address    = toAddr7;
  e->failedStep = VL53L0X_STEP_NONE;
  e->lastError  = VL53L0X_ERROR_NONE;
  return true;
}

void tofManagerStartSides ( void ) {
  for ( uint8_t i = ( uint8_t ) TOF_LEFT; i < ( uint8_t ) TOF_DOWN; i++ ) {
    tofEntry_t *e = &tofTable [ i ];
    if ( e->status != TOF_STATUS_OK ) continue;
    const VL53L0X_Error status = VL53L0X_StartMeasurement ( &tofDevArea.l0x [ i ] );    // continuous, back-to-back
    if ( status != VL53L0X_ERROR_NONE ) {
      e->status     = TOF_STATUS_BOOT_FAILED;
      e->failedStep = VL53L0X_STEP_START_RANGING;
      e->lastError  = status;
    }
  }
}

void tofManagerMarkFailed ( tofId_e id, uint8_t step ) {
  if ( ! tofIdValid ( id ) ) return;
  tofTable [ id ].status     = TOF_STATUS_BOOT_FAILED;
  tofTable [ id ].failedStep = step;
  tofTable [ id ].lastError  = VL53L0X_ERROR_NONE;
}

void tofManagerPollDown ( void ) {
  tofEntry_t *e = &tofTable [ TOF_DOWN ];
  if ( e->status != TOF_STATUS_OK ) return;
  if ( ! downPoll.set ( TOF_DOWN_PERIOD_MS, true ) ) return;

  VL53L0X_RangingMeasurementData_t data;
  VL53L0X_Error status = vl53l0xStartAndRead ( &tofDevArea.l0x [ TOF_DOWN ], &data );
  if ( status != VL53L0X_ERROR_NONE ) {
    e->status    = TOF_STATUS_I2C_ERROR;    // never retried in flight; altitude hold sees out of range
    e->lastError = status;
    e->rangeMm   = TOF_RANGE_INVALID;
    return;
  }

  e->newSample   = true;
  e->rangeStatus = data.RangeStatus;
  e->rawMm       = data.RangeMilliMeter;
  e->samples++;
  if ( data.RangeStatus == 0 && data.RangeMilliMeter < TOF_DOWN_MAX_MM ) {
    if ( downReseedPending ) {
      downReseedPending = false;
      downFiltState     = ( float ) data.RangeMilliMeter;
    } else {
      downFiltState = downFiltState * ( 1.0f - TOF_DOWN_LPS ) + ( float ) data.RangeMilliMeter * TOF_DOWN_LPS;
    }
    downFiltMm    = ( uint16_t ) ( downFiltState + 0.5f );
    e->outOfRange = false;
    e->rangeMm    = ( int16_t ) data.RangeMilliMeter;
  } else {
    e->outOfRange     = true;
    e->rangeMm        = TOF_RANGE_INVALID;
    downReseedPending = true;
  }
}

void tofManagerPollSides ( void ) {
  static uint8_t next = ( uint8_t ) TOF_LEFT;    // round-robin position over LEFT … EXTERNAL
  if ( ! sidePoll.set ( TOF_SIDE_POLL_MS, true ) ) return;

  // The next running side laser, at most one full turn.
  tofId_e id         = TOF_COUNT;
  for ( uint8_t k = 0; k < ( uint8_t ) TOF_DOWN; k++ ) {
    const uint8_t i = next;
    next            = ( uint8_t ) ( ( next + 1 ) % ( uint8_t ) TOF_DOWN );
    if ( tofTable [ i ].status == TOF_STATUS_OK ) {
      id = ( tofId_e ) i;
      break;
    }
  }
  if ( id == TOF_COUNT ) return;

  tofEntry_t *e = &tofTable [ id ];
  VL53L0X_RangingMeasurementData_t data;
  bool ready           = false;
  VL53L0X_Error status = vl53l0xReadIfReady ( &tofDevArea.l0x [ id ], &data, &ready );
  if ( status != VL53L0X_ERROR_NONE ) {
    e->status    = TOF_STATUS_I2C_ERROR;    // this laser only; never retried in flight
    e->lastError = status;
    e->rangeMm   = TOF_RANGE_INVALID;
    return;
  }
  if ( ! ready ) return;

  e->newSample   = true;
  e->rangeStatus = data.RangeStatus;
  e->rawMm       = data.RangeMilliMeter;
  e->samples++;
  e->outOfRange  = ( data.RangeStatus != 0 );
  e->rangeMm     = e->outOfRange ? ( int16_t ) TOF_RANGE_INVALID : ( int16_t ) data.RangeMilliMeter;
}

bool tofDownNew ( void ) {
  return tofTable [ TOF_DOWN ].newSample;
}

void tofDownClearNew ( void ) {
  tofTable [ TOF_DOWN ].newSample = false;
}

bool tofDownOutOfRange ( void ) {
  return tofTable [ TOF_DOWN ].status != TOF_STATUS_OK || tofTable [ TOF_DOWN ].outOfRange;
}

void tofDownReseed ( void ) {
  downReseedPending = true;
}

uint16_t tofDownFiltMm ( void ) {
  return downFiltMm;
}

uint16_t tofDownRawMm ( void ) {
  return tofTable [ TOF_DOWN ].rawMm;
}
