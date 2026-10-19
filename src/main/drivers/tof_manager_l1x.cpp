/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                               #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\tof_manager_l1x.cpp                                #
 #  Created Date: Fri, 16th Oct 2026                                           #
 #  Brief: Single owner of every VL53L1X on I2C1 ( X1Ranging ). Each table    #
 #         entry has its own device struct, status and results, so altitude   #
 #         hold only ever sees DOWN's samples. DOWN keeps the former           #
 #         ranging_init_L1 / getRange_L1 configuration, timing and filtering.  #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 19th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-16	AJ	Created ( vl53l1x-xranging-api task 2 )                    #
 #  2026-10-19	AJ	Compiled in every build ( vl53l1x-xranging-api t15 )       #
 #  2026-10-19	AJ	Device table shared with the other laser family            #
 #            	  	( vl53l1x-xranging-api t16 )                               #
*******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "vl53l1_platform.h"
#include "vl53l1_api.h"
#include "vl53l1_def.h"
#include "vl53l1_core.h"                 // VL53L1_clear_interrupt
#include "vl53l1_preset_setup.h"         // VL53L1_TUNING_PROXY_MIN
#include "vl53l1_register_map.h"         // result, identification and firmware-status register addresses
#include "vl53l1_register_settings.h"    // VL53L1_RANGE_STATUS__RANGE_STATUS_MASK

#include "drivers/bus_i2c.h"
#include "drivers/system.h"
#include "drivers/tof_manager_l1x.h"
#include "drivers/tof_dev_area.h"    // tofDevArea.l1x: the device table, shared with the VL53L0X manager
#include "API/API-Utils.h"           // rangingFamily ( )
#include "API/Scheduler-Timer.h"

#define TOF_L1X_DOWN_POLL_MS 10    // ms, DOWN data-ready poll period ( getRange_L1's RANGE_POLL )
#define TOF_L1X_SIDE_POLL_MS 10    // ms, one side laser per 10 ms: 5 side slots -> each every <= 50 ms ( one period )
#define TOF_L1X_BOOT_TRIES   10    // FIRMWARE__SYSTEM_STATUS reads, 1 ms apart ( the chip boots in <= 1.2 ms )
#define TOF_L1X_MODEL_ID     0xEACC    // IDENTIFICATION__MODEL_ID ( 0x010F ) : MODULE_TYPE ( 0x0110 )

// mm, slant range. A covered or blocked window reads 0-10 mm with status 0 ( log-1, log-4 ), so it is out of range;
// the craft sitting on the floor reads 25-28 mm and stays valid ( landing and touchdown see the laser as on the L0X ).
#define L1X_MIN_VALID_MM 15
#define L1X_MAX_VALID_MM 4500    // mm, at or above this a result is out of range

// Distance mode: Medium unless the target overrides it ( Long uses longer VCSEL periods, 15 / 13, with the same
// sigma / signal limits; SetDistanceMode keeps the budget and period, vl53l1_api.c:1068-1094 ).
#ifndef L1X_DISTANCE_MODE
#define L1X_DISTANCE_MODE VL53L1_DISTANCEMODE_MEDIUM
#endif

// ST refuses to start TIMED ranging unless period >= budget + 4 ms ( vl53l1_api.c:1772-1781 ), and the laser would
// stay failed for the whole flight: catch a bad target override at compile time ( +5 covers the budget read-back rounding ).
static_assert ( L1X_SAMPLE_PERIOD_MS >= L1X_TIMING_BUDGET_US / 1000 + 5, "VL53L1X: period must be >= budget + 4 ms" );
// Nominal: each re-arm is >= TOF_L1X_SIDE_POLL_MS plus loop jitter ( and a loop shared with the DOWN poll is
// skipped ), so a full turn over 5 fitted sides is ~55 ms against the 51-53 ms measured period. A side laser can
// miss a result now and then; that is well inside the L1X_STALE_MS stale time.
static_assert ( ( int ) TOF_DOWN * TOF_L1X_SIDE_POLL_MS <= L1X_SAMPLE_PERIOD_MS, "every side slot must be polled once per period" );

// Per-result read: RESULT__RANGE_STATUS ( 0x0089 ) up to and including
// RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 ( 0x0098-0x0099 ), one 17-byte I2C read in place of
// VL53L1_GetRangingMeasurementData's 133-byte system + core + debug results read ( the former getRange_L1 read ).
#define L1X_RES_FIRST_REG    VL53L1_RESULT__RANGE_STATUS
#define L1X_RES_LEN          ( VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 + 2 - L1X_RES_FIRST_REG )
#define L1X_RES_RANGE_STATUS ( VL53L1_RESULT__RANGE_STATUS - L1X_RES_FIRST_REG )
#define L1X_RES_STREAM_COUNT ( VL53L1_RESULT__STREAM_COUNT - L1X_RES_FIRST_REG )
#define L1X_RES_RANGE        ( VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 - L1X_RES_FIRST_REG )

static_assert ( L1X_RES_LEN == 17, "VL53L1X result block is 0x0089..0x0099" );

// The device structs live in tofDevArea.l1x ( drivers/tof_dev_area.h ), shared with the VL53L0X manager: touched
// only by a bring-up in the VL53L1X family, and afterwards only for an entry whose status that bring-up set OK.
static tofEntry_t tofTable [ TOF_COUNT ];
static uint32_t tofLastMs [ TOF_COUNT ];         // ms, millis ( ) at the last new result ( 0 = none yet )
static int32_t tofProxyMinMm [ TOF_COUNT ];     // mm, ST TUNING_PROXY_MIN, read from the API at bring-up
static bool downUsedBus = false;                 // the DOWN poll did I2C in this loop ( consumed by tofL1xPollSides )

static Interval downPoll;
static Interval sidePoll;
static bool downStartPending = true;    // DOWN starts TIMED ranging at its first poll ( as getRange_L1 did )
static bool sidesStarted     = false;
static uint16_t downValidMm  = 0;       // last valid DOWN range, mm ( the former NewSensorRange_L1 )

static inline bool tofIdValid ( tofId_e id ) {
  return ( unsigned ) id < ( unsigned ) TOF_COUNT;
}

static uint16_t l1xBe16 ( const uint8_t *p ) {
  return ( uint16_t ) ( ( ( uint32_t ) p [ 0 ] << 8 ) | ( uint32_t ) p [ 1 ] );
}

// RangeStatus from RESULT__RANGE_STATUS exactly as the full API derives it in TIMED mode: the device-error remap of
// VL53L1_copy_sys_and_core_results_to_range_results + SetSimpleData, then ConvertStatusLite. The sigma and signal
// limits are checked on the device ( written into the timing config at StartMeasurement ) and come back here as
// SIGMATHRESHOLDCHECK / MSRCNOTARGET. Their values are the preset tuning defaults, sigma 90 mm / signal 1.5 MCPS
// in both Medium ( tp_lite_med_*, vl53l1_api_preset_modes.c:545-553 ) and Long ( tp_lite_long_*, :781-784 ): the
// preset overwrites DataInit's 18 mm / 0.25 MCPS.
static uint8_t l1xRangeStatus ( uint8_t deviceRangeStatus, uint8_t streamCount ) {
  switch ( deviceRangeStatus & VL53L1_RANGE_STATUS__RANGE_STATUS_MASK ) {
    case VL53L1_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
    case VL53L1_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
    case VL53L1_DEVICEERROR_NOVHVVALUEFOUND:
    case VL53L1_DEVICEERROR_MULTCLIPFAIL:
      return VL53L1_RANGESTATUS_HARDWARE_FAIL;
    case VL53L1_DEVICEERROR_USERROICLIP:
      return VL53L1_RANGESTATUS_MIN_RANGE_FAIL;
    case VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY:
      return VL53L1_RANGESTATUS_SYNCRONISATION_INT;
    case VL53L1_DEVICEERROR_RANGECOMPLETE:    // the first range after start ( stream count 0 ) has no wrap check
      return ( streamCount == 0 ) ? VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL : VL53L1_RANGESTATUS_RANGE_VALID;
    case VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK:
      return VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
    case VL53L1_DEVICEERROR_RANGEPHASECHECK:
      return VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;
    case VL53L1_DEVICEERROR_MSRCNOTARGET:
      return VL53L1_RANGESTATUS_SIGNAL_FAIL;
    case VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK:
      return VL53L1_RANGESTATUS_SIGMA_FAIL;
    case VL53L1_DEVICEERROR_PHASECONSISTENCY:
      return VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
    case VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD:
      return VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL;
    case VL53L1_DEVICEERROR_MINCLIP:
      return VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED;
    default:
      return VL53L1_RANGESTATUS_NONE;
  }
}

// RangeMilliMeter and RangeStatus of one result block, with the former l1xDecodeResult ( )'s arithmetic ( the
// API's VL53L1_copy_sys_and_core_results_to_range_results ): the crosstalk-corrected range ( offset and crosstalk
// are corrected on the device ) times this chip's software gain factor, then the negative-range floor. The other
// fields of VL53L1_RangingMeasurementData_t had no reader and are not decoded.
static void l1xDecodeResult ( tofId_e id, const uint8_t *res, int16_t *rangeMm, uint8_t *rangeStatus ) {
  const uint8_t streamCount = res [ L1X_RES_STREAM_COUNT ];

  int32_t mm = ( int32_t ) l1xBe16 ( &res [ L1X_RES_RANGE ] );
  mm *= ( int32_t ) tofDevArea.l1x [ id ].Data.LLData.gain_cal.standard_ranging_gain_factor;
  mm += 0x0400;
  mm /= 0x0800;

  int16_t range  = ( int16_t ) mm;
  uint8_t status = l1xRangeStatus ( res [ L1X_RES_RANGE_STATUS ], streamCount );

  if ( ( status == VL53L1_RANGESTATUS_RANGE_VALID ) && ( range < 0 ) ) {
    if ( range < tofProxyMinMm [ id ] )
      status = VL53L1_RANGESTATUS_RANGE_INVALID;
    else
      range = 0;
  }
  *rangeMm     = range;
  *rangeStatus = status;
}

// A bus error after a good bring-up: this laser only, never retried in flight.
static void tofL1xBusError ( tofEntry_t *e, VL53L1_Error status ) {
  e->status    = TOF_STATUS_I2C_ERROR;
  e->lastError = status;
  e->rangeMm   = TOF_RANGE_INVALID;
}

// One ready result: read the block, release it ( SYSTEM__INTERRUPT_CLEAR, so data-ready drops until the next
// measurement completes ), decode and cache it. Short path instead of VL53L1_GetRangingMeasurementData +
// VL53L1_ClearInterruptAndStartMeasurement ( 5.4 ms ): in the TIMED preset GPH is disabled and the per-range
// GENERAL_ONWARDS rewrite only re-sends the unchanged configuration. The LL driver's stream-count state is not
// advanced; it is only checked in back-to-back mode.
static void tofL1xReadResult ( tofId_e id ) {
  tofEntry_t *e     = &tofTable [ id ];
  VL53L1_Dev_t *dev = &tofDevArea.l1x [ id ];

  uint8_t result [ L1X_RES_LEN ];
  VL53L1_Error status = VL53L1_ReadMulti ( dev, L1X_RES_FIRST_REG, result, L1X_RES_LEN );
  if ( status != VL53L1_ERROR_NONE ) {
    tofL1xBusError ( e, status );
    return;
  }
  status = VL53L1_clear_interrupt ( dev );

  int16_t rangeMm     = 0;    // mm, may be negative close in
  uint8_t rangeStatus = 0;
  l1xDecodeResult ( id, result, &rangeMm, &rangeStatus );

  // One genuine new result ( a failed clear then stops the laser, which the readers see as out of range ).
  const bool valid = ( rangeStatus == 0 ) && ( rangeMm >= L1X_MIN_VALID_MM ) && ( rangeMm < L1X_MAX_VALID_MM );
  e->newSample     = true;
  e->rangeStatus   = rangeStatus;
  e->rawMm         = ( rangeMm < 0 ) ? ( uint16_t ) 0 : ( uint16_t ) rangeMm;
  e->samples++;
  tofLastMs [ id ] = millis ( );
  e->outOfRange    = ! valid;
  e->rangeMm       = valid ? rangeMm : ( int16_t ) TOF_RANGE_INVALID;
  if ( valid && id == TOF_DOWN ) downValidMm = ( uint16_t ) rangeMm;    // positive here; kept across invalid results

  if ( status != VL53L1_ERROR_NONE ) tofL1xBusError ( e, status );
}

// No new result for L1X_STALE_MS ( a stall that raises no error ): the cached range is out of range.
static void tofL1xCheckStale ( tofId_e id, uint32_t nowMs ) {
  if ( ( nowMs - tofLastMs [ id ] ) > L1X_STALE_MS ) {
    tofTable [ id ].outOfRange = true;
    tofTable [ id ].rangeMm    = TOF_RANGE_INVALID;
  }
}

// FIRMWARE__SYSTEM_STATUS bit 0, bounded: TOF_L1X_BOOT_TRIES reads 1 ms apart. This platform's
// VL53L1_WaitDeviceBooted ( ) does not wait ( VL53L1_WaitUs / WaitValueMaskEx are stubs ), so the poll is here.
static VL53L1_Error tofL1xWaitBooted ( VL53L1_Dev_t *dev ) {
  VL53L1_Error status = VL53L1_ERROR_TIME_OUT;
  for ( uint8_t k = 0; k < TOF_L1X_BOOT_TRIES; k++ ) {
    uint8_t fwStatus = 0;
    status           = VL53L1_RdByte ( dev, VL53L1_FIRMWARE__SYSTEM_STATUS, &fwStatus );
    if ( status == VL53L1_ERROR_NONE ) {
      if ( fwStatus & 0x01 ) return VL53L1_ERROR_NONE;
      status = VL53L1_ERROR_TIME_OUT;
    }
    delay ( 1 );
  }
  return status;
}

void tofL1xReset ( void ) {
  memset ( tofTable, 0, sizeof ( tofTable ) );
  for ( uint8_t i = 0; i < ( uint8_t ) TOF_COUNT; i++ ) {
    tofTable [ i ].xshutGpio = TOF_XSHUT_NONE;
    tofTable [ i ].rangeMm   = TOF_RANGE_INVALID;
    tofLastMs [ i ]          = 0;
  }
  downStartPending = true;
  sidesStarted     = false;
  downValidMm      = 0;
}

void tofL1xRequest ( tofId_e id, int8_t xshutGpio ) {
  if ( ! tofIdValid ( id ) ) return;
  tofTable [ id ].requested = true;
  tofTable [ id ].xshutGpio = xshutGpio;
}

const tofEntry_t *tofL1xEntry ( tofId_e id ) {
  return tofIdValid ( id ) ? &tofTable [ id ] : NULL;
}

uint32_t tofL1xLastSampleMs ( tofId_e id ) {
  return tofIdValid ( id ) ? tofLastMs [ id ] : 0;
}

bool tofL1xProbe ( uint8_t addr7 ) {
  uint8_t id [ 2 ] = { 0, 0 };
  if ( i2cRead_v2 ( addr7, VL53L1_IDENTIFICATION__MODEL_ID, 2, id ) == 0 ) {
    return false;
  }
  return l1xBe16 ( id ) == TOF_L1X_MODEL_ID;
}

bool tofL1xBringUp ( tofId_e id, uint8_t fromAddr7, uint8_t toAddr7 ) {
  if ( ! tofIdValid ( id ) ) return false;
  if ( rangingFamily ( ) != RANGING_VL53L1X ) return false;    // tofDevArea is the VL53L0X table in that family

  tofEntry_t *e     = &tofTable [ id ];
  VL53L1_Dev_t *dev = &tofDevArea.l1x [ id ];
  uint8_t step      = TOF_L1X_STEP_NONE;

  dev->I2cDevAddr      = fromAddr7;
  dev->comms_type      = 1;
  dev->comms_speed_khz = 400;

  VL53L1_Error status = tofL1xWaitBooted ( dev );
  if ( status == VL53L1_ERROR_NONE ) {
    status = VL53L1_WaitDeviceBooted ( dev );    // as ranging_init_L1: sets the LL driver state ( no wait here )
  }
  if ( status != VL53L1_ERROR_NONE ) step = TOF_L1X_STEP_BOOT_WAIT;

  if ( status == VL53L1_ERROR_NONE ) {
    uint16_t modelId = 0;
    status           = VL53L1_RdWord ( dev, VL53L1_IDENTIFICATION__MODEL_ID, &modelId );
    if ( status == VL53L1_ERROR_NONE && modelId != TOF_L1X_MODEL_ID ) status = VL53L1_ERROR_NOT_SUPPORTED;
    if ( status != VL53L1_ERROR_NONE ) step = TOF_L1X_STEP_MODEL_ID;
  }

  // The address moves before DataInit: StartMeasurement writes the address register back from the static NVM
  // copy DataInit reads ( DEVICECONFIGLEVEL_FULL from 0x0001 ), and SetDeviceAddress does not update that copy.
  if ( status == VL53L1_ERROR_NONE && fromAddr7 != toAddr7 ) {
    status = VL53L1_SetDeviceAddress ( dev, ( uint8_t ) ( toAddr7 * 2 ) );    // 8-bit form ( vl53l1_api.c SetDeviceAddress writes / 2 )
    if ( status == VL53L1_ERROR_NONE ) {
      dev->I2cDevAddr = toAddr7;    // the host follows only on success
      if ( ! tofL1xProbe ( toAddr7 ) ) status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    if ( status != VL53L1_ERROR_NONE ) step = TOF_L1X_STEP_SET_ADDRESS;
  }

  // The former ranging_init_L1 ( ) configuration, in its order, for every laser.
  if ( status == VL53L1_ERROR_NONE ) {
    status = VL53L1_DataInit ( dev );
    if ( status != VL53L1_ERROR_NONE ) step = TOF_L1X_STEP_DATA_INIT;
  }
  if ( status == VL53L1_ERROR_NONE ) {
    status = VL53L1_StaticInit ( dev );
    if ( status != VL53L1_ERROR_NONE ) step = TOF_L1X_STEP_STATIC_INIT;
  }
  if ( status == VL53L1_ERROR_NONE ) {
    status = VL53L1_SetDistanceMode ( dev, L1X_DISTANCE_MODE );    // Medium unless the target overrides it
    if ( status != VL53L1_ERROR_NONE ) step = TOF_L1X_STEP_DISTANCE_MODE;
  }
  if ( status == VL53L1_ERROR_NONE ) {
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds ( dev, L1X_TIMING_BUDGET_US );
    if ( status != VL53L1_ERROR_NONE ) step = TOF_L1X_STEP_TIMING_BUDGET;
  }
  if ( status == VL53L1_ERROR_NONE ) {
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds ( dev, L1X_SAMPLE_PERIOD_MS );    // TIMED-mode period
    if ( status != VL53L1_ERROR_NONE ) step = TOF_L1X_STEP_INTER_MEASUREMENT;
  }
  if ( status == VL53L1_ERROR_NONE ) {    // negative-range floor used by l1xDecodeResult ( )
    status = VL53L1_GetTuningParameter ( dev, VL53L1_TUNING_PROXY_MIN, &tofProxyMinMm [ id ] );
    if ( status != VL53L1_ERROR_NONE ) step = TOF_L1X_STEP_TUNING_READ;
  }

  e->outOfRange    = false;
  e->newSample     = false;
  e->rawMm         = 0;
  e->rangeMm       = TOF_RANGE_INVALID;
  e->samples       = 0;
  tofLastMs [ id ] = 0;
  if ( id == TOF_DOWN ) {
    downStartPending = true;
    downValidMm      = 0;
  }

  if ( status != VL53L1_ERROR_NONE ) {
    e->status     = TOF_STATUS_BOOT_FAILED;
    e->failedStep = step;
    e->lastError  = status;
    return false;
  }
  e->status     = TOF_STATUS_OK;
  e->address    = toAddr7;
  e->failedStep = TOF_L1X_STEP_NONE;
  e->lastError  = VL53L1_ERROR_NONE;
  return true;
}

void tofL1xStartSides ( void ) {
  for ( uint8_t i = ( uint8_t ) TOF_LEFT; i < ( uint8_t ) TOF_DOWN; i++ ) {
    tofEntry_t *e = &tofTable [ i ];
    if ( e->status != TOF_STATUS_OK ) continue;
    const VL53L1_Error status = VL53L1_StartMeasurement ( &tofDevArea.l1x [ i ] );    // TIMED, one result per period
    if ( status != VL53L1_ERROR_NONE ) {
      e->status     = TOF_STATUS_BOOT_FAILED;
      e->failedStep = TOF_L1X_STEP_START_RANGING;
      e->lastError  = status;
    }
    tofLastMs [ i ] = millis ( );    // the stale time counts from the start
  }
  sidesStarted = true;
}

void tofL1xMarkFailed ( tofId_e id, uint8_t step ) {
  if ( ! tofIdValid ( id ) ) return;
  tofTable [ id ].status     = TOF_STATUS_BOOT_FAILED;
  tofTable [ id ].failedStep = step;
  tofTable [ id ].lastError  = VL53L1_ERROR_NONE;
}

void tofL1xPollDown ( void ) {
  tofEntry_t *e = &tofTable [ TOF_DOWN ];
  if ( e->status != TOF_STATUS_OK ) return;
  if ( ! downPoll.set ( TOF_L1X_DOWN_POLL_MS, true ) ) return;    // check for new data every 10 ms
  downUsedBus = true;                                               // tofL1xPollSides ( ) skips this loop

  VL53L1_Dev_t *dev = &tofDevArea.l1x [ TOF_DOWN ];
  if ( downStartPending ) {
    downStartPending          = false;
    const VL53L1_Error status = VL53L1_StartMeasurement ( dev );
    if ( status != VL53L1_ERROR_NONE ) {
      e->status     = TOF_STATUS_BOOT_FAILED;
      e->failedStep = TOF_L1X_STEP_START_RANGING;
      e->lastError  = status;
      e->rangeMm    = TOF_RANGE_INVALID;
      return;
    }
  }

  uint8_t ready             = 0;
  const VL53L1_Error status = VL53L1_GetMeasurementDataReady ( dev, &ready );
  if ( status != VL53L1_ERROR_NONE ) {
    tofL1xBusError ( e, status );
    return;
  }
  if ( ready ) tofL1xReadResult ( TOF_DOWN );    // re-query data-ready on the next poll, never re-read a result
  tofL1xCheckStale ( TOF_DOWN, millis ( ) );
}

void tofL1xPollSides ( void ) {
  static uint8_t next = ( uint8_t ) TOF_LEFT;    // round-robin position over LEFT … EXTERNAL
  if ( ! sidesStarted ) return;
  // Never in the same loop as a DOWN poll: checked before the interval, so a due side poll runs on the next loop
  // and the two polls stay one loop apart from then on.
  if ( downUsedBus ) {
    downUsedBus = false;
    return;
  }
  if ( ! sidePoll.set ( TOF_L1X_SIDE_POLL_MS, true ) ) return;

  // The next running side laser, at most one full turn.
  tofId_e id = TOF_COUNT;
  for ( uint8_t k = 0; k < ( uint8_t ) TOF_DOWN; k++ ) {
    const uint8_t i = next;
    next            = ( uint8_t ) ( ( next + 1 ) % ( uint8_t ) TOF_DOWN );
    if ( tofTable [ i ].status == TOF_STATUS_OK ) {
      id = ( tofId_e ) i;
      break;
    }
  }
  if ( id == TOF_COUNT ) return;

  uint8_t ready             = 0;
  const VL53L1_Error status = VL53L1_GetMeasurementDataReady ( &tofDevArea.l1x [ id ], &ready );
  if ( status != VL53L1_ERROR_NONE ) {
    tofL1xBusError ( &tofTable [ id ], status );
  } else if ( ready ) {
    tofL1xReadResult ( id );
  }

  // Stale check on every running side laser ( no bus traffic ).
  const uint32_t nowMs = millis ( );
  for ( uint8_t i = ( uint8_t ) TOF_LEFT; i < ( uint8_t ) TOF_DOWN; i++ ) {
    if ( tofTable [ i ].status == TOF_STATUS_OK ) tofL1xCheckStale ( ( tofId_e ) i, nowMs );
  }
}

bool tofL1xDownNew ( void ) {
  return tofTable [ TOF_DOWN ].newSample;
}

void tofL1xDownClearNew ( void ) {
  tofTable [ TOF_DOWN ].newSample = false;
}

bool tofL1xDownOutOfRange ( void ) {
  // Also when DOWN has stopped ( failed bring-up, start or bus error ), or when no new result has arrived for
  // L1X_STALE_MS ( a stall that raises no error ): the consumer then falls back to the baro.
  return tofTable [ TOF_DOWN ].status != TOF_STATUS_OK || tofTable [ TOF_DOWN ].outOfRange
         || ( ( millis ( ) - tofLastMs [ TOF_DOWN ] ) > L1X_STALE_MS );
}

uint16_t tofL1xDownRangeMm ( void ) {
  return downValidMm;
}

uint16_t tofL1xDownRawMm ( void ) {
  return tofTable [ TOF_DOWN ].rawMm;
}
