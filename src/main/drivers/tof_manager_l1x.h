/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                               #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\tof_manager_l1x.h                                  #
 #  Created Date: Fri, 16th Oct 2026                                           #
 #  Brief: Single owner of every VL53L1X on I2C1 ( X1Ranging ): one table     #
 #         entry per laser ( 4 side, EXTERNAL, DOWN ) with its own device,     #
 #         status and results. One laser's error never stops another.          #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 19th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-16	AJ	Created ( vl53l1x-xranging-api task 2 )                    #
 #  2026-10-19	AJ	Compiled in every build ( vl53l1x-xranging-api t15 )       #
*******************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"    // target.h first, so a target override of the L1X_* timing is always seen

#include "drivers/tof_manager.h"    // tofId_e, tofStatus_e, tofEntry_t, TOF_XSHUT_NONE, TOF_RANGE_INVALID, TOF_DOWN_ADDRESS

// TIMED-mode measurement period and budget, every laser ( the target may override both; ST: period >= budget + 4 ms ).
#ifndef L1X_SAMPLE_PERIOD_MS
#define L1X_SAMPLE_PERIOD_MS 50    // ms
#endif
#ifndef L1X_TIMING_BUDGET_US
#define L1X_TIMING_BUDGET_US 45000    // us, per measurement
#endif
#ifndef L1X_STALE_MS
#define L1X_STALE_MS ( 3 * L1X_SAMPLE_PERIOD_MS + 10 )    // ms, no new result for this long counts as out of range
#endif

#define TOF_L1X_BOOT_ADDRESS 0x29    // 7-bit, every VL53L1X after XSHUT goes high; side lasers use 0x2A + n

#ifdef __cplusplus
extern "C" {
#endif

// The bring-up step that failed, for diagnostics ( tofEntry_t.failedStep, 0 = none ), in call order.
typedef enum {
  TOF_L1X_STEP_NONE = 0,
  TOF_L1X_STEP_BUS_BUSY,             // not released: another chip already answered at 0x29
  TOF_L1X_STEP_BOOT_WAIT,            // FIRMWARE__SYSTEM_STATUS never reported booted ( or no answer )
  TOF_L1X_STEP_MODEL_ID,             // the chip is not a VL53L1X ( 0x010F / 0x0110 != 0xEA / 0xCC )
  TOF_L1X_STEP_SET_ADDRESS,          // the address write failed, or the chip did not answer at the new one
  TOF_L1X_STEP_DATA_INIT,
  TOF_L1X_STEP_STATIC_INIT,
  TOF_L1X_STEP_DISTANCE_MODE,
  TOF_L1X_STEP_TIMING_BUDGET,
  TOF_L1X_STEP_INTER_MEASUREMENT,
  TOF_L1X_STEP_TUNING_READ,          // TUNING_PROXY_MIN read-back ( the negative-range floor )
  TOF_L1X_STEP_START_RANGING         // TIMED ranging did not start
} tofL1xStep_e;

// ---- Table ----
void tofL1xReset ( void );                               // every entry back to NOT_ENABLED
void tofL1xRequest ( tofId_e id, int8_t xshutGpio );     // mark a laser wanted, with its XSHUT pin
const tofEntry_t *tofL1xEntry ( tofId_e id );            // read-only view, NULL for a bad id; lastError is a VL53L1_Error
uint32_t tofL1xLastSampleMs ( tofId_e id );              // ms, millis ( ) at the last new result ( 0 = none yet )

// True if a VL53L1X answers at addr7: model ID 0x010F = 0xEA and module type 0x0110 = 0xCC ( a VL53L0X does not
// pass ). The 16-bit index reaches a VL53L0X as index 0x01 plus one data byte, so do not probe a known VL53L0X.
bool tofL1xProbe ( uint8_t addr7 );

// ---- Bring-up ( one entry; the XSHUT pins and the order across entries are the caller's ) ----
// The chip answering at fromAddr7 is waited for ( boot ), checked ( model ID ), moved to toAddr7 ( skipped if
// equal ) and configured there: DataInit, StaticInit, L1X_DISTANCE_MODE ( Medium ), L1X_TIMING_BUDGET_US,
// L1X_SAMPLE_PERIOD_MS. The address moves before DataInit, because StartMeasurement rewrites the address
// register from the NVM copy DataInit reads. Nothing is started here: DOWN starts at its first
// tofL1xPollDown ( ) ( as getRange_L1 did ), the sides in tofL1xStartSides ( ).
// Sets status OK, or BOOT_FAILED with failedStep / lastError. Returns true on OK.
bool tofL1xBringUp ( tofId_e id, uint8_t fromAddr7, uint8_t toAddr7 );
// Start TIMED ranging on every side laser that came up. Called once, after the last bring-up, so no laser
// fires while another one is configured. A failure is BOOT_FAILED ( TOF_L1X_STEP_START_RANGING ).
void tofL1xStartSides ( void );
// Mark a laser BOOT_FAILED without touching the bus ( e.g. TOF_L1X_STEP_BUS_BUSY ).
void tofL1xMarkFailed ( tofId_e id, uint8_t step );

// ---- Polling ----
// DOWN: every 10 ms, check data-ready and read + clear a ready result ( the former getRange_L1 ( ) ).
void tofL1xPollDown ( void );
// Side lasers: at most one laser's I2C per call ( paced to TOF_L1X_SIDE_POLL_MS ), round-robin over the
// running ones; a result is read only when the chip has one ready. Readers use the cached entry.
// Call it right after tofL1xPollDown ( ) in the same loop: it skips a loop in which the DOWN poll used the bus,
// so one loop never carries both lasers' I2C.
void tofL1xPollSides ( void );

// ---- DOWN accessors for altitude hold ( one-for-one with the former *_L1 globals ) ----
bool tofL1xDownNew ( void );           // isTofDataNew_L1 ( )
void tofL1xDownClearNew ( void );      // isTofDataNewflag_L1 = false
bool tofL1xDownOutOfRange ( void );    // isOutofRange_L1 ( ): also while DOWN is not OK, or no result for L1X_STALE_MS
// The L1X has no driver filter: altitude hold's AltTofL1x::tofFiltMm ( ) AND tofRawMm ( ) both read tofL1xDownRangeMm ( ),
// as both read NewSensorRange_L1 before. tofL1xDownRawMm ( ) is for diagnostics only: it is the last result
// unchecked ( covered-window 0-10 mm, out-of-range values ) and must never feed the fusion.
uint16_t tofL1xDownRangeMm ( void );   // last valid range, mm ( NewSensorRange_L1; kept across invalid results )
uint16_t tofL1xDownRawMm ( void );     // diagnostics only: last result's range, mm, unchecked ( a negative range reads 0 )

#ifdef __cplusplus
}
#endif
