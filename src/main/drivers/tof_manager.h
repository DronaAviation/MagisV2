/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                               #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\tof_manager.h                                      #
 #  Brief: Single owner of every VL53L0X on I2C1: one table entry per laser    #
 #         ( 4 side, EXTERNAL, DOWN ) with its own device, status and results.#
 #         One laser's error never stops another.                              #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-14	AJ	Created ( tof-single-pipeline task 2 )                     #
*******************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Same values as laser_e in API/XRanging.h ( LEFT 0 … EXTERNAL 4, DOWN 5 ).
typedef enum {
  TOF_LEFT = 0,
  TOF_RIGHT,
  TOF_FRONT,
  TOF_BACK,
  TOF_EXTERNAL,
  TOF_DOWN,
  TOF_COUNT
} tofId_e;

typedef enum {
  TOF_STATUS_NOT_ENABLED = 0,    // not requested, or not brought up yet
  TOF_STATUS_OK,
  TOF_STATUS_BOOT_FAILED,        // probe, address change or init failed ( failedStep says where )
  TOF_STATUS_I2C_ERROR           // failed after a good bring-up; never retried
} tofStatus_e;

#define TOF_XSHUT_NONE     ( -1 )
#define TOF_RANGE_INVALID  ( -100 )    // range when there is no valid sample ( XRanging's value )
#define TOF_DOWN_ADDRESS   0x30        // 7-bit; side lasers use 0x2A + n

typedef struct {
  bool requested;         // user code asked for this laser ( set before the bring-up )
  int8_t xshutGpio;       // peripheral_gpio_pin_e, or TOF_XSHUT_NONE
  uint8_t address;        // 7-bit, valid once status is OK
  uint8_t status;         // tofStatus_e
  uint8_t failedStep;     // vl53l0xStep_e of a BOOT_FAILED, else 0
  int8_t lastError;       // VL53L0X_Error that set BOOT_FAILED / I2C_ERROR
  uint8_t rangeStatus;    // RangeStatus of the last sample ( 0 = valid )
  bool outOfRange;        // last sample was not a valid range
  bool newSample;         // a sample arrived since the consumer last cleared it
  uint16_t rawMm;         // last sample's RangeMilliMeter, unfiltered, mm
  int16_t rangeMm;        // last valid range in mm, or TOF_RANGE_INVALID ( XRanging.getRange ( ) )
  uint16_t samples;       // samples read since bring-up ( wraps ), for update-rate checks
} tofEntry_t;

// ---- Table ----
void tofManagerReset ( void );                               // every entry back to NOT_ENABLED
void tofManagerRequest ( tofId_e id, int8_t xshutGpio );     // mark a laser wanted, with its XSHUT pin
const tofEntry_t *tofManagerEntry ( tofId_e id );            // read-only view, NULL for a bad id

// ---- Bring-up ( one entry; the order across entries is the caller's ) ----
// The chip answering at fromAddr7 is moved to toAddr7 ( skipped if equal ) and configured for its
// role there: DOWN = single ranging, 33 ms ( the former ranging_init ); side = continuous, 20 ms.
// Sets status OK, or BOOT_FAILED with failedStep / lastError. Returns true on OK.
bool tofManagerBringUp ( tofId_e id, uint8_t fromAddr7, uint8_t toAddr7 );
// Start continuous ranging on every side laser that came up. Called once, after the last bring-up,
// so no laser fires while another one calibrates. A failure is BOOT_FAILED ( STEP_START_RANGING ).
void tofManagerStartSides ( void );
// Mark a laser BOOT_FAILED without touching the bus ( e.g. VL53L0X_STEP_BUS_BUSY ).
void tofManagerMarkFailed ( tofId_e id, uint8_t step );

// ---- Polling ----
// DOWN: every 33 ms, start a measurement and read the result, then the 0.1 IIR ( the former getRange ( ) ).
void tofManagerPollDown ( void );
// Side lasers: at most one laser's I2C per call ( paced to TOF_SIDE_POLL_MS ), round-robin over the
// running ones; a sample is read only when the chip has one ready. Readers use the cached entry.
void tofManagerPollSides ( void );

// ---- DOWN accessors for altitude hold ----
bool tofDownNew ( void );
void tofDownClearNew ( void );
bool tofDownOutOfRange ( void );    // also true while DOWN's status is not OK
void tofDownReseed ( void );        // seed the IIR from the next valid sample
uint16_t tofDownFiltMm ( void );    // 0.1 IIR of the range, mm ( was NewSensorRange )
uint16_t tofDownRawMm ( void );     // last unfiltered sample, mm

#ifdef __cplusplus
}
#endif
