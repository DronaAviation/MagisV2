/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\ObjectAvoidanceCore.h                              #
 #  Created Date: Mon, 19th Oct 2026                                           #
 #  Brief: The object-avoidance push / brake loop shared by XRanging           #
 #         ( VL53L0X ) and X1Ranging ( VL53L1X ). Header only: each API        #
 #         instantiates it with its own state, laser set and range source.     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 19th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-19	AJ	Created ( vl53l1x-xranging-api task 6 )                    #
 #  2026-10-19	AJ	3 out-of-range ticks clear the filter, 1st sample seeds ( t14 ) #
*******************************************************************************/

#pragma once

#include <stdint.h>

#include "API/API-Utils.h"      // RC_ARRAY, userRCassert ( ), millis ( ), constrain ( )
#include "API/Peripherals.h"    // GPIO_State_e
#include "API/XRanging.h"       // laser_e, laser_status_e ( shared by X1Ranging )
#include "io/rc_controls.h"     // rcData, ROLL / PITCH

// ============================================================
// Object Avoidance configuration
// ============================================================
#define AVOID_RC_MAX          25       // Maximum RC value deviation allowed during avoidance
#define OA_MAX_PUSH_TIME_MS   500      // Maximum time in milliseconds to push forward during obstacle avoidance
#define OA_CLEAR_HOLD_MS      500      // Time in milliseconds to hold clear state after an obstacle is no longer detected
#define OA_BRAKE_GAIN         0.45f    // Gain factor applied to braking during obstacle avoidance

#define USER_OA_BLEND_PUSH    0.3f    // User-defined blend factor for pushing action in obstacle avoidance
#define USER_OA_BLEND_BRAKE   0.2f    // User-defined blend factor for braking action in obstacle avoidance

#define AVOIDANCE_INTERVAL_MS 100    // Interval in milliseconds between successive obstacle avoidance checks

#define OA_FILTER_COUNT       4    // one filter per OA laser: 0 LEFT, 1 RIGHT, 2 FRONT, 3 BACK
#define OA_OOR_CLEAR_TICKS    3    // out-of-range OA ticks ( 100 ms each ) in a row that clear a side's filter

// The API that owns one copy of the avoidance state.
typedef enum {
  OA_CORE_XRANGING  = 0,    // XRanging ( VL53L0X ), applyObjectAvoidance ( )
  OA_CORE_X1RANGING = 1,    // X1Ranging ( VL53L1X ), x1RangingApplyObjectAvoidance ( )
} oaCoreUser_e;

/**
 * @brief The avoidance loop's own state, one copy per API ( U ). Each member is its own object
 * ( C++17 inline variable ), as the file statics of XRanging's loop were, so the core compiles to
 * the same code. All constant-initialised to 0 ( .bss ): no constructor.
 */
template < oaCoreUser_e U >
struct oaCoreState {
  static inline uint32_t lastRunMs                 = 0;        // ms, millis ( ) of the last 100 ms tick
  static inline int16_t filtMm [ OA_FILTER_COUNT ] = { 0 };    // mm, filtered distance per OA laser ( LEFT, RIGHT, FRONT, BACK )
  static inline uint8_t oorTicks [ OA_FILTER_COUNT ] = { 0 };  // out-of-range ticks in a row per OA laser ( saturates )
  static inline uint32_t rollStartTs               = 0;        // ms, start of the roll push, 0 = no avoidance in progress
  static inline uint32_t pitchStartTs              = 0;        // ms, start of the pitch push, 0 = no avoidance in progress
  static inline int16_t lastRollOA                 = 0;        // RC offset of the last sensed roll tick ( brake reference )
  static inline int16_t lastPitchOA                = 0;        // RC offset of the last sensed pitch tick ( brake reference )
};

/**
 * @brief Filters the input distance with a first-order IIR ( 3/4 previous + 1/4 new ).
 *
 * This function applies the IIR filter to smooth out the
 * distance readings. If the provided distance (d) is less than or equal to zero,
 * it returns the previously filtered value for that index without updating.
 * A filter at 0 ( start-up, or cleared by oaCoreDistance ) is seeded with the first
 * valid sample: averaging it against 0 would quarter it, and a wall at 2 m would read
 * 500 mm and fire one false push.
 *
 * @tparam U  The API whose filter values are used ( oaCoreState < U > ).
 * @param idx Index of the filter array to update and return the filtered value.
 * @param d   New distance measurement to be filtered ( mm ).
 * @return    The filtered distance value at the specified index ( mm ).
 */
template < oaCoreUser_e U >
static inline int16_t oaCoreFilter ( uint8_t idx, int16_t d ) {
  using S = oaCoreState < U >;
  if ( d <= 0 ) return S::filtMm [ idx ];
  if ( S::filtMm [ idx ] == 0 ) {    // seed ( see above ); a seeded filter never returns to 0 while d > 0
    S::filtMm [ idx ] = d;
    return d;
  }
  S::filtMm [ idx ] = ( int16_t ) ( ( S::filtMm [ idx ] * 3 + d ) / 4 );
  return S::filtMm [ idx ];
}

// OA distance for one side laser ( vl53l1x-xranging-api task 14, both APIs ).
//  - A laser that stopped working ( boot failed, I2C error, not enabled ) gives "no obstacle" at once and its
//    filter restarts.
//  - Out of range ( nothing within reach, a dark or glossy target, stale ): the filter is held for up to
//    OA_OOR_CLEAR_TICKS - 1 ticks, so a short dropout at a real obstacle keeps the push ( the brake phase
//    always runs at full strength, so dropped push ticks would leave a net roll toward the obstacle ). On the
//    OA_OOR_CLEAR_TICKS-th tick in a row it clears: a removed obstacle is released within 300 ms. Holding it
//    for good, as before, kept a removed obstacle "seen" and repeated the push / brake cycle.
// Ranging is XRanging_P or X1Ranging_P ( the same getStatus / getRange ).
template < oaCoreUser_e U, typename Ranging >
static inline int16_t oaCoreDistance ( uint8_t idx, Ranging &ranging, laser_e laser ) {
  using S                 = oaCoreState < U >;
  const laser_status_e st = ranging.getStatus ( laser );
  if ( st == LASER_OK ) {
    S::oorTicks [ idx ] = 0;
    return oaCoreFilter < U > ( idx, ranging.getRange ( laser ) );
  }
  if ( st == LASER_OUT_OF_RANGE && ++S::oorTicks [ idx ] < OA_OOR_CLEAR_TICKS ) {
    return S::filtMm [ idx ];    // a short dropout: hold
  }
  S::oorTicks [ idx ] = OA_OOR_CLEAR_TICKS;    // saturate: no wrap however long it stays out of range
  S::filtMm [ idx ]   = 0;
  return 0;
}

/**
 * @brief Calculates the remote control offset for object avoidance based on distance.
 *
 * If the distance is less than or equal to zero, or greater than the threshold, it returns
 * zero. The offset is inverted if the 'invert' parameter is true.
 *
 * @param distance    The measured distance to the object ( mm ).
 * @param thresholdMm The avoidance threshold ( mm, initObjectAvoidance cm * 10 ).
 * @param invert      Boolean flag indicating whether to invert the offset direction.
 * @return            The calculated remote control offset for object avoidance.
 */
static inline int16_t oaCoreMap ( int16_t distance, uint16_t thresholdMm, bool invert ) {
  if ( distance <= 0 ) return 0;
  if ( distance > thresholdMm ) return 0;

  int16_t rcOffset = AVOID_RC_MAX;
  if ( invert ) rcOffset = -rcOffset;
  return rcOffset;
}

/**
 * @brief Applies object avoidance logic to adjust roll and pitch remote control values.
 *
 * This function periodically checks the status of laser distance sensors to determine
 * if any objects are within a predefined range. It calculates offsets for roll and pitch
 * based on sensor input, adjusting the remote control values accordingly to avoid obstacles.
 * The function ensures that object avoidance is only applied at specified intervals and
 * blends user inputs with calculated offsets for smooth operation.
 *
 * @tparam U          The API: its own state, oaCoreState < U >.
 * @param enabled     The API's enableOA ( ) / disableOA ( ) flag.
 * @param thresholdMm The API's threshold ( mm ).
 * @param laserInit   The API's enabled-laser set, indexed by laser_e ( LEFT … BACK are read ).
 * @param ranging     The API object ( getStatus / getRange over its cached samples ).
 * @param ledWrite    The API's indicator LED write ( side index, state ).
 *
 * `enabled` and `thresholdMm` are references so that they are read where the loop reads them
 * ( after the 100 ms timestamp, and per laser ), as before the move into this core.
 */
template < oaCoreUser_e U, typename Ranging >
static inline void oaCoreRun ( const bool &enabled, const uint16_t &thresholdMm, const bool *laserInit,
                               Ranging &ranging, void ( *ledWrite ) ( uint8_t, GPIO_State_e ) ) {
  using S = oaCoreState < U >;

  // Get the current time in milliseconds
  uint32_t now = millis ( );

  // Check if the avoidance logic should run based on the predefined interval
  if ( now - S::lastRunMs < AVOIDANCE_INTERVAL_MS )
    return;

  // Update the last run timestamp
  S::lastRunMs = now;

  // Return if object avoidance is not enabled
  if ( ! enabled ) return;

  // Calculate user input adjustments for roll and pitch
  int16_t userRoll  = rcData [ ROLL ] - 1500;
  int16_t userPitch = rcData [ PITCH ] - 1500;

  // Initialize offsets for roll and pitch
  int16_t rollOffset  = 0;
  int16_t pitchOffset = 0;

  // Flags to indicate if an obstacle is detected
  bool rollSense  = false;
  bool pitchSense = false;

  // Check and process left laser sensor
  if ( laserInit [ LEFT ] ) {
    int16_t o = oaCoreMap ( oaCoreDistance < U > ( 0, ranging, LEFT ), thresholdMm, false );
    rollSense |= ( o != 0 );
    ledWrite ( LEFT, o ? STATE_HIGH : STATE_LOW );
    rollOffset += o;
  }

  // Check and process right laser sensor
  if ( laserInit [ RIGHT ] ) {
    int16_t o = oaCoreMap ( oaCoreDistance < U > ( 1, ranging, RIGHT ), thresholdMm, true );
    rollSense |= ( o != 0 );
    ledWrite ( RIGHT, o ? STATE_HIGH : STATE_LOW );
    rollOffset += o;
  }

  // Check and process front laser sensor
  if ( laserInit [ FRONT ] ) {
    int16_t o = oaCoreMap ( oaCoreDistance < U > ( 2, ranging, FRONT ), thresholdMm, true );
    pitchSense |= ( o != 0 );
    ledWrite ( FRONT, o ? STATE_HIGH : STATE_LOW );
    pitchOffset += o;
  }

  // Check and process back laser sensor
  if ( laserInit [ BACK ] ) {
    int16_t o = oaCoreMap ( oaCoreDistance < U > ( 3, ranging, BACK ), thresholdMm, false );
    pitchSense |= ( o != 0 );
    ledWrite ( BACK, o ? STATE_HIGH : STATE_LOW );
    pitchOffset += o;
  }

  // If a roll obstacle is sensed, update the start timestamp and last offset
  if ( rollSense ) {
    if ( S::rollStartTs == 0 ) S::rollStartTs = now;
    S::lastRollOA = rollOffset;
  }

  // If a pitch obstacle is sensed, update the start timestamp and last offset
  if ( pitchSense ) {
    if ( S::pitchStartTs == 0 ) S::pitchStartTs = now;
    S::lastPitchOA = pitchOffset;
  }

  // Adjust roll control based on the object avoidance state ( float -> RC_ARRAY int32, truncated )
  if ( S::rollStartTs && ( now - S::rollStartTs < OA_MAX_PUSH_TIME_MS ) )
    RC_ARRAY [ ROLL ] = ( int32_t ) ( ( userRoll * USER_OA_BLEND_PUSH ) + rollOffset );
  else if ( S::rollStartTs && ( now - S::rollStartTs < OA_MAX_PUSH_TIME_MS + OA_CLEAR_HOLD_MS ) )
    RC_ARRAY [ ROLL ] = ( int32_t ) ( ( userRoll * USER_OA_BLEND_BRAKE ) + ( -S::lastRollOA * OA_BRAKE_GAIN ) );
  else {
    S::rollStartTs     = 0;
    S::lastRollOA      = 0;
    RC_ARRAY [ ROLL ] = ( int32_t ) ( userRoll * USER_OA_BLEND_PUSH );
  }

  // Adjust pitch control based on the object avoidance state
  if ( S::pitchStartTs && ( now - S::pitchStartTs < OA_MAX_PUSH_TIME_MS ) )
    RC_ARRAY [ PITCH ] = ( int32_t ) ( ( userPitch * USER_OA_BLEND_PUSH ) + pitchOffset );
  else if ( S::pitchStartTs && ( now - S::pitchStartTs < OA_MAX_PUSH_TIME_MS + OA_CLEAR_HOLD_MS ) )
    RC_ARRAY [ PITCH ] = ( int32_t ) ( ( userPitch * USER_OA_BLEND_BRAKE ) + ( -S::lastPitchOA * OA_BRAKE_GAIN ) );
  else {
    S::pitchStartTs     = 0;
    S::lastPitchOA      = 0;
    RC_ARRAY [ PITCH ] = ( int32_t ) ( userPitch * USER_OA_BLEND_PUSH );
  }

  // Constrain the roll and pitch values within acceptable limits
  RC_ARRAY [ ROLL ]  = constrain ( RC_ARRAY [ ROLL ], -500, 500 );
  RC_ARRAY [ PITCH ] = constrain ( RC_ARRAY [ PITCH ], -500, 500 );

  // Latch the override and keep it fresh against the watchdog. Pilot authority
  // is 0 here because the pilot's stick is already folded into RC_ARRAY above
  // (USER_OA_BLEND_PUSH / USER_OA_BLEND_BRAKE); letting mw.cpp cross-fade it in
  // again would both double-count the pilot and let a full stick fade the
  // obstacle push away, which is exactly when it is needed most.
  userRCassert ( ROLL, 0.0f );
  userRCassert ( PITCH, 0.0f );
}
