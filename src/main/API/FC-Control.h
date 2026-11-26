/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\FC-Control.h                                           #
 #  Created Date: Sat, 23rd Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 5th Nov 2026                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-11-05	AJ	Flight status order and values documented ( API 1.8.4 )    #
 #  2026-10-16	AJ	Take-off / land / flip rules documented ( API 1.7.0 )      #
*******************************************************************************/
#ifndef FC_CONTROL_H
#define FC_CONTROL_H

// Define an enumeration for various flight statuses.
typedef enum {
  FS_ACCEL_GYRO_CALIBRATION = 0,    // Indicates that the accelerometer and gyroscope are being calibrated.
  FS_MAG_CALIBARATION,              // Indicates that the magnetometer is being calibrated.
  FS_LOW_BATTERY,                   // Indicates a low battery condition.
  FS_INFLIGHT_LOW_BATTERY,          // Indicates a low battery condition while in flight.
  FS_CRASHED,                       // Indicates that the aircraft has crashed.
  FS_SIGNAL_LOSS,                   // Indicates a loss of control signal.
  FS_NOT_OK_TO_ARM,                 // Indicates that it's not safe to arm the aircraft.
  FS_OK_TO_ARM,                     // Indicates that it's safe to arm the aircraft.
  FS_ARMED                          // Indicates that the aircraft is armed.
} flightstatus_e;

// Define an enumeration for different flight modes
typedef enum {
  ANGLE,            // Angle mode: stabilizes the craft to maintain level flight
  RATE,             // Rate mode: allows manual control of the craft's rate of rotation
  MAGHOLD,          // Magnetic hold mode: maintains the current heading using a compass
  HEADFREE,         // Head-free mode: controls are independent of the craft's orientation
  ATLTITUDEHOLD,    // Altitude hold mode: maintains a set altitude automatically
  THROTTLE_MODE     // Throttle mode: defines how throttle input is handled
} flight_mode_e;

// Define an enumeration for flip directions
typedef enum {
  BACK_FLIP    // Back flip direction: performs a back flip maneuver
} flip_direction_e;

/**
 * @brief Gets the current flight status: the most important of the statuses that are active.
 *
 * Several statuses can be active at once ( for example crashed and OK to arm ); this returns the first
 * active one in this order:
 *   1. FS_ACCEL_GYRO_CALIBRATION, FS_MAG_CALIBARATION
 *   2. FS_CRASHED: set when the crash detector disarms ( crash failsafe on, the default; armed in ANGLE
 *      mode, not during a flip; tilt over 70 degrees or an impact ).
 *      Cleared about 0.3 s after the disarm once the drone is level, so read it every loop.
 *   3. FS_LOW_BATTERY ( warning level ), then FS_INFLIGHT_LOW_BATTERY ( critical level ). Both only with
 *      the in-flight low-battery failsafe on, the default. Once critical is confirmed the drone lands, and
 *      the status is reported again after the landing until the battery is changed. A provisional critical,
 *      before the pack resistance is measured, does not land and can clear when disarmed.
 *   4. FS_SIGNAL_LOSS
 *   5. FS_NOT_OK_TO_ARM / FS_OK_TO_ARM, then FS_ARMED
 *
 * The same status the app receives, except that the app is told FS_LOW_BATTERY instead of
 * FS_INFLIGHT_LOW_BATTERY while armed.
 *
 * @return Current `flightstatus_e`. Right after a disarm with the ARM switch still on, one read can
 *         find no status active: the value is then 0xFFFF, which matches no FS_ value ( not armed ).
 */
flightstatus_e FlightStatus_Get ( void );

/**
 * @brief Checks whether one flight status is active, whatever its priority.
 *
 * For example FlightStatus_Check ( FS_ARMED ) is true while armed even when FlightStatus_Get ( )
 * returns FS_LOW_BATTERY.
 *
 * @param status Flight status of type `flightstatus_e`.
 * @return True if that status is active; false if not, or if `status` is not a `flightstatus_e` value.
 */
bool FlightStatus_Check ( flightstatus_e status );

/**
 * @brief Checks if the specified flight mode is active.
 *
 * This function evaluates the given flight mode by comparing it against various predefined modes,
 * returning a boolean value indicating its activation status.
 *
 * @param MODE The flight mode to check, of type `flight_mode_e`.
 * @return True if the specified mode is active, false otherwise.
 */
bool FlightMode_Check ( flight_mode_e MODE );

/**
 * @brief Sets the specified flight mode and updates user flight mode states.
 *
 * This function activates or deactivates specific flight modes based on the input,
 * while also updating an array that tracks which user-defined flight modes are currently set.
 *
 * @param MODE The flight mode to set, of type `flight_mode_e`.
 */
void FlightMode_Set ( flight_mode_e MODE );

/**
 * @brief Takes off and climbs to a height, then holds it ( fly with ALT HOLD on ).
 *
 * The command is queued and runs at the end of the loop ( docs/API/FC_CONTROL_API_WIKI.md ).
 * - Disarmed: arms the drone once the ARM switch is on and the throttle is centred, then climbs. Until
 *   then it waits.
 * - Already armed ( Command_Arm ( ), the ARM switch ): climbs if armed less than 3 s ago or at the
 *   ARM-switch ground idle; in the air it sets no height.
 * - Once per arm. The height is set only with the throttle centred ( alt-hold deadband, default
 *   1460-1540 ); on an armed drone an off-centre stick uses the take-off up with no height.
 * - Ignored while a landing is in progress.
 * - Dropped if the ARM switch is on but arming is refused for 2 s ( calibrating, critical battery ), whatever
 *   the stick, and at a Developer Mode switch-off while it waits. With the battery OK, arming is allowed
 *   again right after a landing: a take-off sent then re-arms the drone and climbs.
 *
 * @param height Take-off height in cm, constrained to 100-250. Baro: above the current height; laser
 *               ( XRanging.initAlt ): above the floor.
 */
void Command_TakeOff ( uint16_t height = 150 );

/**
 * @brief Lands: descends, confirms touchdown and disarms.
 *
 * Accepted only while armed and when no other command is running; ignored while a flip is in progress
 * ( send it again after the flip ). The descent throttle starts at
 * 1305 - landSpeed and bleeds down to 1150; in ALT HOLD that is a descent of ( 1500 - throttle ) / 4 cm/s,
 * at most 100 cm/s. Touchdown is confirmed no earlier than 1 s after the start, or at once on an impact.
 * Neither Command_TakeOff nor Command_Flip can interrupt it.
 *
 * @param landSpeed 5 starts at 1300 ( about -50 cm/s, like the failsafe landing ); the default 105 starts
 *                  at 1200 ( about -75 cm/s ).
 */
void Command_Land ( uint8_t landSpeed = 105 );

/**
 * @brief Starts a back flip.
 *
 * Runs only in MAG ( heading hold ) mode and when no flip is in progress; otherwise it is ignored.
 * Ignored while a landing is in progress. A flip ends after at most 8 s: in level flight if within
 * 70 degrees, otherwise the drone is disarmed.
 *
 * @param direction The direction of the flip; only BACK_FLIP exists.
 */
void Command_Flip ( flip_direction_e direction );

/**
 * @brief Arms the drone now, if the ARM switch is on and arming is allowed.
 *
 * Arming needs the drone calibrated and the battery not critical. With the battery OK it is also allowed again
 * right after a landing, with the ARM switch still on. A Command_TakeOff that follows within 3 s still climbs.
 *
 * @return True if the drone is armed.
 */
bool Command_Arm ( void );

/**
 * @brief Disarms the system.
 *
 * This function disarms the system regardless of its current state.
 * It returns the negation of the arming status.
 *
 * @return True if the system is disarmed, false otherwise.
 */
bool Command_DisArm ( void );

/**
 * @brief Sets the head-free mode heading.
 *
 * Assigns the provided heading to userHeadFreeHoldHeading
 * and updates the flag to indicate it's set.
 *
 * @param heading The desired heading as int16_t.
 */
void setheadFreeModeHeading ( int16_t heading );

#endif