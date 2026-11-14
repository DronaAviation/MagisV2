/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\FC-Config.h                                            #
 #  Created Date: Sun, 24th Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sun, 15th Nov 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  14-11-2026	AJ	setDefault_PIDProfile ( ): documented the five restored    #
 #              	profiles, RAM only, armed and LuxFloat caveats.            #
 #  15-11-2026	AJ	setDesiredRate / getDesiredRate: range, clamp, hold,       #
 #              	where the rate acts, flip and save caveats ( T16 ).        #
*******************************************************************************/
#ifndef FC_CONFIG_H
#define FC_CONFIG_H

// Define a structure named PID to hold PID controller parameters
typedef struct {
  uint8_t p;    // Proportional gain
  uint8_t i;    // Integral gain
  uint8_t d;    // Derivative gain
} PID;

// Define an enumeration for different types of PID profiles
typedef enum {
  PID_ROLL,     // PID profile for roll control
  PID_PITCH,    // PID profile for pitch control
  PID_YAW,      // PID profile for yaw control
  PID_ALT,      // PID profile for altitude control
  PID_USER      // User-defined PID profile
} pid_profile_e;

typedef enum {
  LOW_BATTERY,             // Indicates a low battery condition, triggering actions to prevent power loss.
  INFLIGHT_LOW_BATTERY,    // Indicates a low battery condition specifically when in flight, requiring immediate attention.
  CRASH,                   // Represents a crash condition, prompting emergency procedures to minimize damage.
  ALL                      // Represents all failsafe conditions, used to enable or check all failsafes simultaneously.
} failsafe_e;

/**
 * @brief Sets the loop frequency within 3.5 to 2000 Hz.
 *
 * Constrains and converts the input frequency from Hz to milliHz
 * for precision, updating the global variable.
 *
 * @param frequency Loop frequency in Hz.
 */
void setUserLoopFrequency ( float frequency );

/**
 * @brief Sets PID gains for a specified profile into the provided PID structure.
 *
 * @param PROFILE The PID profile type to retrieve gains from.
 * @param pid Pointer to the PID structure to be updated.
 */
void get_PIDProfile ( pid_profile_e PROFILE, PID *pid );

/**
 * @brief Updates the PID profile with gains from the provided PID structure.
 *
 * @param PROFILE The PID profile type to update.
 * @param pid Pointer to the PID structure containing new gain values.
 */
void set_PIDProfile ( pid_profile_e PROFILE, PID *pid );

/**
 * @brief Restores the firmware default gains of the five PID profiles above.
 *
 * Restores PID_ROLL and PID_PITCH to 40 / 10 / 30, PID_YAW to 150 / 70 / 50,
 * PID_ALT to 128 / 0 / 30 and PID_USER to 60 / 30 / 5 ( P / I / D ). The
 * firmware's internal gains ( level, velocity, position, navigation and
 * heading ) are not touched.
 *
 * The change is made in RAM only and is not saved: a power cycle brings back
 * the stored gains, unless something else saves the configuration first ( an
 * app save, a stick trim, an accelerometer or compass calibration, or a
 * profile change, which stores these gains into the profile being left ).
 *
 * Gains are read live every loop, so calling this while armed steps the
 * tuning in flight. Call it from plutoInit ( ) or while disarmed.
 *
 * With the LuxFloat PID controller ( pidController 2, not the default ), roll,
 * pitch and yaw use separate float gains, so the restored PID_ROLL, PID_PITCH
 * and PID_YAW values have no effect there.
 */
void setDefault_PIDProfile ( void );

/**
 * @brief Retrieves the angle the attitude controller is asked for.
 *
 * For AG_ROLL and AG_PITCH: the pilot's stick target plus the user tilt being
 * flown ( setDesiredAngle ), in deci-degrees, as computed by the controller in
 * its last loop. It is the request **before** the max_angle_inclination limit,
 * so it can read beyond the angle the drone actually flies. For AG_YAW it
 * returns 0: yaw is flown as a rate, not an angle.
 *
 * @param ANGLE An enumerated type specifying the angle type.
 * @return int32_t The requested angle; see above.
 */
int32_t getDesiredAngle ( angle_e ANGLE );

/**
 * @brief Sets the desired angle: a roll / pitch tilt, or a yaw bearing.
 *
 * For AG_ROLL and AG_PITCH, the drone tilts by `angle` ( deci-degrees, 100 is
 * 10 deg ) until Estimate_Get ( Angle, ... ) on that axis reads the value, and
 * holds it until a new call replaces it; setDesiredAngle ( AG_ROLL, 0 ) levels
 * it. The tilt is added to the pilot's stick, and the sum keeps the pilot's
 * angle limit ( max_angle_inclination, 20 deg by default ). It is flown only:
 *
 * - in ANGLE mode, with the default PID controller;
 * - once the drone is airborne ( throttle raised and at least 20 cm above
 *   where it armed ), so a take-off does not pivot on a leg;
 * - while Developer Mode runs; a Wi-Fi hiccup up to 0.6 s keeps it, a longer
 *   stall levels the drone until the link returns;
 * - outside a flip or a landing.
 *
 * When it is not flown it stays held. It is dropped when Developer Mode is
 * switched off ( the drone levels ) and when the drone disarms.
 *
 * For AG_YAW, heading hold is engaged automatically while Developer Mode
 * is on and the target is held ( a pilot-selected HEADFREE is kept ). The
 * pilot's yaw stick overrides it while deflected; once it is released and
 * the craft has stopped turning, the craft returns to this bearing.
 *
 * @param ANGLE An enumerated type specifying the angle type.
 * @param angle The angle value to be set. Deci-degrees for AG_ROLL and
 *              AG_PITCH ( bounded to +/-1800; the flown angle is limited
 *              as above ); whole degrees ( 0..359, compass referenced ) for
 *              AG_YAW. Other yaw values are wrapped ( 450 is 90, -90 is 270 ).
 */
void setDesiredAngle ( angle_e ANGLE, int32_t angle );

/**
 * @brief Turns the craft by an angle measured from its own heading.
 *
 * Where setDesiredAngle ( AG_YAW, x ) commands an absolute compass bearing,
 * this commands a turn of x degrees relative to the craft: positive turns
 * clockwise ( to the right ), negative anti-clockwise. The relative offset is
 * resolved to an absolute bearing once, at the moment of the call, and then
 * held - so a single call is a single turn, not a continuous rotation.
 *
 * Repeated calls compose against the setpoint already commanded, so three
 * calls of +90 add up to a full turn. The first call after entering Developer
 * Mode is measured from the craft's actual heading. A Wi-Fi hiccup re-runs
 * onLoopStart ( ) but keeps the target, so a relative call placed there adds
 * another turn after each hiccup: guard it, or use setDesiredAngle there.
 *
 * Like the absolute setter, heading hold is engaged for it automatically
 * while Developer Mode is on and a yaw target is held ( a pilot-selected
 * HEADFREE is kept, as it also holds heading ), so no app switch or
 * FlightMode_Set call is needed.
 *
 * The pilot's yaw stick overrides the turn while deflected. Once it is
 * released and the craft has stopped turning, the target is re-based to
 * where the pilot left the nose: the craft does not swing back, and the
 * next relative call is measured from there. A call made while the stick
 * is held, or while the craft is still turning from it, is measured from
 * the nose at that moment, and that target then stands after the release.
 * Do not mix with a yaw RcCommand_Set override: heading hold replaces it
 * once engaged, but the override is flown while heading hold waits to
 * re-arm ( yaw rate above ~17 deg/s ), and a fast override keeps it waiting.
 *
 * Only AG_YAW is supported; AG_ROLL and AG_PITCH are ignored.
 *
 * @param ANGLE An enumerated type specifying the angle type. Only AG_YAW acts.
 * @param angle The turn to make, in whole degrees, relative to the craft.
 */
void DesiredAngle_setRelative ( angle_e ANGLE, int32_t angle );

/**
 * @brief Retrieves the stick rate setting of an axis ( see setDesiredRate ).
 *
 * Returns the value the last setDesiredRate ( ) call on that axis holds, after
 * its clamp. If it was never called: the rate in use, which outside a flip is 75
 * for AG_ROLL and AG_PITCH and the stored profile value for AG_YAW ( 0 by
 * default ), and during a flip is the flip's own roll / pitch rate. It is the
 * setting, not how fast the drone turns: for the measured rotation use
 * Estimate_Get ( Rate, ... ).
 *
 * @param ANGLE AG_ROLL, AG_PITCH or AG_YAW; anything else returns 0.
 * @return int32_t The rate setting: 0-100 roll / pitch, 0-255 yaw.
 */
int32_t getDesiredRate ( angle_e ANGLE );

/**
 * @brief Sets the stick rate of an axis: how fast the drone turns at full stick.
 *
 * `rate` is the Cleanflight rate setting, 0-100 for AG_ROLL / AG_PITCH and
 * 0-255 for AG_YAW; a value outside the range is clamped to it. It acts at once
 * and holds until the next call or a reboot, through arming, disarming and
 * flips. With no call, roll / pitch run at 75 and yaw at the stored profile
 * value ( 0 by default ). With the default PID controller:
 *
 * - AG_YAW scales the yaw stick in every flight mode: the full-stick turn rate
 *   grows with rate + 20, so 20 doubles the default yaw stick.
 * - AG_ROLL / AG_PITCH act only outside ANGLE mode, e.g. after
 *   FlightMode_Set ( RATE ): the full-stick spin rate grows with rate + 20,
 *   so 30 spins at about half the default 75. In ANGLE mode, the default, they
 *   have no effect.
 * - A flip sets its own rates at its start and returns to this value when it
 *   ends. A roll / pitch call during a flip is kept ( getDesiredRate returns it )
 *   but applied only when the flip ends, so it cannot change the flip's spin
 *   rate. A flip that does not finish in time either hands the value back at
 *   once or disarms the drone, and the value is applied then.
 * - Under a non-default pidController ( app / CLI ) the rates also scale that
 *   controller's dynamic P / I / D gains in every mode, ANGLE included.
 *
 * This call does not save anything. It writes the live settings, so if they are
 * saved while it is set ( a settings save or profile change from the app, a stick
 * trim or a calibration ) the value is stored and survives a reboot. If the app
 * then loads other settings, AG_YAW takes the loaded value, while roll and pitch
 * are written back from the held value on the next disarmed loop; in both cases
 * getDesiredRate ( ) keeps returning the value set here.
 *
 * @param ANGLE AG_ROLL, AG_PITCH or AG_YAW; anything else is ignored.
 * @param rate  The rate setting; see above for the range.
 */
void setDesiredRate ( angle_e ANGLE, int32_t rate );

/**
 * @brief Retrieves the desired position for a specific axis.
 *
 * This function returns the desired position based on the specified
 * axis. If the axis is Z, it returns the currently set altitude;
 * otherwise, it returns 0 as a default value.
 *
 * @param AXIS An enumerated type specifying the axis (e.g., X, Y, Z).
 * @return The desired position for the specified axis or 0 if not Z.
 */
int32_t getDesiredPositions ( axis_e AXIS );

/**
 * @brief Sets the desired position for a specific axis.
 *
 * This inline function sets the desired position for the given axis.
 * If the specified axis is Z, it updates the set altitude using the
 * provided position. For other axes, no action is taken.
 *
 * @param AXIS An enumerated type specifying the axis (e.g., X, Y, Z).
 * @param position The position value to set for the specified axis.
 */
void setDesiredPosition ( axis_e AXIS, int32_t position );

/**
 * @brief Sets the desired position relative to a specific axis.
 *
 * This inline function adjusts the desired position relative to the given
 * axis. If the specified axis is Z, it modifies the relative altitude
 * using the provided position value. For other axes, no action is executed.
 *
 * @param AXIS An enumerated type specifying the axis (e.g., X, Y, Z).
 * @param position The position value to adjust for the specified axis.
 */
void DesiredPosition_setRelative ( axis_e AXIS, int32_t position );

/**
 * @brief Enables specified failsafe conditions.
 *
 * This function enables specific failsafe mechanisms based on the provided
 * FAILSAFE type. If the FAILSAFE type is ALL, all failsafes are enabled.
 * Otherwise, only the specified failsafe will be enabled while others
 * remain unchanged.
 *
 * @param FAILSAFE The failsafe condition to enable.
 */
void Failsafe_enable ( failsafe_e FAILSAFE );

/**
 * @brief Disables specified failsafe conditions.
 *
 * This function disables specific failsafe mechanisms based on the provided
 * FAILSAFE type. If the FAILSAFE type is ALL, all failsafes are disabled.
 * Otherwise, only the specified failsafe will be disabled while others
 * retain their current state.
 *
 * @param FAILSAFE The failsafe condition to disable.
 */
void Failsafe_disable ( failsafe_e FAILSAFE );
#endif
