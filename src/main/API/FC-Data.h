/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\FC-Data.h                                              #
 #  Created Date: Sat, 23rd Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Wed, 4th Nov 2026                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#ifndef FC_DATA_H
#define FC_DATA_H

#include <stdint.h>
#include "common/axis.h"
// Enumeration to represent different types of sensors available in the flight controller
typedef enum {
  Accelerometer,    // Sensor for measuring acceleration forces
  Gyroscope,        // Sensor for measuring rotational motion
  Magnetometer,     // Sensor for measuring magnetic fields
  Barometer         // Sensor for measuring atmospheric pressure
} FC_Sensors_e;

// Enumeration to represent different types of data that can be read from a barometer
typedef enum {
  Pressure,      // Data representing atmospheric pressure
  Temperature    // Data representing temperature
} BARO_Data_e;

typedef enum {
  Angle,
  Rate,
  Position,
  Velocity
} FC_Estimate_e;

/**
 * @brief Retrieve sensor data for accelerometer, gyroscope, or magnetometer based on the specified axis.
 *
 * This function returns scaled sensor readings for the provided sensor type and axis.
 * It supports accelerometer, gyroscope, and magnetometer sensors. If an unsupported
 * sensor type is requested, the function returns 0.
 *
 * @param _sensor The type of sensor (Accelerometer, Gyroscope, Magnetometer).
 * @param _axis The axis for which to retrieve the data (e.g., X, Y, Z, Net_Acc for accelerometer).
 * @return Signed (int32_t) Scaled sensor reading or 0 if the sensor type is unsupported.
 *
 * - For the Accelerometer:
 *   - If _axis is Net_Acc, it returns the net acceleration magnitude.
 *   - Otherwise, it returns the scaled accelerometer value for the specified axis.
 *
 * - For the Gyroscope:
 *   - Returns the scaled gyroscope value for the specified axis.
 *
 * - For the Magnetometer:
 *   - Returns the scaled magnetometer value for the specified axis.
 *
 * - Returns 0 for unsupported sensor types.
 */
int32_t Sensor_Get ( FC_Sensors_e _sensor, axis_e _axis );
/**
 * @brief Retrieves barometer data for the specified sensor and data type.
 *
 * This function checks whether the provided sensor is a barometer and, if so,
 * returns the requested barometric data (pressure or temperature) in specified units.
 *
 * @param _sensor The sensor type, which should be Barometer for valid data retrieval.
 * @param _data The type of barometric data to retrieve (Pressure or Temperature).
 * @return Signed (int32_t) The retrieved barometric pressure in Pa ( = mbar x100, e.g. 100470 ),
 *         temperature in deg C x100 ( e.g. 3270 = 32.70 C ), both rounded to nearest,
 *         or 0 for unsupported types.
 */
int32_t Sensor_Get ( FC_Sensors_e _sensor, BARO_Data_e _data );

/**
 * @brief Retrieves the estimated value based on the type of estimate and specified axis.
 *
 * This function uses a switch statement to determine the estimated value from different
 * types of estimates (Rate, Position, Velocity) across specified axes (X, Y, Z).
 * Depending on the combination of estimate type and axis, it returns the corresponding
 * value or 0 for unhandled cases.
 *
 * @param _estimateOf The type of estimate requested (Rate, Position, Velocity).
 * @param _axis The specific axis for which the estimate is required (X, Y, Z).
 *
 * @return int16_t The estimated value:
 *         - For `Rate` estimates: the measured rotation rate in deg/s x10 ( e.g. 900 = 90 deg/s ), from the gyro:
 *           roll rate if X, pitch rate if Y, yaw rate if Z. Rounded to nearest.
 *           Signs: X > 0 rolling right ( right side down ), Y > 0 pitching nose down, both the same way as
 *           the roll and pitch angles; Z < 0 turning right ( clockwise seen from above ), the opposite way
 *           to the heading ( Angle AG_YAW ), which counts up clockwise.
 *         - For `Position` estimates: position along X and Y in cm, estimated altitude for Z in cm.
 *         - For `Velocity` estimates: velocity along X and Y in cm/s, estimated vertical velocity for Z in cm/s.
 *         Returns 0 for any unrecognized estimate types or axes.
 *         Position and Velocity on X and Y stay 0 unless a positioning sensor ( UWB ) feeds the estimator.
 *
 * @warning `Position` and `Velocity` on X and Y need UWB ( the only X / Y source in this firmware ); Z is always valid.
 */
int16_t Estimate_Get ( FC_Estimate_e _estimateOf, axis_e _axis );

/**
 * @brief Retrieves the estimated angle measurement in specified units.
 *
 * This function returns the estimated angle measurement based on the type of estimate
 * and the specific angle requested. The angles can be roll, pitch, or yaw,
 * and they are returned in their respective units (deciDegrees for roll and pitch, degrees for yaw).
 *
 * @param _estimateOf Specifies the type of estimate, expected to be Angle for this function.
 * @param _angle The specific angle measurement requested (AG_ROLL, AG_PITCH, AG_YAW).
 *
 * @return int16_t The estimated angle measurement:
 *         - Roll in deciDegrees if _angle is AG_ROLL.
 *         - Pitch in deciDegrees if _angle is AG_PITCH.
 *         - Heading in degrees if _angle is AG_YAW.
 *         Roll > 0 is right side down, pitch > 0 is nose down; the heading is 0 to 359, rising clockwise.
 *         Returns 0 if the estimate type is not Angle or if the angle is unrecognized.
 */
int16_t Estimate_Get ( FC_Estimate_e _estimateOf, angle_e _angle );

#endif