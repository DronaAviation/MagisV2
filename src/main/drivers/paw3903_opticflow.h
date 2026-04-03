/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\paw3903_opticflow.h                                #
 #  Created Date: Thu, 6th Nov 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Fri, 14th Nov 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#ifndef PAW3903_OPTICFLOW_H
#define PAW3903_OPTICFLOW_H

#include <stdint.h>
#include "API/Peripherals.h"
#include "drivers/bridge_sc18is602b.h"

#define PAW3903_REG_Product_ID    0x00    // Reset value: 0x49
#define PAW3903_REG_Revision_ID   0x01    // Reset value: 0x01
#define PAW3903_REG_Motion        0x02
#define PAW3903_REG_Delta_X_L     0x03
#define PAW3903_REG_Delta_X_H     0x04
#define PAW3903_REG_Delta_Y_L     0x05
#define PAW3903_REG_Delta_Y_H     0x06
#define PAW3903_REG_Squal         0x07
#define PAW3903_REG_Shutter_Lower 0x0B
#define PAW3903_REG_Shutter_Upper 0x0C
#define PAW3903_REG_Observation   0x15
#define PAW3903_REG_PowerUpReset  0x3A
#define PAW3903_REG_Shutdown      0x3B
#define PAW3903_REG_Motion_Burst  0x16
#define PAW3903_REG_Resolution    0x4E
#define PAW3903_REG_Orientation   0x5B
#define PAW3903_REG_LightMode     0x22

/**
 * @brief Enumerates common resolution (CPI/DPI) settings for the PAW3903 sensor.
 *
 * Each step in the register represents 50 counts per inch (CPI).
 * The effective resolution = enum value * 50.
 *
 * Register: 0x4E (Resolution)
 * Range: 0x01 (50 CPI) → 0x80 (6400 CPI)
 */
typedef enum {
  PAW3903_RES_400_CPI  = 0x08, /**<  400 CPI  */
  PAW3903_RES_800_CPI  = 0x10, /**<  800 CPI  */
  PAW3903_RES_1000_CPI = 0x14, /**< 1000 CPI (default after reset) */
  PAW3903_RES_1200_CPI = 0x18, /**< 1200 CPI  */
  PAW3903_RES_1600_CPI = 0x20, /**< 1600 CPI  */
  PAW3903_RES_2000_CPI = 0x28, /**< 2000 CPI  */
  PAW3903_RES_2400_CPI = 0x30, /**< 2400 CPI  */
  PAW3903_RES_3000_CPI = 0x3C, /**< 3000 CPI  */
  PAW3903_RES_3200_CPI = 0x40, /**< 3200 CPI  */
  PAW3903_RES_4000_CPI = 0x50, /**< 4000 CPI  */
  PAW3903_RES_5000_CPI = 0x64, /**< 5000 CPI  */
  PAW3903_RES_6400_CPI = 0x80  /**< 6400 CPI (max recommended) */
} PAW3903_ResolutionCPI_t;

/**
 * @brief Enumerates the three operation modes of the PAW3903 sensor.
 *
 * Register: 0x22 (Operation Mode)
 *
 * These modes configure internal frame rate, gain, and illumination
 * to optimize performance under different lighting conditions.
 *
 * | Mode | Frame Rate | Typical Lux | Description |
 * |:----:|:-----------:|:-----------:|:-------------|
 * | 0 | 126 fps | ~60 | Bright Mode — for well-lit surfaces |
 * | 1 | 126 fps | ~30 | Low Light Mode — balanced, default |
 * | 2 | 50 fps  | ~9  | Super Low Light Mode — for dark environments |
 *
 * Default after reset: PAW3903_MODE_LOW_LIGHT
 */

typedef enum {
  PAW3903_MODE_BRIGHT          = 0x00,
  PAW3903_MODE_LOW_LIGHT       = 0x01,
  PAW3903_MODE_SUPER_LOW_LIGHT = 0x02
} PAW3903_OperationMode_t;

/**
 * @brief Enumerates axis orientation control bits for the PAW3903 sensor.
 *
 * Register: 0x5B (Orientation)
 * Reset value: 0xE0  (default = normal orientation)
 *
 * Each bit controls axis swapping or inversion:
 *  - Bit 5 → Invert X-axis
 *  - Bit 4 → Invert Y-axis
 *  - Bit 3 → Swap X and Y axes
 *
 * These bits allow you to compensate for the sensor's physical
 * mounting direction on the PCB or board.
 */
typedef enum {
  PAW3903_ORIENT_NORMAL   = 0x00, /**< Default orientation (no swap, no invert) */
  PAW3903_ORIENT_INVERT_X = 0x20, /**< Invert X-axis (bit 5 = 1) */
  PAW3903_ORIENT_INVERT_Y = 0x10, /**< Invert Y-axis (bit 4 = 1) */
  PAW3903_ORIENT_SWAP_XY  = 0x08, /**< Swap X and Y axes (bit 3 = 1) */

  /* Common combinations */
  PAW3903_ORIENT_INVERT_XY      = 0x30, /**< Invert both X and Y axes */
  PAW3903_ORIENT_SWAP_INVERT_X  = 0x28, /**< Swap X/Y, invert X only */
  PAW3903_ORIENT_SWAP_INVERT_Y  = 0x18, /**< Swap X/Y, invert Y only */
  PAW3903_ORIENT_SWAP_INVERT_XY = 0x38  /**< Swap and invert both axes */
} PAW3903_Orientation_t;

typedef struct {
  uint8_t motion;
  int16_t deltaX;
  int16_t deltaY;
  uint8_t squal;
  uint16_t shutter;
  uint8_t observation;
} PAW3903_Data;

/**
 * @brief Checks the product and revision IDs of the PAW3903 sensor.
 *
 * This function reads the product and revision IDs from the PAW3903 sensor
 * registers and optionally stores them in provided variables. It verifies if
 * the sensor's product ID is 0x49 and revision ID is 0x01.
 *
 * @param[out] product_id Pointer to store the read product ID, if not null.
 * @param[out] revision_id Pointer to store the read revision ID, if not null.
 * @return True if the product ID is 0x49 and the revision ID is 0x01; false otherwise.
 */
bool paw3903_check_id ( uint8_t *product_id, uint8_t *revision_id );

/**
 * Initializes the PAW3903 sensor by setting up SPI communication.
 *
 * @return true if the initialization is successful and the product ID
 *         and revision ID are verified, false otherwise.
 */
bool paw3903_init ( void );

/**
 * @brief Select one of the PAW3903 operation modes.
 *
 * Mode 0: Bright Mode (126 fps) – for bright surfaces (~60 lux)
 * Mode 1: Low Light Mode (126 fps, default) – for moderate lighting (~30 lux)
 * Mode 2: Super Low Light Mode (50 fps) – for dark environments (~9 lux)
 *
 * @param mode :`PAW3903_MODE_BRIGHT`, `PAW3903_MODE_LOW_LIGHT`, `PAW3903_MODE_SUPER_LOW_LIGHT`
 * @return true if successful
 */
bool paw3903_set_mode ( PAW3903_OperationMode_t mode );

/**
 * @brief Read current operating mode (if readable in your version).
 * @return PAW3903_OperationMode_t `PAW3903_MODE_BRIGHT`, `PAW3903_MODE_LOW_LIGHT`, `PAW3903_MODE_SUPER_LOW_LIGHT`
 */
PAW3903_OperationMode_t paw3903_get_mode ( void );

/**
 * Performs a power-up reset on the PAW3903 sensor.
 *
 * @return true if the reset command is successfully written to the
 *         PowerUpReset register, false otherwise.
 */
bool paw3903_power_up_reset ( void );

/**
 * Initiates the shutdown sequence for the PAW3903 sensor.
 *
 * @return true if the shutdown command is successfully written to the
 *         Shutdown register, false otherwise.
 */
bool paw3903_shutdown ( void );

/**
 * Reads the Surface Quality (SQUAL) value from the PAW3903 sensor.
 *
 * This function retrieves the SQUAL value, which represents the quality
 * of the surface beneath the sensor. Higher values typically indicate
 * better tracking conditions.
 *
 * @return The SQUAL value as an 8-bit unsigned integer.
 */
uint8_t paw3903_read_squal ( void );

/**
 * Reads the Shutter value from the PAW3903 sensor.
 *
 * This function retrieves the shutter value, which is a measure of the
 * sensor's exposure time. It combines the lower and upper byte registers
 * to form a 16-bit unsigned integer representing the full shutter value.
 *
 * @return The shutter value as a 16-bit unsigned integer.
 */
uint16_t paw3903_read_shutter ( void );

/**
 * @brief Read the Observation register (0x15) from the PAW3903.
 *
 * The Observation register provides a snapshot of the internal tracking
 * quality and image acquisition status of the optical flow engine.
 * It is mainly used for debugging, surface detection, and performance monitoring.
 *
 * Bit meaning (approximate, based on PixArt documentation and empirical behavior):
 * -------------------------------------------------------------------------------
 * Bit 7:6  → Surface quality category
 *            00 = very poor tracking (lifted or dark surface)
 *            01 = weak tracking (low SQUAL or motion blur)
 *            10 = moderate tracking (usable)
 *            11 = good tracking (stable surface detection)
 *
 * Bit 5    → Shutter saturation flag
 *            1 = Shutter near maximum (too dark or too high above surface)
 *            0 = Normal shutter operation
 *
 * Bit 4    → Frame invalid flag
 *            1 = Current frame invalid (motion blur, out of focus, or too bright)
 *            0 = Valid frame captured
 *
 * Bit 3:0  → Raw SQUAL bits / internal correlation
 *            Higher value = better surface quality and correlation strength
 *
 * Typical interpretation of register value:
 *   - 0x00–0x0F → No surface detected or poor tracking
 *   - 0x10–0x3F → Weak surface tracking
 *   - 0x40–0x7F → Good surface tracking
 *   - Bit5=1 often indicates low light / high shutter exposure
 *
 * @return Current value of Observation register (0x15)
 */
uint8_t paw3903_read_observation ( void );

/**
 * @brief Read all motion data from PAW3903 using Motion_Burst (0x16).
 *
 * This function performs a burst read starting at register 0x16.
 * The sensor outputs a sequence of bytes containing motion data, delta X/Y,
 * surface quality (SQUAL), shutter time, and observation state.
 *
 * Using Motion_Burst ensures all data comes from the same frame capture,
 * improving motion accuracy and reducing SPI overhead.
 *
 * Burst format (byte index):
 *  [0] Motion         - Status bits:
 *                       bit7 = Motion occurred (1 = new motion)
 *                       bit6 = Overflow on X/Y (1 = data overflow)
 *                       bit5 = Frame available flag
 *  [1] Delta_X_L      - X-axis motion (low byte)
 *  [2] Delta_X_H      - X-axis motion (high byte, signed)
 *  [3] Delta_Y_L      - Y-axis motion (low byte)
 *  [4] Delta_Y_H      - Y-axis motion (high byte, signed)
 *  [5] SQUAL          - Surface quality (0–255, higher = better)
 *  [6] Shutter_Lower  - Exposure time low byte
 *  [7] Shutter_Upper  - Exposure time high byte
 *  [8] Observation    - Tracking/validity status
 *
 */

bool paw3903_read_motion_burst ( PAW3903_Data &out );
// bool paw3903_read_motion_burst ( PAW3903_Data &out, bool debug = false );

/**
 * @brief Set the motion resolution (CPI) of PAW3903.
 *
 * The resolution determines how many counts per inch (CPI/DPI)
 * the sensor reports. The effective resolution is:
 *      Resolution = reg_value * 50
 *
 * Default after reset: 0x20 = 1600 CPI
 * Range: typically 0x01 (50 CPI) to 0x80 (6400 CPI)
 *
 * Use lower CPI (≈ 1200–1600) for long-range, low-noise tracking (e.g., drone navigation).
 * Use higher CPI (≈ 3200–4000) for short-range, high-precision tracking (e.g., surface mice).
 *
 * @param res Desired resolution in counts per inch (multiple of 50)
 * @return true if successful, false if out of range
 */
bool paw3903_set_resolution ( PAW3903_ResolutionCPI_t res );

/**
 * @brief Read current motion resolution setting.
 * @return Resolution in counts per inch (CPI)
 */
PAW3903_ResolutionCPI_t paw3903_get_resolution ( void );

/**
 * @brief Set PAW3903 orientation configuration.
 *
 * @param orient Orientation value (from PAW3903_Orientation_t)
 * @return true if successful
 */
bool paw3903_set_orientation ( PAW3903_Orientation_t orient );

/**
 * @brief Read current orientation register value.
 *
 * @return Current PAW3903 orientation configuration
 */
PAW3903_Orientation_t paw3903_get_orientation ( void );

/**
 * Configures the motion detection pin for the PAW3903 sensor.
 *
 * This function sets up a specified GPIO pin for motion detection by enabling
 * it and configuring it as an input-only pin. The default GPIO pin can be overridden
 * by providing a different one as a parameter.
 *
 * @param sc18GPIO The GPIO pin to configure, defaults to SS1 if not specified.
 * @return True if the configuration is successful; otherwise, false.
 */
bool paw3903_config_motion_pin ( SC18IS601B_GPIO sc18GPIO = SS1 );

/**
 * Configures the motion detection pin for the PAW3903 sensor.
 *
 * This function initializes a specified GPIO pin for motion detection by setting
 * it up as an input. It assigns a specific configuration to the motion GPIO.
 *
 * @param GPIO The GPIO pin to configure for motion detection.
 */
void paw3903_config_motion_pin ( peripheral_gpio_pin_e GPIO );

/**
 * Reads the state of the motion detection pin for the PAW3903 sensor.
 *
 * This function checks the configuration of the motion GPIO and returns the
 * current state of the motion pin. It uses different methods to read the pin
 * based on its configuration.
 *
 * @return true if motion is detected, false otherwise.
 */
bool paw3903_read_motion_pin ( void );

#endif