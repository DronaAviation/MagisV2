/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\XRanging.h                                             #
 #  Created Date: Sat, 31st Jan 2026                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 19th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-14	AJ	DOWN laser: initAlt ( ), getStatus ( ) ( tof-single-pipeline ) #
 #  2026-10-16	AJ	OA unused-laser default DOWN, not ( laser_e ) -1 ( -Wconversion ) #
 #  2026-10-19	AJ	X1Ranging used too refuses both ( vl53l1x-xranging-api t15 ) #
*******************************************************************************/
#ifndef X_RANGING_H
#define X_RANGING_H

#include <stdint.h>

#include "API/Peripherals.h"    // peripheral_gpio_pin_e

#ifdef __cplusplus
extern "C" {
#endif

typedef enum laser_sensors {
  LEFT = 0,
  RIGHT,
  FRONT,
  BACK,
  EXTERNAL,
  DOWN    // the downward altitude laser, enabled only with initAlt ( )
} laser_e;

// Health of one laser, from getStatus ( ).
typedef enum laser_status {
  LASER_NOT_ENABLED = 0,    // not requested, refused, or not started yet
  LASER_OK,                 // ranging, last sample valid
  LASER_BOOT_FAILED,        // did not answer or failed its start-up; not retried
  LASER_I2C_ERROR,          // stopped answering after a good start; not retried until power-off
  LASER_OUT_OF_RANGE        // ranging, but the last sample had no valid range
} laser_status_e;

class XRanging_P {
 private:
  int16_t triggerThreshold [ 6 ] = { -1, -1, -1, -1, -1, -1 };

 public:
  /**
   * @brief Initializes the laser ranging system by setting up initial states and trigger thresholds.
   *
   * This function configures the laser modules located at LEFT, RIGHT, FRONT, and BACK positions.
   * It marks each module as initialized and assigns a default trigger threshold value for each.
   */
  void init ( void );

  /**
   * @brief Initializes a specific laser module with a given threshold.
   *
   * This function sets the initialization state and assigns a trigger threshold
   * for a specified laser module, identified by the `laser` parameter.
   *
   * @param laser The enumeration value representing the laser module to initialize.(`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @param threshold The trigger threshold value (in `cm`) to set for the specified laser module.
   */
  void init ( laser_e laser, int16_t threshold = 20 );

  /**
   * @brief Determines if the specified laser module is triggered based on its range.
   *
   * This function checks the initialization status and trigger threshold of the given
   * laser module. It retrieves the range measurement and evaluates whether it falls
   * below a predefined threshold, indicating a trigger event. If the laser is triggered,
   * an associated GPIO pin may be set to high or low state.
   *
   * @param laser The enumeration value representing the laser module to check.(`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @return `true` if the laser is triggered; otherwise, `false`.
   */
  bool isTriggered ( laser_e laser );

  /**
   * @brief Retrieves the latest range of a laser module.
   * Returns the cached sample: the firmware reads the lasers itself ( DOWN every 33 ms, one side laser
   * per 10 ms ), so this call never waits on the I2C bus.
   * @param laser The enumeration value representing the laser module to measure. (`LEFT`,`RIGHT`,`FRONT`,`BACK`,
   *        `EXTERNAL`, or `DOWN` after `initAlt ( )` )
   * @return The range in mm; -100 when the laser has no valid sample; -1 if the laser is not initialized.
   */
  int16_t getRange ( laser_e laser );

  /**
   * @brief Uses the downward laser for altitude: the laser and the altitude-hold fusion together.
   * Call it only from `plutoInit ( )`. Without it the firmware holds altitude on the barometer alone.
   * The request is fixed at start-up and never changes while the drone runs.
   *
   * The pin wired to the down-laser's XSHUT lets the firmware hold the down-laser off while the
   * side lasers start. That pin then belongs to the down-laser: a side laser's LED on the same pin
   * ( `GPIO_5` is FRONT's LED ) is never driven.
   * @param xshutGpio The GPIO wired to the down-laser's XSHUT ( e.g. `GPIO_5` ).
   * @return `false` if refused: called after `plutoInit ( )`, the pin is a side laser's XSHUT, an
   *         RGB-LED slot or UART2 pin ( GPIO_1, 3, 6, 10-13, 16-18 ), or in use as ADC / PWM
   *         ( GPIO_2, 4, 5, 8 remain ). The pin is checked again after `plutoInit ( )`; check
   *         `getStatus ( DOWN )`. A refused call keeps an earlier accepted request. If `plutoInit ( )`
   *         also uses X1Ranging ( VL53L1X ), XRanging and X1Ranging are both refused at start-up: no
   *         laser starts and altitude hold uses the barometer. After start-up the pin is reserved:
   *         `Peripheral_Init` / `Peripheral_Write` on it ( or on its ADC / PWM alias ) are ignored.
   */
  bool initAlt ( peripheral_gpio_pin_e xshutGpio );

  /**
   * @brief As `initAlt ( xshutGpio )`, for a down-laser with no XSHUT wire. The firmware moves it
   * off the shared start-up address before the side lasers start.
   */
  bool initAlt ( void );

  /**
   * @brief Health of a laser, including `DOWN`.
   * @param laser The laser ( `LEFT`, `RIGHT`, `FRONT`, `BACK`, `EXTERNAL`, `DOWN` ).
   * @return `LASER_OK`, `LASER_OUT_OF_RANGE`, `LASER_BOOT_FAILED`, `LASER_I2C_ERROR` or `LASER_NOT_ENABLED`.
   */
  laser_status_e getStatus ( laser_e laser );

  /**
   * @brief Initializes the object avoidance system with specified parameters.
   *
   * This function sets the object avoidance distance to `_avoidDist` and initializes
   * up to four lasers for obstacle detection. Each laser is enabled by setting its
   * corresponding entry in `isXLaserInit` to `true`, provided it is a side laser
   * ( `LEFT` … `EXTERNAL` ); `DOWN` ( the default ) or -1 means unused.
   *
   * @param _avoidDist The distance ( in `cm`) parameter for triggering object avoidance.
   * @param _laser1 The first laser to be initialized for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @param _laser2 The second laser to be initialized for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @param _laser3 The third laser to be initialized for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @param _laser4 The fourth laser to be initialized for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   */
  void initObjectAvoidance ( uint16_t _avoidDist, laser_e _laser1, laser_e _laser2 = DOWN, laser_e _laser3 = DOWN, laser_e _laser4 = DOWN );
  /**
   * @brief Enables the obstacle avoidance feature.
   *
   * This function sets the internal flag `_enOA` to `true`, activating the
   * obstacle avoidance mechanism within the XRanging_P class.
   */
  void enableOA ( void );

  /**
   * @brief Disables the obstacle avoidance feature and turns off laser LEDs.
   *
   * This function sets the internal flag `_enOA` to `false`, deactivating the
   * obstacle avoidance mechanism within the XRanging_P class. It also iterates
   * over all lasers, turning off the LED for each initialized laser with a valid
   * GPIO pin by setting it to `STATE_LOW`.
   */
  void disableOA ( void );
};

extern XRanging_P XRanging;

#ifdef __cplusplus
}
#endif

#endif