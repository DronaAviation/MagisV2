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
 #  Last Modified: Mon, 2nd Feb 2026                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#ifndef X_RANGING_H
#define X_RANGING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum laser_sensors {
  LEFT = 0,
  RIGHT,
  FRONT,
  BACK,
  EXTERNAL
} laser_e;

class XRanging_P {
 private:
  int16_t triggerThreshold [ 5 ] = { -1, -1, -1, -1, -1 };

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
   * @brief Retrieves the range measurement for a specified laser module.
   *
   * This function checks if the specified laser module is initialized. If it is,
   * it iterates through available lasers to find the matching one and initiates
   * a ranging process to obtain the distance measurement. Returns -1 if the laser
   * is not initialized or not found.
   *
   * @param laser The enumeration value representing the laser module to measure. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @return The range measurement as an `int16_t` if successful, otherwise -1.
   */
  int16_t getRange ( laser_e laser );

  /**
   * @brief Initializes the object avoidance system with specified parameters.
   *
   * This function sets the object avoidance distance to `_avoidDist` and initializes
   * up to four lasers for obstacle detection. Each laser is enabled by setting its
   * corresponding entry in `isXLaserInit` to `true`, provided it is a valid laser
   * identifier (non-negative).
   *
   * @param _avoidDist The distance ( in `cm`) parameter for triggering object avoidance.
   * @param _laser1 The first laser to be initialized for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @param _laser2 The second laser to be initialized for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @param _laser3 The third laser to be initialized for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @param _laser4 The fourth laser to be initialized for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   */
  void initObjectAvoidance ( uint16_t _avoidDist, laser_e _laser1, laser_e _laser2 = ( laser_e ) -1, laser_e _laser3 = ( laser_e ) -1, laser_e _laser4 = ( laser_e ) -1 );
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