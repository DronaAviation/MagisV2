/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\X1Ranging.h                                            #
 #  Created Date: Fri, 16th Oct 2026                                           #
 #  Brief: VL53L1X laser API: the XRanging methods and laser names for an     #
 #         all-VL53L1X drone, chosen at run time by calling it in plutoInit.   #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 19th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-16	AJ	Created ( vl53l1x-xranging-api task 3 )                    #
 #  2026-10-19	AJ	No build flag: chosen by calling it; with XRanging both off #
*******************************************************************************/
#ifndef X1_RANGING_H
#define X1_RANGING_H

#include <stdint.h>

#include "API/Peripherals.h"    // peripheral_gpio_pin_e
#include "API/XRanging.h"       // laser_e, laser_status_e ( the same laser names and status values )

/**
 * VL53L1X lasers, used the same way as `XRanging` ( VL53L0X ). There is no build option: calling
 * X1Ranging from `plutoInit ( )` makes every laser on the drone a VL53L1X. Use one laser API only.
 * If `plutoInit ( )` calls both X1Ranging and XRanging, both are refused at start-up: no laser
 * starts, `getRange` returns -1, `getStatus` `LASER_NOT_ENABLED`, `isTriggered` `false`, and
 * altitude hold uses the barometer.
 *
 * Every VL53L1X ranges in Medium mode with a 45 ms budget and a new sample every 50 ms. The lasers
 * are started once, after `plutoInit ( )`, with the same pins as XRanging: XSHUT LEFT = `GPIO_7`,
 * RIGHT = `GPIO_9`, FRONT = `GPIO_16`, BACK = `GPIO_15`, EXTERNAL = `GPIO_14`. While X1Ranging is used,
 * SPI2 ( `GPIO_5` - `GPIO_8`: the user SPI API and the flash ) is switched off.
 */
class X1Ranging_P {
 public:
  /**
   * @brief Enables the VL53L1X lasers at LEFT, RIGHT, FRONT and BACK, each with a 20 cm trigger
   * threshold. Call it from `plutoInit ( )`: the lasers are started once, after it.
   */
  void init ( void );

  /**
   * @brief Enables one side VL53L1X laser with a trigger threshold.
   * @param laser The laser to enable ( `LEFT`, `RIGHT`, `FRONT`, `BACK`, `EXTERNAL` ); `DOWN` comes
   *        from `initAlt ( )` and is ignored here.
   * @param threshold The trigger threshold in `cm`, for `isTriggered ( )`.
   */
  void init ( laser_e laser, int16_t threshold = 20 );

  /**
   * @brief True while the laser's range is below its trigger threshold. Drives the laser's indicator
   * LED ( LEFT `GPIO_6`, RIGHT `GPIO_10`, FRONT `GPIO_5`, BACK `GPIO_8` ) high when triggered, low otherwise;
   * an LED on the down-laser's XSHUT pin is never driven.
   * @param laser A side laser ( `LEFT`, `RIGHT`, `FRONT`, `BACK`, `EXTERNAL` ).
   * @return `false` if the laser is not enabled, has no threshold or has no valid sample.
   */
  bool isTriggered ( laser_e laser );

  /**
   * @brief Latest range of a VL53L1X laser. Returns the cached sample: the firmware reads the lasers
   * itself ( each laser has a new sample every 50 ms; DOWN is checked every 10 ms, the side lasers
   * one per 10 ms in turn ), so this call never waits on the I2C bus.
   * @param laser `LEFT`, `RIGHT`, `FRONT`, `BACK`, `EXTERNAL`, or `DOWN` after `initAlt ( )`.
   * @return The range in mm; -100 when the laser has no valid sample; -1 if the laser is not enabled
   *         ( also when XRanging is used too ).
   */
  int16_t getRange ( laser_e laser );

  /**
   * @brief Uses the downward VL53L1X for altitude: the laser and the altitude-hold fusion together.
   * Call it only from `plutoInit ( )`. Without it the firmware holds altitude on the barometer alone.
   * The request is fixed at start-up and never changes while the drone runs.
   *
   * The pin wired to the down-laser's XSHUT lets the firmware hold the down-laser off while the
   * side lasers start. That pin then belongs to the down-laser: a side laser's LED on the same pin
   * ( `GPIO_5` is FRONT's LED ) is never driven.
   * @param xshutGpio The GPIO wired to the down-laser's XSHUT: `GPIO_2`, `GPIO_4`, `GPIO_5` or `GPIO_8`.
   * @return `false` if refused: called after `plutoInit ( )`,
   *         the pin is a side laser's XSHUT, an RGB-LED slot or UART2 pin ( GPIO_1, 3, 6, 10-13,
   *         16-18 ), or in use as ADC / PWM. The pin is checked again after `plutoInit ( )`; check
   *         `getStatus ( DOWN )`. A refused call keeps an earlier accepted request. After start-up
   *         the pin is reserved: `Peripheral_Init` / `Peripheral_Write` on it ( or on its ADC / PWM
   *         alias ) are ignored. An accepted request is still dropped at start-up if XRanging is
   *         used too ( `getStatus ( DOWN )` then reads `LASER_NOT_ENABLED` ).
   */
  bool initAlt ( peripheral_gpio_pin_e xshutGpio );

  /**
   * @brief As `initAlt ( xshutGpio )`, for a down-laser with no XSHUT wire. The firmware moves it
   * off the shared start-up address before the side lasers start.
   */
  bool initAlt ( void );

  /**
   * @brief Health of a VL53L1X laser, including `DOWN`.
   * @param laser The laser ( `LEFT`, `RIGHT`, `FRONT`, `BACK`, `EXTERNAL`, `DOWN` ).
   * @return `LASER_OK`, `LASER_OUT_OF_RANGE` ( also when no sample arrived for 160 ms ), `LASER_BOOT_FAILED`,
   *         `LASER_I2C_ERROR` or `LASER_NOT_ENABLED` ( always, when XRanging is used too ).
   */
  laser_status_e getStatus ( laser_e laser );

  /**
   * @brief Sets the object-avoidance distance and enables up to four side VL53L1X lasers for it.
   *
   * Each argument that is a side laser ( `LEFT` … `EXTERNAL` ) is enabled. `DOWN` ( the default ) and
   * -1 mean unused: the down-laser is not an avoidance laser. Call it from `plutoInit ( )` so the
   * lasers are started.
   *
   * @param _avoidDist The distance ( in `cm` ) below which an obstacle is avoided.
   * @param _laser1 The first laser for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @param _laser2 The second laser for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @param _laser3 The third laser for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   * @param _laser4 The fourth laser for object avoidance. (`LEFT`,`RIGHT`,`FRONT`,`BACK`)
   */
  void initObjectAvoidance ( uint16_t _avoidDist, laser_e _laser1, laser_e _laser2 = DOWN, laser_e _laser3 = DOWN, laser_e _laser4 = DOWN );

  /**
   * @brief Enables object avoidance on the lasers given to `initObjectAvoidance ( )`.
   * Has no effect when XRanging is used too.
   */
  void enableOA ( void );

  /**
   * @brief Disables object avoidance and turns off the LED of every enabled side laser.
   */
  void disableOA ( void );
};

// The object has no data members ( the state is file-static in X1Ranging.cpp ), so it needs no
// constructor: C++ static constructors never run in this firmware ( -nostartfiles ).
extern X1Ranging_P X1Ranging;

#endif
