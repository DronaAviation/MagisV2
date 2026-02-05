/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\XRanging.cpp                                       #
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

#include "API/API-Utils.h"
#include "API/Peripherals.h"
#include "API/XRanging.h"
#include "io/rc_controls.h"
#include "common/maths.h"

// Laser sensor instances
static LaserSensor laserLEFT;
static LaserSensor laserRIGHT;
static LaserSensor laserFRONT;
static LaserSensor laserBACK;
static LaserSensor laserEXTERNAL;

/**
 * @brief Mapping structure between logical laser IDs and hardware.
 *
 * This structure maps:
 *  - Logical laser ID (LEFT, RIGHT, FRONT, BACK, EXTERNAL)
 *  - Status LED GPIO
 *  - XSHUT GPIO
 *  - Associated LaserSensor object
 *
 * This enables table-driven logic instead of switch-case handling.
 */
struct LaserMap {
  laser_e id;
  peripheral_gpio_pin_e ledGpio;
  peripheral_gpio_pin_e xshutGpio;
  LaserSensor *sensor;
};

// Laser mapping table
static LaserMap lasers [] = {
  { LEFT, GPIO_6, GPIO_7, &laserLEFT },
  { RIGHT, GPIO_10, GPIO_9, &laserRIGHT },
  { FRONT, GPIO_5, GPIO_16, &laserFRONT },
  { BACK, GPIO_8, GPIO_15, &laserBACK },
  { EXTERNAL, ( peripheral_gpio_pin_e ) -1, GPIO_14, &laserEXTERNAL },
};

// Calculate the number of elements in the 'lasers' array.
static constexpr uint8_t LASER_COUNT = sizeof ( lasers ) / sizeof ( lasers [ 0 ] );

// Function to check if a given GPIO pin is valid.
static inline bool isValidGpio ( peripheral_gpio_pin_e gpio ) {
  return ( gpio >= 0 && gpio < GPIO_COUNT );
}

/**
 * @brief Initializes the ranging sensors by resetting them, bringing them up one-by-one,
 * and performing an LED animation sequence.
 *
 * This function performs the following phases:
 * 1. **Reset Phase**: Turns off all lasers and LEDs by writing STATE_LOW to their respective GPIO pins.
 * 2. **Initialization Phase**: Powers on each laser module individually, initializes it, and assigns a unique I2C address.
 * 3. **LED Animation Phase**: Toggles LEDs in a blinking pattern for visual confirmation of initialization.
 */
void xRangingInit ( void ) {

  uint8_t i2cAddress = 42;

  // Phase 1: reset
  for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
    laser_e id = lasers [ i ].id;
    if ( ! isXLaserInit [ id ] ) continue;

    Peripheral_Init ( lasers [ i ].xshutGpio, OUTPUT );
    Peripheral_Write ( lasers [ i ].xshutGpio, STATE_LOW );

    if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
      Peripheral_Init ( lasers [ i ].ledGpio, OUTPUT );
      Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );
    }
    delay ( 10 );
  }

  // Phase 2: bring up one-by-one
  for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
    laser_e id = lasers [ i ].id;
    if ( ! isXLaserInit [ id ] ) continue;

    Peripheral_Write ( lasers [ i ].xshutGpio, STATE_HIGH );
    if ( isValidGpio ( lasers [ i ].ledGpio ) )
      Peripheral_Write ( lasers [ i ].ledGpio, STATE_HIGH );

    delay ( 30 );

    lasers [ i ].sensor->init ( );
    lasers [ i ].sensor->setAddress ( i2cAddress++ );

    if ( isValidGpio ( lasers [ i ].ledGpio ) )
      Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );

    delay ( 30 );
  }

  // Phase 3: LED animation
  for ( uint8_t blink = 0; blink < 4; blink++ ) {
    for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
      if ( ! isXLaserInit [ lasers [ i ].id ] ) continue;
      if ( isValidGpio ( lasers [ i ].ledGpio ) )
        Peripheral_Write ( lasers [ i ].ledGpio, STATE_TOGGLE );
    }
    delay ( 75 );
  }
}

// ============================================================
// XRanging public API
// ============================================================

void XRanging_P::init ( void ) {
  isXLaserInit [ LEFT ]      = true;
  triggerThreshold [ LEFT ]  = 200;
  isXLaserInit [ RIGHT ]     = true;
  triggerThreshold [ RIGHT ] = 200;
  isXLaserInit [ FRONT ]     = true;
  triggerThreshold [ FRONT ] = 200;
  isXLaserInit [ BACK ]      = true;
  triggerThreshold [ BACK ]  = 200;
}

void XRanging_P::init ( laser_e laser, int16_t threshold ) {
  isXLaserInit [ laser ]     = true;
  triggerThreshold [ laser ] = threshold * 10;
}

int16_t XRanging_P::getRange ( laser_e laser ) {
  if ( ! isXLaserInit [ laser ] ) return -1;

  for ( uint8_t i = 0; i < LASER_COUNT; i++ )
    if ( lasers [ i ].id == laser )
      return lasers [ i ].sensor->startRanging ( );

  return -1;
}

bool XRanging_P::isTriggered ( laser_e laser ) {
  if ( ! isXLaserInit [ laser ] ) return false;
  if ( triggerThreshold [ laser ] < 0 ) return false;

  int16_t range = getRange ( laser );
  if ( range < 0 ) return false;

  bool triggered = ( range < triggerThreshold [ laser ] );
  if ( isValidGpio ( lasers [ laser ].ledGpio ) )
    Peripheral_Write ( lasers [ laser ].ledGpio, triggered ? STATE_HIGH : STATE_LOW );

  return triggered;
}

XRanging_P XRanging;

// ============================================================
// Object Avoidance configuration
// ============================================================
#define RC_MID                1500    // Midpoint value for RC signal, typically neutral position
#define RC_MIN                1000    // Minimum value for RC signal, lower bound
#define RC_MAX                2000    // Maximum value for RC signal, upper bound

#define AVOID_RC_MAX          50       // Maximum RC value deviation allowed during avoidance
#define OA_MAX_PUSH_TIME_MS   500      // Maximum time in milliseconds to push forward during obstacle avoidance
#define OA_CLEAR_HOLD_MS      500      // Time in milliseconds to hold clear state after an obstacle is no longer detected
#define OA_BRAKE_GAIN         0.45f    // Gain factor applied to braking during obstacle avoidance

#define USER_OA_BLEND_PUSH    0.3f    // User-defined blend factor for pushing action in obstacle avoidance
#define USER_OA_BLEND_BRAKE   0.2f    // User-defined blend factor for braking action in obstacle avoidance

#define AVOIDANCE_INTERVAL_MS 100    // Interval in milliseconds between successive obstacle avoidance checks

static uint16_t _objectAoidDistance = 0;        // Distance to the nearest detected obstacle
static uint32_t lastAvoidanceRunMs  = 0;        // Timestamp of the last obstacle avoidance logic execution
static bool _enOA                   = false;    // Flag indicating whether obstacle avoidance is enabled

/**
 * @brief Filters the input distance using a simple moving average filter.
 *
 * This function applies a weighted moving average filter to smooth out the
 * distance readings. The filter uses a fixed size array to store the
 * filtered values for different indices. If the provided distance (d) is
 * less than or equal to zero, it returns the previously filtered value
 * for that index without updating.
 *
 * @param idx Index of the filter array to update and return the filtered value.
 * @param d   New distance measurement to be filtered.
 * @return    The filtered distance value at the specified index.
 */
static int16_t filterDistance ( uint8_t idx, int16_t d ) {
  static int16_t dFilt [ 5 ] = { 0 };
  if ( d <= 0 ) return dFilt [ idx ];
  dFilt [ idx ] = ( dFilt [ idx ] * 3 + d ) / 4;
  return dFilt [ idx ];
}

/**
 * @brief Calculates the remote control offset for object avoidance based on distance.
 *
 * This function determines the offset to be applied for object avoidance,
 * depending on the input distance and an inversion flag. If the distance is
 * less than or equal to zero, or greater than a predefined threshold (_objectAoidDistance),
 * it returns zero. The offset is inverted if the 'invert' parameter is true.
 *
 * @param distance  The measured distance to the object.
 * @param invert    Boolean flag indicating whether to invert the offset direction.
 * @return          The calculated remote control offset for object avoidance.
 */
static int16_t mapAvoidance ( int16_t distance, bool invert ) {
  if ( distance <= 0 ) return 0;
  if ( distance > _objectAoidDistance ) return 0;

  int16_t rcOffset = AVOID_RC_MAX;
  if ( invert ) rcOffset = -rcOffset;
  return rcOffset;
}

void XRanging_P::enableOA ( void ) {
  _enOA = true;
}

void XRanging_P::disableOA ( void ) {
  _enOA = false;
  for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
    if ( ! isXLaserInit [ lasers [ i ].id ] ) continue;
    if ( isValidGpio ( lasers [ i ].ledGpio ) )
      Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );
  }
}

void XRanging_P::initObjectAvoidance ( uint16_t _avoidDist, laser_e _laser1, laser_e _laser2, laser_e _laser3, laser_e _laser4 ) {
  auto enableLaser = [ & ] ( laser_e l ) {
    if ( l < 0 ) return;
    isXLaserInit [ l ] = true;
  };

  _objectAoidDistance = _avoidDist * 10;
  enableLaser ( _laser1 );
  enableLaser ( _laser2 );
  enableLaser ( _laser3 );
  enableLaser ( _laser4 );
}

/**
 * @brief Applies object avoidance logic to adjust roll and pitch remote control values.
 *
 * This function periodically checks the status of laser distance sensors to determine
 * if any objects are within a predefined range. It calculates offsets for roll and pitch
 * based on sensor input, adjusting the remote control values accordingly to avoid obstacles.
 * The function ensures that object avoidance is only applied at specified intervals and
 * blends user inputs with calculated offsets for smooth operation.
 */
void applyObjectAvoidance ( void ) {

  // Get the current time in milliseconds
  uint32_t now = millis ( );

  // Check if the avoidance logic should run based on the predefined interval
  if ( now - lastAvoidanceRunMs < AVOIDANCE_INTERVAL_MS )
    return;

  // Update the last run timestamp
  lastAvoidanceRunMs = now;

  // Return if object avoidance is not enabled
  if ( ! _enOA ) return;

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
  if ( isXLaserInit [ LEFT ] ) {
    int16_t o = mapAvoidance ( filterDistance ( 0, XRanging.getRange ( LEFT ) ), false );
    rollSense |= ( o != 0 );
    Peripheral_Write ( lasers [ LEFT ].ledGpio, o ? STATE_HIGH : STATE_LOW );
    rollOffset += o;
  }

  // Check and process right laser sensor
  if ( isXLaserInit [ RIGHT ] ) {
    int16_t o = mapAvoidance ( filterDistance ( 1, XRanging.getRange ( RIGHT ) ), true );
    rollSense |= ( o != 0 );
    Peripheral_Write ( lasers [ RIGHT ].ledGpio, o ? STATE_HIGH : STATE_LOW );
    rollOffset += o;
  }

  // Check and process front laser sensor
  if ( isXLaserInit [ FRONT ] ) {
    int16_t o = mapAvoidance ( filterDistance ( 2, XRanging.getRange ( FRONT ) ), true );
    pitchSense |= ( o != 0 );
    Peripheral_Write ( lasers [ FRONT ].ledGpio, o ? STATE_HIGH : STATE_LOW );
    pitchOffset += o;
  }

  // Check and process back laser sensor
  if ( isXLaserInit [ BACK ] ) {
    int16_t o = mapAvoidance ( filterDistance ( 3, XRanging.getRange ( BACK ) ), false );
    pitchSense |= ( o != 0 );
    Peripheral_Write ( lasers [ BACK ].ledGpio, o ? STATE_HIGH : STATE_LOW );
    pitchOffset += o;
  }

  // Static variables to track the last known state for roll and pitch avoidance
  static uint32_t rollOAStartTs  = 0;
  static uint32_t pitchOAStartTs = 0;
  static int16_t lastRollOA      = 0;
  static int16_t lastPitchOA     = 0;

  // If a roll obstacle is sensed, update the start timestamp and last offset
  if ( rollSense ) {
    if ( rollOAStartTs == 0 ) rollOAStartTs = now;
    lastRollOA = rollOffset;
  }

  // If a pitch obstacle is sensed, update the start timestamp and last offset
  if ( pitchSense ) {
    if ( pitchOAStartTs == 0 ) pitchOAStartTs = now;
    lastPitchOA = pitchOffset;
  }

  // Adjust roll control based on the object avoidance state
  if ( rollOAStartTs && ( now - rollOAStartTs < OA_MAX_PUSH_TIME_MS ) )
    RC_ARRAY [ ROLL ] = ( userRoll * USER_OA_BLEND_PUSH ) + rollOffset;
  else if ( rollOAStartTs && ( now - rollOAStartTs < OA_MAX_PUSH_TIME_MS + OA_CLEAR_HOLD_MS ) )
    RC_ARRAY [ ROLL ] = ( userRoll * USER_OA_BLEND_BRAKE ) + ( -lastRollOA * OA_BRAKE_GAIN );
  else {
    rollOAStartTs     = 0;
    lastRollOA        = 0;
    RC_ARRAY [ ROLL ] = userRoll * USER_OA_BLEND_PUSH;
  }

  // Adjust pitch control based on the object avoidance state
  if ( pitchOAStartTs && ( now - pitchOAStartTs < OA_MAX_PUSH_TIME_MS ) )
    RC_ARRAY [ PITCH ] = ( userPitch * USER_OA_BLEND_PUSH ) + pitchOffset;
  else if ( pitchOAStartTs && ( now - pitchOAStartTs < OA_MAX_PUSH_TIME_MS + OA_CLEAR_HOLD_MS ) )
    RC_ARRAY [ PITCH ] = ( userPitch * USER_OA_BLEND_BRAKE ) + ( -lastPitchOA * OA_BRAKE_GAIN );
  else {
    pitchOAStartTs     = 0;
    lastPitchOA        = 0;
    RC_ARRAY [ PITCH ] = userPitch * USER_OA_BLEND_PUSH;
  }

  // Constrain the roll and pitch values within acceptable limits
  RC_ARRAY [ ROLL ]  = constrain ( RC_ARRAY [ ROLL ], -500, 500 );
  RC_ARRAY [ PITCH ] = constrain ( RC_ARRAY [ PITCH ], -500, 500 );

  // Set flags indicating that the roll and pitch values have been updated
  userRCflag [ ROLL ]  = true;
  userRCflag [ PITCH ] = true;
}

// #include "API/API-Utils.h"
// #include "API/Peripherals.h"
// #include "API/XRanging.h"
// #include "io/rc_controls.h"
// #include "common/maths.h"

//           /* ============================================================
//            * Laser sensor instances
//            * ============================================================ */
//           static LaserSensor laserLEFT;
// static LaserSensor laserRIGHT;
// static LaserSensor laserFRONT;
// static LaserSensor laserBACK;
// static LaserSensor laserEXTERNAL;

// /* ============================================================
//  * Laser mapping table
//  * - Defines GPIOs and sensor object per logical laser
//  * - This enables table-driven logic instead of switch-case hell
//  * ============================================================ */
// struct LaserMap {
//   laser_e id;                         // Logical laser ID
//   peripheral_gpio_pin_e ledGpio;      // Status LED GPIO (-1 if unused)
//   peripheral_gpio_pin_e xshutGpio;    // XSHUT GPIO
//   LaserSensor *sensor;                // Sensor object
// };

// static LaserMap lasers [] = {
//   { LEFT, GPIO_6, GPIO_7, &laserLEFT },
//   { RIGHT, GPIO_10, GPIO_9, &laserRIGHT },
//   { FRONT, GPIO_5, GPIO_16, &laserFRONT },
//   { BACK, GPIO_8, GPIO_15, &laserBACK },
//   { EXTERNAL, ( peripheral_gpio_pin_e ) -1, GPIO_14, &laserEXTERNAL },
// };

// static constexpr uint8_t LASER_COUNT = sizeof ( lasers ) / sizeof ( lasers [ 0 ] );

// /* ============================================================
//  * Helper: validate GPIO before touching hardware
//  * ============================================================ */
// static inline bool isValidGpio ( peripheral_gpio_pin_e gpio ) {
//   return ( gpio >= 0 && gpio < GPIO_COUNT );
// }

// /* ============================================================
//  * XRanging low-level init
//  * - Powers up sensors one-by-one
//  * - Assigns unique I2C addresses
//  * ============================================================ */
// void xRangingInit ( void ) {

//   uint8_t i2cAddress = 42;

//   // Phase 1: Reset all enabled sensors (XSHUT LOW)
//   for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
//     laser_e id = lasers [ i ].id;
//     if ( ! isXLaserInit [ id ] ) continue;

//     Peripheral_Init ( lasers [ i ].xshutGpio, OUTPUT );
//     Peripheral_Write ( lasers [ i ].xshutGpio, STATE_LOW );

//     if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
//       Peripheral_Init ( lasers [ i ].ledGpio, OUTPUT );
//       Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );
//     }

//     delay ( 10 );
//   }

//   // Phase 2: Bring up sensors one at a time
//   //           → init
//   //           → assign unique I2C address
//   for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
//     laser_e id = lasers [ i ].id;
//     if ( ! isXLaserInit [ id ] ) continue;

//     Peripheral_Write ( lasers [ i ].xshutGpio, STATE_HIGH );
//     if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
//       Peripheral_Write ( lasers [ i ].ledGpio, STATE_HIGH );
//     }

//     delay ( 30 );

//     lasers [ i ].sensor->init ( );
//     lasers [ i ].sensor->setAddress ( i2cAddress++ );

//     if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
//       Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );
//     }

//     delay ( 30 );
//   }

//   // Phase 3: Startup LED animation (LEFT/RIGHT/FRONT/BACK)
//   for ( uint8_t blink = 0; blink < 4; blink++ ) {
//     for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
//       if ( ! isXLaserInit [ lasers [ i ].id ] ) continue;
//       if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
//         Peripheral_Write ( lasers [ i ].ledGpio, STATE_TOGGLE );
//       }
//     }
//     delay ( 75 );
//   }
// }

// /* ============================================================
//  * XRanging public API
//  * ============================================================ */
// void XRanging_P::init ( void ) {
//   isXLaserInit [ LEFT ]      = true;
//   triggerThreshold [ LEFT ]  = 200;
//   isXLaserInit [ RIGHT ]     = true;
//   triggerThreshold [ RIGHT ] = 200;
//   isXLaserInit [ FRONT ]     = true;
//   triggerThreshold [ FRONT ] = 200;
//   isXLaserInit [ BACK ]      = true;
//   triggerThreshold [ BACK ]  = 200;
// }

// void XRanging_P::init ( laser_e laser, int16_t threshold ) {
//   isXLaserInit [ laser ]     = true;
//   triggerThreshold [ laser ] = threshold;
// }

// /* ============================================================
//  * Read distance from a laser
//  * - Returns -1 if laser is disabled
//  * ============================================================ */
// int16_t XRanging_P::getRange ( laser_e laser ) {

//   if ( ! isXLaserInit [ laser ] ) return -1;

//   for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
//     if ( lasers [ i ].id == laser ) {
//       return lasers [ i ].sensor->startRanging ( );
//     }
//   }

//   return -1;
// }

// /* ============================================================
//  * Trigger logic
//  * - Compares range against configured threshold
//  * - Updates LED state
//  * ============================================================ */
// bool XRanging_P::isTriggered ( laser_e laser ) {

//   if ( ! isXLaserInit [ laser ] ) return false;
//   if ( triggerThreshold [ laser ] < 0 ) return false;

//   int16_t range = getRange ( laser );
//   if ( range < 0 ) return false;

//   bool triggered = ( range < triggerThreshold [ laser ] );

//   if ( isValidGpio ( lasers [ laser ].ledGpio ) ) {
//     Peripheral_Write ( lasers [ laser ].ledGpio, triggered ? STATE_HIGH : STATE_LOW );
//   }

//   return triggered;
// }

// /* ============================================================
//  * Singleton instance
//  * ============================================================ */
// XRanging_P XRanging;

// #define RC_MID                1500
// #define RC_MIN                1000
// #define RC_MAX                2000

// #define AVOID_RC_MAX          50    // max RC offset

// #define OA_MAX_PUSH_TIME_MS   500    // max time OA is allowed to push
// #define OA_CLEAR_HOLD_MS      300
// #define OA_BRAKE_GAIN         0.35f

// #define USER_OA_BLEND_PUSH    0.3f    // user authority during OA push
// #define USER_OA_BLEND_BRAKE   0.2f    // even less during braking

// #define AVOIDANCE_INTERVAL_MS 100

// static uint16_t _objectAoidDistance = 0;
// static uint32_t lastAvoidanceRunMs  = 0;

// static int16_t filterDistance ( uint8_t idx, int16_t d ) {
//   static int16_t dFilt [ 5 ] = { 0 };

//   if ( d <= 0 ) return dFilt [ idx ];
//   dFilt [ idx ] = ( dFilt [ idx ] * 3 + d ) / 4;
//   return dFilt [ idx ];
// }

// static int16_t mapAvoidance ( int16_t distance, bool invert ) {

//   if ( distance <= 0 )
//     return 0;

//   // Full OA as soon as object is within 400 mm
//   if ( distance > _objectAoidDistance )
//     return 0;

//   int16_t rcOffset = AVOID_RC_MAX;

//   if ( invert )
//     rcOffset = -rcOffset;

//   return rcOffset;
// }

// bool _enOA = false;

// void enableOA ( ) {
//   _enOA = true;
// }
// void disableOA ( ) {
//   _enOA = false;
//   for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
//     if ( ! isXLaserInit [ lasers [ i ].id ] ) continue;
//     if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
//       Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );
//     }
//   }
// }

// void XRanging_P::initObjectAvoidanceBraking ( uint16_t _avoidDist, laser_e _laser1, laser_e _laser2, laser_e _laser3, laser_e _laser4 ) {
//   // Helper macro to enable a laser safely
//   auto enableLaser = [ & ] ( laser_e l ) {
//     if ( l < 0 )
//       return;

//     isXLaserInit [ l ] = true;
//   };
//   _objectAoidDistance = _avoidDist;
//   // Enable only requested lasers
//   enableLaser ( _laser1 );
//   enableLaser ( _laser2 );
//   enableLaser ( _laser3 );
//   enableLaser ( _laser4 );
// }

// void applyObjectAvoidance ( void ) {

//   uint32_t now = millis ( );
//   if ( now - lastAvoidanceRunMs < AVOIDANCE_INTERVAL_MS )
//     return;
//   lastAvoidanceRunMs = now;

//   if ( ! _enOA )
//     return;

//   /* ---------------- User RC (centered) ---------------- */
//   int16_t userRoll  = rcData [ ROLL ] - 1500;
//   int16_t userPitch = rcData [ PITCH ] - 1500;

//   int16_t rollOffset  = 0;
//   int16_t pitchOffset = 0;

//   bool rollSense  = false;
//   bool pitchSense = false;

//   /* ---------------- Sensors ---------------- */
//   if ( isXLaserInit [ LEFT ] ) {
//     int16_t o = mapAvoidance (
//               filterDistance ( 0, XRanging.getRange ( LEFT ) ), false );
//     if ( o ) rollSense = true;
//     Peripheral_Write ( lasers [ LEFT ].ledGpio, o ? STATE_HIGH : STATE_LOW );
//     rollOffset += o;
//   }

//   if ( isXLaserInit [ RIGHT ] ) {
//     int16_t o = mapAvoidance (
//               filterDistance ( 1, XRanging.getRange ( RIGHT ) ), true );
//     if ( o ) rollSense = true;
//     Peripheral_Write ( lasers [ RIGHT ].ledGpio, o ? STATE_HIGH : STATE_LOW );
//     rollOffset += o;
//   }

//   if ( isXLaserInit [ FRONT ] ) {
//     int16_t o = mapAvoidance (
//               filterDistance ( 2, XRanging.getRange ( FRONT ) ), true );
//     if ( o ) pitchSense = true;
//     Peripheral_Write ( lasers [ FRONT ].ledGpio, o ? STATE_HIGH : STATE_LOW );
//     pitchOffset += o;
//   }

//   if ( isXLaserInit [ BACK ] ) {
//     int16_t o = mapAvoidance (
//               filterDistance ( 3, XRanging.getRange ( BACK ) ), false );
//     if ( o ) pitchSense = true;
//     Peripheral_Write ( lasers [ BACK ].ledGpio, o ? STATE_HIGH : STATE_LOW );
//     pitchOffset += o;
//   }

//   /* ---------------- Latch OA timing & force ---------------- */
//   static uint32_t rollOAStartTs  = 0;
//   static uint32_t pitchOAStartTs = 0;
//   static int16_t lastRollOA      = 0;
//   static int16_t lastPitchOA     = 0;

//   if ( rollSense ) {
//     if ( rollOAStartTs == 0 ) rollOAStartTs = now;
//     lastRollOA = rollOffset;
//   }

//   if ( pitchSense ) {
//     if ( pitchOAStartTs == 0 ) pitchOAStartTs = now;
//     lastPitchOA = pitchOffset;
//   }

//   /* ========================== ROLL ========================== */
//   if ( rollOAStartTs != 0 && ( now - rollOAStartTs < OA_MAX_PUSH_TIME_MS ) ) {

//     // PUSH phase
//     RC_ARRAY [ ROLL ] = ( userRoll * USER_OA_BLEND_PUSH ) + rollOffset;
//   } else if ( rollOAStartTs != 0 && ( now - rollOAStartTs < OA_MAX_PUSH_TIME_MS + OA_CLEAR_HOLD_MS ) ) {

//     // BRAKE phase
//     RC_ARRAY [ ROLL ] = ( userRoll * USER_OA_BLEND_BRAKE ) + ( -lastRollOA * OA_BRAKE_GAIN );
//   } else {
//     // OA finished
//     rollOAStartTs     = 0;
//     lastRollOA        = 0;
//     RC_ARRAY [ ROLL ] = userRoll;
//   }

//   /* ========================== PITCH ========================== */
//   if ( pitchOAStartTs != 0 && ( now - pitchOAStartTs < OA_MAX_PUSH_TIME_MS ) ) {

//     // PUSH phase
//     RC_ARRAY [ PITCH ] = ( userPitch * USER_OA_BLEND_PUSH ) + pitchOffset;
//   } else if ( pitchOAStartTs != 0 && ( now - pitchOAStartTs < OA_MAX_PUSH_TIME_MS + OA_CLEAR_HOLD_MS ) ) {

//     // BRAKE phase
//     RC_ARRAY [ PITCH ] = ( userPitch * USER_OA_BLEND_BRAKE ) + ( -lastPitchOA * OA_BRAKE_GAIN );
//   } else {
//     // OA finished
//     pitchOAStartTs     = 0;
//     lastPitchOA        = 0;
//     RC_ARRAY [ PITCH ] = userPitch;
//   }

//   /* ---------------- Clamp & commit ---------------- */
//   RC_ARRAY [ ROLL ]  = constrain ( RC_ARRAY [ ROLL ], -500, 500 );
//   RC_ARRAY [ PITCH ] = constrain ( RC_ARRAY [ PITCH ], -500, 500 );

//   userRCflag [ ROLL ]  = true;
//   userRCflag [ PITCH ] = true;
// }
