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
 #  Last Modified: Wed, 14th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-14	AJ	initAlt / getStatus / DOWN, bounds checks, XSHUT beats LED  #
*******************************************************************************/

#include "API/API-Utils.h"
#include "API/Peripherals.h"
#include "API/XRanging.h"
#include "io/rc_controls.h"
#include "common/maths.h"
#include "drivers/tof_manager.h"
#include "drivers/ranging_vl53l0x.h"    // VL53L0X_BOOT_ADDRESS, vl53l0xProbe ( )
#include "drivers/bus_spi.h"           // spiRelease ( ): SPI2 shares GPIO_5-8
#include "drivers/flash_m25p16.h"      // m25p16_release ( )

// flight/altitudehold.h ( C linkage ); not included: it needs the mixer / PID headers first.
extern "C" void altLaserFusionLatch ( bool on );

// laser_e is used as a tofId_e index.
static_assert ( ( int ) LEFT == ( int ) TOF_LEFT && ( int ) RIGHT == ( int ) TOF_RIGHT && ( int ) FRONT == ( int ) TOF_FRONT
                    && ( int ) BACK == ( int ) TOF_BACK && ( int ) EXTERNAL == ( int ) TOF_EXTERNAL && ( int ) DOWN == ( int ) TOF_DOWN,
                "laser_e and tofId_e must match" );
#define XLASER_ID_COUNT ( ( uint8_t ) DOWN + 1 )    // laser_e values, DOWN included
static_assert ( sizeof ( isXLaserInit ) / sizeof ( isXLaserInit [ 0 ] ) == XLASER_ID_COUNT, "isXLaserInit: one entry per laser_e" );

/**
 * @brief Mapping structure between the side lasers and their pins.
 *
 * This structure maps:
 *  - Logical laser ID (LEFT, RIGHT, FRONT, BACK, EXTERNAL)
 *  - Status LED GPIO
 *  - XSHUT GPIO
 *
 * Each laser's device, status and results live in drivers/tof_manager ( one entry per laser_e,
 * DOWN included ). The DOWN laser's XSHUT comes from initAlt ( ), not from this table.
 */
struct LaserMap {
  laser_e id;
  peripheral_gpio_pin_e ledGpio;
  peripheral_gpio_pin_e xshutGpio;
};

// Laser mapping table: entry i is laser_e i ( isTriggered ( ) and the OA LED writes index it that way ).
static constexpr LaserMap lasers [] = {
  { LEFT, GPIO_6, GPIO_7 },
  { RIGHT, GPIO_10, GPIO_9 },
  { FRONT, GPIO_5, GPIO_16 },
  { BACK, GPIO_8, GPIO_15 },
  { EXTERNAL, ( peripheral_gpio_pin_e ) -1, GPIO_14 },
};

// Calculate the number of elements in the 'lasers' array.
static constexpr uint8_t LASER_COUNT = sizeof ( lasers ) / sizeof ( lasers [ 0 ] );
static_assert ( LASER_COUNT == ( uint8_t ) DOWN, "one table entry per side laser" );
static_assert ( lasers [ 0 ].id == LEFT && lasers [ 1 ].id == RIGHT && lasers [ 2 ].id == FRONT && lasers [ 3 ].id == BACK
                    && lasers [ 4 ].id == EXTERNAL,
                "lasers [ i ] must be laser_e i" );

// Function to check if a given GPIO pin is valid.
static inline bool isValidGpio ( peripheral_gpio_pin_e gpio ) {
  return ( gpio >= 0 && gpio < GPIO_COUNT );
}

// ---- Down-laser request ( initAlt ) ----
static bool altRequested                  = false;                            // an initAlt ( ) call was accepted
static int8_t altXshutGpio                = -1;                               // its XSHUT pin ( peripheral_gpio_pin_e ), -1 = no wire
static bool altLatched                    = false;                            // the bring-up ran; initAlt is closed
static bool altEnabled                    = false;                            // final, after the latch check

// GPIOs that are also an ADC or PWM pin ( PIN_MAP.md ); -1 = none.
struct GpioAlias {
  int8_t adc;    // peripheral_adc_pin
  int8_t pwm;    // peripheral_pwm_pin_e
};
static const GpioAlias gpioAlias [ GPIO_COUNT ] = {
  [GPIO_1]  = { -1, PWM_1 },     [GPIO_2]  = { ADC_1, -1 },    [GPIO_3]  = { -1, PWM_2 },
  [GPIO_4]  = { -1, PWM_3 },     [GPIO_5]  = { ADC_2, PWM_4 }, [GPIO_6]  = { ADC_3, PWM_5 },
  [GPIO_7]  = { ADC_4, -1 },     [GPIO_8]  = { ADC_5, -1 },    [GPIO_9]  = { ADC_6, -1 },
  [GPIO_10] = { -1, PWM_6 },     [GPIO_11] = { -1, PWM_7 },    [GPIO_12] = { -1, PWM_8 },
  [GPIO_13] = { -1, PWM_9 },     [GPIO_14] = { ADC_7, -1 },    [GPIO_15] = { -1, -1 },
  [GPIO_16] = { -1, PWM_10 },    [GPIO_17] = { ADC_8, -1 },    [GPIO_18] = { ADC_9, -1 },
};

// GPIOs another driver can take over at any time, also after start-up, without asking XRanging:
// the RGB-LED data slots ( RGB_1 … RGB_8: PA8, PB6, PB14, PA13, PA14, PB4, PB5, PA15 ) and UART2
// ( PA2 / PA3: user Uart_Init and the UWB link ). A timer or UART line on an XSHUT resets the laser.
static const int8_t altReservedGpio [] = { GPIO_1, GPIO_3, GPIO_6, GPIO_10, GPIO_11, GPIO_12, GPIO_13, GPIO_16, GPIO_17, GPIO_18 };

// True if the down-laser cannot have this XSHUT pin: invalid, a side laser's XSHUT ( enabled or
// not: user code may enable it later ), an RGB / UART2 pin, or in use as ADC / PWM.
static bool altPinConflict ( int8_t gpio ) {
  if ( gpio < 0 || gpio >= ( int8_t ) GPIO_COUNT ) return true;
  for ( uint8_t i = 0; i < LASER_COUNT; i++ )
    if ( ( int ) lasers [ i ].xshutGpio == gpio ) return true;
  for ( uint8_t i = 0; i < ( uint8_t ) sizeof ( altReservedGpio ); i++ )
    if ( altReservedGpio [ i ] == gpio ) return true;
  const GpioAlias &alias = gpioAlias [ gpio ];
  if ( alias.adc >= 0 && adcPinEnabled ( ( uint8_t ) alias.adc ) ) return true;
  if ( alias.pwm >= 0 && isPwmInit [ alias.pwm ] ) return true;
  return false;
}

// An XSHUT claim beats an LED: a LOW on the down-laser's XSHUT resets it and wipes its address,
// so no LED write may reach that pin ( GPIO_5 is FRONT's LED and the down XSHUT on this board ).
static inline bool isLedUsable ( peripheral_gpio_pin_e led ) {
  return isValidGpio ( led ) && ( int ) led != altXshutGpio;    // altXshutGpio -1 matches no LED
}

static void ledInit ( uint8_t i ) {
  if ( isLedUsable ( lasers [ i ].ledGpio ) ) {
    Peripheral_Init ( lasers [ i ].ledGpio, OUTPUT );
    Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );
  }
}

static void ledWrite ( uint8_t i, GPIO_State_e state ) {
  if ( isLedUsable ( lasers [ i ].ledGpio ) ) Peripheral_Write ( lasers [ i ].ledGpio, state );
}

bool xRangingAltLatch ( void ) {
  if ( ! altLatched ) {
    altLatched = true;
    // Re-check the pin now that all of plutoInit ( ) has run ( ADC / PWM may be enabled after initAlt ).
    altEnabled = altRequested && ( altXshutGpio < 0 || ! altPinConflict ( altXshutGpio ) );
    if ( ! altEnabled ) {
      altRequested          = false;
      isXLaserInit [ DOWN ] = false;
    }
  }
  return altEnabled;
}

int8_t xRangingAltXshut ( void ) {
  return altEnabled ? altXshutGpio : ( int8_t ) -1;
}

bool xRangingGpioClaimed ( uint8_t gpioPin ) {
  const int8_t x = xRangingAltXshut ( );
  return x >= 0 && gpioPin == ( uint8_t ) x;
}

bool xRangingAdcClaimed ( uint8_t adcPin ) {
  const int8_t x = xRangingAltXshut ( );
  return x >= 0 && gpioAlias [ x ].adc >= 0 && adcPin == ( uint8_t ) gpioAlias [ x ].adc;
}

bool xRangingPwmClaimed ( uint8_t pwmPin ) {
  const int8_t x = xRangingAltXshut ( );
  return x >= 0 && gpioAlias [ x ].pwm >= 0 && pwmPin == ( uint8_t ) gpioAlias [ x ].pwm;
}

#define SIDE_FIRST_ADDRESS 0x2A    // side lasers get 0x2A + n, n = enabled side lasers before it in the table
#define XSHUT_BOOT_MS      30      // after XSHUT goes high ( the VL53L0X boots in <= 1.2 ms )

/**
 * @brief The one ToF bring-up ( tof-single-pipeline ). Runs once, after plutoInit ( ), so every
 * XRanging request is known.
 *
 * 1. **Latch**: the initAlt ( ) request and the altitude-laser fusion are fixed for the run. An
 *    accepted initAlt keeps the fusion on even if the laser then fails to start ( a dropout, D6 ).
 *    If any laser is requested, SPI2 ( which shares GPIO_5-8 ) and the M25P16 flash are switched off.
 * 2. **Reset**: every enabled laser is held in reset ( XSHUT low ), so nothing answers at 0x29.
 * 3. **DOWN alone**: released ( XSHUT wire ) or found ( no wire: its moved address first, which it
 *    keeps across a reset without power loss, then 0x29 ), and moved to TOF_DOWN_ADDRESS.
 * 4. **Side lasers one at a time**: each boots alone at 0x29 and moves to 0x2A + n. Once all are up,
 *    the side lasers start continuous ranging.
 * 5. **LED animation**.
 *
 * A laser that fails is marked BOOT_FAILED in the manager and, if it has an XSHUT, held in reset so
 * the next one is alone at 0x29. Before each release 0x29 must be silent: a fitted laser whose XSHUT
 * nothing drives ( not enabled; PA15 has a pull-up, PB13 is pulled up once SPI2 is released ) would answer too
 * and be re-addressed with it. Then that release is skipped ( BOOT_FAILED, VL53L0X_STEP_BUS_BUSY ).
 * A down-laser without a wire cannot be checked this way: it is at 0x29 itself. Nothing is retried.
 */
void xRangingInit ( void ) {

  const bool altOn = xRangingAltLatch ( );
  altLaserFusionLatch ( altOn );

  // SPI2's pins are GPIO_5-8 ( PB15 MOSI, PB14 MISO, PB13 SCK, PB12 NSS ). Whenever XRanging is used,
  // SPI2 is switched off for the run before any laser pin is touched: an SPI line resting low would
  // hold an XSHUT in reset. The user SPI API and the M25P16 flash stop working ( XRanging wins ).
  bool xRangingUsed = false;
  for ( uint8_t id = 0; id < XLASER_ID_COUNT; id++ ) xRangingUsed = xRangingUsed || isXLaserInit [ id ];
  if ( xRangingUsed ) {
#ifdef USE_SPI_DEVICE_2
    spiRelease ( SPI2 );
#endif
#ifdef USE_FLASH_M25P16
    m25p16_release ( );
#endif
  }

  tofManagerReset ( );
  const int8_t downXshut = xRangingAltXshut ( );    // -1 = no XSHUT wire

  // Phase 1: reset
  for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
    laser_e id = lasers [ i ].id;
    if ( ! isXLaserInit [ id ] ) continue;

    tofManagerRequest ( ( tofId_e ) id, ( int8_t ) lasers [ i ].xshutGpio );
    Peripheral_Init ( lasers [ i ].xshutGpio, OUTPUT );
    Peripheral_Write ( lasers [ i ].xshutGpio, STATE_LOW );

    ledInit ( i );
    delay ( 10 );
  }
  if ( altOn ) {
    tofManagerRequest ( TOF_DOWN, downXshut );
    if ( downXshut >= 0 ) {
      xshutPinInit ( ( uint8_t ) downXshut );    // the claimed pin: Peripheral_Write would refuse it
      xshutPinWrite ( ( uint8_t ) downXshut, false );
    }
    delay ( 10 );
  }

  // Phase 2: the down-laser alone
  if ( altOn ) {
    if ( downXshut >= 0 ) {
      if ( vl53l0xProbe ( VL53L0X_BOOT_ADDRESS ) ) {
        tofManagerMarkFailed ( TOF_DOWN, VL53L0X_STEP_BUS_BUSY );    // stays in reset
      } else {
        xshutPinWrite ( ( uint8_t ) downXshut, true );
        delay ( XSHUT_BOOT_MS );
        if ( ! tofManagerBringUp ( TOF_DOWN, VL53L0X_BOOT_ADDRESS, TOF_DOWN_ADDRESS ) ) {
          xshutPinWrite ( ( uint8_t ) downXshut, false );    // keep a failed chip off the bus
        }
      }
    } else if ( vl53l0xProbe ( TOF_DOWN_ADDRESS ) ) {
      tofManagerBringUp ( TOF_DOWN, TOF_DOWN_ADDRESS, TOF_DOWN_ADDRESS );    // warm reset: already moved
    } else {
      tofManagerBringUp ( TOF_DOWN, VL53L0X_BOOT_ADDRESS, TOF_DOWN_ADDRESS );
    }
  }

  // Phase 3: side lasers one by one
  uint8_t i2cAddress = SIDE_FIRST_ADDRESS;
  for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
    laser_e id = lasers [ i ].id;
    if ( ! isXLaserInit [ id ] ) continue;

    if ( vl53l0xProbe ( VL53L0X_BOOT_ADDRESS ) ) {    // not alone: leave this one in reset
      tofManagerMarkFailed ( ( tofId_e ) id, VL53L0X_STEP_BUS_BUSY );
      i2cAddress++;
      continue;
    }

    Peripheral_Write ( lasers [ i ].xshutGpio, STATE_HIGH );
    ledWrite ( i, STATE_HIGH );

    delay ( XSHUT_BOOT_MS );

    if ( ! tofManagerBringUp ( ( tofId_e ) id, VL53L0X_BOOT_ADDRESS, i2cAddress ) ) {
      Peripheral_Write ( lasers [ i ].xshutGpio, STATE_LOW );    // keep a failed chip off 0x29
    }
    i2cAddress++;

    ledWrite ( i, STATE_LOW );

    delay ( XSHUT_BOOT_MS );
  }

  tofManagerStartSides ( );    // continuous ranging, now that no laser is calibrating

  // Phase 4: LED animation
  for ( uint8_t blink = 0; blink < 4; blink++ ) {
    for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
      if ( ! isXLaserInit [ lasers [ i ].id ] ) continue;
      ledWrite ( i, STATE_TOGGLE );
    }
    delay ( 75 );
  }
}

// ============================================================
// XRanging public API
// ============================================================

void XRanging_P::init ( void ) {
  static_assert ( sizeof ( triggerThreshold ) / sizeof ( triggerThreshold [ 0 ] ) == XLASER_ID_COUNT, "triggerThreshold: one entry per laser_e" );
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
  if ( ( unsigned ) laser >= LASER_COUNT ) return;    // side lasers only; DOWN comes from initAlt ( )
  isXLaserInit [ laser ]     = true;
  triggerThreshold [ laser ] = ( int16_t ) ( threshold * 10 );
}

// Records the down-laser request; gpio -1 = no XSHUT wire.
static bool altRequest ( int8_t gpio ) {
#if defined( LASER_TOF_L1x )
  ( void ) gpio;
  return false;    // the VL53L1X build already owns the one down-laser
#else
  if ( altLatched ) return false;    // only before the bring-up, i.e. from plutoInit ( )
  if ( gpio >= 0 && altPinConflict ( gpio ) ) return false;
  altRequested          = true;
  altXshutGpio          = gpio;
  isXLaserInit [ DOWN ] = true;
  return true;
#endif
}

bool XRanging_P::initAlt ( peripheral_gpio_pin_e xshutGpio ) {
  if ( ( unsigned ) xshutGpio >= ( unsigned ) GPIO_COUNT ) return false;
  return altRequest ( ( int8_t ) xshutGpio );
}

bool XRanging_P::initAlt ( void ) {
  return altRequest ( -1 );
}

laser_status_e XRanging_P::getStatus ( laser_e laser ) {
  if ( ( unsigned ) laser >= XLASER_ID_COUNT ) return LASER_NOT_ENABLED;
  const tofEntry_t *e = tofManagerEntry ( ( tofId_e ) laser );
  switch ( ( tofStatus_e ) e->status ) {
    case TOF_STATUS_OK: return e->outOfRange ? LASER_OUT_OF_RANGE : LASER_OK;
    case TOF_STATUS_BOOT_FAILED: return LASER_BOOT_FAILED;
    case TOF_STATUS_I2C_ERROR: return LASER_I2C_ERROR;
    default: return LASER_NOT_ENABLED;
  }
}

int16_t XRanging_P::getRange ( laser_e laser ) {
  if ( ( unsigned ) laser >= XLASER_ID_COUNT || ! isXLaserInit [ laser ] ) return -1;

  // The cached sample: no I2C here ( the periodic laser task polls the chips ).
  const tofEntry_t *e = tofManagerEntry ( ( tofId_e ) laser );
  return ( e->status == TOF_STATUS_OK ) ? e->rangeMm : ( int16_t ) TOF_RANGE_INVALID;
}

bool XRanging_P::isTriggered ( laser_e laser ) {
  if ( ( unsigned ) laser >= LASER_COUNT ) return false;    // side lasers only ( the LED table )
  if ( ! isXLaserInit [ laser ] ) return false;
  if ( triggerThreshold [ laser ] < 0 ) return false;

  int16_t range = getRange ( laser );
  if ( range < 0 ) return false;

  bool triggered = ( range < triggerThreshold [ laser ] );
  ledWrite ( ( uint8_t ) laser, triggered ? STATE_HIGH : STATE_LOW );

  return triggered;
}

XRanging_P XRanging;

// ============================================================
// Object Avoidance configuration
// ============================================================
#define RC_MID                1500    // Midpoint value for RC signal, typically neutral position
#define RC_MIN                1000    // Minimum value for RC signal, lower bound
#define RC_MAX                2000    // Maximum value for RC signal, upper bound

#define AVOID_RC_MAX          25       // Maximum RC value deviation allowed during avoidance
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
static int16_t dFilt [ XLASER_ID_COUNT ] = { 0 };

static int16_t filterDistance ( uint8_t idx, int16_t d ) {
  if ( d <= 0 ) return dFilt [ idx ];
  dFilt [ idx ] = ( int16_t ) ( ( dFilt [ idx ] * 3 + d ) / 4 );
  return dFilt [ idx ];
}

// OA distance for one side laser. A laser that stopped working ( boot failed, I2C error ) gives
// "no obstacle" and its filter restarts: otherwise its last distance would keep OA pushing for good.
static int16_t oaDistance ( uint8_t idx, laser_e laser ) {
  const laser_status_e st = XRanging.getStatus ( laser );
  if ( st != LASER_OK && st != LASER_OUT_OF_RANGE ) {
    dFilt [ idx ] = 0;
    return 0;
  }
  return filterDistance ( idx, XRanging.getRange ( laser ) );
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
    ledWrite ( i, STATE_LOW );
  }
}

void XRanging_P::initObjectAvoidance ( uint16_t _avoidDist, laser_e _laser1, laser_e _laser2, laser_e _laser3, laser_e _laser4 ) {
  auto enableLaser = [ & ] ( laser_e l ) {
    if ( ( unsigned ) l >= LASER_COUNT ) return;    // -1 = unused argument; DOWN is not an OA laser
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
    int16_t o = mapAvoidance ( oaDistance ( 0, LEFT ), false );
    rollSense |= ( o != 0 );
    ledWrite ( LEFT, o ? STATE_HIGH : STATE_LOW );
    rollOffset += o;
  }

  // Check and process right laser sensor
  if ( isXLaserInit [ RIGHT ] ) {
    int16_t o = mapAvoidance ( oaDistance ( 1, RIGHT ), true );
    rollSense |= ( o != 0 );
    ledWrite ( RIGHT, o ? STATE_HIGH : STATE_LOW );
    rollOffset += o;
  }

  // Check and process front laser sensor
  if ( isXLaserInit [ FRONT ] ) {
    int16_t o = mapAvoidance ( oaDistance ( 2, FRONT ), true );
    pitchSense |= ( o != 0 );
    ledWrite ( FRONT, o ? STATE_HIGH : STATE_LOW );
    pitchOffset += o;
  }

  // Check and process back laser sensor
  if ( isXLaserInit [ BACK ] ) {
    int16_t o = mapAvoidance ( oaDistance ( 3, BACK ), false );
    pitchSense |= ( o != 0 );
    ledWrite ( BACK, o ? STATE_HIGH : STATE_LOW );
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

  // Latch the override and keep it fresh against the watchdog. Pilot authority
  // is 0 here because the pilot's stick is already folded into RC_ARRAY above
  // (USER_OA_BLEND_PUSH / USER_OA_BLEND_BRAKE); letting mw.cpp cross-fade it in
  // again would both double-count the pilot and let a full stick fade the
  // obstacle push away, which is exactly when it is needed most.
  userRCassert ( ROLL, 0.0f );
  userRCassert ( PITCH, 0.0f );
}
