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
 #  Last Modified: Mon, 19th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-14	AJ	initAlt / getStatus / DOWN, bounds checks, XSHUT beats LED  #
 #  2026-10-16	AJ	Pin table shared with X1Ranging ( vl53l1x-xranging-api t3 ) #
 #  2026-10-19	AJ	Every request refused in a VL53L1X build ( t4 )             #
 #  2026-10-19	AJ	OA loop moved to ObjectAvoidanceCore.h ( t6 )               #
 #  2026-10-19	AJ	Run-time laser family; both APIs used refuses both ( t15 )  #
 #  2026-10-19	AJ	initAlt latches the VL53L0X fusion copy ( t17 )             #
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
#include "API-Src/ObjectAvoidanceCore.h"    // oaCoreRun ( ), the shared avoidance loop

// flight/altitudehold.h ( C linkage ); not included: it needs the mixer / PID headers first.
extern "C" void altLaserFusionLatch ( bool on, bool vl53l1x );

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

// X1Ranging's down XSHUT ( VL53L1X ) is reserved the same way: x1RangingAltXshut ( ) is -1 unless
// X1Ranging latched a down-laser with an XSHUT wire ( only one family runs, so at most one is set ).
bool xRangingGpioClaimed ( uint8_t gpioPin ) {
  const int8_t x  = xRangingAltXshut ( );
  const int8_t x1 = x1RangingAltXshut ( );
  if ( x1 >= 0 && gpioPin == ( uint8_t ) x1 ) return true;
  return x >= 0 && gpioPin == ( uint8_t ) x;
}

bool xRangingAdcClaimed ( uint8_t adcPin ) {
  const int8_t x  = xRangingAltXshut ( );
  const int8_t x1 = x1RangingAltXshut ( );
  if ( x1 >= 0 && gpioAlias [ x1 ].adc >= 0 && adcPin == ( uint8_t ) gpioAlias [ x1 ].adc ) return true;
  return x >= 0 && gpioAlias [ x ].adc >= 0 && adcPin == ( uint8_t ) gpioAlias [ x ].adc;
}

bool xRangingPwmClaimed ( uint8_t pwmPin ) {
  const int8_t x  = xRangingAltXshut ( );
  const int8_t x1 = x1RangingAltXshut ( );
  if ( x1 >= 0 && gpioAlias [ x1 ].pwm >= 0 && pwmPin == ( uint8_t ) gpioAlias [ x1 ].pwm ) return true;
  return x >= 0 && gpioAlias [ x ].pwm >= 0 && pwmPin == ( uint8_t ) gpioAlias [ x ].pwm;
}

// X1Ranging ( VL53L1X ) uses the same side-laser pins and down-XSHUT rules: one table, one check.
int8_t xRangingSideXshut ( uint8_t side ) {
  return ( side < LASER_COUNT ) ? ( int8_t ) lasers [ side ].xshutGpio : ( int8_t ) -1;
}

int8_t xRangingSideLed ( uint8_t side ) {
  return ( side < LASER_COUNT ) ? ( int8_t ) lasers [ side ].ledGpio : ( int8_t ) -1;
}

bool xRangingAltPinConflict ( int8_t gpio ) {
  return altPinConflict ( gpio );
}

// ---- Laser family ( vl53l1x-xranging-api t15 ) ----
static rangingFamily_e latchedFamily = RANGING_NONE;    // RANGING_NONE until rangingFamilyLatch ( )
static bool familyLatched            = false;

// Once, right after plutoInit ( ): which laser API has requests. Requests are recorded in plutoInit ( )
// whatever the family ( it is not known until here ); xRangingInit ( ) / x1RangingInit ( ) apply it.
void rangingFamilyLatch ( void ) {
  if ( familyLatched ) return;
  familyLatched = true;
  bool xUsed    = altRequested;    // any XRanging request, DOWN ( initAlt ) included
  for ( uint8_t id = 0; id < XLASER_ID_COUNT; id++ ) xUsed = xUsed || isXLaserInit [ id ];
  const bool x1Used = x1RangingRequested ( );
  if ( xUsed && x1Used ) latchedFamily = RANGING_BOTH;
  else if ( xUsed ) latchedFamily = RANGING_VL53L0X;
  else if ( x1Used ) latchedFamily = RANGING_VL53L1X;
  else latchedFamily = RANGING_NONE;
}

rangingFamily_e rangingFamily ( void ) {
  return latchedFamily;
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

  const rangingFamily_e family = rangingFamily ( );
  if ( family == RANGING_VL53L1X || family == RANGING_BOTH ) {
    // X1Ranging is used. Alone, it owns the bus and the altitude-fusion latch ( x1RangingInit ( ) ); with
    // XRanging too, both are refused. Either way XRanging starts nothing and drops its requests, so its
    // getters read -1 / LASER_NOT_ENABLED, initAlt is closed, and the fusion latch is not touched here.
    for ( uint8_t id = 0; id < XLASER_ID_COUNT; id++ ) isXLaserInit [ id ] = false;
    altRequested = false;
    altXshutGpio = -1;
    altEnabled   = false;
    altLatched   = true;
    return;
  }

  const bool altOn = xRangingAltLatch ( );
  altLaserFusionLatch ( altOn, false );

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

// Requests are recorded in plutoInit ( ) whatever the family; if X1Ranging is used too, xRangingInit ( )
// drops them at the latch ( rangingFamily ( ) RANGING_VL53L1X or RANGING_BOTH ).

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
  if ( altLatched ) return false;    // only before the bring-up, i.e. from plutoInit ( )
  if ( gpio >= 0 && altPinConflict ( gpio ) ) return false;
  altRequested          = true;
  altXshutGpio          = gpio;
  isXLaserInit [ DOWN ] = true;
  return true;
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
// Object Avoidance ( the push / brake loop and its constants: API-Src/ObjectAvoidanceCore.h )
// ============================================================
#define RC_MID 1500    // Midpoint value for RC signal, typically neutral position
#define RC_MIN 1000    // Minimum value for RC signal, lower bound
#define RC_MAX 2000    // Maximum value for RC signal, upper bound

static uint16_t _objectAoidDistance = 0;        // mm, an obstacle at or below this is avoided ( initObjectAvoidance cm * 10 )
static bool _enOA                   = false;    // Flag indicating whether obstacle avoidance is enabled

void XRanging_P::enableOA ( void ) {
  _enOA = true;    // applyObjectAvoidance ( ) runs only for RANGING_NONE / RANGING_VL53L0X ( mw.cpp )
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
 * @brief XRanging's object avoidance, once per loop from mw.cpp: the shared core
 * ( oaCoreRun, ObjectAvoidanceCore.h ) over the VL53L0X cache, the lasers set by
 * initObjectAvoidance ( ) and the side-laser LEDs.
 */
void applyObjectAvoidance ( void ) {
  oaCoreRun < OA_CORE_XRANGING > ( _enOA, _objectAoidDistance, isXLaserInit, XRanging, ledWrite );
}
