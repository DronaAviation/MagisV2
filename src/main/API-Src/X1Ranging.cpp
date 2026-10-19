/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\X1Ranging.cpp                                      #
 #  Created Date: Fri, 16th Oct 2026                                           #
 #  Brief: X1Ranging ( VL53L1X ) over drivers/tof_manager_l1x: requests from   #
 #         plutoInit ( ), one latched bring-up in x1RangingInit ( ), cached    #
 #         getters. Chosen at run time by calling it; with XRanging, refused.  #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 19th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-16	AJ	Created ( vl53l1x-xranging-api task 3 )                    #
 #  2026-10-19	AJ	Object avoidance through ObjectAvoidanceCore.h ( t6 )      #
 #  2026-10-19	AJ	Every build; runs only for the VL53L1X family ( t15 )       #
 #  2026-10-19	AJ	initAlt latches the VL53L1X fusion copy ( t17 )             #
*******************************************************************************/

#include "platform.h"

#include "API/API-Utils.h"
#include "API/Peripherals.h"
#include "API/X1Ranging.h"
#include "drivers/tof_manager_l1x.h"
#include "drivers/bus_spi.h"         // spiRelease ( ): SPI2 shares GPIO_5-8
#include "drivers/flash_m25p16.h"    // m25p16_release ( )
#include "API-Src/ObjectAvoidanceCore.h"    // oaCoreRun ( ), the loop shared with XRanging

// flight/altitudehold.h ( C linkage ); not included: it needs the mixer / PID headers first.
extern "C" void altLaserFusionLatch ( bool on, bool vl53l1x );

// laser_e is used as a tofId_e index.
static_assert ( ( int ) LEFT == ( int ) TOF_LEFT && ( int ) RIGHT == ( int ) TOF_RIGHT && ( int ) FRONT == ( int ) TOF_FRONT
                    && ( int ) BACK == ( int ) TOF_BACK && ( int ) EXTERNAL == ( int ) TOF_EXTERNAL && ( int ) DOWN == ( int ) TOF_DOWN,
                "laser_e and tofId_e must match" );

#define X1LASER_ID_COUNT   ( ( uint8_t ) DOWN + 1 )    // laser_e values, DOWN included
#define X1SIDE_COUNT       ( ( uint8_t ) DOWN )        // LEFT … EXTERNAL: XRanging's pin table ( xRangingSide* ( ) )
#define SIDE_FIRST_ADDRESS 0x2A                        // side lasers get 0x2A + n, n = side lasers before it ( as XRanging.cpp SIDE_FIRST_ADDRESS )
#define XSHUT_BOOT_MS      30                          // ms after XSHUT goes high ( as XRanging.cpp XSHUT_BOOT_MS; the bring-up also polls boot )

// ---- Requests ( from plutoInit ( ) ) ----
// Own arrays, not isXLaserInit [ ]: xRangingInit ( ) and applyObjectAvoidance ( ) act on that one.
static bool x1LaserInit [ X1LASER_ID_COUNT ]          = { false, false, false, false, false, false };    // indexed by laser_e
static int16_t triggerThresholdMm [ X1LASER_ID_COUNT ] = { -1, -1, -1, -1, -1, -1 };                      // mm, -1 = none

// ---- Down-laser request ( initAlt ) ----
static bool altRequested   = false;    // an initAlt ( ) call was accepted
static int8_t altXshutGpio = -1;       // its XSHUT pin ( peripheral_gpio_pin_e ), -1 = no wire
static bool altLatched     = false;    // x1RangingInit ( ) ran; initAlt is closed
static bool altEnabled     = false;    // final, after the latch check

// ---- Object avoidance configuration ( the avoidance loop is the shared core, ObjectAvoidanceCore.h ) ----
// Side lasers in use: x1LaserInit [ LEFT … BACK ]; range and health per side: X1Ranging.getRange / getStatus.
static uint16_t oaThresholdMm = 0;        // mm, an obstacle at or below this is avoided ( initObjectAvoidance cm * 10 )
static bool oaEnabled         = false;    // enableOA ( ) / disableOA ( )

static inline bool isValidGpio ( int8_t gpio ) {
  return gpio >= 0 && gpio < ( int8_t ) GPIO_COUNT;
}

// An XSHUT claim beats an LED: a LOW on the down-laser's XSHUT resets it and wipes its address,
// so no LED write may reach that pin ( GPIO_5 is FRONT's LED ).
static inline bool isLedUsable ( int8_t led ) {
  return isValidGpio ( led ) && led != altXshutGpio;    // altXshutGpio -1 matches no LED
}

static void ledInit ( uint8_t i ) {
  const int8_t led = xRangingSideLed ( i );
  if ( isLedUsable ( led ) ) {
    Peripheral_Init ( ( peripheral_gpio_pin_e ) led, OUTPUT );
    Peripheral_Write ( ( peripheral_gpio_pin_e ) led, STATE_LOW );
  }
}

static void ledWrite ( uint8_t i, GPIO_State_e state ) {
  const int8_t led = xRangingSideLed ( i );
  if ( isLedUsable ( led ) ) Peripheral_Write ( ( peripheral_gpio_pin_e ) led, state );
}

static bool x1RangingAltLatch ( void ) {
  if ( ! altLatched ) {
    altLatched = true;
    // Re-check the pin now that all of plutoInit ( ) has run ( ADC / PWM may be enabled after initAlt ).
    altEnabled = altRequested && ( altXshutGpio < 0 || ! xRangingAltPinConflict ( altXshutGpio ) );
    if ( ! altEnabled ) {
      altRequested         = false;
      x1LaserInit [ DOWN ] = false;
    }
  }
  return altEnabled;
}

int8_t x1RangingAltXshut ( void ) {
  return altEnabled ? altXshutGpio : ( int8_t ) -1;
}

bool x1RangingRequested ( void ) {
  bool used = false;
  for ( uint8_t id = 0; id < X1LASER_ID_COUNT; id++ ) used = used || x1LaserInit [ id ];
  return used;
}

/**
 * @brief The one VL53L1X bring-up, as xRangingInit ( ) for the VL53L0X. Runs once, after
 * plutoInit ( ), so every X1Ranging request is known, and only when rangingFamily ( ) is
 * RANGING_VL53L1X ( X1Ranging used, XRanging not ); otherwise it drops X1Ranging's requests.
 *
 * 1. **Latch**: the initAlt ( ) request and the altitude-laser fusion are fixed for the run. If any
 *    laser is requested, SPI2 ( which shares GPIO_5-8 ) and the M25P16 flash are switched off.
 * 2. **Reset**: every requested laser is held in reset ( XSHUT low ), so nothing answers at 0x29.
 * 3. **DOWN alone**: released ( XSHUT wire ) or found ( no wire: its moved address first, which it
 *    keeps across a reset without power loss, then 0x29 ), and moved to TOF_DOWN_ADDRESS.
 * 4. **Side lasers one at a time**: each boots alone at 0x29 and moves to 0x2A + n. Once all are
 *    up, the side lasers start TIMED ranging.
 * 5. **LED animation**.
 *
 * Before each release 0x29 must be silent ( tofL1xProbe ): a fitted laser whose XSHUT nothing drives
 * would answer too and be re-addressed with it. That release is then skipped ( BOOT_FAILED,
 * TOF_L1X_STEP_BUS_BUSY ). A failed laser with an XSHUT is held in reset. Nothing is retried.
 */
void x1RangingInit ( void ) {
  if ( altLatched ) return;    // once

  if ( rangingFamily ( ) != RANGING_VL53L1X ) {
    // NONE / VL53L0X: X1Ranging has no requests, and xRangingInit ( ) owns the bus and the fusion latch.
    // BOTH ( XRanging used too ): both are refused, and nobody latches the fusion ( baro only ).
    // Either way X1Ranging starts nothing and drops any request, so its getters read -1 /
    // LASER_NOT_ENABLED, and initAlt is closed.
    for ( uint8_t id = 0; id < X1LASER_ID_COUNT; id++ ) x1LaserInit [ id ] = false;
    altRequested = false;
    altXshutGpio = -1;
    altEnabled   = false;
    altLatched   = true;
    return;
  }

  const bool altOn = x1RangingAltLatch ( );
  altLaserFusionLatch ( altOn, true );    // the VL53L1X fusion copy ( flight/altitudehold.cpp, AltTofL1x )

  tofL1xReset ( );
  if ( ! x1RangingRequested ( ) ) return;

  // SPI2's pins are GPIO_5-8: an SPI line resting low would hold an XSHUT in reset. The user SPI API
  // and the M25P16 flash stop working ( X1Ranging wins ), as with XRanging.
#ifdef USE_SPI_DEVICE_2
  spiRelease ( SPI2 );
#endif
#ifdef USE_FLASH_M25P16
  m25p16_release ( );
#endif

  const int8_t downXshut = x1RangingAltXshut ( );    // -1 = no XSHUT wire

  // Phase 1: reset
  for ( uint8_t i = 0; i < X1SIDE_COUNT; i++ ) {
    if ( ! x1LaserInit [ i ] ) continue;
    const int8_t xshut = xRangingSideXshut ( i );

    tofL1xRequest ( ( tofId_e ) i, xshut );
    Peripheral_Init ( ( peripheral_gpio_pin_e ) xshut, OUTPUT );
    Peripheral_Write ( ( peripheral_gpio_pin_e ) xshut, STATE_LOW );

    ledInit ( i );
    delay ( 10 );
  }
  if ( altOn ) {
    tofL1xRequest ( TOF_DOWN, downXshut );
    if ( downXshut >= 0 ) {
      xshutPinInit ( ( uint8_t ) downXshut );    // the claimed pin: Peripheral_Write would refuse it
      xshutPinWrite ( ( uint8_t ) downXshut, false );
    }
    delay ( 10 );
  }

  // Phase 2: the down-laser alone
  if ( altOn ) {
    if ( downXshut >= 0 ) {
      if ( tofL1xProbe ( TOF_L1X_BOOT_ADDRESS ) ) {
        tofL1xMarkFailed ( TOF_DOWN, TOF_L1X_STEP_BUS_BUSY );    // stays in reset
      } else {
        xshutPinWrite ( ( uint8_t ) downXshut, true );
        delay ( XSHUT_BOOT_MS );
        if ( ! tofL1xBringUp ( TOF_DOWN, TOF_L1X_BOOT_ADDRESS, TOF_DOWN_ADDRESS ) ) {
          xshutPinWrite ( ( uint8_t ) downXshut, false );    // keep a failed chip off the bus
        }
      }
    } else if ( tofL1xProbe ( TOF_DOWN_ADDRESS ) ) {
      tofL1xBringUp ( TOF_DOWN, TOF_DOWN_ADDRESS, TOF_DOWN_ADDRESS );    // warm reset: already moved
    } else {
      tofL1xBringUp ( TOF_DOWN, TOF_L1X_BOOT_ADDRESS, TOF_DOWN_ADDRESS );
    }
  }

  // Phase 3: side lasers one by one
  uint8_t i2cAddress = SIDE_FIRST_ADDRESS;
  for ( uint8_t i = 0; i < X1SIDE_COUNT; i++ ) {
    if ( ! x1LaserInit [ i ] ) continue;
    const int8_t xshut = xRangingSideXshut ( i );

    if ( tofL1xProbe ( TOF_L1X_BOOT_ADDRESS ) ) {    // not alone: leave this one in reset
      tofL1xMarkFailed ( ( tofId_e ) i, TOF_L1X_STEP_BUS_BUSY );
      i2cAddress++;
      continue;
    }

    Peripheral_Write ( ( peripheral_gpio_pin_e ) xshut, STATE_HIGH );
    ledWrite ( i, STATE_HIGH );

    delay ( XSHUT_BOOT_MS );

    if ( ! tofL1xBringUp ( ( tofId_e ) i, TOF_L1X_BOOT_ADDRESS, i2cAddress ) ) {
      Peripheral_Write ( ( peripheral_gpio_pin_e ) xshut, STATE_LOW );    // keep a failed chip off 0x29
    }
    i2cAddress++;

    ledWrite ( i, STATE_LOW );

    delay ( XSHUT_BOOT_MS );
  }

  tofL1xStartSides ( );    // TIMED ranging, now that no laser is being configured

  // Phase 4: LED animation
  for ( uint8_t blink = 0; blink < 4; blink++ ) {
    for ( uint8_t i = 0; i < X1SIDE_COUNT; i++ ) {
      if ( ! x1LaserInit [ i ] ) continue;
      ledWrite ( i, STATE_TOGGLE );
    }
    delay ( 75 );
  }
}

// ============================================================
// X1Ranging public API
// ============================================================

void X1Ranging_P::init ( void ) {
  x1LaserInit [ LEFT ]         = true;
  triggerThresholdMm [ LEFT ]  = 200;
  x1LaserInit [ RIGHT ]        = true;
  triggerThresholdMm [ RIGHT ] = 200;
  x1LaserInit [ FRONT ]        = true;
  triggerThresholdMm [ FRONT ] = 200;
  x1LaserInit [ BACK ]         = true;
  triggerThresholdMm [ BACK ]  = 200;
}

void X1Ranging_P::init ( laser_e laser, int16_t threshold ) {
  if ( ( unsigned ) laser >= X1SIDE_COUNT ) return;    // side lasers only; DOWN comes from initAlt ( )
  x1LaserInit [ laser ]        = true;
  triggerThresholdMm [ laser ] = ( int16_t ) ( threshold * 10 );    // cm -> mm
}

// Records the down-laser request; gpio -1 = no XSHUT wire. A refused call keeps an earlier accepted one.
static bool altRequest ( int8_t gpio ) {
  if ( altLatched ) return false;    // only before the bring-up, i.e. from plutoInit ( )
  if ( gpio >= 0 && xRangingAltPinConflict ( gpio ) ) return false;
  altRequested         = true;
  altXshutGpio         = gpio;
  x1LaserInit [ DOWN ] = true;
  return true;
}

bool X1Ranging_P::initAlt ( peripheral_gpio_pin_e xshutGpio ) {
  if ( ( unsigned ) xshutGpio >= ( unsigned ) GPIO_COUNT ) return false;
  return altRequest ( ( int8_t ) xshutGpio );
}

bool X1Ranging_P::initAlt ( void ) {
  return altRequest ( -1 );
}

laser_status_e X1Ranging_P::getStatus ( laser_e laser ) {
  if ( ( unsigned ) laser >= X1LASER_ID_COUNT ) return LASER_NOT_ENABLED;
  const tofEntry_t *e = tofL1xEntry ( ( tofId_e ) laser );
  switch ( ( tofStatus_e ) e->status ) {
    case TOF_STATUS_OK: return e->outOfRange ? LASER_OUT_OF_RANGE : LASER_OK;
    case TOF_STATUS_BOOT_FAILED: return LASER_BOOT_FAILED;
    case TOF_STATUS_I2C_ERROR: return LASER_I2C_ERROR;
    default: return LASER_NOT_ENABLED;
  }
}

int16_t X1Ranging_P::getRange ( laser_e laser ) {
  if ( ( unsigned ) laser >= X1LASER_ID_COUNT || ! x1LaserInit [ laser ] ) return -1;

  // The cached sample: no I2C here ( the periodic laser task polls the chips ).
  const tofEntry_t *e = tofL1xEntry ( ( tofId_e ) laser );
  return ( e->status == TOF_STATUS_OK ) ? e->rangeMm : ( int16_t ) TOF_RANGE_INVALID;
}

bool X1Ranging_P::isTriggered ( laser_e laser ) {
  if ( ( unsigned ) laser >= X1SIDE_COUNT ) return false;    // side lasers only ( the LED table )
  if ( ! x1LaserInit [ laser ] ) return false;
  if ( triggerThresholdMm [ laser ] < 0 ) return false;

  const int16_t range = getRange ( laser );
  if ( range < 0 ) return false;

  const bool triggered = ( range < triggerThresholdMm [ laser ] );
  ledWrite ( ( uint8_t ) laser, triggered ? STATE_HIGH : STATE_LOW );

  return triggered;
}

void X1Ranging_P::enableOA ( void ) {
  oaEnabled = true;
}

void X1Ranging_P::disableOA ( void ) {
  oaEnabled = false;
  for ( uint8_t i = 0; i < X1SIDE_COUNT; i++ ) {
    if ( ! x1LaserInit [ i ] ) continue;
    ledWrite ( i, STATE_LOW );
  }
}

void X1Ranging_P::initObjectAvoidance ( uint16_t _avoidDist, laser_e _laser1, laser_e _laser2, laser_e _laser3, laser_e _laser4 ) {
  auto enableLaser = [] ( laser_e l ) {
    if ( ( unsigned ) l >= X1SIDE_COUNT ) return;    // -1 = unused argument; DOWN is not an OA laser
    x1LaserInit [ l ] = true;
  };

  oaThresholdMm = ( uint16_t ) ( _avoidDist * 10 );    // cm -> mm
  enableLaser ( _laser1 );
  enableLaser ( _laser2 );
  enableLaser ( _laser3 );
  enableLaser ( _laser4 );
}

/**
 * @brief X1Ranging's object avoidance, once per loop from mw.cpp ( family RANGING_VL53L1X ): the
 * core XRanging uses ( oaCoreRun, ObjectAvoidanceCore.h ) over the VL53L1X cache, the lasers set
 * by initObjectAvoidance ( ) and the side-laser LEDs.
 */
void x1RangingApplyObjectAvoidance ( void ) {
  oaCoreRun < OA_CORE_X1RANGING > ( oaEnabled, oaThresholdMm, x1LaserInit, X1Ranging, ledWrite );
}

X1Ranging_P X1Ranging;
