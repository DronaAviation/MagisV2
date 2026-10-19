/*
 * This file is part of Magis.
 *
 * Magis is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Magis is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "platform.h"
#include "build_config.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"
#include "blackbox/blackbox_fielddefs.h"

#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/ranging_vl53l0x.h"
#include "drivers/nvic.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/pwm_output.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RESET_CHECK 3

/* --- User RC override (RC_ARRAY / userRCflag) ----------------------------- *
 * How much of the pilot's stick deflection is allowed to cross-fade a user RC
 * override away. 1.0f means a fully deflected stick takes the channel back
 * completely; 0.0f means the pilot cannot fade the override at all, which is
 * what producers that already fold the pilot's input into RC_ARRAY themselves
 * (applyObjectAvoidance, x1RangingApplyObjectAvoidance) ask for.            */
#define USER_RC_AUTHORITY_DEFAULT 1.0f

/* Stick travel, either side of centre, at which the pilot has taken a channel
 * back completely. Deliberately shorter than the 500 counts of physical stick
 * travel so the handover is decisive: at half stick the override is gone
 * entirely, and a user command starts reversing well before that. Raise it
 * towards 500 for a gentler, more gradual handover.                         */
#define USER_RC_STICK_TRAVEL      200

/* An override is dropped if it has not been re-asserted within this window.
 * The actual timeout is max ( this, 2 * userLoopFrequency ) so that
 * re-asserting once per plutoLoop() always keeps the override alive; the
 * floor also covers applyObjectAvoidance() / x1RangingApplyObjectAvoidance(),
 * which re-assert every 100 ms.                                             */
#define USER_RC_HOLD_MIN_US       250000

extern uint8_t resetCounter;
extern uint8_t intLogCounter;
extern uint8_t floatLogCounter;
extern int16_t appHeading;
extern int16_t AUX3_VALUE;
extern int16_t userHeading;
extern uint32_t userLoopFrequency;
extern int32_t user_GPS_coord [ 2 ];
extern uint32_t autoRcTimerLoop;
extern int32_t MOTOR_ARRAY [ 4 ];
extern int32_t app_GPS_coord [ 2 ];
extern int32_t RC_ARRAY [ 4 ];
extern uint32_t userRCsetTime [ 4 ];    // micros() when each override was last asserted
extern float userRCauthority [ 4 ];     // how far the pilot's stick may fade each override out
extern int16_t userRCthrottleRef;       // pilot throttle captured when the throttle override latched

extern bool runUserCode;
extern bool developerMode;
extern bool useAutoRC;
extern bool External_RC_FLAG [ 4 ];
extern bool userRCflag [ 4 ];
extern bool callibrateAccelero;
extern bool hasTakeOff;
extern bool AutoAccCalibration;
extern bool callOnPilotStart;
extern bool callonPilotFinish;
extern bool fsLowBattery;
extern bool fsInFlightLowBattery;
extern bool fsCrash;
extern bool isUserHeadingSet;
extern bool isUserHeadingRelative;
extern bool userHeadingStale;
extern bool isUserGPSCoordSet;
extern bool startShieldRanging;
extern bool reverseReferenceFrame;
extern bool motorMixer;
extern bool isLocalisationOn;
extern bool DONT_USE_STATUS_LED;

extern bool isPwmInit [ 11 ];
extern bool isUserFlightModeSet [ 6 ];
extern bool isXLaserInit [ 6 ];    // indexed by laser_e: LEFT … EXTERNAL, DOWN

extern int32_t userDesiredAngle [ 3 ];
extern int32_t userDesiredRate [ 3 ];
extern int32_t userMotorPwm [ 4 ];
extern int32_t userSetVelocity;
extern int16_t userHeadFreeHoldHeading;
extern bool isUserDesiredAngle [ 3 ];
extern bool isUserDesiredRate [ 3 ];
extern bool isUserMotorPwm [ 4 ];
extern bool isUserSetVelocity;
extern bool isUserHeadFreeHoldSet;

void userEnabledLand ( );
void resetUserRCflag ( void );
void userRCassert ( uint8_t channel, float pilotAuthority );
void userRCrelease ( uint8_t channel );

// Laser family ( vl53l1x-xranging-api t15 ): which laser API user code asked for in plutoInit ( ).
// rangingFamilyLatch ( ) fixes it once, right after plutoInit ( ) and before xRangingInit ( ); later
// calls do nothing. Before the latch rangingFamily ( ) is RANGING_NONE. RANGING_BOTH ( XRanging and
// X1Ranging both requested ) refuses both: no laser starts and altitude hold stays on the baro.
typedef enum {
  RANGING_NONE = 0,    // no laser requested
  RANGING_VL53L0X,     // XRanging only
  RANGING_VL53L1X,     // X1Ranging only
  RANGING_BOTH         // both requested: both refused
} rangingFamily_e;
void rangingFamilyLatch ( void );
rangingFamily_e rangingFamily ( void );

void xRangingInit ( void );
void applyObjectAvoidance ( );
// Down-laser ( XRanging.initAlt ) state. xRangingAltLatch ( ) runs once at the bring-up, after
// plutoInit ( ): it re-checks the XSHUT pin and fixes the result for the rest of the run.
bool xRangingAltLatch ( void );
int8_t xRangingAltXshut ( void );    // peripheral_gpio_pin_e, or -1 for no XSHUT wire
// True once the latch gave this pin to the down-laser's XSHUT ( as a GPIO, or through its ADC /
// PWM alias ). The public Peripheral_Init / Peripheral_Write then refuse it: any other drive of
// that pin resets the altitude laser in flight. X1Ranging's down XSHUT counts too.
bool xRangingGpioClaimed ( uint8_t gpioPin );
bool xRangingAdcClaimed ( uint8_t adcPin );
bool xRangingPwmClaimed ( uint8_t pwmPin );

// X1Ranging ( VL53L1X, API-Src/X1Ranging.cpp ): the one bring-up, run once after plutoInit ( ) as
// xRangingInit ( ); it starts lasers only when the family is RANGING_VL53L1X.
void x1RangingInit ( void );
bool x1RangingRequested ( void );     // user code asked X1Ranging for any laser ( DOWN included )
int8_t x1RangingAltXshut ( void );    // X1Ranging's down XSHUT once latched ( peripheral_gpio_pin_e ), else -1
void x1RangingApplyObjectAvoidance ( void );    // X1Ranging's applyObjectAvoidance ( ): the same core, the L1X lasers
// XRanging's side-laser pin table and down-XSHUT pin rules, shared with X1Ranging ( API-Src/XRanging.cpp ).
int8_t xRangingSideXshut ( uint8_t side );      // side = laser_e LEFT … EXTERNAL; peripheral_gpio_pin_e, -1 out of range
int8_t xRangingSideLed ( uint8_t side );        // the side laser's indicator LED, -1 none
bool xRangingAltPinConflict ( int8_t gpio );    // true if a down-laser XSHUT may not use this GPIO
// XSHUT pins for the laser bring-up only: no claim check ( peripheral_gpio_pin_e ).
void xshutPinInit ( uint8_t gpioPin );
void xshutPinWrite ( uint8_t gpioPin, bool high );
// True if user code enabled this ADC pin ( peripheral_adc_pin ) with Peripheral_Init.
bool adcPinEnabled ( uint8_t adcPin );

// int getGPIOport ( unibus_e pin );
// GPIO_Pin getGPIOpin ( unibus_e pin );
// uint32_t getGPIOclock ( unibus_e pin );
// uint8_t getGPIOpinSource ( unibus_e pin );
// uint16_t getTimerCh ( unibus_e pin );

void resetUser ( void );

#ifdef __cplusplus
}
#endif
