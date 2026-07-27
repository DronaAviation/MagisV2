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
 * (applyObjectAvoidance) ask for.                                           */
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
 * floor also covers applyObjectAvoidance(), which re-asserts every 100 ms. */
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
extern bool isUserGPSCoordSet;
extern bool startShieldRanging;
extern bool reverseReferenceFrame;
extern bool motorMixer;
extern bool isLocalisationOn;
extern bool DONT_USE_STATUS_LED;

extern bool isPwmInit [ 11 ];
extern bool isUserFlightModeSet [ 6 ];
extern bool isXLaserInit [ 5 ];

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

void xRangingInit ( void );
void applyObjectAvoidance ( );

// int getGPIOport ( unibus_e pin );
// GPIO_Pin getGPIOpin ( unibus_e pin );
// uint32_t getGPIOclock ( unibus_e pin );
// uint8_t getGPIOpinSource ( unibus_e pin );
// uint16_t getTimerCh ( unibus_e pin );

void resetUser ( void );

#ifdef __cplusplus
}
#endif
