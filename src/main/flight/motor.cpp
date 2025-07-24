/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\flight\motor.cpp                                           #
 #  Created Date: Tue, 15th Apr 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sun, 20th Apr 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#include "config/runtime_config.h"

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"

#include "io/rc_controls.h"

#include "flight/motor.h"
#include "flight/mixer.h"

bool hasInitialBoostCompleted = false;    // Flags whether the initial 100ms motor boost is complete

bool initialBoostResetDone = false;    // Flags whether the boost logic has been reset after disarming

unsigned long currentMillis         = 0;    // Tracks the current time in milliseconds
unsigned long initialBoostStartTime = 0;    // Stores the start time of the initial motor boost

/**
 * @brief Sets all motor PWM values to the given value.
 *
 * @param pwmValue The PWM value to apply to all disarmed motors (typically between 1000–2000).
 */
void setAllMotors ( int pwmValue ) {
  for ( int i = 0; i < 4; i++ ) {
    motor_disarmed [ i ] = pwmValue;
  }
}

/**
 * @brief Handles motor wake-up logic based on arming status, RC mode, and safety checks.
 *
 * This function provides a short 100ms boost to all motors when arming is initiated,
 * then transitions to a low idle throttle. It ensures the motors remain disarmed when
 * conditions are not met.
 *
 * Conditions:
 * - If arming is allowed and arm mode is active, motors boost to 1700 for 100ms,
 *   then drop to 1100 idle.
 * - If disarmed or arm mode is not active, motors reset to 1000 and boost logic is cleared.
 */
void MotroWakeUp ( void ) {
  currentMillis = millis ( );

  // Evaluate the system state
  bool isArmModeActive  = IS_RC_MODE_ACTIVE ( BOXARM );
  bool isSystemReady    = status_FSI ( Ok_to_arm );
  bool isCurrentlyArmed = ARMING_FLAG ( ARMED );

  // If system is ready, arm mode is active, and not yet armed
  if ( isSystemReady && isArmModeActive && ! isCurrentlyArmed ) {
    if ( ! hasInitialBoostCompleted ) {
      // First-time boost: mark the start time and set high PWM
      if ( initialBoostStartTime == 0 ) {
        initialBoostStartTime = currentMillis;
        setAllMotors ( 1650 );    // Initial 100ms boost
      }
      // If 100ms has passed, mark boost as complete
      else if ( currentMillis - initialBoostStartTime >= 100 ) {
        hasInitialBoostCompleted = true;
        initialBoostResetDone    = false;
      }
    } else {
      // After initial boost, maintain idle speed
      setAllMotors ( 1100 );
      initialBoostResetDone = false;
    }
  }

  // If disarmed or arm mode is off, reset boost logic and shut motors
  if ( ( ! isArmModeActive || ! isSystemReady ) /* && ! initialBoostResetDone */ ) {
    hasInitialBoostCompleted = false;
    initialBoostStartTime    = 0;
    setAllMotors ( 1000 );    // Fully disarmed throttle
    initialBoostResetDone = true;
  }
}
