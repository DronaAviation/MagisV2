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
 #  Last Modified: Thu, 18th Sep 2025                                          #
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

#include "API/Motor.h"

struct Rev_Motor_Gpio {
  GPIO_TypeDef *gpio;
  uint16_t pin;
  uint32_t RCC_AHBPeriph;
};

static Rev_Motor_Gpio motors_gpio [ 6 ];

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
  // Get the current time in milliseconds
  currentMillis = millis ( );
  

  // Read the raw state of the arm switch and the system readiness status
  bool armSwitchRaw     = IS_RC_MODE_ACTIVE ( BOXARM );
  bool isSystemReady    = status_FSI ( Ok_to_arm );
  bool isCurrentlyArmed = ARMING_FLAG ( ARMED );

  // Static variables for debouncing and boost control
  static bool armSwitchDebounced          = false;    // Debounced state of the arm switch
  static bool boostActive                 = false;    // Flag indicating if boost is currently active
  static uint32_t lastArmSwitchChangeTime = 0;        // Timestamp of the last change in arm switch state
  static bool lastArmSwitchRaw            = false;    // Last recorded state of the raw arm switch

  // Debounce logic: Update the last change time if the arm switch state has changed
  if ( armSwitchRaw != lastArmSwitchRaw ) {
    lastArmSwitchChangeTime = currentMillis;    // Record the time of the change
    lastArmSwitchRaw        = armSwitchRaw;     // Update the last raw state
  }

  // Check if the arm switch state has been stable for at least 200ms
  if ( ( currentMillis - lastArmSwitchChangeTime ) >= 200 ) {
    armSwitchDebounced = armSwitchRaw;    // Set the debounced state
  }

  // Core logic for motor control based on debounced arm input
  if ( isSystemReady && armSwitchDebounced && ! isCurrentlyArmed ) {
    // If the system is ready and the arm switch is activated
    if ( usingMotorAPI ) {
      Motor_SetDir ( M5, ANTICLOCK_WISE );
      Motor_SetDir ( M6, CLOCK_WISE );
      Motor_SetDir ( M5, ANTICLOCK_WISE );
      Motor_SetDir ( M8, CLOCK_WISE );
      usingMotorAPI = false;
    }
    if ( ! hasInitialBoostCompleted && ! boostActive ) {
      initialBoostStartTime = currentMillis;    // Start boost timer
      boostActive           = true;             // Activate boost
      setAllMotors ( 1650 );                    // Set motors to a higher throttle for boost
    } else if ( boostActive && currentMillis - initialBoostStartTime >= 100 ) {
      // If boost is active and has been running for 100ms
      hasInitialBoostCompleted = true;     // Mark boost as completed
      initialBoostResetDone    = false;    // Prepare for potential reset
      boostActive              = false;    // Deactivate boost
    } else if ( hasInitialBoostCompleted ) {
      // If boost has been completed, set motors to a lower throttle
      setAllMotors ( 1100 );            // Maintain a safe operating throttle
      initialBoostResetDone = false;    // Allow for a reset if needed
    }
  }

  // Reset logic if disarmed or arm mode is off (using debounced value)
  if ( ( ! armSwitchDebounced || ! isSystemReady ) && ! initialBoostResetDone ) {
    hasInitialBoostCompleted = false;    // Reset completion status
    initialBoostStartTime    = 0;        // Reset boost start time
    boostActive              = false;    // Deactivate boost
    setAllMotors ( 1000 );               // Set motors to a safe disarmed throttle
    initialBoostResetDone = true;        // Mark reset as done
  }
}

void reverseMotorGPIOInit ( void ) {
  GPIO_TypeDef *gpio;
  gpio_config_t cfg;

  motors_gpio [ 2 ].gpio          = GPIOB;
  motors_gpio [ 2 ].pin           = Pin_4;
  motors_gpio [ 2 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;
  // M1
  cfg.pin   = motors_gpio [ 2 ].pin;
  cfg.mode  = Mode_Out_PP;
  cfg.speed = Speed_2MHz;
  RCC_AHBPeriphClockCmd ( motors_gpio [ 2 ].RCC_AHBPeriph, ENABLE );
  gpioInit ( motors_gpio [ 2 ].gpio, &cfg );
  digitalHi ( motors_gpio [ 2 ].gpio, motors_gpio [ 2 ].pin );

  motors_gpio [ 3 ].gpio          = GPIOB;
  motors_gpio [ 3 ].pin           = Pin_5;
  motors_gpio [ 3 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;
  // M2
  cfg.pin   = motors_gpio [ 3 ].pin;
  cfg.mode  = Mode_Out_PP;
  cfg.speed = Speed_2MHz;
  RCC_AHBPeriphClockCmd ( motors_gpio [ 3 ].RCC_AHBPeriph, ENABLE );
  gpioInit ( motors_gpio [ 3 ].gpio, &cfg );
  digitalLo ( motors_gpio [ 3 ].gpio, motors_gpio [ 3 ].pin );

  motors_gpio [ 4 ].gpio          = GPIOB;
  motors_gpio [ 4 ].pin           = Pin_6;
  motors_gpio [ 4 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;
  // M3
  cfg.pin   = motors_gpio [ 4 ].pin;
  cfg.mode  = Mode_Out_PP;
  cfg.speed = Speed_2MHz;
  RCC_AHBPeriphClockCmd ( motors_gpio [ 4 ].RCC_AHBPeriph, ENABLE );
  gpioInit ( motors_gpio [ 4 ].gpio, &cfg );
  digitalHi ( motors_gpio [ 4 ].gpio, motors_gpio [ 4 ].pin );

  motors_gpio [ 5 ].gpio          = GPIOB;
  motors_gpio [ 5 ].pin           = Pin_7;
  motors_gpio [ 5 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;
  // M4
  cfg.pin   = motors_gpio [ 5 ].pin;
  cfg.mode  = Mode_Out_PP;
  cfg.speed = Speed_2MHz;
  RCC_AHBPeriphClockCmd ( motors_gpio [ 5 ].RCC_AHBPeriph, ENABLE );
  gpioInit ( motors_gpio [ 5 ].gpio, &cfg );
  digitalLo ( motors_gpio [ 5 ].gpio, motors_gpio [ 5 ].pin );
}
