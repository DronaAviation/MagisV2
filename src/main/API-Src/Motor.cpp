/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\Motor.cpp                                              #
 #  Created Date: Tue, 20th May 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 18th Sep 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/pwm_rx.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/rc_controls.h"
#include "io/serial.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/imu.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "API/Motor.h"

struct Rev_Motor_Gpio {
  GPIO_TypeDef *gpio;
  uint16_t pin;
  uint32_t RCC_AHBPeriph;
};

pwmOutputPort_t *userMotor [ 2 ];    // Array to hold PWM output configurations for two motors

// Initialize an array of Motor structs
static Rev_Motor_Gpio motors_gpio [ 6 ];

/**
 * @brief Initializes the specified motor by configuring the necessary GPIO and timer settings.
 *
 * This function sets up the hardware required for controlling a bidirectional motor.
 * It configures the appropriate GPIO pins and PWM timer based on the motor type specified.
 *
 * @param _motor The bidirectional motor to initialize (e.g., M1, M2).
 */
void Motor_Init ( bidirectional_motor_e _motor ) {
  timerHardware_t timerHardware;
  GPIO_TypeDef *gpio;
  gpio_config_t cfg;
  uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;

  switch ( _motor ) {
    case M1:
      RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOA, ENABLE );
      RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM3, ENABLE );
      timerHardware                   = { TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_2 };
      motors_gpio [ 0 ].gpio          = GPIOB;
      motors_gpio [ 0 ].pin           = Pin_0;
      motors_gpio [ 0 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;
      // M1
      cfg.pin   = motors_gpio [ 0 ].pin;
      cfg.mode  = Mode_Out_PP;
      cfg.speed = Speed_2MHz;
      RCC_AHBPeriphClockCmd ( motors_gpio [ 0 ].RCC_AHBPeriph, ENABLE );
      gpioInit ( motors_gpio [ 0 ].gpio, &cfg );
      digitalLo ( motors_gpio [ 0 ].gpio, motors_gpio [ 0 ].pin );

      GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );
      userMotor [ 0 ] = pwmOutConfig ( &timerHardware, PWM_BRUSHED_TIMER_MHZ, hz / masterConfig.motor_pwm_rate, 0 );
      break;

    case M2:
      RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOA, ENABLE );
      RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM3, ENABLE );
      timerHardware                   = { TIM3, GPIOA, Pin_7, TIM_Channel_2, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource7, GPIO_AF_2 };
      motors_gpio [ 1 ].gpio          = GPIOB;
      motors_gpio [ 1 ].pin           = Pin_1;
      motors_gpio [ 1 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;
      cfg.pin                         = motors_gpio [ 1 ].pin;
      cfg.mode                        = Mode_Out_PP;
      cfg.speed                       = Speed_2MHz;
      RCC_AHBPeriphClockCmd ( motors_gpio [ 1 ].RCC_AHBPeriph, ENABLE );
      gpioInit ( motors_gpio [ 1 ].gpio, &cfg );
      digitalHi ( motors_gpio [ 1 ].gpio, motors_gpio [ 1 ].pin );

      GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );
      userMotor [ 1 ] = pwmOutConfig ( &timerHardware, PWM_BRUSHED_TIMER_MHZ, hz / masterConfig.motor_pwm_rate, 0 );
      break;

    default:
      break;
  }
}

bool usingMotorAPI  = false;    // Flag to indicate if the motor API is being used
bool morto_arm_stat = false;    // Status flag for motor arming

/**
 * @brief Sets the PWM value for the specified motor.
 *
 * This function updates the duty cycle of the specified motor based on the given PWM value.
 * It also manages motor disarming status based on RC mode activity.
 *
 * @param _motor The bidirectional motor to set the PWM value for (e.g., M1, M2, M5, M6, M7, M8).
 * @param pwmValue The desired PWM value within the range of 1000 to 2000.
 */
void Motor_Set ( bidirectional_motor_e _motor, int16_t pwmValue ) {
  // Constrain the PWM value to be within the valid range of 1000 to 2000
  pwmValue = constrain ( pwmValue, 1000, 2000 );

  // Calculate the index for userMotor based on the motor type
  int motorIndex = static_cast< int > ( _motor );    // Assuming M1, M2, etc., are enumerated types starting from 0

  // Update the PWM value for the specified motor if it is either M1 or M2
  if ( _motor == M1 || _motor == M2 ) {
    // Update the CCR register with the calculated PWM value based on the period
    *( userMotor [ motorIndex ]->ccr ) = ( pwmValue - 1000 ) * userMotor [ motorIndex ]->period / 1000;
  }

  // Check if RC mode is not active
  if ( ! IS_RC_MODE_ACTIVE ( BOXARM ) ) {
    morto_arm_stat = false;    // Set the motor armed status to false

    // Disarm motors based on their specific identifiers
    switch ( _motor ) {
      case M5:
        usingMotorAPI        = true;        // Indicate that the motor API is being used
        motor_disarmed [ 3 ] = pwmValue;    // Store the PWM value for motor M5
        break;

      case M6:
        usingMotorAPI        = true;
        motor_disarmed [ 2 ] = pwmValue;    // Store the PWM value for motor M6
        break;

      case M7:
        usingMotorAPI        = true;
        motor_disarmed [ 0 ] = pwmValue;    // Store the PWM value for motor M7
        break;

      case M8:
        usingMotorAPI        = true;
        motor_disarmed [ 1 ] = pwmValue;    // Store the PWM value for motor M8
        break;

      default:
        break;    // No action needed for other motors
    }
  }
  // Check if RC mode is active and the motors are not already armed
  else if ( IS_RC_MODE_ACTIVE ( BOXARM ) && ! morto_arm_stat ) {
    morto_arm_stat = true;    // Arm the motors
    // Disarm all motors by setting their PWM values to the minimum of 1000
    motor_disarmed [ 3 ] = 1000;
    motor_disarmed [ 2 ] = 1000;
    motor_disarmed [ 0 ] = 1000;
    motor_disarmed [ 1 ] = 1000;
  }
}

/**
 * @brief Sets the direction of the specified motor.
 *
 * This function controls the direction of the specified motor by setting its GPIO pin high or low.
 *
 * @param motor The bidirectional motor whose direction is to be set.
 * @param direction The desired direction of the motor (true for forward, false for reverse).
 */
void Motor_SetDir ( bidirectional_motor_e motor, motor_direction_e direction ) {
  // Check if the motor index is within the valid range
  if ( motor >= 0 && motor < sizeof ( motors_gpio ) / sizeof ( Rev_Motor_Gpio ) ) {
    // If direction is true, set the GPIO pin high to move the motor in one direction
    if ( direction )
      digitalHi ( motors_gpio [ motor ].gpio, motors_gpio [ motor ].pin );
    // If direction is false, set the GPIO pin low to move the motor in the opposite direction
    else
      digitalLo ( motors_gpio [ motor ].gpio, motors_gpio [ motor ].pin );
  }
}
