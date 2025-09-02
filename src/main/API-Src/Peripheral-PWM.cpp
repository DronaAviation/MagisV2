/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\Peripheral-PWM.cpp                                 #
 #  Created Date: Tue, 2nd Sep 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 2nd Sep 2025                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#include "platform.h"
#include "build_config.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"

#include "API/API-Utils.h"
#include "API/Peripherals.h"

static pwmOutputPort_t *pwm [ 10 ];

// void Peripheral_Init ( peripheral_pwm_pin_e _pin, uint16_t pwmRate ) {
//   timerHardware_t timerHardware;
//   uint32_t hz;

//   switch ( _pin ) {

//     case PWM_1:

//       if ( ! isPwmInit [ 0 ] ) {

//         RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOA, ENABLE );

//         RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM1, ENABLE );

//         timerHardware = { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource8, GPIO_AF_6 };

//         GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

//         hz = PWM_TIMER_MHZ * 1000000;

//         pwm [ 0 ] = pwmOutConfig ( &timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500 );

//         isPwmInit [ 0 ] = true;
//       }

//       break;

//     case PWM_2:

//       if ( ! isPwmInit [ 1 ] ) {

//         RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );

//         RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM4, ENABLE );

//         timerHardware = { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn, 1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_2 };

//         GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

//         hz = PWM_TIMER_MHZ * 1000000;

//         pwm [ 1 ] = pwmOutConfig ( &timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500 );

//         isPwmInit [ 1 ] = true;
//       }

//       break;

//     case PWM_3:

//       if ( ! isPwmInit [ 2 ] ) {

//         RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );

//         RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM4, ENABLE );

//         timerHardware = { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn, 1, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_2 };

//         GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

//         hz = PWM_TIMER_MHZ * 1000000;

//         pwm [ 2 ] = pwmOutConfig ( &timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500 );

//         isPwmInit [ 2 ] = true;
//       }

//       break;

//     case PWM_4:

//       if ( ! isPwmInit [ 3 ] ) {

//         RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );

//         RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM15, ENABLE );

//         timerHardware = { TIM15, GPIOB, Pin_15, TIM_Channel_2, TIM1_BRK_TIM15_IRQn, 1, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_1 };

//         GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

//         hz = PWM_TIMER_MHZ * 1000000;

//         pwm [ 3 ] = pwmOutConfig ( &timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500 );

//         isPwmInit [ 3 ] = true;
//       }

//       break;

//     case PWM_5:

//       if ( ! isPwmInit [ 4 ] ) {

//         RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );

//         RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM15, ENABLE );

//         timerHardware = { TIM15, GPIOB, Pin_14, TIM_Channel_1, TIM1_BRK_TIM15_IRQn, 1, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_1 };

//         GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

//         hz = PWM_TIMER_MHZ * 1000000;

//         pwm [ 4 ] = pwmOutConfig ( &timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500 );

//         isPwmInit [ 4 ] = true;
//       }

//       break;

//     case PWM_6:

//       if ( ! isPwmInit [ 5 ] ) {

//         RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOA, ENABLE );

//         RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM4, ENABLE );

//         timerHardware = { TIM4, GPIOA, Pin_13, TIM_Channel_3, TIM4_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource13, GPIO_AF_10 };

//         GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

//         hz = PWM_TIMER_MHZ * 1000000;

//         pwm [ 5 ] = pwmOutConfig ( &timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500 );

//         isPwmInit [ 5 ] = true;
//       }

//       break;

//     case PWM_7:

//       if ( ! isPwmInit [ 6 ] ) {

//         RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOA, ENABLE );

//         RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM8, ENABLE );

//         timerHardware = { TIM8, GPIOA, Pin_14, TIM_Channel_2, TIM8_CC_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource14, GPIO_AF_5 };

//         GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

//         hz = PWM_TIMER_MHZ * 1000000;

//         pwm [ 6 ] = pwmOutConfig ( &timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500 );

//         isPwmInit [ 6 ] = true;
//       }

//       break;

//     case PWM_8:

//       if ( ! isPwmInit [ 7 ] ) {

//         RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );

//         RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM3, ENABLE );

//         timerHardware = { TIM3, GPIOB, Pin_4, TIM_Channel_1, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource4, GPIO_AF_3 };

//         GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

//         hz = PWM_TIMER_MHZ * 1000000;

//         pwm [ 7 ] = pwmOutConfig ( &timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500 );

//         isPwmInit [ 7 ] = true;
//       }

//       break;

//     case PWM_9:

//       RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );

//       RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM17, ENABLE );

//       timerHardware = { TIM17, GPIOB, Pin_5, TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource5, GPIO_AF_10 };

//       GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

//       hz = PWM_TIMER_MHZ * 1000000;

//       pwm [ 8 ] = pwmOutConfig ( &timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500 );

//       isPwmInit [ 8 ] = true;

//       break;

//     case PWM_10:

//       if ( ! isPwmInit [ 9 ] ) {

//         RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOA, ENABLE );

//         RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM15, ENABLE );

//         timerHardware = { TIM15, GPIOA, Pin_15, TIM_Channel_1, TIM1_BRK_TIM15_IRQn, 1, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_2 };

//         GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

//         hz = PWM_TIMER_MHZ * 1000000;

//         pwm [ 9 ] = pwmOutConfig ( &timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500 );

//         isPwmInit [ 9 ] = true;
//       }

//       break;
//   }
// }

// void Peripheral_Write ( peripheral_pwm_pin_e _pwm_pin, uint16_t _pwm_value ) {

//   _pwm_value = constrain ( _pwm_value, 500, 2500 );

//   switch ( _pwm_pin ) {

//     case PWM_1:
//       if ( isPwmInit [ 0 ] ) {
//         *pwm [ 0 ]->ccr = _pwm_value;
//       }

//       break;

//     case PWM_2:

//       if ( isPwmInit [ 1 ] ) {

//         *pwm [ 1 ]->ccr = _pwm_value;
//       }

//       break;

//     case PWM_3:

//       if ( isPwmInit [ 2 ] ) {

//         *pwm [ 2 ]->ccr = _pwm_value;
//       }

//       break;

//     case PWM_4:

//       if ( isPwmInit [ 3 ] ) {

//         *pwm [ 3 ]->ccr = _pwm_value;
//       }

//       break;

//     case PWM_5:

//       if ( isPwmInit [ 4 ] ) {

//         *pwm [ 4 ]->ccr = _pwm_value;
//       }

//       break;

//     case PWM_6:

//       if ( isPwmInit [ 5 ] ) {

//         *pwm [ 5 ]->ccr = _pwm_value;
//       }

//       break;

//     case PWM_7:

//       if ( isPwmInit [ 6 ] ) {

//         *pwm [ 6 ]->ccr = _pwm_value;
//       }

//       break;

//     case PWM_8:

//       if ( isPwmInit [ 7 ] ) {

//         *pwm [ 7 ]->ccr = _pwm_value;
//       }

//       break;
//     case PWM_9:

//       if ( isPwmInit [ 9 ] ) {

//         *pwm [ 8 ]->ccr = _pwm_value;
//       }

//       break;

//     case PWM_10:

//       if ( isPwmInit [ 10 ] ) {

//         *pwm [ 9 ]->ccr = _pwm_value;
//       }

//       break;

//     default:

//       break;
//   }
// }

typedef struct {
  TIM_TypeDef *tim;
  GPIO_TypeDef *gpio;
  uint16_t pin;
  uint8_t channel;
  IRQn_Type irq;
  uint8_t outputEnable;
  GPIO_Mode mode;
  uint8_t pinSource;
  uint8_t alternateFunction;
  uint32_t gpioClk;
  uint32_t timClk;
  FunctionalState timClkState;
  FunctionalState gpioClkState;
} PwmConfig_t;

// LUT for all PWM pins
static const PwmConfig_t pwmConfig [] = {
  // TIM,   GPIO, Pin,  Channel, IRQ,                 OutEn, Mode,        PinSrc, AF,   GPIOClk,                TIMClk
  { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource8, GPIO_AF_6, RCC_AHBPeriph_GPIOA, RCC_APB2Periph_TIM1 },
  { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn, 1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_2, RCC_AHBPeriph_GPIOB, RCC_APB1Periph_TIM4 },
  { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn, 1, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_2, RCC_AHBPeriph_GPIOB, RCC_APB1Periph_TIM4 },
  { TIM15, GPIOB, Pin_15, TIM_Channel_2, TIM1_BRK_TIM15_IRQn, 1, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_1, RCC_AHBPeriph_GPIOB, RCC_APB2Periph_TIM15 },
  { TIM15, GPIOB, Pin_14, TIM_Channel_1, TIM1_BRK_TIM15_IRQn, 1, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_1, RCC_AHBPeriph_GPIOB, RCC_APB2Periph_TIM15 },
  { TIM4, GPIOA, Pin_13, TIM_Channel_3, TIM4_IRQn, 1, Mode_AF_PP_PD, GPIO_PinSource13, GPIO_AF_10, RCC_AHBPeriph_GPIOA, RCC_APB1Periph_TIM4 },
  { TIM8, GPIOA, Pin_14, TIM_Channel_2, TIM8_CC_IRQn, 1, Mode_AF_PP_PD, GPIO_PinSource14, GPIO_AF_5, RCC_AHBPeriph_GPIOA, RCC_APB2Periph_TIM8 },
  { TIM3, GPIOB, Pin_4, TIM_Channel_1, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource4, GPIO_AF_3, RCC_AHBPeriph_GPIOB, RCC_APB1Periph_TIM3 },
  { TIM17, GPIOB, Pin_5, TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource5, GPIO_AF_10, RCC_AHBPeriph_GPIOB, RCC_APB2Periph_TIM17 },
  { TIM15, GPIOA, Pin_15, TIM_Channel_1, TIM1_BRK_TIM15_IRQn, 1, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_2, RCC_AHBPeriph_GPIOA, RCC_APB2Periph_TIM15 },
};

void Peripheral_Init ( peripheral_pwm_pin_e _pin, uint16_t pwmRate ) {
  // Check if the pin is within valid range (PWM_1 to PWM_10)
  // If not, return immediately for safety
  if ( _pin < PWM_1 || _pin > PWM_10 ) return;

  // Use the enum value directly as an index for arrays
  uint8_t idx = _pin;

  // Proceed only if the PWM at this index has not been initialized yet
  if ( ! isPwmInit [ idx ] ) {
    // Get the configuration settings for the specified PWM pin
    const PwmConfig_t *cfg = &pwmConfig [ idx ];

    // Enable the GPIO clock for the specified peripheral
    RCC_AHBPeriphClockCmd ( cfg->gpioClk, ENABLE );

    // Enable the appropriate timer clock based on the timer's base address
    if ( ( ( uint32_t ) cfg->tim & 0xFFFF0000 ) == ( uint32_t ) APB2PERIPH_BASE )
      RCC_APB2PeriphClockCmd ( cfg->timClk, ENABLE );
    else
      RCC_APB1PeriphClockCmd ( cfg->timClk, ENABLE );

    // Populate the timer hardware structure with configuration data
    timerHardware_t timerHardware = { cfg->tim, cfg->gpio, cfg->pin, cfg->channel,
                                      cfg->irq, cfg->outputEnable, cfg->mode,
                                      cfg->pinSource, cfg->alternateFunction };

    // Configure the GPIO pin for alternate function (used for PWM output)
    GPIO_PinAFConfig ( cfg->gpio, ( uint16_t ) cfg->pinSource, cfg->alternateFunction );

    // Compute the PWM frequency in Hertz
    uint32_t hz = PWM_TIMER_MHZ * 1000000;

    // Configure and initialize the PWM output
    // Sets the PWM frequency and initial duty cycle
    pwm [ idx ] = pwmOutConfig ( &timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500 );

    // Mark this PWM channel as initialized
    isPwmInit [ idx ] = true;
  }
}

void Peripheral_Write ( peripheral_pwm_pin_e _pwm_pin, uint16_t _pwm_value ) {
  // Constrain the PWM value to be within the 500–2500 range
  _pwm_value = constrain ( _pwm_value, 500, 2500 );

  // Use the enum value directly as an index for arrays
  uint8_t idx = _pwm_pin;

  // Check if the index is valid and the PWM channel has been initialized
  if ( idx < 10 && isPwmInit [ idx ] ) {
    // Set the CCR register of the PWM channel to the constrained PWM value
    *pwm [ idx ]->ccr = _pwm_value;
  }
}
