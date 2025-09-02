/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\Peripherals.h                                          #
 #  Created Date: Thu, 22nd May 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 2nd Sep 2025                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

/**
 * @enum peripheral_gpio
 * @brief Enumeration of available GPIO pins on the peripheral.
 *
 * This enumeration defines the General Purpose Input/Output (GPIO)
 * pins for use. Each pin can be utilized for input/output operations.
 *
 * Usage: Refer to constants (e.g., `GPIO_1`, `GPIO_2`) when configuring
 * or using GPIO functionalities.
 */
typedef enum peripheral_gpio {
  GPIO_1,       // Represents GPIO pin 1
  GPIO_2,       // Represents GPIO pin 2
  GPIO_3,       // Represents GPIO pin 3
  GPIO_4,       // Represents GPIO pin 4
  GPIO_5,       // Represents GPIO pin 5
  GPIO_6,       // Represents GPIO pin 6
  GPIO_7,       // Represents GPIO pin 7
  GPIO_8,       // Represents GPIO pin 8
  GPIO_9,       // Represents GPIO pin 9
  GPIO_10,      // Represents GPIO pin 10
  GPIO_11,      // Represents GPIO pin 11
  GPIO_12,      // Represents GPIO pin 12
  GPIO_13,      // Represents GPIO pin 13
  GPIO_14,      // Represents GPIO pin 14
  GPIO_15,      // Represents GPIO pin 15
  GPIO_16,      // Represents GPIO pin 16
  GPIO_17,      // Represents GPIO pin 17
  GPIO_18,      // Represents GPIO pin 18
  GPIO_COUNT    // Total number of GPIO pins defined
} peripheral_gpio_pin_e;

/**
 * @enum gpio_mode
 * @brief GPIO configuration modes.
 *
 * Defines the modes for configuring GPIO pins:
 * - `INPUT`: Floating input mode.
 * - `INPUT_PULL_UP`: Input with pull-up resistor.
 * - `INPUT_PULL_DOWN`: Input with pull-down resistor.
 * - `OUTPUT`: Push-pull output mode.
 */
typedef enum gpio_mode {
  INPUT,              // Input mode (floating)
  INPUT_PULL_UP,      // Input mode with pull-up resistor enabled
  INPUT_PULL_DOWN,    // Input mode with pull-down resistor enabled
  OUTPUT,             // Output mode (push-pull)
} GPIO_Mode_e;

/**
 * @enum gpio_state
 * @brief Represents the state of a GPIO pin.
 *
 * This enumeration defines the possible states for a GPIO pin:
 * - `STATE_LOW`: The pin is in a logic low state (0V).
 * - `STATE_HIGH`: The pin is in a logic high state (typically 3.3V or 5V).
 * - `STATE_TOGGLE`: The pin toggles between low and high states.
 */
typedef enum gpio_state {
  STATE_LOW,      // Logic low state
  STATE_HIGH,     // Logic high state
  STATE_TOGGLE    // Toggle state (switch between low and high)
} GPIO_State_e;

typedef enum peripheral_adc {
  ADC_1,
  ADC_2,
  ADC_3,
  ADC_4,
  ADC_5,
  ADC_6,
  ADC_7,
  ADC_8,
  ADC_9,
  ADC_10,
  ADC_11
} peripheral_adc_pin;

typedef enum peripheral_adc_channel {
  ADC2_IN12,
  ADC4_IN5,
  ADC4_IN4,
  ADC3_IN5,
  ADC4_IN3,
  ADC2_IN1,
  ADC2_IN2,
  ADC1_IN4,
  ADC1_IN3,
  ADC2_IN4,
  ADC3_IN1
} Peripheral_ADC_Channel;

#define ADC_CHANNEL_COUNT 11

extern bool _isAdcEnable [ ADC_CHANNEL_COUNT ];
extern uint8_t _adcDmaIndex [ ADC_CHANNEL_COUNT ];

#define ADC1_CHANNEL_COUNT 2
#define ADC2_CHANNEL_COUNT 3
#define ADC3_CHANNEL_COUNT 1
#define ADC4_CHANNEL_COUNT 3

extern volatile uint16_t _adc1Values [ ADC1_CHANNEL_COUNT ];
extern volatile uint16_t _adc2Values [ ADC2_CHANNEL_COUNT ];
extern volatile uint16_t _adc3Values [ ADC3_CHANNEL_COUNT ];
extern volatile uint16_t _adc4Values [ ADC4_CHANNEL_COUNT ];

/**
 * @brief Initializes a GPIO pin with a specified mode.
 *
 * Configures the specified GPIO pin. Returns if the pin is invalid.
 *
 * @param _gpio_pin The GPIO pin to initialize.
 * @param _mode The mode for the GPIO pin.
 */
void Peripheral_Init ( peripheral_gpio_pin_e _gpio_pin, GPIO_Mode_e _mode );
/**
 * @brief Reads the state of a GPIO pin.
 *
 * Returns true if the pin is high, false if low or invalid.
 *
 * @param _gpio_pin The GPIO pin to read.
 * @return bool The state of the pin.
 */
bool Peripheral_Read ( peripheral_gpio_pin_e _gpio_pin );
/**
 * @brief Writes a state to a GPIO pin.
 *
 * Sets the specified GPIO pin to high or low. Returns if the pin is invalid.
 *
 * @param _gpio_pin The GPIO pin to write to.
 * @param _state The state to write (high/low).
 */
void Peripheral_Write ( peripheral_gpio_pin_e _gpio_pin, GPIO_State_e _state );

/**
 * @brief Initializes the specified ADC pin.
 * @param _adc_pin Pin ranging from ADC_1 to ADC_11.
 */
void Peripheral_Init ( peripheral_adc_pin _pin );

/**
 * @brief Reads the latest value from the specified ADC pin.
 * @param _adc_pin Pin ranging from ADC_1 to ADC_11.
 * @return Latest ADC value or 0 if invalid pin.
 */
uint16_t Peripheral_Read ( peripheral_adc_pin _adc_pin );

void APIAdcInit ( void );

#endif