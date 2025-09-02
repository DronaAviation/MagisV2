/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\Peripheral-GPIO.cpp                                #
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
#include <stdint.h>

#include "platform.h"

#include "build_config.h"
#include "common/utils.h"
#include "common/atomic.h"

#include "drivers/system.h"
#include "drivers/gpio.h"

#include "config/config.h"

#include "API/Peripherals.h"

// Structure to hold mapping information for GPIO pins
typedef struct {
  GPIO_TypeDef *port;    // Pointer to the GPIO port
  uint16_t pin;          // Pin number within the port
  uint32_t clock;        // Clock identifier for the GPIO port
} GpioMapEntry;

// Static constant array to map peripheral GPIO pins to their settings
static const GpioMapEntry gpioMap [ GPIO_COUNT ] = {
  [GPIO_1]  = { GPIOA, Pin_8, RCC_AHBPeriph_GPIOA },
  [GPIO_2]  = { GPIOB, Pin_2, RCC_AHBPeriph_GPIOB },
  [GPIO_3]  = { GPIOB, Pin_6, RCC_AHBPeriph_GPIOB },
  [GPIO_4]  = { GPIOB, Pin_7, RCC_AHBPeriph_GPIOB },
  [GPIO_5]  = { GPIOB, Pin_15, RCC_AHBPeriph_GPIOB },
  [GPIO_6]  = { GPIOB, Pin_14, RCC_AHBPeriph_GPIOB },
  [GPIO_7]  = { GPIOB, Pin_13, RCC_AHBPeriph_GPIOB },
  [GPIO_8]  = { GPIOB, Pin_12, RCC_AHBPeriph_GPIOB },
  [GPIO_9]  = { GPIOA, Pin_4, RCC_AHBPeriph_GPIOA },
  [GPIO_10] = { GPIOA, Pin_13, RCC_AHBPeriph_GPIOA },
  [GPIO_11] = { GPIOA, Pin_14, RCC_AHBPeriph_GPIOA },
  [GPIO_12] = { GPIOB, Pin_4, RCC_AHBPeriph_GPIOB },
  [GPIO_13] = { GPIOB, Pin_5, RCC_AHBPeriph_GPIOB },
  [GPIO_14] = { GPIOA, Pin_5, RCC_AHBPeriph_GPIOA },
  [GPIO_15] = { GPIOB, Pin_3, RCC_AHBPeriph_GPIOB },
  [GPIO_16] = { GPIOA, Pin_15, RCC_AHBPeriph_GPIOA },
  [GPIO_17] = { GPIOA, Pin_3, RCC_AHBPeriph_GPIOA },
  [GPIO_18] = { GPIOA, Pin_2, RCC_AHBPeriph_GPIOA },
};

// Mapping of GPIO modes to configuration settings
static const GPIO_Mode modeMap [] = {
  [INPUT]           = Mode_IN_FLOATING,
  [INPUT_PULL_UP]   = Mode_IPU,
  [INPUT_PULL_DOWN] = Mode_IPD,
  [OUTPUT]          = Mode_Out_PP,
};

// Function pointer type for writing to GPIO pins
typedef void ( *GpioWriteFn ) ( GPIO_TypeDef *, uint16_t );

// Static constant array mapping states to their corresponding write functions
static const GpioWriteFn writeMap [] = {
  [STATE_LOW]    = digitalLo,
  [STATE_HIGH]   = digitalHi,
  [STATE_TOGGLE] = digitalToggle
};

// Global variable to hold the current GPIO entry
GpioMapEntry entry;

void Peripheral_Init ( peripheral_gpio_pin_e _gpio_pin, GPIO_Mode_e _mode ) {
  // Check if the provided GPIO pin is valid
  if ( _gpio_pin >= GPIO_COUNT ) {
    return;    // Exit if invalid
  }

  // Get the corresponding entry from the gpioMap based on the provided pin
  entry = gpioMap [ _gpio_pin ];

  // Configure GPIO settings
  gpio_config_t cfg = {
    .pin   = entry.pin,            // Set the pin
    .mode  = modeMap [ _mode ],    // Set the mode based on the mapping
    .speed = Speed_2MHz            // Set the speed for the GPIO
  };

  // Enable the clock for the GPIO port
  RCC_AHBPeriphClockCmd ( entry.clock, ENABLE );
  // Initialize the GPIO with the configuration
  gpioInit ( entry.port, &cfg );
}

bool Peripheral_Read ( peripheral_gpio_pin_e _gpio_pin ) {
  // Check if the provided GPIO pin is valid
  if ( _gpio_pin >= GPIO_COUNT ) {
    return false;    // Return false if invalid
  }

  // Get the corresponding entry from the gpioMap
  entry = gpioMap [ _gpio_pin ];

  // Return the state of the specified GPIO pin
  return digitalIn ( entry.port, entry.pin );
}

void Peripheral_Write ( peripheral_gpio_pin_e _gpio_pin, GPIO_State_e _state ) {
  // Check if the provided GPIO pin is valid
  if ( _gpio_pin >= GPIO_COUNT ) {
    return;    // Exit if invalid
  }

  // Get the corresponding entry from the gpioMap
  entry = gpioMap [ _gpio_pin ];

  // Call the appropriate function to set the GPIO state
  writeMap [ _state ]( entry.port, entry.pin );
}
