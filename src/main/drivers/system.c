/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\system.c                                           #
 #  Created Date: Mon, 6th Oct 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 27th Nov 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build_config.h"

#include "gpio.h"
#include "light_led.h"
#include "sound_beeper.h"
#include "nvic.h"

#include "system.h"

#ifndef EXTI_CALLBACK_HANDLER_COUNT
  #define EXTI_CALLBACK_HANDLER_COUNT 1
#endif

typedef struct extiCallbackHandlerConfig_s {
  IRQn_Type irqn;
  extiCallbackHandlerFunc *fn;
} extiCallbackHandlerConfig_t;

static extiCallbackHandlerConfig_t extiHandlerConfigs [ EXTI_CALLBACK_HANDLER_COUNT ];

void registerExtiCallbackHandler ( IRQn_Type irqn, extiCallbackHandlerFunc *fn ) {
  for ( int index = 0; index < EXTI_CALLBACK_HANDLER_COUNT; index++ ) {
    extiCallbackHandlerConfig_t *candidate = &extiHandlerConfigs [ index ];
    if ( ! candidate->fn ) {
      candidate->fn   = fn;
      candidate->irqn = irqn;
      return;
    }
  }
  failureMode ( FAILURE_DEVELOPER );    // EXTI_CALLBACK_HANDLER_COUNT is too low for the amount of handlers required.
}

void unregisterExtiCallbackHandler ( IRQn_Type irqn, extiCallbackHandlerFunc *fn ) {
  for ( int index = 0; index < EXTI_CALLBACK_HANDLER_COUNT; index++ ) {
    extiCallbackHandlerConfig_t *candidate = &extiHandlerConfigs [ index ];
    if ( candidate->fn == fn && candidate->irqn == irqn ) {
      candidate->fn   = NULL;
      candidate->irqn = ( IRQn_Type ) 0;
      return;
    }
  }
}

static void extiHandler ( IRQn_Type irqn ) {
  for ( int index = 0; index < EXTI_CALLBACK_HANDLER_COUNT; index++ ) {
    extiCallbackHandlerConfig_t *candidate = &extiHandlerConfigs [ index ];
    if ( candidate->fn && candidate->irqn == irqn ) {
      candidate->fn ( );
    }
  }
}

void EXTI15_10_IRQHandler ( void ) {
  extiHandler ( EXTI15_10_IRQn );
}

void EXTI3_IRQHandler ( void ) {
  extiHandler ( EXTI3_IRQn );
}

// cycles per microsecond
static uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
// cached value of RCC->CSR
uint32_t cachedRccCsrValue;

static void cycleCounterInit ( void ) {
  RCC_ClocksTypeDef clocks;
  RCC_GetClocksFreq ( &clocks );
  usTicks = clocks.SYSCLK_Frequency / 1000000;
}

// SysTick
void SysTick_Handler ( void ) {
  sysTickUptime++;
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros ( void ) {
  register uint32_t ms, cycle_cnt;
  do {
    ms        = sysTickUptime;
    cycle_cnt = SysTick->VAL;

    /*
     * If the SysTick timer expired during the previous instruction, we need to give it a little time for that
     * interrupt to be delivered before we can recheck sysTickUptime:
     */
    asm volatile ( "\tnop\n" );
  } while ( ms != sysTickUptime );
  return ( ms * 1000 ) + ( usTicks * 1000 - cycle_cnt ) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis ( void ) {
  return sysTickUptime;
}

void systemInit ( void ) {
#ifdef CC3D
  /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
  extern void *isr_vector_table_base;

  NVIC_SetVectorTable ( ( uint32_t ) &isr_vector_table_base, 0x0 );
#endif
  // Configure NVIC preempt/priority groups
  NVIC_PriorityGroupConfig ( NVIC_PRIORITY_GROUPING );

#ifdef STM32F10X
  // Turn on clocks for stuff we use
  RCC_APB2PeriphClockCmd ( RCC_APB2Periph_AFIO, ENABLE );
#endif

  // cache RCC->CSR value to use it in isMPUSoftreset() and others
  cachedRccCsrValue = RCC->CSR;
  RCC_ClearFlag ( );

  enableGPIOPowerUsageAndNoiseReductions ( );

#ifdef STM32F10X
  // Set USART1 TX (PA9) to output and high state to prevent a rs232 break condition on reset.
  // See issue https://github.com/cleanflight/cleanflight/issues/1433
  gpio_config_t gpio;

  gpio.mode  = Mode_Out_PP;
  gpio.speed = Speed_2MHz;
  gpio.pin   = Pin_9;
  digitalHi ( GPIOA, gpio.pin );
  gpioInit ( GPIOA, &gpio );

  // Turn off JTAG port 'cause we're using the GPIO for leds
  #define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW ( 0x2 << 24 )
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;
#endif

  // Init cycle counter
  cycleCounterInit ( );

  memset ( extiHandlerConfigs, 0x00, sizeof ( extiHandlerConfigs ) );
  // SysTick
  SysTick_Config ( SystemCoreClock / 1000 );
}

#if 1
void delayMicroseconds ( uint32_t us ) {
  uint32_t now = micros ( );
  for ( ; micros ( ) - now < us; );
}
#else
void delayMicroseconds ( uint32_t us ) {
  uint32_t elapsed   = 0;
  uint32_t lastCount = SysTick->VAL;

  for ( ;; ) {
    register uint32_t current_count = SysTick->VAL;
    uint32_t elapsed_us;

    // measure the time elapsed since the last time we checked
    elapsed += current_count - lastCount;
    lastCount = current_count;

    // convert to microseconds
    elapsed_us = elapsed / usTicks;
    if ( elapsed_us >= us )
      break;

    // reduce the delay by the elapsed time
    us -= elapsed_us;

    // keep fractional microseconds for the next iteration
    elapsed %= usTicks;
  }
}
#endif

void delay ( uint32_t ms ) {
  while ( ms-- )
    delayMicroseconds ( 1000 );
}

// --- Timing (ms) ---

// Define time intervals in milliseconds for different LED patterns and gaps
#define HDR_MS              200    // Duration for HDR pattern
#define DETAIL_MS           200    // Duration for detail pattern
#define DETAIL_GREEN_OFF_MS 250    // Green LED off duration
#define DETAIL_RED_OFF_MS   350    // Red LED off duration
#define DETAIL_BLUE_OFF_MS  500    // Blue LED off duration
#define CYCLE_GAP_MS        750    // Gap between cycles

// Enumeration for LED colors
typedef enum {
  LED_COLOR_RED,      // Represents the red LED
  LED_COLOR_GREEN,    // Represents the green LED
  LED_COLOR_BLUE      // Represents the blue LED
} LedColor;

// Structure to define an error pattern
typedef struct {
  uint8_t bit;          // Bitmask index in `mode` for identifying specific errors
  uint8_t blueCount;    // Number of blue blinks indicating the error count or type
} ErrorPattern;

/**
 * @brief Blinks an LED a specified number of times with defined on and off durations.
 *
 * This function controls the blinking of a specified color LED (red, green, or blue)
 * by turning it on for a given duration and then off for another duration. The process
 * repeats for the specified number of times.
 *
 * @param color  The color of the LED to blink (LED_COLOR_RED, LED_COLOR_GREEN, LED_COLOR_BLUE).
 * @param n      The number of times the LED should blink.
 * @param on_ms  The duration in milliseconds for which the LED remains on during each blink.
 * @param off_ms The duration in milliseconds for which the LED remains off between blinks.
 */
static inline void blink_led ( LedColor color, uint8_t n, uint16_t on_ms, uint16_t off_ms ) {
  while ( n-- ) {
    switch ( color ) {
      case LED_COLOR_RED: LED_R_ON; break;
      case LED_COLOR_GREEN: LED_G_ON; break;
      case LED_COLOR_BLUE: LED_B_ON; break;
    }
    delay ( on_ms );
    switch ( color ) {
      case LED_COLOR_RED: LED_R_OFF; break;
      case LED_COLOR_GREEN: LED_G_OFF; break;
      case LED_COLOR_BLUE: LED_B_OFF; break;
    }
    delay ( off_ms );
  }
}

static const ErrorPattern errorTable [] = {
  { FAILURE_INA219, 1 },              // INA failure
  { FAILURE_MISSING_ACC, 2 },         // IMU missing
  { FAILURE_ACC_INCOMPATIBLE, 2 },    // IMU missing
  { FAILURE_BARO, 3 },                // Baro failure
  { FAILURE_BARO_DRIFT, 3 },          // Baro drift
  { FAILURE_EXTCLCK, 4 },             // Crystal failure
  { FAILURE_VL53L1X, 5 },             // Crystal failure
  { FAILURE_PAW3903, 6 }              // Crystal failure
};

static const uint8_t errorTableCount = sizeof ( errorTable ) / sizeof ( errorTable [ 0 ] );

/**
 * @brief Executes a failure mode indication based on specified error modes.
 *
 * This function interprets the error modes and translates them into LED blink patterns.
 * It first determines unique failure codes from an error table and then continuously
 * blinks LEDs to represent these failures.
 *
 * @param mode A bitmask representing different error modes to be checked against the error table.
 */
void failureMode ( uint16_t mode ) {
  // Array to store unique failure codes
  uint8_t failures [ 20 ];

  // Counter to track the number of unique failures detected
  uint8_t failCount = 0;

  // Iterate over the error table to check for active error bits in the mode
  for ( uint8_t i = 0; i < errorTableCount; i++ ) {
    // Check if the current error bit is set in the mode
    if ( mode & ( 1 << errorTable [ i ].bit ) ) {
      // Flag to check if the failure code already exists in the failures array
      bool exists = false;

      // Check for existence of the current failure code in the failures array
      for ( uint8_t j = 0; j < failCount; j++ ) {
        if ( failures [ j ] == errorTable [ i ].blueCount ) {
          exists = true;
          break;
        }
      }

      // If the failure code already exists, skip adding it
      if ( exists ) continue;

      // Add new unique failure code to the failures array and increment the counter
      failures [ failCount ] = errorTable [ i ].blueCount;
      failCount++;
    }
  }

  // If no failures were identified, exit the function
  if ( failCount == 0 )
    return;

  // Continuously loop to indicate failures using LED blink patterns
  while ( 1 ) {
    // Blink green LED to indicate the number of unique failures detected
    blink_led ( LED_COLOR_GREEN, failCount, HDR_MS, DETAIL_MS );

    // Delay before blinking red and blue LEDs for detailed failure indications
    delay ( DETAIL_MS );
    for ( uint8_t i = 0; i < failCount; i++ ) {
      // Determine the number of red blinks indicating the failure index
      uint8_t redBlink = i + 1;
      // Determine the number of blue blinks corresponding to the failure code
      uint8_t blueBlink = failures [ i ];

      // Blink red LED with determined blink count
      blink_led ( LED_COLOR_RED, redBlink, HDR_MS, DETAIL_RED_OFF_MS );

      // Delay before blinking blue LED
      delay ( DETAIL_MS );

      // Blink blue LED with determined blink count
      blink_led ( LED_COLOR_BLUE, blueBlink, HDR_MS, DETAIL_BLUE_OFF_MS );

      // Delay after blue LED indication
      delay ( DETAIL_MS );
    }

    // Delay before starting the next cycle of blink indications
    delay ( CYCLE_GAP_MS );
  }
}
