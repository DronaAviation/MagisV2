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
 #  Last Modified: Sat, 11th Oct 2025                                          #
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

// Time duration for which the header signal is on (in milliseconds)
#define HDR_ON_MS 200
// Time duration for which the header signal is off (in milliseconds)
#define HDR_OFF_MS 200
// Time duration for which each detail signal (blue blink) is on (in milliseconds)
#define DETAIL_ON_MS 200
// Standard time duration for which each detail signal is off (in milliseconds)
#define DETAIL_OFF_MS 500
// Special time duration for which the detail signal is off when indicating a crystal failure (in milliseconds)
#define DETAIL_OFF_CRYSTAL 300
// Time gap between complete sequences of signals (in milliseconds)
#define CYCLE_GAP_MS 1000

// Small helpers (toggle-based since only *_TOGGLE macros exist)

/**
 * @brief Blinks a red LED a specified number of times.
 *
 * This function toggles the red LED on and off a given number of times,
 * with customizable durations for the "on" and "off" states.
 *
 * @param n Number of times to blink the LED.
 * @param on_ms Duration in milliseconds for which the LED stays on.
 * @param off_ms Duration in milliseconds for which the LED stays off.
 */
static inline void blink_red ( uint8_t n, uint16_t on_ms, uint16_t off_ms ) {
  while ( n-- ) {
    LED_R_TOGGLE;
    delay ( on_ms );
    LED_R_TOGGLE;
    delay ( off_ms );
  }
}

/**
 * @brief Blinks a blue LED a specified number of times.
 *
 * This function toggles the blue LED on and off a given number of times,
 * with customizable durations for the "on" and "off" states.
 *
 * @param n Number of times to blink the LED.
 * @param on_ms Duration in milliseconds for which the LED stays on.
 * @param off_ms Duration in milliseconds for which the LED stays off.
 */
static inline void blink_blue ( uint8_t n, uint16_t on_ms, uint16_t off_ms ) {
  while ( n-- ) {
    LED_B_TOGGLE;
    delay ( on_ms );
    LED_B_TOGGLE;
    delay ( off_ms );
  }
}

/**
 * @brief Indicates system failure modes using LED blink patterns.
 *
 * This function determines and executes a specific blink pattern to
 * indicate various system failures. The pattern is based on the mode
 * parameter, and different failures have different priorities and
 * associated blink sequences:
 *
 * - IMU + BARO: 5× Blue (highest priority)
 * - IMU: 3× Blue
 * - BARO: 4× Blue
 * - INA219: 2× Blue
 * - Crystal: 6× Blue (shorter OFF time)
 * - BARO Drift: 7× Blue (shorter OFF time)
 *
 * If multiple failures are detected simultaneously, the one with the
 * highest priority will be indicated. A red LED blinks twice at the
 * start of each cycle to signal that an error state is active.
 *
 * @param mode Encoded failure mode bits indicating which systems have failed.
 */
void failureMode ( uint8_t mode ) {

  const bool hasIMU        = ( mode & ( 1 << FAILURE_MISSING_ACC ) ) || ( mode & ( 1 << FAILURE_ACC_INCOMPATIBLE ) );
  const bool hasBARO       = ( mode & ( 1 << FAILURE_BARO ) );
  const bool hasBARO_Drift = ( mode & ( 1 << FAILURE_BARO_DRIFT ) );
  const bool hasINA        = ( mode & ( 1 << FAILURE_INA219 ) );
  const bool hasXTAL       = ( mode & ( 1 << FAILURE_EXTCLCK ) );

  while ( 1 ) {
    // Blink red twice as a header to indicate "error state active"
    blink_red ( 2, HDR_ON_MS, HDR_OFF_MS );

    // Decide detail pattern (blue count + off timing)
    uint8_t blueCount = 0;
    uint16_t blueOff  = DETAIL_OFF_MS;

    if ( hasINA ) {
      blueCount = 2;    // INA219 failure
    } else if ( hasXTAL ) {
      blueCount = 6;                     // Crystal failure
      blueOff   = DETAIL_OFF_CRYSTAL;    // Special gap for crystal failure
    } else if ( hasIMU && hasBARO ) {
      blueCount = 5;    // Combined IMU + Baro failure (highest priority)
    } else if ( hasIMU ) {
      blueCount = 3;    // IMU failure
    } else if ( hasBARO ) {
      blueCount = 4;    // Baro failure
    } else if ( hasBARO_Drift ) {
      blueCount = 7;    // Baro drift failure
    } else {
      // No specific failure: only the header and cycle gap will occur.
    }

    if ( blueCount ) {
      blink_blue ( blueCount, DETAIL_ON_MS, blueOff );
    }

    // Delay between complete sequences
    delay ( CYCLE_GAP_MS );
  }
}
