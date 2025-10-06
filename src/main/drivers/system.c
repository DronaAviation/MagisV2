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
 #  Last Modified: Mon, 6th Oct 2025                                           #
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

// #define SHORT_FLASH_DURATION 30      // reduce these to make the led blink faster
// #define CODE_FLASH_DURATION  150

// void failureMode ( uint8_t mode )    // DD
// {
//   while ( 1 ) {
//     int codeRepeatsRemaining = 10;
//     int codeFlashesRemaining;
//     int shortFlashesRemaining;
//     while ( codeRepeatsRemaining-- ) {

//       shortFlashesRemaining = 5;
//       codeFlashesRemaining  = 2;
//       uint8_t flashDuration = SHORT_FLASH_DURATION;

//       while ( shortFlashesRemaining || codeFlashesRemaining ) {
//         if ( ( mode & 64 ) == 64 ) {
//           LED_R_TOGGLE;
//         }
//         if ( ( mode & 32 ) == 32 ) {
//           LED_B_TOGGLE;
//         }
//         if ( ( mode & 2 ) == 2 ) {
//           LED_G_TOGGLE;
//         }    // drona
//         BEEP_ON;
//         delay ( flashDuration );

//         if ( ( mode & 64 ) == 64 ) {
//           LED_R_TOGGLE;
//         }
//         if ( ( mode & 32 ) == 32 ) {
//           LED_B_TOGGLE;
//         }
//         if ( ( mode & 2 ) == 2 ) {
//           LED_G_TOGGLE;
//         }
//         BEEP_OFF;
//         delay ( flashDuration );

//         if ( shortFlashesRemaining ) {
//           shortFlashesRemaining--;
//           if ( shortFlashesRemaining == 0 ) {
//             delay ( 500 );
//             flashDuration = CODE_FLASH_DURATION;
//           }
//         } else {
//           codeFlashesRemaining--;
//         }
//       }
//       delay ( 1000 );
//     }
//   }

// #ifdef DEBUG
//   systemReset ( );
// #else
//   systemResetToBootloader ( );
// #endif
// }

// --- Timing (ms) ---
#define HDR_ON_MS          200
#define HDR_OFF_MS         200
#define DETAIL_ON_MS       200
#define DETAIL_OFF_MS      500
#define DETAIL_OFF_CRYSTAL 300
#define CYCLE_GAP_MS       1000

// Small helpers (toggle-based since only *_TOGGLE macros exist)
static inline void blink_red ( uint8_t n, uint16_t on_ms, uint16_t off_ms ) {
  while ( n-- ) {
    LED_R_TOGGLE;
    delay ( on_ms );
    LED_R_TOGGLE;
    delay ( off_ms );
  }
}
static inline void blink_blue ( uint8_t n, uint16_t on_ms, uint16_t off_ms ) {
  while ( n-- ) {
    LED_B_TOGGLE;
    delay ( on_ms );
    LED_B_TOGGLE;
    delay ( off_ms );
  }
}
void failureMode ( uint8_t mode ) {
  // Decode/priority:
  // 1) IMU + BARO → 5× Blue (override)
  // 2) IMU → 3× Blue   (FAILURE_MISSING_ACC and/or FAILURE_ACC_INCOMPATIBLE)
  // 3) BARO → 4× Blue
  // 4) INA219 → 2× Blue
  // 5) CRYSTAL → 6× Blue (shorter OFF time)
  // If multiple are set, the highest item above wins.

  const bool hasIMU  = ( mode & ( 1 << FAILURE_MISSING_ACC ) ) || ( mode & ( 1 << FAILURE_ACC_INCOMPATIBLE ) );
  const bool hasBARO = ( mode & ( 1 << FAILURE_BARO ) );
  const bool hasINA  = ( mode & ( 1 << FAILURE_INA219 ) );
  const bool hasXTAL = ( mode & ( 1 << FAILURE_EXTCLCK ) );

  while ( 1 ) {
    // 2× Red header (indicates "error state active")
    blink_red ( 2, HDR_ON_MS, HDR_OFF_MS );

    // Decide detail pattern (blue count + off timing)
    uint8_t blueCount = 0;
    uint16_t blueOff  = DETAIL_OFF_MS;

    if ( hasIMU && hasBARO ) {
      blueCount = 5;    // IMU + Baro override
    } else if ( hasINA ) {
      blueCount = 2;    // INA219
    } else if ( hasIMU ) {
      blueCount = 3;    // IMU
    } else if ( hasBARO ) {
      blueCount = 4;    // Baro
    } else if ( hasXTAL ) {
      blueCount = 6;                     // Crystal
      blueOff   = DETAIL_OFF_CRYSTAL;    // special gap
    } else {
      // No mapped code: keep only the header and cycle gap.
    }

    if ( blueCount ) {
      blink_blue ( blueCount, DETAIL_ON_MS, blueOff );
    }

    // Gap between complete sequences
    delay ( CYCLE_GAP_MS );
  }
}
