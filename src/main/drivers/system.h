/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\system.h                                           #
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
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void systemInit ( void );
void delayMicroseconds ( uint32_t us );
void delay ( uint32_t ms );

uint32_t micros ( void );
uint32_t millis ( void );

#ifdef TEST_ENABLE
// failure
void failureMode ( uint16_t mode );
#endif

// bootloader/IAP
void systemReset ( void );
void systemResetToBootloader ( void );
bool isMPUSoftReset ( void );

void enableGPIOPowerUsageAndNoiseReductions ( void );
// current crystal frequency - 8 or 12MHz
extern uint32_t hse_value;

typedef void extiCallbackHandlerFunc ( void );

void registerExtiCallbackHandler ( IRQn_Type irqn, extiCallbackHandlerFunc *fn );
void unregisterExtiCallbackHandler ( IRQn_Type irqn, extiCallbackHandlerFunc *fn );

extern uint32_t cachedRccCsrValue;

typedef enum {    // DD
  FAILURE_DEVELOPER = 0,
  FAILURE_MISSING_ACC,
  // FAILURE_ACC_INIT,
  FAILURE_ACC_INCOMPATIBLE,
  FAILURE_INVALID_EEPROM_CONTENTS,
  FAILURE_FLASH_WRITE_FAILED,
  // FAILURE_GYRO_INIT_FAILED
  FAILURE_BARO,
  FAILURE_EXTCLCK,
  FAILURE_INA219,
  FAILURE_BARO_DRIFT,
  FAILURE_PAW3903,
  FAILURE_VL53L1X
} failureMode_e;

extern uint16_t failureFlag;

#ifdef __cplusplus
}
#endif
