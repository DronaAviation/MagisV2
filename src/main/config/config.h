/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 MechAsh (j.mechash@gmail.com)                 #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\config\config.h                                            #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sun, 10th Aug 2025                                          #
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
#define MAX_PROFILE_COUNT                        3
#define MAX_CONTROL_RATE_PROFILE_COUNT           3
#define ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS 1500

typedef enum {
  FEATURE_RX_PPM             = 1 << 0,
  FEATURE_INA219_VBAT        = 1 << 1,
  FEATURE_INFLIGHT_ACC_CAL   = 1 << 2,
  FEATURE_RX_SERIAL          = 1 << 3,
  FEATURE_MOTOR_STOP         = 1 << 4,
  FEATURE_SERVO_TILT         = 1 << 5,
  FEATURE_SOFTSERIAL         = 1 << 6,
  FEATURE_GPS                = 1 << 7,
  FEATURE_FAILSAFE           = 1 << 8,
  FEATURE_SONAR              = 1 << 9,
  FEATURE_TELEMETRY          = 1 << 10,
  FEATURE_INA219_CBAT        = 1 << 11,
  FEATURE_3D                 = 1 << 12,
  FEATURE_RX_PARALLEL_PWM    = 1 << 13,
  FEATURE_RX_MSP             = 1 << 14,
  FEATURE_RSSI_ADC           = 1 << 15,
  FEATURE_LED_STRIP          = 1 << 16,
  FEATURE_DISPLAY            = 1 << 17,
  FEATURE_ONESHOT125         = 1 << 18,
  FEATURE_BLACKBOX           = 1 << 19,
  FEATURE_CHANNEL_FORWARDING = 1 << 20
} features_e;

void handleOneshotFeatureChangeOnRestart ( void );
void latchActiveFeatures ( void );
bool featureConfigured ( uint32_t mask );
bool feature ( uint32_t mask );
void featureSet ( uint32_t mask );
void featureClear ( uint32_t mask );
void featureClearAll ( void );
uint32_t featureMask ( void );

void copyCurrentProfileToProfileSlot ( uint8_t profileSlotIndex );

void initEEPROM ( void );
void resetEEPROM ( void );
void readEEPROM ( void );
void readEEPROMAndNotify ( void );
void writeEEPROM ( );
void ensureEEPROMContainsValidData ( void );
void saveConfigAndNotify ( void );

uint8_t getCurrentProfile ( void );
void changeProfile ( uint8_t profileIndex );

uint8_t getCurrentControlRateProfile ( void );
void changeControlRateProfile ( uint8_t profileIndex );

bool canSoftwareSerialBeUsed ( void );

uint16_t getCurrentMinthrottle ( void );
#ifdef __cplusplus
}
#endif
