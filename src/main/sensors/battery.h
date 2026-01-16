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
 #  File: \src\main\sensors\battery.h                                          #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Fri, 16th Jan 2026                                          #
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
#include "rx/rx.h"

#define VBAT_SCALE_DEFAULT            25    // Drona
#define VBAT_RESDIVVAL_DEFAULT        10
#define VBAT_RESDIVMULTIPLIER_DEFAULT 10
#define VBAT_SCALE_MIN                0
#define VBAT_SCALE_MAX                255

typedef enum {
  CURRENT_SENSOR_NONE    = 0,
  CURRENT_SENSOR_ADC     = 1,
  CURRENT_SENSOR_VIRTUAL = 2,
  CURRENT_SENSOR_INA219  = 3,
  CURRENT_SENSOR_MAX     = CURRENT_SENSOR_INA219
} currentSensor_e;

typedef struct batteryConfig_s {
  // uint8_t vbatscale;                 // adjust this to match battery voltage to reported value
  // uint8_t vbatresdivval;             // resistor divider R2 (default NAZE 10(K))
  // uint8_t vbatresdivmultiplier;      // multiplier for scale (e.g. 2.5:1 ratio with multiplier of 4 can use '100' instead of '25' in ratio) to get better precision
  // uint8_t vbatmaxcellvoltage;        // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
  uint8_t vBatMaxVoltage;            // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
  // uint8_t vbatmincellvoltage;        // minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
  uint8_t vBatMinVoltage;            // minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
  // uint8_t vbatwarningcellvoltage;    // warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)
  uint8_t vBatWarningVoltage;        // warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)

  // int16_t currentMeterScale;           // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
  // uint16_t currentMeterOffset;         // offset of the current sensor in millivolt steps
  currentSensor_e currentMeterType;    // type of current meter used, either ADC or virtual

  // FIXME this doesn't belong in here since it's a concern of MSP, not of the battery code.
  uint8_t multiwiiCurrentMeterOutput;    // if set to 1 output the mAmpRaw in milliamp steps instead of 0.01A steps via msp
  // uint16_t batteryCapacity;              // mAh
  uint16_t BatteryCapacity;              // mAh
} batteryConfig_t;

typedef enum {
  BATTERY_OK = 0,
  BATTERY_WARNING,
  BATTERY_CRITICAL,
  BATTERY_NOT_PRESENT
} batteryState_e;

extern uint16_t vBatRaw;
extern uint16_t vBatComp;
// extern uint16_t vbatscaled;
extern uint16_t vbatRaw;
// extern uint16_t vbatLatestADC;
extern uint8_t batteryCellCount;
extern uint16_t batteryMaxVoltage;
extern uint16_t batteryWarningVoltage;
extern uint16_t batteryCriticalVoltage;
extern uint16_t batteryCapacity_mAh;
// extern uint16_t amperageLatestADC;
extern uint16_t mAmpRaw;
extern uint16_t mAmpWithGain;
extern uint16_t mAhDrawn;
extern uint16_t mAhRemain;
extern uint16_t battery_capacity_mAh;
extern uint16_t EstBatteryCapacity;
extern uint8_t BatteryWarningMode;
extern float soc_battery_percentage;
extern float soc_mAh_percentage;
extern float soc_Fused;
extern float _ina219_current_gain;
extern float wAh;

batteryState_e getBatteryState ( void );

void batteryInit ( batteryConfig_t *initialBatteryConfig );

// void updateBatteryState ( uint16_t _vBatComp );
// uint16_t computeVbatComp_mV ( uint32_t now, uint32_t vbatTs, uint32_t ibatTs, bool armed, int throttle_us );
void BMS_Update ( uint32_t now, uint32_t vbatTs, uint32_t ibatTs, bool armed, int throttle_us );

void updateINA219Voltage ( void );
void updateINA219Current ( uint32_t now, bool armed );

#ifdef __cplusplus
}
#endif
