/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\sensors\battery.cpp                                        #
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

#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "flight/failsafe.h"
#include "platform.h"

#include "common/maths.h"

#include "drivers/adc.h"
#include "drivers/system.h"
#include "drivers/ina219.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/battery.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "flight/lowpass.h"
#include "io/beeper.h"

#define BATTERY_CAP_mAH   600
#define BATTERY_V_Max_mV  4200
#define BATTERY_V_Warn_mV 3400
#define BATTERY_V_Min_mV  3200

// #include "drivers/ina219.h"

// ---- Accumulator state ----
static uint32_t last_ms     = 0;
static uint64_t mA_ms_accum = 0;

// Internal state
static uint16_t samples [ BATTERY_BUFFER_SIZE ];    // Array to store voltage samples
static uint32_t sum  = 0;                           // Sum of the samples
static uint8_t head  = 0;                           // Index for circular buffer
static uint8_t count = 0;                           // Number of valid samples

uint16_t initialBatteryCapicity;

#define VBATT_PRESENT_THRESHOLD_MV 10
#define VBATT_LPF_FREQ             10

// Battery monitoring stuff
uint8_t batteryCellCount = 3;    // cell count
uint16_t batteryMaxVoltage;
uint16_t batteryWarningVoltage;
uint16_t batteryCriticalVoltage;
uint16_t batteryCapacity;
uint16_t EstBatteryCapacity;

uint16_t vbat              = 0;    // battery voltage in 0.1V steps (filtered)
uint16_t vbatscaled        = 0;
uint16_t vbatLatestADC     = 0;    // most recent unsmoothed raw reading from vbat ADC
uint16_t amperageLatestADC = 0;    // most recent raw reading from current ADC

int32_t amperage;        // amperage read by current sensor in centiampere (1/100th A)
int32_t mAhDrawn = 0;    // milliampere hours drawn from the battery since start
int32_t mAhRemain = 0;    // milliampere hours drawn from the battery since start

batteryConfig_t *batteryConfig;

static batteryState_e batteryState;
// static lowpass_t lowpassFilter;

// uint16_t batteryAdcToVoltage ( uint16_t src ) {
//   // calculate battery voltage based on ADC reading
//   // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
//   vbatscaled = ( ( ( uint32_t ) src * 330 ) / 4095 );
//   return ( ( ( ( ( uint32_t ) src * batteryConfig->vbatscale * 343 + ( 0xFFF * 50 ) ) / ( 0xFFF * batteryConfig->vbatresdivval ) ) ) / batteryConfig->vbatresdivmultiplier );
// }

// static void updateBatteryVoltage ( void ) {
//   uint16_t vbatSample;
//   uint16_t vbatFiltered;
//   //! Old Adc battery detection
//   // store the battery voltage with some other recent battery voltage readings
//   // vbatSample = vbatLatestADC = adcGetChannel(ADC_BATTERY);
//   //   vbatFiltered = (uint16_t) lowpassFixed(&lowpassFilter, vbatSample, VBATT_LPF_FREQ);
//   // vbat = batteryAdcToVoltage(vbatFiltered);
//   // vbat = batteryAdcToVoltage(3628);
//   //! NEW : INA219 Battery Measuring
//   // vbat = bus_voltage ( );
//   vbat = ProcessedVoltage ( );
// }

#define VBATTERY_STABLE_DELAY 40
/* Batt Hysteresis of +/-100mV */
#define VBATT_HYSTERESIS 100

// void updateBattery ( void ) {
//   updateBatteryVoltage ( );

//   /* battery has just been connected*/
//   if ( batteryState == BATTERY_NOT_PRESENT && vbat > VBATT_PRESENT_THRESHOLD_MV ) {
//     /* Actual battery state is calculated below, this is really BATTERY_PRESENT */
//     batteryState = BATTERY_OK;
//     /* wait for VBatt to stabilise then we can calc number of cells
//      (using the filtered value takes a long time to ramp up)
//      We only do this on the ground so don't care if we do block, not
//      worse than original code anyway*/
//     delay ( VBATTERY_STABLE_DELAY );
//     updateBatteryVoltage ( );

//     unsigned cells = ( batteryAdcToVoltage ( vbatLatestADC ) / batteryConfig->vbatmaxcellvoltage ) + 1;
//     if ( cells > 8 ) {
//       // something is wrong, we expect 8 cells maximum (and autodetection will be problematic at 6+ cells)
//       cells = 8;
//     }
//     batteryCellCount       = cells;
//     batteryWarningVoltage  = batteryCellCount * batteryConfig->vbatwarningcellvoltage;
//     batteryCriticalVoltage = batteryCellCount * batteryConfig->vbatmincellvoltage;
//   }
//   /* battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of VBATT_PRESENT_THRESHOLD_MV */
//   else if ( batteryState != BATTERY_NOT_PRESENT && vbat <= VBATT_PRESENT_THRESHOLD_MV ) {
//     batteryState           = BATTERY_NOT_PRESENT;
//     batteryCellCount       = 0;
//     batteryWarningVoltage  = 0;
//     batteryCriticalVoltage = 0;
//   }

//   switch ( batteryState ) {
//     case BATTERY_OK:
//       if ( vbat <= ( batteryWarningVoltage - VBATT_HYSTERESIS ) ) {
//         batteryState = BATTERY_WARNING;
//         beeper ( BEEPER_BAT_LOW );
//       }
//       break;
//     case BATTERY_WARNING:
//       DISABLE_ARMING_FLAG ( PREVENT_ARMING );
//       if ( vbat <= ( batteryCriticalVoltage - VBATT_HYSTERESIS ) ) {
//         batteryState = BATTERY_CRITICAL;
//         beeper ( BEEPER_BAT_CRIT_LOW );
//       } else if ( vbat > ( batteryWarningVoltage + VBATT_HYSTERESIS ) ) {
//         batteryState = BATTERY_OK;
//       } else {
//         beeper ( BEEPER_BAT_LOW );
//         // failsafeOnLowBattery();
//       }
//       failsafeOnLowBattery ( );
//       break;
//     case BATTERY_CRITICAL:

//       if ( vbat > ( batteryCriticalVoltage + VBATT_HYSTERESIS ) ) {
//         batteryState = BATTERY_WARNING;
//         beeper ( BEEPER_BAT_LOW );
//       } else {
//         beeper ( BEEPER_BAT_CRIT_LOW );
//       }
//       failsafeOnLowBattery ( );

//       break;
//     case BATTERY_NOT_PRESENT:
//       break;
//   }
// }

batteryState_e getBatteryState ( void ) {
  return batteryState;
}

const char *const batteryStateStrings [] = { "OK", "WARNING", "CRITICAL", "NOT PRESENT" };

// const char *getBatteryStateString ( void ) {
//   return batteryStateStrings [ batteryState ];
// }

void batteryInit ( batteryConfig_t *initialBatteryConfig ) {
  batteryConfig          = initialBatteryConfig;
  batteryState           = BATTERY_NOT_PRESENT;
  batteryCellCount       = 1;
  batteryMaxVoltage      = 0;
  batteryWarningVoltage  = 0;
  batteryCriticalVoltage = 0;
  EstBatteryCapacity     = 0;
  initialBatteryCapicity = 0;

  memset ( samples, 0, sizeof ( samples ) );    // Use memset for efficient initialization
  sum   = 0;
  head  = 0;
  count = 0;
#ifdef INA219_Current
  last_ms     = millis ( );
  mA_ms_accum = 0;
#endif
}

#define ADCVREF 3300    // in mV
int32_t currentSensorToCentiamps ( uint16_t src ) {
  int32_t millivolts;

  millivolts = ( ( uint32_t ) src * ADCVREF ) / 4096;
  millivolts -= batteryConfig->currentMeterOffset;

  return ( millivolts * 1000 ) / ( int32_t ) batteryConfig->currentMeterScale;    // current in 0.01A steps
}

// void updateCurrentMeter ( int32_t lastUpdateAt, rxConfig_t *rxConfig, uint16_t deadband3d_throttle ) {
//   static int32_t amperageRaw = 0;
//   static int64_t mAhdrawnRaw = 0;
//   int32_t throttleOffset     = ( int32_t ) rcCommand [ THROTTLE ] - 1000;
//   int32_t throttleFactor     = 0;

//   switch ( batteryConfig->currentMeterType ) {
//     case CURRENT_SENSOR_ADC:
//       amperageRaw -= amperageRaw / 8;
//       amperageRaw += ( amperageLatestADC = adcGetChannel ( ADC_CURRENT ) );
//       amperage = currentSensorToCentiamps ( amperageRaw / 8 );
//       break;
//     case CURRENT_SENSOR_VIRTUAL:
//       amperage = ( int32_t ) batteryConfig->currentMeterOffset;
//       if ( ARMING_FLAG ( ARMED ) ) {
//         throttleStatus_e throttleStatus = calculateThrottleStatus ( rxConfig, deadband3d_throttle );
//         if ( throttleStatus == THROTTLE_LOW && feature ( FEATURE_MOTOR_STOP ) )
//           throttleOffset = 0;
//         throttleFactor = throttleOffset + ( throttleOffset * throttleOffset / 50 );
//         amperage += throttleFactor * ( int32_t ) batteryConfig->currentMeterScale / 1000;
//       }
//       break;
//     case CURRENT_SENSOR_INA219:
//       break;
//     case CURRENT_SENSOR_NONE:
//       amperage = 0;
//       break;
//   }

//   mAhdrawnRaw += ( amperage * lastUpdateAt ) / 1000;
//   // mAhDrawn = mAhdrawnRaw / ( 3600 * 100 );
//   mAhDrawn = current_reading ( );
// }

// uint8_t calculateBatteryPercentage ( void ) {
//   return ( ( ( uint32_t ) vbat - ( batteryConfig->vbatmincellvoltage * batteryCellCount ) ) * 100 ) / ( ( batteryConfig->vbatmaxcellvoltage - batteryConfig->vbatmincellvoltage ) * batteryCellCount );
// }

// uint8_t calculateBatteryCapacityRemainingPercentage ( void ) {
//   uint16_t batteryCapacity = batteryConfig->batteryCapacity;

//   return constrain ( ( batteryCapacity - constrain ( mAhDrawn, 0, 0xFFFF ) ) * 100.0f / batteryCapacity, 0, 100 );
// }

/**
 * @brief Updates the battery voltage readings with a new ADC reading.
 *
 * @param adc_reading The latest ADC reading of the battery voltage.
 *
 * This function updates the circular buffer with the new ADC reading,
 * adjusts the `sum` by removing the oldest sample and adding the new one,
 * and updates the `head` index. If the buffer is not yet full, it increments
 * the `count` of valid samples.
 */
void UpdateINA219Battery ( uint16_t adc_reading ) {
  sum -= samples [ head ];           // Remove old value from sum
  samples [ head ] = adc_reading;    // Insert new value
  sum += adc_reading;                // Add new value to sum

  head = ( head + 1 ) % BATTERY_BUFFER_SIZE;    // Move head circularly

  if ( count < BATTERY_BUFFER_SIZE ) {
    count++;
  }
}

/**
 * @brief Processes and returns the average battery voltage.
 *
 * @return The average voltage based on current samples.
 *
 * This function updates the voltage readings with the current bus voltage,
 * then calculates and returns the average voltage. If no valid samples are
 * available, it returns 0.
 */
uint16_t ProcessedINA219Voltage ( void ) {
  UpdateINA219Battery ( bus_voltage ( ) );    // Update with current voltage reading

  if ( count == 0 ) return 0;    // Return 0 if no valid samples

  return ( uint16_t ) ( sum / count );    // Calculate and return the average
}

uint16_t ProcessedINA219Current ( void ) {
  int16_t mV = shunt_voltage ( );    // signed mV from shunt

  if ( mV <= 0 ) return 0;    // no reverse/negative current in telemetry

  // // mA = mV × 1000 / R_mΩ
  amperage = ( uint32_t ) ( ( mV / 10.0f ) / INA219_SHUNT_RESISTOR_MILLIOHM );

  if ( amperage > 0xFFFF ) amperage = 0xFFFF;    // clamp to uint16_t max

  return ( uint16_t ) amperage;
}

void updateINA219Voltage ( void ) {

  vbat = ProcessedINA219Voltage ( );

  /* battery has just been connected*/
  if ( batteryState == BATTERY_NOT_PRESENT && vbat > VBATT_PRESENT_THRESHOLD_MV ) {
    /* Actual battery state is calculated below, this is really BATTERY_PRESENT */
    batteryState           = BATTERY_OK;
    batteryCapacity        = BATTERY_CAP_mAH;
    batteryMaxVoltage      = BATTERY_V_Max_mV;
    batteryCriticalVoltage = batteryCellCount * BATTERY_V_Min_mV;
    batteryWarningVoltage  = batteryCellCount * BATTERY_V_Warn_mV;
    /* wait for VBatt to stabilise then we can calc number of cells
     (using the filtered value takes a long time to ramp up)
     We only do this on the ground so don't care if we do block, not
     worse than original code anyway*/
    delay ( VBATTERY_STABLE_DELAY );
    // updateBatteryVoltage ( );

    unsigned cells = ( vbat / ( batteryMaxVoltage / 100u ) ) + 1;
    if ( cells > 2 ) {
      // something is wrong, we expect 8 cells maximum (and autodetection will be problematic at 6+ cells)
      cells = 8;
    }
    batteryCellCount = cells;
#ifdef INA219_Current
    uint16_t v_u100mV      = vbat * 100;
    initialBatteryCapicity = ( ( v_u100mV - batteryCriticalVoltage ) * batteryCapacity ) / ( batteryMaxVoltage - batteryCriticalVoltage );
#endif
  }
  /* battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of VBATT_PRESENT_THRESHOLD_MV */
  else if ( batteryState != BATTERY_NOT_PRESENT && vbat <= VBATT_PRESENT_THRESHOLD_MV ) {
    batteryState           = BATTERY_NOT_PRESENT;
    batteryCellCount       = 0;
    batteryWarningVoltage  = 0;
    batteryCriticalVoltage = 0;
    batteryMaxVoltage      = 0;
    batteryCapacity        = 0;
    initialBatteryCapicity = 0;
  }

  switch ( batteryState ) {
    case BATTERY_OK:
      if ( vbat <= ( batteryWarningVoltage - VBATT_HYSTERESIS ) ) {
        batteryState = BATTERY_WARNING;
        beeper ( BEEPER_BAT_LOW );
      }
      break;
    case BATTERY_WARNING:
      DISABLE_ARMING_FLAG ( PREVENT_ARMING );
      if ( vbat <= ( batteryCriticalVoltage - VBATT_HYSTERESIS ) ) {
        batteryState = BATTERY_CRITICAL;
        beeper ( BEEPER_BAT_CRIT_LOW );
      } else if ( vbat > ( batteryWarningVoltage + VBATT_HYSTERESIS ) ) {
        batteryState = BATTERY_OK;
      } else {
        beeper ( BEEPER_BAT_LOW );
        // failsafeOnLowBattery();
      }
      failsafeOnLowBattery ( );
      break;
    case BATTERY_CRITICAL:

      if ( vbat > ( batteryCriticalVoltage + VBATT_HYSTERESIS ) ) {
        batteryState = BATTERY_WARNING;
        beeper ( BEEPER_BAT_LOW );
      } else {
        beeper ( BEEPER_BAT_CRIT_LOW );
      }
      failsafeOnLowBattery ( );

      break;
    case BATTERY_NOT_PRESENT:
      break;
  }
}

void updateINA219Current ( void ) {
  uint32_t now = millis ( );
  uint32_t dt  = now - last_ms;    // ok across wrap (unsigned arithmetic)
  last_ms      = now;
  uint16_t mA  = ProcessedINA219Current ( );    // your integer mA (clamped, non-negative)
  mA_ms_accum += ( uint64_t ) mA * ( uint64_t ) dt;

  mAhDrawn = mA_ms_accum / 3600000ULL;    // convert mA·ms → mAh
  if ( mAhDrawn > 0xFFFFULL ) {
    mAhDrawn = 0xFFFF;
  }

  // Get whole mAh (uint16_t), clamped for MSP serialize16()
  mAhRemain = ( uint16_t ) initialBatteryCapicity - mAhDrawn;
}

// void BatteryCapacityEst ( void ) {
//   uint16_t v_u100mV = bus_voltage ( ) * 100;    // from your INA219 in mV
//   // uint32_t v_mV          = ( uint32_t ) v_u100mV * 100u;
// }

uint8_t BatteryCellCount ( void ) {
  return batteryCellCount;
}

uint16_t InitialBatteryCap ( void ) {
  return initialBatteryCapicity;
}