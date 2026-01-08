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
 #  Last Modified: Thu, 8th Jan 2026                                           #
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

#include "API/API-Utils.h"

batteryConfig_t *batteryConfig;

ring_avg_u16_t vbatRawAvgRing;
#define vBAT_RAW_BUFFER_SIZE 50
static uint16_t vBatRawSamples [ vBAT_RAW_BUFFER_SIZE ];

ring_avg_u16_t vShuntRawAvgRing;
#define vSHUNT_RAW_BUFFER_SIZE 50
static uint16_t vShuntRawSamples [ vSHUNT_RAW_BUFFER_SIZE ];

ring_avg_u16_t vbatSagAvgRing;
#define vBAT_SAG_BUFFER_SIZE 20
static uint16_t vBatCompSamples [ vBAT_SAG_BUFFER_SIZE ];

ring_avg_u16_t mAhAvgRing;
#define mAh_BUFFER_SIZE 10
static uint16_t mAhSamples [ mAh_BUFFER_SIZE ];

// uint16_t vbatscaled        = 0;
// uint16_t vbatLatestADC     = 0;    // most recent unsmoothed raw reading from vBatRaw ADC
// uint16_t amperageLatestADC = 0;    // most recent raw reading from current ADC

// #define VBATT_LPF_FREQ             10

// Function getBatteryState
static batteryState_e batteryState;

// Function batteryInit

// Function handleBatteryConnected
#define VBATTERY_STABLE_DELAY 40
#define VBATT_HYSTERESIS      100    // Batt Hysteresis of +/-100mV

uint16_t battery_capacity_mAh   = 0;
uint8_t batteryCellCount        = 1;    // cell count
uint16_t batteryMaxVoltage      = 0;
uint16_t batteryWarningVoltage  = 0;
uint16_t batteryCriticalVoltage = 0;
uint16_t batteryCapacity_mAh    = 0;
uint16_t EstBatteryCapacity     = 0;

// Function handleBatteryDisconnected

// Function updateINA219Voltage
#define VBATT_PRESENT_THRESHOLD_MV 10
uint16_t vBatRaw = 0;    // battery voltage in 0.1V steps (filtered)

// Function ina219_auto_calibrate_current
#define SYSTEM_R_MOHM           100.0f    // effective system resistance (battery + wiring + PCB)
#define CURR_CAL_ALPHA          0.002f    // convergence speed (0.001–0.005 recommended)
#define CURR_CAL_MIN_GAIN       0.95f
#define CURR_CAL_MAX_GAIN       1.05f
#define CURR_CAL_MIN_CURRENT_MA 1000    // only learn above this current
#define CURR_CAL_MIN_SAG_MV     20      // ignore noise floor

float _ina219_current_gain = 1.0f;

// Function ProcessedINA219Current
uint16_t vShuntRaw    = 0;
uint16_t mAmpRaw      = 0;    // mAmpRaw read by current sensor in centiampere (1/100th A)
uint16_t mAmpWithGain = 0;
uint16_t mAhDrawn     = 0;    // milliampere hours drawn from the battery since start
uint16_t mAhRemain    = 0;    // milliampere hours drawn from the battery since start

// Function updateINA219Current
static uint32_t last_ms     = 0;
static uint64_t mA_ms_accum = 0;

// Function computeVbatComp_mV
#define VBAT_SAG_R_MOHM ( uint32_t ) SYSTEM_R_MOHM    // from your calc (~100 mΩ)
#define VBAT_SAG_MAX_MV 700                           // clamp compensation

#define VBAT_STALE_MS   200    // how old vBat can be to trust compensation
#define IBAT_STALE_MS   200    // how old mAmpRaw can be to trust current-sag

#define SAG_THR_MAX_MV  450    // throttle-only sag at full stick (fallback/assist)
#define THR_MIN_US      1000
#define THR_MAX_US      2000
#define THR_IDLE_US     1150

#define THR_ASSIST_PCT  20    // extra % throttle sag added when current is valid

// Function soc_from_mAh

// Function soc_linear_from_voltage

// Function fuse_soc_vbatComp_smart
float wAh = 0;

// Function updateBatteryStateSoc
#define SOC_WARN_PCT 20
#define SOC_CRIT_PCT 10
#define SOC_HYST_PCT 2

uint8_t BatteryWarningMode = 0;

// Function BMS_Update variables
float soc_battery_percentage = 0;
float soc_mAh_percentage     = 0;
float soc_Fused              = 0;
uint16_t vBatComp            = 0;

batteryState_e getBatteryState ( void ) {
  // Return the current battery state stored in the variable 'batteryState'
  return batteryState;
}

/**
 * @brief Initializes the battery configuration and related parameters.
 *
 * This function sets up the battery configuration by assigning the provided initial configuration to the
 * global battery configuration pointer. It initializes the battery state to indicate absence and prepares
 * various averaging rings for voltage and current measurements, ensuring accurate monitoring of battery
 * parameters.
 *
 * @param initialBatteryConfig Pointer to the initial battery configuration structure.
 */
void batteryInit ( batteryConfig_t *initialBatteryConfig ) {
  // Assign the initial configuration to the global battery configuration pointer
  batteryConfig        = initialBatteryConfig;
  battery_capacity_mAh = batteryConfig->BatteryCapacity;

  // Initialize battery state and parameters
  batteryState = BATTERY_NOT_PRESENT;    // Set the initial state as battery not present

  ring_avg_u16_init ( &vbatRawAvgRing, vBatRawSamples, vBAT_RAW_BUFFER_SIZE );
  ring_avg_u16_init ( &vShuntRawAvgRing, vShuntRawSamples, vSHUNT_RAW_BUFFER_SIZE );
  ring_avg_u16_init ( &vbatSagAvgRing, vBatCompSamples, vBAT_SAG_BUFFER_SIZE );
  ring_avg_u16_init ( &mAhAvgRing, mAhSamples, mAh_BUFFER_SIZE );
}

/**
 * @brief Handles the initialization and configuration of battery parameters upon connection.
 *
 * This function sets the initial battery state to indicate that the battery is connected. It initializes
 * various battery parameters, such as capacity, maximum voltage, and critical/warning voltage levels based
 * on predefined configurations. The function also estimates the number of battery cells and calculates the
 * estimated remaining battery capacity if the INA219_Current feature is enabled.
 */
static inline void handleBatteryConnected ( ) {
  // Set the initial battery state to indicate that the battery is connected and operational.
  batteryState = BATTERY_OK;

  // Initialize battery parameters with predefined values.
  batteryCapacity_mAh = battery_capacity_mAh;                                    // Set the battery capacity in milliamp hours.
  batteryMaxVoltage   = ( uint16_t ) ( batteryConfig->vBatMaxVoltage * 100 );    // Set the maximum voltage of the battery in millivolts.

  // Calculate critical and warning voltage levels based on the number of cells and predefined constants.
  batteryCriticalVoltage = batteryCellCount * ( uint16_t ) ( batteryConfig->vBatMinVoltage * 100 );        // Voltage level considered critically low.
  batteryWarningVoltage  = batteryCellCount * ( uint16_t ) ( batteryConfig->vBatWarningVoltage * 100 );    // Voltage level considered a warning threshold.

  // Delay to allow the battery voltage to stabilize after connection.
  delay ( VBATTERY_STABLE_DELAY );

  // Estimate the number of battery cells based on the current voltage.
  unsigned cells = ( vBatRaw / ( batteryMaxVoltage / 100u ) ) + 1;    // Calculate the cell count estimation.
  if ( cells > 8 ) {
    cells = 8;    // Clamp the number of cells to a maximum of 8 if estimation exceeds this value.
  }
  batteryCellCount = cells;    // Update the battery cell count.

#ifdef INA219_Current
  // If the INA219_Current feature is enabled, estimate the remaining battery capacity.
  uint16_t v_u100mV  = vBatRaw * 100;    // Convert battery voltage to hundred-millivolt units for calculations.
  EstBatteryCapacity = ( ( v_u100mV - batteryCriticalVoltage ) * batteryCapacity_mAh ) / ( batteryMaxVoltage - batteryCriticalVoltage );
  // Calculate estimated battery capacity as a percentage of total capacity.
#endif
}

static inline void handleBatteryDisconnected ( ) {
  batteryState           = BATTERY_NOT_PRESENT;
  batteryCellCount       = 0;
  batteryWarningVoltage  = 0;
  batteryCriticalVoltage = 0;
  batteryMaxVoltage      = 0;
  batteryCapacity_mAh    = 0;
  EstBatteryCapacity     = 0;
}

/**
 * @brief Updates the battery voltage reading from the INA219 sensor and manages battery connection state.
 *
 * This function processes the voltage reading from the INA219 sensor and updates the raw battery voltage
 * (`vBatRaw`). It also checks the current battery connection state to determine if a battery has been
 * connected or disconnected based on the voltage threshold. The appropriate handling functions are called
 * to manage these state changes.
 */
void updateINA219Voltage ( ) {
  // Process the voltage reading from the INA219 sensor and store it in vBatRaw
  // vBatRaw = ProcessedINA219Voltage ( );
  vBatRaw = ring_avg_u16_get ( &vbatRawAvgRing, bus_voltage ( ) );

  // Check if the battery is currently not present but the voltage indicates otherwise
  if ( batteryState == BATTERY_NOT_PRESENT && vBatRaw > VBATT_PRESENT_THRESHOLD_MV ) {
    // Handle scenario where a battery has been connected
    handleBatteryConnected ( );
  }
  // Check if the battery is present but the voltage indicates disconnection
  else if ( batteryState != BATTERY_NOT_PRESENT && vBatRaw <= VBATT_PRESENT_THRESHOLD_MV ) {
    // Handle scenario where the battery has been disconnected
    handleBatteryDisconnected ( );
  }
}

/**
 * @brief Automatically calibrates the INA219 current measurement using observed versus expected voltage sag.
 *
 * This function dynamically adjusts the gain applied to current measurements from the INA219 sensor
 * based on the difference between observed and expected voltage sag. It ensures that the calibrated
 * current values are accurate, particularly in armed states where precision is critical. The gain is
 * clamped to prevent runaway learning and updated using an Exponential Moving Average (EMA) approach.
 *
 * @param _mAmpRaw The raw current measurement in milliamps.
 * @param armed A boolean indicating whether the system is armed for precise calibration.
 * @return float The updated gain value for current calibration.
 */
static inline float ina219_auto_calibrate_current ( uint16_t _mAmpRaw, bool armed ) {
  static float ina219_current_gain = 1.0f;

  if ( ! armed || _mAmpRaw < CURR_CAL_MIN_CURRENT_MA || vShuntRaw <= vBatRaw ) {
    return ina219_current_gain;
  }

  float sag_obs_mV = static_cast< float > ( vShuntRaw - vBatRaw );
  if ( sag_obs_mV < CURR_CAL_MIN_SAG_MV )
    return ina219_current_gain;

  float sag_exp_mV = ( static_cast< float > ( _mAmpRaw ) * SYSTEM_R_MOHM ) / 1000.0f;
  if ( sag_exp_mV < CURR_CAL_MIN_SAG_MV )
    return ina219_current_gain;

  float gain_err = sag_obs_mV / sag_exp_mV;

  // Hard clamp to prevent runaway learning
  if ( gain_err < CURR_CAL_MIN_GAIN ) gain_err = CURR_CAL_MIN_GAIN;
  if ( gain_err > CURR_CAL_MAX_GAIN ) gain_err = CURR_CAL_MAX_GAIN;

  // Slow convergence (EMA-style)
  ina219_current_gain += INA219_SHUNT_RESISTOR_MILLIOHM * ( gain_err - ina219_current_gain );
  _ina219_current_gain = ina219_current_gain;
  return ina219_current_gain;
}

/**
 * @brief Processes the INA219 current reading, applying calibration and limits.
 *
 * This function calculates the processed current from the raw shunt voltage readings obtained from the
 * INA219 sensor. It converts the voltage to milliamps, applies a gain factor through auto-calibration,
 * and ensures the resulting value fits within a 16-bit unsigned integer limit. The function is designed
 * to handle both armed and unarmed states for accurate current measurement.
 *
 * @param armed A boolean indicating whether the system is armed.
 * @return uint16_t The processed current in milliamps as a 16-bit unsigned integer.
 */
static inline uint16_t ProcessedINA219Current ( bool armed ) {
  vShuntRaw = ring_avg_u16_get ( &vShuntRawAvgRing, shunt_voltage ( ) );

  if ( vShuntRaw <= 0 ) {
    return 0;    // Return 0 if the current is negative or zero.
  }

  mAmpRaw = static_cast< uint32_t > ( ( vShuntRaw / 10.0f ) / INA219_SHUNT_RESISTOR_MILLIOHM );

  if ( mAmpRaw > 0xFFFF ) {
    mAmpRaw = 0xFFFF;    // Clamp to the maximum value of a 16-bit unsigned integer.
  }

  float mAmpGain = ina219_auto_calibrate_current ( mAmpRaw, armed );

  return static_cast< uint16_t > ( mAmpRaw * mAmpGain );    // Return the calculated current as a 16-bit unsigned integer.
}

/**
 * @brief Updates the INA219 current measurements and computes battery consumption metrics.
 *
 * This function initializes tracking variables on its first call and subsequently updates the accumulated
 * current draw over time. It calculates the milliamps drawn (mAh) and the remaining battery capacity based
 * on the elapsed time and current readings from the INA219 sensor. The function is designed to be called
 * regularly with the current timestamp and armed state of the system to ensure accurate energy consumption
 * tracking.
 *
 * @param nowUs Current timestamp in microseconds.
 * @param armed A boolean indicating whether the system is armed.
 */
void updateINA219Current ( uint32_t nowUs, bool armed ) {
  static bool init = false;

  if ( ! init ) {
    init        = true;
    last_ms     = nowUs;    // rename last_ms -> last_us ideally
    mA_ms_accum = 0;
    mAhDrawn    = 0;
    mAhRemain   = EstBatteryCapacity;
    return;
  }

  uint32_t dtUs = nowUs - last_ms;
  last_ms       = nowUs;

  // Convert µs -> ms (integer)
  uint32_t dtMs = dtUs / 1000U;
  if ( dtMs == 0 ) return;    // too fast to matter / avoid zero ms integration

  mAmpWithGain = ProcessedINA219Current ( armed );

  mA_ms_accum += ( uint64_t ) mAmpWithGain * ( uint64_t ) dtMs;

  uint32_t mAh32 = ( uint32_t ) ( mA_ms_accum / 3600000ULL );
  if ( mAh32 > 0xFFFFU ) mAh32 = 0xFFFFU;

  mAhDrawn = ( uint16_t ) mAh32;

  mAhRemain = ( uint16_t ) ( EstBatteryCapacity - mAhDrawn );
}

/**
 * @brief Computes the compensated battery voltage in millivolts, accounting for throttle and current-induced voltage sag.
 *
 * This function calculates the voltage compensation based on throttle input and, if available, current sensing data.
 * It incorporates throttle sag as a fallback and adds current sag when both voltage and current readings are fresh.
 * The result is constrained to avoid exceeding the maximum defined sag. The final compensated voltage is averaged
 * using a ring buffer to provide a stable output.
 *
 * @param now Current time in milliseconds.
 * @param vbatTs Timestamp of the last battery voltage reading.
 * @param ibatTs Timestamp of the last battery current reading.
 * @param armed A boolean indicating whether the system is armed.
 * @param throttle_us Throttle input in microseconds.
 *
 * @return Compensated battery voltage in millivolts as an unsigned 16-bit integer.
 */
static inline uint16_t computeVbatComp_mV ( uint32_t now, uint32_t vbatTs, uint32_t ibatTs, bool armed, int throttle_us ) {
  // 1) Throttle sag (always available: fallback + assist)
  int32_t t        = constrain ( throttle_us, THR_MIN_US, THR_MAX_US );
  uint32_t thr01   = ( uint32_t ) ( t - THR_MIN_US );    // 0..1000
  uint32_t sagT_mV = ( SAG_THR_MAX_MV * thr01 ) / 1000U;

  if ( ! armed || throttle_us < THR_IDLE_US ) {
    sagT_mV = 0;
  }

  // 2) Current sag (only if feature enabled AND both signals are fresh)
  uint32_t sagI_mV      = 0;
  const bool hasCurrent = feature ( FEATURE_INA219_CBAT );

  if ( hasCurrent && armed && throttle_us >= THR_IDLE_US ) {

    const bool vFresh = ( now - vbatTs ) <= VBAT_STALE_MS;
    const bool iFresh = ( now - ibatTs ) <= IBAT_STALE_MS;

    if ( vFresh && iFresh ) {
      // sag(mV) = I(mA) * R(mΩ) / 1000
      sagI_mV = ( ( uint16_t ) mAmpRaw * ( uint32_t ) VBAT_SAG_R_MOHM ) / 1000U;
    }
  }

  // 3) Fuse
  // - If current sag valid: use it + small throttle assist
  // - Else: use throttle sag only
  uint32_t sag_mV = sagT_mV;
  if ( sagI_mV > 0 ) {
    sag_mV = sagI_mV + ( sagT_mV * THR_ASSIST_PCT ) / 100U;
  }

  sag_mV = constrain ( sag_mV, 0, VBAT_SAG_MAX_MV );

  // 4) Compensated voltage
  uint32_t vcomp = ( uint32_t ) ( vBatRaw * 100 ) + sag_mV;
  if ( vcomp > 65535U ) vcomp = 65535U;

  return ( uint16_t ) ring_avg_u16_get ( &vbatSagAvgRing, vcomp );
}

/**
 * @brief Computes the state of charge (SoC) percentage from the remaining battery capacity in milliamp-hours.
 *
 * This function calculates the SoC based on the remaining milliamp-hours (mAh) compared to the total battery capacity.
 * It handles edge cases where the battery capacity is zero or when the remaining mAh is greater than or equal to the capacity,
 * returning 0% and 100% respectively. Otherwise, it returns the calculated percentage.
 *
 * @return The calculated SoC percentage as a float.
 */
static inline float soc_from_mAh ( ) {
  // Return 0% if battery capacity is zero to avoid division by zero
  if ( batteryCapacity_mAh == 0 )
    return 0.0f;

  // Return 100% if remaining mAh is greater than or equal to battery capacity
  if ( mAhRemain >= batteryCapacity_mAh )
    return 100.0f;

  // Calculate the percentage of remaining mAh
  constexpr float max_pct = 100.0f;
  float pct               = ( static_cast< float > ( mAhRemain ) * max_pct ) / static_cast< float > ( batteryCapacity_mAh );

  // Return the calculated percentage
  return pct;
}

/**
 * @brief Computes the state of charge (SoC) percentage from a given battery voltage in millivolts.
 *
 * This function calculates the linear SoC percentage based on the provided battery voltage.
 * It uses predefined constants to determine the critical and maximum voltage levels,
 * and ensures that the returned percentage does not exceed 100%.
 *
 * @param v_mV The battery voltage in millivolts.
 * @return The calculated SoC percentage as a float.
 */
static inline float soc_linear_from_voltage ( uint16_t v_mV ) {
  // Calculate percentage using constants directly
  constexpr float max_pct = 100.0f;
  float num               = static_cast< float > ( v_mV - ( batteryCriticalVoltage - VBATT_HYSTERESIS ) ) * max_pct;
  float den               = static_cast< float > ( batteryMaxVoltage - ( batteryCriticalVoltage - VBATT_HYSTERESIS ) );

  // Return calculated percentage, ensuring it does not exceed maximum percentage
  return num / den;
}

/**
 * @brief Computes the fused state of charge (SoC) based on voltage and ampere-hour values, adjusting for armed status.
 *
 * This function estimates the fused SoC by blending the SoC derived from ampere-hours (socAh)
 * and the SoC derived from voltage (socV). It considers several factors including:
 * - Base weighting from Ah SoC with smooth transitions between zones.
 * - Voltage dominance to apply a gentle bias when operating in low voltage zones.
 * - Adjustments for armed status, introducing a small bias.
 * - Clamping and smoothing to ensure a stable ramp-up or ramp-down of the weighting factor.
 * - Temporal realism by limiting the rate of change when armed or on the ground.
 *
 * @param socV The state of charge calculated from voltage.
 * @param socAh The state of charge calculated from ampere-hours.
 * @param armed A boolean indicating if the system is armed (true) or not (false).
 * @return The fused state of charge as an 8-bit unsigned integer.
 */
static inline uint8_t fuse_soc_vbatComp_smart ( float socV, float socAh, bool armed ) {
  static float wAhPrev        = 55.0f;
  static uint8_t socFusedPrev = 100;

  float wAhTarget;

  // 1. Base weighting from Ah SOC (smooth zones)
  if ( socAh <= 15.0f )
    wAhTarget = 30.0f;
  else if ( socAh >= 85.0f )
    wAhTarget = 70.0f;
  else
    wAhTarget = 55.0f;

  // 2. Voltage dominance zone (gentle bias)
  if ( vBatComp <= batteryWarningVoltage ) {
    uint16_t depth = ( vBatComp <= ( batteryCriticalVoltage - VBATT_HYSTERESIS ) ) ? ( batteryWarningVoltage - ( batteryCriticalVoltage - VBATT_HYSTERESIS ) ) : ( batteryWarningVoltage - vBatComp );

    float bias = static_cast< float > ( depth * 12U ) / static_cast< float > ( batteryWarningVoltage - ( batteryCriticalVoltage - VBATT_HYSTERESIS ) );

    if ( wAhTarget > ( 25.0f + bias ) )
      wAhTarget -= bias;
  }

  // 3. Armed bias (small but real)
  if ( armed && wAhTarget > 25.0f )
    wAhTarget -= 4.0f;

  // 4. Clamp + smooth ramp (KEY CHANGE)
  wAhTarget = ( wAhTarget < 20.0f ) ? 20.0f :
              ( wAhTarget > 75.0f ) ? 75.0f :
                                      wAhTarget;

  // Ramp wAh slowly (linearity magic)
  if ( wAhTarget > wAhPrev + 2.0f )
    wAhPrev += 2.0f;
  else if ( wAhTarget + 2.0f < wAhPrev )
    wAhPrev -= 2.0f;
  else
    wAhPrev = wAhTarget;

  wAh = wAhPrev;

  // 5. Weighted fusion
  float fused = socAh * wAh + socV * ( 100.0f - wAh );
  fused /= 100.0f;
  if ( fused > 100.0f ) fused = 100.0f;

  uint8_t out = static_cast< uint8_t > ( fused );

  // 6. Temporal realism (linear decay)
  if ( armed ) {
    // Never increase in flight
    if ( out > socFusedPrev )
      out = socFusedPrev;

    // Tight drop limit (graph-friendly)
    if ( socFusedPrev > out ) {
      uint8_t maxDrop = 2;
      if ( socFusedPrev - out > maxDrop )
        out = socFusedPrev - maxDrop;
    }
  } else {
    // Ground recovery: slow
    const uint8_t MAX_RISE = 1;
    if ( out > socFusedPrev + MAX_RISE )
      out = socFusedPrev + MAX_RISE;
  }

  socFusedPrev = out;
  return out;
}

/**
 * @brief Updates the battery state based on the state of charge (SoC) percentage.
 *
 * This function changes the battery state and triggers appropriate actions such as
 * enabling/disabling arming flags, setting flight status indicators, and activating
 * beepers based on the current SoC percentage. It manages transitions between different
 * battery states: OK, WARNING, CRITICAL, and NOT_PRESENT.
 *
 * @param socPct State of charge percentage of the battery.
 */
static inline void updateBatteryStateSoc ( uint8_t socPct ) {

  switch ( batteryState ) {

    case BATTERY_OK:
      ENABLE_ARMING_FLAG ( OK_TO_ARM );
      if ( ( socPct <= ( SOC_WARN_PCT - SOC_HYST_PCT ) ) && fsInFlightLowBattery ) {
        batteryState = BATTERY_WARNING;
        set_FSI ( Low_battery );
        beeper ( BEEPER_BAT_LOW );
      }
      break;

    case BATTERY_WARNING:
      DISABLE_ARMING_FLAG ( PREVENT_ARMING );
      if ( socPct <= ( SOC_CRIT_PCT - SOC_HYST_PCT ) ) {
        batteryState = BATTERY_CRITICAL;
        set_FSI ( LowBattery_inFlight );
        reset_FSI ( Low_battery );
        beeper ( BEEPER_BAT_CRIT_LOW );
      } else if ( socPct > ( SOC_WARN_PCT + SOC_HYST_PCT ) ) {
        reset_FSI ( Low_battery );
        batteryState       = BATTERY_OK;
        BatteryWarningMode = 0;
      } else {
        beeper ( BEEPER_BAT_LOW );
        if ( socPct <= 14 ) {
          BatteryWarningMode = 2;
        } else {
          BatteryWarningMode = 1;
        }
      }
      break;

    case BATTERY_CRITICAL:
      DISABLE_ARMING_FLAG ( PREVENT_ARMING );
      if ( socPct > ( SOC_CRIT_PCT + SOC_HYST_PCT ) ) {
        batteryState = BATTERY_WARNING;
        set_FSI ( Low_battery );
        reset_FSI ( LowBattery_inFlight );
        beeper ( BEEPER_BAT_LOW );
      } else {
        beeper ( BEEPER_BAT_CRIT_LOW );
      }
      break;

    case BATTERY_NOT_PRESENT:
      break;
  }
}

/**
 * @brief Updates the Battery Management System (BMS) parameters.
 *
 * This function calculates the compensated battery voltage and state of charge (SoC)
 * percentages based on the current timestamp, voltage, and current timestamps. It also
 * fuses the SoC values if a specific feature is enabled and updates the battery state
 * accordingly.
 *
 * @param now Current time in milliseconds since system start.
 * @param vbatTs Timestamp for the last battery voltage measurement.
 * @param ibatTs Timestamp for the last battery current measurement.
 * @param armed Boolean indicating whether the system is armed.
 * @param throttle_us Throttle position in microseconds.
 */
void BMS_Update ( uint32_t now, uint32_t vbatTs, uint32_t ibatTs, bool armed, int throttle_us ) {
  uint16_t _vBatComp = computeVbatComp_mV ( now, vbatTs, ibatTs, armed, throttle_us );

  vBatComp = 0.35f * _vBatComp + 0.65f * ( ( vBatRaw * 100 ) + vShuntRaw );

  soc_battery_percentage = soc_linear_from_voltage ( vBatComp );
  if ( feature ( FEATURE_INA219_CBAT ) ) {

    soc_mAh_percentage = soc_from_mAh ( );
    soc_Fused          = fuse_soc_vbatComp_smart ( soc_battery_percentage, soc_mAh_percentage, armed );
    updateBatteryStateSoc ( soc_Fused );
  } else {
    updateBatteryStateSoc ( soc_battery_percentage );
  }
}