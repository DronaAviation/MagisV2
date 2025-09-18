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
 #  Last Modified: Fri, 22nd Aug 2025                                          #
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

#define DEFAULT_BATTERY_CAP_mAH 600
#define BATTERY_V_Max_mV        4200
#define BATTERY_V_Warn_mV       3400
#define BATTERY_V_Min_mV        3200

// #include "drivers/ina219.h"

// ---- Accumulator state ----
static uint32_t last_ms     = 0;
static uint64_t mA_ms_accum = 0;

// Internal state
static uint16_t samples [ BATTERY_BUFFER_SIZE ];    // Array to store voltage samples
static uint32_t sum  = 0;                           // Sum of the samples
static uint8_t head  = 0;                           // Index for circular buffer
static uint8_t count = 0;                           // Number of valid samples

static batteryState_e batteryState;

#define VBATT_PRESENT_THRESHOLD_MV 10
#define VBATT_LPF_FREQ             10

#define VBATTERY_STABLE_DELAY      40
#define VBATT_HYSTERESIS           100    // Batt Hysteresis of +/-100mV

// Battery monitoring stuff
uint8_t batteryCellCount = 1;    // cell count
uint16_t batteryMaxVoltage;
uint16_t batteryWarningVoltage;
uint16_t batteryCriticalVoltage;
uint16_t batteryCapacity;
uint16_t EstBatteryCapacity;

uint16_t vbat              = 0;    // battery voltage in 0.1V steps (filtered)
uint16_t vbatscaled        = 0;
uint16_t vbatLatestADC     = 0;    // most recent unsmoothed raw reading from vbat ADC
uint16_t amperageLatestADC = 0;    // most recent raw reading from current ADC

int32_t amperage;         // amperage read by current sensor in centiampere (1/100th A)
int32_t mAhDrawn  = 0;    // milliampere hours drawn from the battery since start
int32_t mAhRemain = 0;    // milliampere hours drawn from the battery since start

batteryConfig_t *batteryConfig;

uint16_t battery_capacity_mAh = DEFAULT_BATTERY_CAP_mAH;

/**
 * @brief Retrieves the current state of the battery.
 *
 * This function returns the current battery state by accessing
 * an existing variable, `batteryState`, which is assumed to be
 * updated elsewhere in the program. The function does not perform
 * any calculations or updates to determine the battery state.
 *
 * @return The current state of the battery as a batteryState_e enum value.
 */
batteryState_e getBatteryState ( void ) {
  // Return the current battery state stored in the variable 'batteryState'
  return batteryState;
}

const char *const batteryStateStrings [] = { "OK", "WARNING", "CRITICAL", "NOT PRESENT" };

/**
 * @brief Initializes the battery configuration and state.
 *
 * This function sets up the initial battery parameters and state based on the provided configuration.
 * It assumes a default single cell battery and initializes various voltage levels and capacity estimates to zero.
 * Additionally, it prepares the sample tracking system by clearing existing samples and resetting counters.
 * If the INA219 current sensor is enabled, it also initializes timing and current accumulation variables.
 *
 * @param initialBatteryConfig Pointer to the initial battery configuration structure.
 */
void batteryInit ( batteryConfig_t *initialBatteryConfig ) {
  // Assign the initial configuration to the global battery configuration pointer
  batteryConfig = initialBatteryConfig;

  // Initialize battery state and parameters
  batteryState           = BATTERY_NOT_PRESENT;    // Set the initial state as battery not present
  batteryCellCount       = 1;                      // Assume a single cell battery by default
  batteryMaxVoltage      = 0;                      // Maximum voltage of the battery set to zero initially
  batteryWarningVoltage  = 0;                      // Warning voltage level set to zero initially
  batteryCriticalVoltage = 0;                      // Critical voltage level set to zero initially
  EstBatteryCapacity     = 0;                      // Estimated battery capacity initialized to zero

  // Clear the samples array, setting all elements to zero using memset
  memset ( samples, 0, sizeof ( samples ) );    // Use memset for efficient initialization of samples array

  // Initialize the sum, head index, and sample count for tracking battery samples
  sum   = 0;    // Sum of samples initialized to zero
  head  = 0;    // Head index in circular buffer initialized to zero
  count = 0;    // Sample count initialized to zero

#ifdef INA219_Current
  // If INA219 current sensor is used, initialize timing and accumulator variables
  last_ms     = millis ( );    // Record the current time in milliseconds
  mA_ms_accum = 0;             // Initialize milliamp-millisecond accumulator to zero
#endif
}

/**
 * @brief Updates the battery sample buffer and sum with a new ADC reading.
 *
 * This function manages a circular buffer to store recent ADC readings for the battery,
 * updating the sum of these samples for further processing. It replaces the oldest value
 * in the buffer with the new reading, adjusts the sum accordingly, and updates the head
 * index to point to the next location in the buffer. The sample count is incremented until
 * it reaches the buffer's capacity.
 *
 * @param adc_reading The new ADC reading to be added to the sample buffer.
 */
void UpdateINA219Battery ( uint16_t adc_reading ) {
  sum -= samples [ head ];           // Remove the old value from the sum by subtracting the oldest sample at the current head position
  samples [ head ] = adc_reading;    // Insert the new ADC reading into the samples array at the current head position
  sum += adc_reading;                // Add the new ADC reading to the sum

  head = ( head + 1 ) % BATTERY_BUFFER_SIZE;    // Increment the head index circularly, wrapping around using modulo operation to stay within buffer size

  if ( count < BATTERY_BUFFER_SIZE ) {    // Check if the buffer is not yet full
    count++;                              // Increment the count of valid samples in the buffer
  }
}

/**
 * @brief Processes and returns the average INA219 bus voltage.
 *
 * This function updates the battery data with the current bus voltage reading
 * obtained from the INA219 sensor. It calculates the average voltage based on
 * the accumulated sum of readings and the count of valid samples. If no valid
 * samples are available, the function returns 0.
 *
 * @return uint16_t The average bus voltage as an unsigned 16-bit integer.
 */
uint16_t ProcessedINA219Voltage ( void ) {
  UpdateINA219Battery ( bus_voltage ( ) );    // Update the battery data with the current bus voltage reading

  if ( count == 0 ) return 0;    // Return 0 if there are no valid samples in the buffer

  return ( uint16_t ) ( sum / count );    // Calculate and return the average voltage as an unsigned 16-bit integer
}

/**
 * @brief Processes the current measurement from an INA219 sensor.
 *
 * This function calculates the current in milliamps based on the shunt voltage
 * measured by an INA219 sensor. It assumes no reverse or negative current is
 * present, clamping the result to a maximum of 65535 milliamps if necessary.
 *
 * @return The processed current value as an unsigned 16-bit integer (uint16_t).
 *         Returns 0 if the measured shunt voltage indicates a negative or zero current.
 */
uint16_t ProcessedINA219Current ( void ) {
  int16_t mV = shunt_voltage ( );    // Retrieve the shunt voltage in millivolts (signed value).

  if ( mV <= 0 ) return 0;    // Return 0 if the current is negative or zero, as reverse/negative current is not considered.

  // Calculate the current in milliamps using the formula: mA = mV × 1000 / R_mΩ.
  amperage = ( uint32_t ) ( ( mV / 10.0f ) / INA219_SHUNT_RESISTOR_MILLIOHM );

  if ( amperage > 0xFFFF ) amperage = 0x0;    // Clamp the calculated current to the maximum value of an unsigned 16-bit integer.

  return ( uint16_t ) amperage;    // Return the calculated current as an unsigned 16-bit integer.
}

/**
 * @brief Handles the initialization and configuration when a battery is connected.
 *
 * This function sets the battery state to indicate a connected and operational status. It initializes
 * battery parameters such as capacity and voltage thresholds based on predefined constants. The function
 * also estimates the number of battery cells and adjusts it if necessary. If the INA219_Current feature
 * is enabled, it calculates the estimated remaining battery capacity.
 */
void handleBatteryConnected ( ) {
  // Set the initial battery state to indicate that the battery is connected and operational.
  batteryState = BATTERY_OK;

  // Initialize battery parameters with predefined values.
  batteryCapacity   = battery_capacity_mAh;    // Set the battery capacity in milliamp hours.
  batteryMaxVoltage = BATTERY_V_Max_mV;        // Set the maximum voltage of the battery in millivolts.

  // Calculate critical and warning voltage levels based on the number of cells and predefined constants.
  batteryCriticalVoltage = batteryCellCount * BATTERY_V_Min_mV;     // Voltage level considered critically low.
  batteryWarningVoltage  = batteryCellCount * BATTERY_V_Warn_mV;    // Voltage level considered a warning threshold.

  // Delay to allow the battery voltage to stabilize after connection.
  delay ( VBATTERY_STABLE_DELAY );

  // Estimate the number of battery cells based on the current voltage.
  unsigned cells = ( vbat / ( batteryMaxVoltage / 100u ) ) + 1;    // Calculate the cell count estimation.
  if ( cells > 8 ) {
    cells = 8;    // Clamp the number of cells to a maximum of 8 if estimation exceeds this value.
  }
  batteryCellCount = cells;    // Update the battery cell count.

#ifdef INA219_Current
  // If the INA219_Current feature is enabled, estimate the remaining battery capacity.
  uint16_t v_u100mV  = vbat * 100;    // Convert battery voltage to hundred-millivolt units for calculations.
  EstBatteryCapacity = ( ( v_u100mV - batteryCriticalVoltage ) * batteryCapacity ) / ( batteryMaxVoltage - batteryCriticalVoltage );
  // Calculate estimated battery capacity as a percentage of total capacity.
#endif
}

/**
 * @brief Handles the configuration and state reset when a battery is disconnected.
 *
 * This function updates the system to reflect the absence of a battery. It resets
 * battery-related parameters, including state, cell count, warning and critical
 * voltage levels, maximum voltage, capacity, and estimated remaining capacity to zero.
 */
void handleBatteryDisconnected ( ) {
  batteryState           = BATTERY_NOT_PRESENT;
  batteryCellCount       = 0;
  batteryWarningVoltage  = 0;
  batteryCriticalVoltage = 0;
  batteryMaxVoltage      = 0;
  batteryCapacity        = 0;
  EstBatteryCapacity     = 0;
}

/**
 * @brief Updates the current battery state based on voltage readings and triggers
 *        appropriate alerts or actions.
 *
 * This function checks the current battery voltage against predefined thresholds
 * to determine the battery state: OK, WARNING, CRITICAL, or NOT_PRESENT. Depending
 * on the battery state, it triggers beepers for low or critical battery levels,
 * disables arming if needed, and initiates failsafe procedures to ensure safety
 * during flight operations.
 */
void updateBatteryState ( ) {
  // Switch based on the current battery state
  switch ( batteryState ) {

    // Case when the battery is in a good state
    case BATTERY_OK:
      // Check if the voltage has dropped below the warning threshold considering hysteresis
      if ( vbat <= ( batteryWarningVoltage - VBATT_HYSTERESIS ) ) {
        // Update battery state to WARNING and trigger low battery beeper
        batteryState = BATTERY_WARNING;
        beeper ( BEEPER_BAT_LOW );
      }
      break;

    // Case when the battery is in a warning state
    case BATTERY_WARNING:
      // Disable arming when battery is in warning state
      DISABLE_ARMING_FLAG ( PREVENT_ARMING );
      // Check if the voltage has dropped below the critical threshold
      if ( vbat <= ( batteryCriticalVoltage - VBATT_HYSTERESIS ) ) {
        // Update battery state to CRITICAL and trigger critical low battery beeper
        batteryState = BATTERY_CRITICAL;
        beeper ( BEEPER_BAT_CRIT_LOW );
        // Check if the voltage has risen above the warning threshold plus hysteresis
      } else if ( vbat > ( batteryWarningVoltage + VBATT_HYSTERESIS ) ) {
        // Revert battery state to OK
        batteryState = BATTERY_OK;
      } else {
        // Continue to warn with low battery beep
        beeper ( BEEPER_BAT_LOW );
      }
      // Trigger failsafe procedures for low battery
      failsafeOnLowBattery ( );
      break;

    // Case when the battery is in a critical state
    case BATTERY_CRITICAL:
      // Check if the voltage has risen above the critical threshold plus hysteresis
      if ( vbat > ( batteryCriticalVoltage + VBATT_HYSTERESIS ) ) {
        // Update battery state to WARNING and trigger low battery beeper
        batteryState = BATTERY_WARNING;
        beeper ( BEEPER_BAT_LOW );
      } else {
        // Continue to warn with critical low battery beep
        beeper ( BEEPER_BAT_CRIT_LOW );
      }
      // Trigger failsafe procedures for critically low battery
      failsafeOnLowBattery ( );
      break;

    // Case when no battery is present, do nothing
    case BATTERY_NOT_PRESENT:
      break;
  }
}

/**
 * @brief Updates the voltage reading from the INA219 sensor and adjusts battery state accordingly.
 *
 * This function processes the voltage reading obtained from the INA219 sensor and updates the
 * global variable `vbat` with this value. Based on the processed voltage, it determines whether
 * a battery has been connected or disconnected by comparing `vbat` against a predefined threshold
 * (`VBATT_PRESENT_THRESHOLD_MV`). If a change in battery connection status is detected, it calls
 * appropriate handler functions to manage these events. Finally, it invokes `updateBatteryState()`
 * to refresh the current battery state.
 */
void updateINA219Voltage ( ) {
  // Process the voltage reading from the INA219 sensor and store it in vbat
  vbat = ProcessedINA219Voltage ( );

  // Check if the battery is currently not present but the voltage indicates otherwise
  if ( batteryState == BATTERY_NOT_PRESENT && vbat > VBATT_PRESENT_THRESHOLD_MV ) {
    // Handle scenario where a battery has been connected
    handleBatteryConnected ( );
  }
  // Check if the battery is present but the voltage indicates disconnection
  else if ( batteryState != BATTERY_NOT_PRESENT && vbat <= VBATT_PRESENT_THRESHOLD_MV ) {
    // Handle scenario where the battery has been disconnected
    handleBatteryDisconnected ( );
  }

  // Update the battery state based on the current voltage readings
  updateBatteryState ( );
}

/**
 * @brief Updates the current measurement from the INA219 sensor and calculates
 *        energy consumption.
 *
 * This function reads the current measurement from the INA219 sensor, calculates
 * the energy consumed over time, and updates the remaining battery capacity. It
 * considers the elapsed time since the last update to compute the energy in
 * milliampere-hours.
 *
 * @note The function resets the drawn energy if it exceeds the maximum value for
 *       a 16-bit integer to prevent overflow.
 */
void updateINA219Current ( ) {
  // Get the current time in milliseconds
  uint32_t now = millis ( );

  // Calculate the time elapsed since the last update
  uint32_t dt = now - last_ms;

  // Update the last timestamp to the current time
  last_ms = now;

  // Read and process the current measurement from the INA219 sensor
  uint16_t mA = ProcessedINA219Current ( );

  // Accumulate the milliampere-milliseconds product for energy calculation
  mA_ms_accum += static_cast< uint64_t > ( mA ) * dt;

  // Convert accumulated milliampere-milliseconds to milliampere-hours
  mAhDrawn = mA_ms_accum / 3600000ULL;

  // If the drawn energy exceeds the maximum value for a 16-bit integer, reset it
  if ( mAhDrawn > 0xFFFFULL ) {
    mAhDrawn = 0x0;
  }

  // Calculate the remaining battery capacity in milliampere-hours
  mAhRemain = static_cast< uint16_t > ( EstBatteryCapacity - mAhDrawn );
}
