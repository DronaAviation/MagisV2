/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\sensors\power.c                                            #
 #  Created Date: Wed, 16th Apr 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sat, 9th Aug 2025                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

// #include <stdbool.h>

// #include <string.h>

// #include "platform.h"

// #include "drivers/ina219.h"
// #include "drivers/system.h"

// #include "sensors/power.h"

// #define BATTERY_FULL_mAH   600
// #define BATTERY_V_FULL_mV  4200
// #define BATTERY_V_EMPTY_mV 3200

// ---- Accumulator state ----
// static uint32_t last_ms     = 0;
// static uint64_t mA_ms_accum = 0;

// Internal state
// static uint16_t samples [ BATTERY_BUFFER_SIZE ];    // Array to store voltage samples
// static uint32_t sum  = 0;                           // Sum of the samples
// static uint8_t head  = 0;                           // Index for circular buffer
// static uint8_t count = 0;                           // Number of valid samples

// uint16_t initialBatteryCapicity = 0;

/**
 * @brief Initializes the battery voltage monitoring system.
 *
 * This function initializes the buffer and associated variables.
 * It sets all elements of the `samples` array to zero, and resets
 * the `sum`, `head`, and `count` variables to their initial states.
 */
// void battery_voltage_init ( void ) {
//   memset ( samples, 0, sizeof ( samples ) );    // Use memset for efficient initialization
//   sum         = 0;
//   head        = 0;
//   count       = 0;
//   last_ms     = millis ( );
//   mA_ms_accum = 0;
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
// void battery_voltage_update ( uint16_t adc_reading ) {
//   sum -= samples [ head ];           // Remove old value from sum
//   samples [ head ] = adc_reading;    // Insert new value
//   sum += adc_reading;                // Add new value to sum

//   head = ( head + 1 ) % BATTERY_BUFFER_SIZE;    // Move head circularly

//   if ( count < BATTERY_BUFFER_SIZE ) {
//     count++;
//   }
// }

/**
 * @brief Processes and returns the average battery voltage.
 *
 * @return The average voltage based on current samples.
 *
 * This function updates the voltage readings with the current bus voltage,
 * then calculates and returns the average voltage. If no valid samples are
 * available, it returns 0.
 */
// uint16_t ProcessedVoltage ( void ) {
//   battery_voltage_update ( bus_voltage ( ) );    // Update with current voltage reading

//   if ( count == 0 ) return 0;    // Return 0 if no valid samples

//   return ( uint16_t ) ( sum / count );    // Calculate and return the average
// }

// uint16_t current_reading ( void ) {
//   int16_t mV = shunt_voltage ( );    // signed mV from shunt

//   if ( mV <= 0 ) return 0;    // no reverse/negative current in telemetry

//   // // mA = mV × 1000 / R_mΩ
//   uint32_t mA = ( uint32_t ) ( ( mV / 10.0f ) / INA219_SHUNT_RESISTOR_MILLIOHM );

//   if ( mA > 0xFFFF ) mA = 0xFFFF;    // clamp to uint16_t max

//   return ( uint16_t ) mA;
// }

// Call this in your main loop as often as you like
// void update_mAh_Meter ( void ) {
//   uint32_t now = millis ( );
//   uint32_t dt  = now - last_ms;    // ok across wrap (unsigned arithmetic)
//   last_ms      = now;
//   uint16_t mA = current_reading ( );    // your integer mA (clamped, non-negative)
//   mA_ms_accum += ( uint64_t ) mA * ( uint64_t ) dt;
// }

// Get whole mAh (uint16_t), clamped for MSP serialize16()
// uint16_t mAh_get ( void ) {
//   uint64_t mAh = mA_ms_accum / 3600000ULL;    // convert mA·ms → mAh
//   if ( mAh > 0xFFFFULL ) return 0xFFFF;
//   return ( uint16_t ) mAh;
// }

// Reset counter (e.g., after a pack change)
// void mAh_reset ( void ) {
//   mA_ms_accum = 0;
//   last_ms     = millis ( );
// }

// void battery_estimate_start_mAh ( void ) {
//   uint16_t v_u100mV      = bus_voltage ( );    // from your INA219 in mV
//   uint32_t v_mV          = ( uint32_t ) v_u100mV * 100u;
//   initialBatteryCapicity = ( uint16_t ) ( ( ( uint32_t ) ( v_mV - BATTERY_V_EMPTY_mV ) * BATTERY_FULL_mAH ) / ( BATTERY_V_FULL_mV - BATTERY_V_EMPTY_mV ) );
// }