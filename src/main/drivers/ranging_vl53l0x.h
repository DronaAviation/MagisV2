/*
 * This file is part of Magis.
 *
 * Magis is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Magis is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "vl53l0x_def.h"
#include "vl53l0x_device.h"
#include "vl53l0x_platform.h"

// ---- Per-device helpers ( tof-single-pipeline ) ----------------------------------------------
// Each helper works on the device it is given and returns that device's status, so one laser's
// error never stops another. The laser table, bring-up and polling live in drivers/tof_manager. Addresses are 7-bit ( 0x29 = the boot address ).

#define VL53L0X_BOOT_ADDRESS 0x29
#define VL53L0X_MODEL_ID     0xEE    // VL53L0X_REG_IDENTIFICATION_MODEL_ID ( 0xC0 ) reads this

typedef struct {
  VL53L0X_DeviceModes deviceMode;    // VL53L0X_DEVICEMODE_SINGLE_RANGING or _CONTINUOUS_RANGING
  uint32_t timingBudgetUs;           // measurement timing budget, µs
  uint8_t vcselPrePclk;              // pre-range VCSEL pulse period, PCLKs
  uint8_t vcselFinalPclk;            // final-range VCSEL pulse period, PCLKs
  bool sampleReadyInterrupt;         // enable the new-sample-ready interrupt ( continuous + vl53l0xReadIfReady ).
                                     // StaticInit leaves it off ( lib DD change ), and then data-ready falls back
                                     // to RESULT_RANGE_STATUS bit 0, which ClearInterruptMask never resets.
} vl53l0xConfig_t;

// The init step that failed, for diagnostics ( 0 = none ).
typedef enum {
  VL53L0X_STEP_NONE = 0,
  VL53L0X_STEP_DATA_INIT,
  VL53L0X_STEP_STATIC_INIT,
  VL53L0X_STEP_REF_SPAD,
  VL53L0X_STEP_REF_CAL,
  VL53L0X_STEP_DEVICE_MODE,
  VL53L0X_STEP_SIGMA_ENABLE,
  VL53L0X_STEP_SIGNAL_ENABLE,
  VL53L0X_STEP_SIGNAL_VALUE,
  VL53L0X_STEP_SIGMA_VALUE,
  VL53L0X_STEP_TIMING_BUDGET,
  VL53L0X_STEP_VCSEL_PRE,
  VL53L0X_STEP_VCSEL_FINAL,
  VL53L0X_STEP_GPIO_CONFIG,
  VL53L0X_STEP_SET_ADDRESS,
  VL53L0X_STEP_BUS_BUSY,     // not released: another chip already answered at 0x29
  VL53L0X_STEP_START_RANGING  // side laser: continuous ranging did not start
} vl53l0xStep_e;

// Full init of the chip answering at addr7: data init, static init, reference SPAD and
// calibration, mode, sigma / signal limit checks, timing budget, VCSEL periods. Stops at the first error; *failedStep gets the step ( vl53l0xStep_e ).
VL53L0X_Error vl53l0xConfigure ( VL53L0X_Dev_t *dev, uint8_t addr7, const vl53l0xConfig_t *cfg, uint8_t *failedStep );
// Moves the chip to newAddr7. The host side follows only if the write succeeded.
VL53L0X_Error vl53l0xSetAddress ( VL53L0X_Dev_t *dev, uint8_t newAddr7 );
// True if a VL53L0X answers at addr7 ( model ID ). A missing chip costs one I2C timeout.
bool vl53l0xProbe ( uint8_t addr7 );
// Start a measurement and read the result straight away ( the DOWN poll, R6 ): in single mode
// the result is the previous measurement's ( F5 ).
VL53L0X_Error vl53l0xStartAndRead ( VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *data );
// Continuous mode ( configured with sampleReadyInterrupt ): read and clear a sample only if one is
// ready. *ready = false costs one read.
VL53L0X_Error vl53l0xReadIfReady ( VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *data, bool *ready );

#ifdef __cplusplus
}
#endif
