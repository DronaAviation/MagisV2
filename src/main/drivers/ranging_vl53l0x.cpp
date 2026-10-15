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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

#include "vl53l0x_api_core.h"
#include "vl53l0x_def.h"
#include "vl53l0x_api.h"
#include "vl53l0x_types.h"

#include "drivers/bus_i2c.h"
#include "ranging_vl53l0x.h"

// ---- Per-device helpers ( tof-single-pipeline ) ----------------------------------------------

VL53L0X_Error vl53l0xConfigure ( VL53L0X_Dev_t *dev, uint8_t addr7, const vl53l0xConfig_t *cfg, uint8_t *failedStep )
{
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint8_t step = VL53L0X_STEP_DATA_INIT;

    dev->I2cDevAddr      = addr7;
    dev->comms_type      = 1;
    dev->comms_speed_khz = 400;

    // Same order and values as the old ranging_init ( ) ( DOWN ) and LaserSensor::init ( ) ( side ).
    VL53L0X_Error status = VL53L0X_DataInit ( dev );

    if ( status == VL53L0X_ERROR_NONE ) {
        step   = VL53L0X_STEP_STATIC_INIT;
        status = VL53L0X_StaticInit ( dev );
    }
    if ( status == VL53L0X_ERROR_NONE ) {
        step   = VL53L0X_STEP_REF_SPAD;
        status = VL53L0X_PerformRefSpadManagement ( dev, &refSpadCount, &isApertureSpads );
    }
    if ( status == VL53L0X_ERROR_NONE ) {
        step   = VL53L0X_STEP_REF_CAL;
        status = VL53L0X_PerformRefCalibration ( dev, &VhvSettings, &PhaseCal );
    }
    if ( status == VL53L0X_ERROR_NONE ) {
        step   = VL53L0X_STEP_DEVICE_MODE;
        status = VL53L0X_SetDeviceMode ( dev, cfg->deviceMode );
    }
    if ( status == VL53L0X_ERROR_NONE ) {
        step   = VL53L0X_STEP_SIGMA_ENABLE;
        status = VL53L0X_SetLimitCheckEnable ( dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );
    }
    if ( status == VL53L0X_ERROR_NONE ) {
        step   = VL53L0X_STEP_SIGNAL_ENABLE;
        status = VL53L0X_SetLimitCheckEnable ( dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );
    }
    if ( status == VL53L0X_ERROR_NONE ) {
        step   = VL53L0X_STEP_SIGNAL_VALUE;
        status = VL53L0X_SetLimitCheckValue ( dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, ( FixPoint1616_t ) ( 0.1f * 65536 ) );    // 0.1 MCPS
    }
    if ( status == VL53L0X_ERROR_NONE ) {
        step   = VL53L0X_STEP_SIGMA_VALUE;
        status = VL53L0X_SetLimitCheckValue ( dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, ( FixPoint1616_t ) ( 60 * 65536 ) );    // 60 mm
    }
    if ( status == VL53L0X_ERROR_NONE ) {
        step   = VL53L0X_STEP_TIMING_BUDGET;
        status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds ( dev, cfg->timingBudgetUs );
    }
    if ( status == VL53L0X_ERROR_NONE ) {
        step   = VL53L0X_STEP_VCSEL_PRE;
        status = VL53L0X_SetVcselPulsePeriod ( dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, cfg->vcselPrePclk );
    }
    if ( status == VL53L0X_ERROR_NONE ) {
        step   = VL53L0X_STEP_VCSEL_FINAL;
        status = VL53L0X_SetVcselPulsePeriod ( dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, cfg->vcselFinalPclk );
    }
    if ( status == VL53L0X_ERROR_NONE && cfg->sampleReadyInterrupt ) {
        step   = VL53L0X_STEP_GPIO_CONFIG;
        status = VL53L0X_SetGpioConfig ( dev, 0, cfg->deviceMode, VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
                                         VL53L0X_INTERRUPTPOLARITY_LOW );
    }

    *failedStep = ( status == VL53L0X_ERROR_NONE ) ? ( uint8_t ) VL53L0X_STEP_NONE : step;
    return status;
}

VL53L0X_Error vl53l0xSetAddress ( VL53L0X_Dev_t *dev, uint8_t newAddr7 )
{
    // The API takes the 8-bit form; the platform layer uses the 7-bit I2cDevAddr.
    VL53L0X_Error status = VL53L0X_SetDeviceAddress ( dev, ( uint8_t ) ( newAddr7 << 1 ) );
    if ( status == VL53L0X_ERROR_NONE ) {
        dev->I2cDevAddr = newAddr7;
    }
    return status;
}

bool vl53l0xProbe ( uint8_t addr7 )
{
    uint8_t modelId = 0;
    if ( ! i2cRead ( addr7, VL53L0X_REG_IDENTIFICATION_MODEL_ID, 1, &modelId ) ) {
        return false;
    }
    return modelId == VL53L0X_MODEL_ID;
}

VL53L0X_Error vl53l0xStartAndRead ( VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *data )
{
    VL53L0X_Error status = VL53L0X_StartMeasurement ( dev );
    if ( status == VL53L0X_ERROR_NONE ) {
        status = VL53L0X_GetRangingMeasurementData ( dev, data );
    }
    return status;
}

VL53L0X_Error vl53l0xReadIfReady ( VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *data, bool *ready )
{
    uint8_t dataReady = 0;
    *ready            = false;

    VL53L0X_Error status = VL53L0X_GetMeasurementDataReady ( dev, &dataReady );
    if ( status == VL53L0X_ERROR_RANGE_ERROR ) {
        // GetInterruptMaskStatus reports RESULT_INTERRUPT_STATUS bits 3-4 ( set by the sensor ) this way;
        // data-ready still comes from bits 0-2. The sample's own RangeStatus marks it invalid.
        status = VL53L0X_ERROR_NONE;
    }
    if ( status != VL53L0X_ERROR_NONE || dataReady == 0 ) {
        return status;
    }
    status = VL53L0X_GetRangingMeasurementData ( dev, data );
    if ( status != VL53L0X_ERROR_NONE ) {
        return status;
    }
    *ready = true;    // the sample is good whatever the clear below reports
    status = VL53L0X_ClearInterruptMask ( dev, 0 );    // resets the interrupt status data-ready reads
    if ( status == VL53L0X_ERROR_INTERRUPT_NOT_CLEARED ) {
        // ST returns this whenever the clear needed a third try, even if that try worked. At worst
        // the next call sees the same sample again; it is not a bus fault.
        status = VL53L0X_ERROR_NONE;
    }
    return status;
}

