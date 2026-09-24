/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\ranging_vl53l1x.cpp                                #
 #  Created Date: Sat, 8th Nov 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 24th Sep 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  24-09-2026	AJ	Range under 15 mm ( covered window ) counts as out of range.  #
 #  24-09-2026	AJ	L1X_DISTANCE_MODE compile-time switch ( default Medium );     #
 #  			L1X_TIMING_BUDGET_US overridable by the target.            #
 #  23-09-2026	AJ	L1X: 45 ms budget / 50 ms period, clear interrupt per      #
 #  		  	sample ( one new sample per measurement ), sample counter, #
 #  		  	latched ST error reported as out of range, warnings fixed. #
 #  23-09-2026	AJ	L1X sample poll: 17-byte result read + interrupt clear     #
 #  		  	instead of the full-API read / GENERAL_ONWARDS rewrite.    #
*******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "vl53l1_platform.h"
// #include "vl53l0x_i2c_platform.h"

#include "vl53l1_api_core.h"
#include "vl53l1_api_strings.h"
#include "vl53l1_def.h"
#include "vl53l1_api.h"
#include "vl53l1_types.h"
#include "vl53l1_core.h"             // VL53L1_clear_interrupt
#include "vl53l1_preset_setup.h"     // VL53L1_TUNING_PROXY_MIN
#include "vl53l1_register_map.h"     // result register addresses
#include "vl53l1_register_settings.h"    // VL53L1_RANGE_STATUS__RANGE_STATUS_MASK

#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/bus_i2c.h"
#include "drivers/system.h"
#include "ranging_vl53l1x.h"
#include "API/Peripherals.h"
#include "API/Debugging.h"
#include "API/Scheduler-Timer.h"

#define RANGE_POLL 10    // ms, data-ready poll period

// mm, slant range. A covered or blocked window reads 0-10 mm with status 0 ( log-1, log-4 ), so it is out of range;
// the craft sitting on the floor reads 25-28 mm and stays valid ( landing and touchdown see the laser as on the L0X ).
#define L1X_MIN_VALID_MM 15

// Distance mode: Medium unless the target overrides it ( Long uses longer VCSEL periods, 15 / 13, with the same
// sigma / signal limits; SetDistanceMode keeps the budget and period, vl53l1_api.c:1068-1094 ).
#ifndef L1X_DISTANCE_MODE
#define L1X_DISTANCE_MODE VL53L1_DISTANCEMODE_MEDIUM
#endif

// us, per measurement. In the full ST API's AUTONOMOUS preset the budget includes a fixed ~26.6 ms guard and
// the rest is split over two phases: 45000 gives ~9.2 ms per phase ( the 41000 default 7.2, 33000 only 3.2 ).
#ifndef L1X_TIMING_BUDGET_US    // the target may override it together with L1X_SAMPLE_PERIOD_MS
#define L1X_TIMING_BUDGET_US 45000
#endif
// ST refuses to start TIMED ranging unless period >= budget + 4 ms ( vl53l1_api.c:1772-1781 ), and the error would
// latch for the whole flight: catch a bad target override at compile time ( +5 covers the budget read-back rounding ).
static_assert ( L1X_SAMPLE_PERIOD_MS >= L1X_TIMING_BUDGET_US / 1000 + 5, "VL53L1X: period must be >= budget + 4 ms" );

#define L1X_INTER_MEASUREMENT_MS L1X_SAMPLE_PERIOD_MS    // ms, TIMED-mode period ( ST needs >= budget + 4 ms: 50 >= 49 by default )

VL53L1_Dev_t MyDevice_L1;
// VL53L0X_Dev_t *pMyDevice = &MyDevice;

VL53L1_Error Global_Status_L1 = 0;
// VL53L1_Error _Global_Status_L1 = 0;
VL53L1_RangingMeasurementData_t RangingMeasurementData_L1;
VL53L1_RangingMeasurementData_t _RangingMeasurementData_L1x;

uint8_t Range_Status_L1    = 0;
uint16_t NewSensorRange_L1 = 0;
uint16_t debug_range_L1    = 0;
bool isTofDataNewflag_L1   = false;
bool out_of_range_L1       = false;
bool startRanging_L1       = false;    // Cleanup later
bool useRangingSensor_L1   = false;    // Cleanup later

Interval rangePoll_L1;

void update_status_L1 ( VL53L1_Error Status ) {
  Global_Status_L1 = Status;
}

LaserSensor_L1 XVision;

#ifdef LASER_TOF_L1x

uint32_t sampleCount_L1  = 0;    // genuine new results ( valid or not ) since boot
uint32_t lastSampleMs_L1 = 0;    // ms, millis ( ) at the last genuine new result

// Per-sample result read: RESULT__RANGE_STATUS ( 0x0089 ) up to and including
// RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 ( 0x0098-0x0099 ), one 17-byte I2C read in place of
// VL53L1_GetRangingMeasurementData's 133-byte system + core + debug results read. Offsets into the block:
#define L1X_RES_FIRST_REG VL53L1_RESULT__RANGE_STATUS
#define L1X_RES_LEN       ( VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 + 2 - L1X_RES_FIRST_REG )
#define L1X_RES_RANGE_STATUS  ( VL53L1_RESULT__RANGE_STATUS - L1X_RES_FIRST_REG )
#define L1X_RES_STREAM_COUNT  ( VL53L1_RESULT__STREAM_COUNT - L1X_RES_FIRST_REG )
#define L1X_RES_SPADS         ( VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 - L1X_RES_FIRST_REG )
#define L1X_RES_AMBIENT       ( VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0 - L1X_RES_FIRST_REG )
#define L1X_RES_SIGMA         ( VL53L1_RESULT__SIGMA_SD0 - L1X_RES_FIRST_REG )
#define L1X_RES_RANGE         ( VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 - L1X_RES_FIRST_REG )
#define L1X_RES_SIGNAL        ( VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 - L1X_RES_FIRST_REG )

static_assert ( L1X_RES_LEN == 17, "VL53L1X result block is 0x0089..0x0099" );

static int32_t l1xProxyMinMm = -30;    // mm, ST TUNING_PROXY_MIN; read from the API at init

static uint16_t l1xBe16 ( const uint8_t *p ) {
  return ( uint16_t ) ( ( ( uint32_t ) p [ 0 ] << 8 ) | ( uint32_t ) p [ 1 ] );
}

// RangeStatus from RESULT__RANGE_STATUS exactly as the full API derives it in TIMED mode: the device-error remap of
// VL53L1_copy_sys_and_core_results_to_range_results + SetSimpleData, then ConvertStatusLite. The sigma and signal
// limits are checked on the device ( written into the timing config at StartMeasurement ) and come back here as
// SIGMATHRESHOLDCHECK / MSRCNOTARGET. Their values are the preset tuning defaults, sigma 90 mm / signal 1.5 MCPS
// in both Medium ( tp_lite_med_*, vl53l1_api_preset_modes.c:545-553 ) and Long ( tp_lite_long_*, :781-784 ): the
// preset overwrites DataInit's 18 mm / 0.25 MCPS.
static uint8_t l1xRangeStatus ( uint8_t deviceRangeStatus, uint8_t streamCount ) {
  switch ( deviceRangeStatus & VL53L1_RANGE_STATUS__RANGE_STATUS_MASK ) {
    case VL53L1_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
    case VL53L1_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
    case VL53L1_DEVICEERROR_NOVHVVALUEFOUND:
    case VL53L1_DEVICEERROR_MULTCLIPFAIL:
      return VL53L1_RANGESTATUS_HARDWARE_FAIL;
    case VL53L1_DEVICEERROR_USERROICLIP:
      return VL53L1_RANGESTATUS_MIN_RANGE_FAIL;
    case VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY:
      return VL53L1_RANGESTATUS_SYNCRONISATION_INT;
    case VL53L1_DEVICEERROR_RANGECOMPLETE:    // the first range after start ( stream count 0 ) has no wrap check
      return ( streamCount == 0 ) ? VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL : VL53L1_RANGESTATUS_RANGE_VALID;
    case VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK:
      return VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
    case VL53L1_DEVICEERROR_RANGEPHASECHECK:
      return VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;
    case VL53L1_DEVICEERROR_MSRCNOTARGET:
      return VL53L1_RANGESTATUS_SIGNAL_FAIL;
    case VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK:
      return VL53L1_RANGESTATUS_SIGMA_FAIL;
    case VL53L1_DEVICEERROR_PHASECONSISTENCY:
      return VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
    case VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD:
      return VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL;
    case VL53L1_DEVICEERROR_MINCLIP:
      return VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED;
    default:
      return VL53L1_RANGESTATUS_NONE;
  }
}

// Decodes the result block into RangingMeasurementData_L1 the way VL53L1_GetRangingMeasurementData does for the
// fields it sets. Not updated: TimeStamp, RangeQualityLevel and the API's LimitChecksCurrent / LimitChecksStatus.
// Simplified: EffectiveSpadRtnCount is always the DSS count ( the API switches to the MM1 / MM2 count on those
// report statuses ). Deliberately left stale: the LL driver's sys_results / ll_state ( checked only back-to-back ).
static void l1xDecodeResult ( const uint8_t *res, VL53L1_RangingMeasurementData_t *out ) {
  const uint8_t streamCount = res [ L1X_RES_STREAM_COUNT ];

  // Crosstalk-corrected range ( offset and crosstalk are corrected on the device ) times the software gain factor,
  // with the API's own arithmetic ( VL53L1_copy_sys_and_core_results_to_range_results ).
  int32_t rangeMm = ( int32_t ) l1xBe16 ( &res [ L1X_RES_RANGE ] );
  rangeMm *= ( int32_t ) MyDevice_L1.Data.LLData.gain_cal.standard_ranging_gain_factor;
  rangeMm += 0x0400;
  rangeMm /= 0x0800;

  uint32_t sigma = ( uint32_t ) l1xBe16 ( &res [ L1X_RES_SIGMA ] ) << 5;    // 14.2 -> 9.7 fixed point, clipped
  if ( sigma > 0xFFFF )
    sigma = 0xFFFF;

  out->StreamCount           = streamCount;
  out->SignalRateRtnMegaCps  = VL53L1_FIXPOINT97TOFIXPOINT1616 ( l1xBe16 ( &res [ L1X_RES_SIGNAL ] ) );
  out->AmbientRateRtnMegaCps = VL53L1_FIXPOINT97TOFIXPOINT1616 ( l1xBe16 ( &res [ L1X_RES_AMBIENT ] ) );
  out->EffectiveSpadRtnCount = l1xBe16 ( &res [ L1X_RES_SPADS ] );
  out->SigmaMilliMeter       = VL53L1_FIXPOINT97TOFIXPOINT1616 ( sigma );
  out->RangeMilliMeter       = ( int16_t ) rangeMm;
  out->RangeFractionalPart   = 0;
  out->RangeStatus           = l1xRangeStatus ( res [ L1X_RES_RANGE_STATUS ], streamCount );

  if ( ( out->RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID ) && ( out->RangeMilliMeter < 0 ) ) {
    if ( out->RangeMilliMeter < l1xProxyMinMm )
      out->RangeStatus = VL53L1_RANGESTATUS_RANGE_INVALID;
    else
      out->RangeMilliMeter = 0;
  }
}

void ranging_init_L1 ( void ) {
  VL53L1_Error Status = Global_Status_L1;

  MyDevice_L1.I2cDevAddr      = 0x29;
  MyDevice_L1.comms_type      = 1;
  MyDevice_L1.comms_speed_khz = 400;

  Status = VL53L1_WaitDeviceBooted ( &MyDevice_L1 );    // Wait till the device boots. Blocking.

  if ( Status == VL53L1_ERROR_NONE )
    Status = VL53L1_DataInit ( &MyDevice_L1 );    // Data initialization

  update_status_L1 ( Status );

  if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {
    Status = VL53L1_StaticInit ( &MyDevice_L1 );    // Device Initialization
    update_status_L1 ( Status );
  }

  if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {
    Status = VL53L1_SetDistanceMode ( &MyDevice_L1, L1X_DISTANCE_MODE );    // Medium unless the target overrides it
    update_status_L1 ( Status );
  }

  if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {
    Status = VL53L1_SetMeasurementTimingBudgetMicroSeconds ( &MyDevice_L1, L1X_TIMING_BUDGET_US );
    update_status_L1 ( Status );
  }

  if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {
    Status = VL53L1_SetInterMeasurementPeriodMilliSeconds ( &MyDevice_L1, L1X_INTER_MEASUREMENT_MS );
    update_status_L1 ( Status );
  }

  if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {    // negative-range floor used by l1xDecodeResult ( )
    Status = VL53L1_GetTuningParameter ( &MyDevice_L1, VL53L1_TUNING_PROXY_MIN, &l1xProxyMinMm );
    update_status_L1 ( Status );
  }
}

void getRange_L1 ( ) {
  VL53L1_Error Status     = Global_Status_L1;
  static uint8_t dataFlag = 0;
  static bool startNow    = true;

  if ( rangePoll_L1.set ( RANGE_POLL, true ) ) {    // Check for new data every 10ms

    if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {
      if ( startNow ) {
        Status = VL53L1_StartMeasurement ( &MyDevice_L1 );
        update_status_L1 ( Status );
        startNow = false;
      }
    }

    if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {    // Check if data is ready
      if ( ! startNow ) {
        Status = VL53L1_GetMeasurementDataReady ( &MyDevice_L1, &dataFlag );
        update_status_L1 ( Status );
      }
    }

    if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {
      if ( dataFlag ) {
        dataFlag = 0;    // re-query data-ready on the next poll, never re-read the same result

        // Short path instead of VL53L1_GetRangingMeasurementData + VL53L1_ClearInterruptAndStartMeasurement ( 5.4 ms ):
        // in the TIMED preset GPH is disabled and the per-range GENERAL_ONWARDS rewrite only re-sends the unchanged
        // configuration, so the range status / range block and the SYSTEM__INTERRUPT_CLEAR handshake are all that is
        // needed. The LL driver's stream-count state is not advanced; it is only checked in back-to-back mode.
        uint8_t result [ L1X_RES_LEN ];
        Status = VL53L1_ReadMulti ( &MyDevice_L1, L1X_RES_FIRST_REG, result, L1X_RES_LEN );
        update_status_L1 ( Status );

        if ( Global_Status_L1 == VL53L1_ERROR_NONE ) {
          // Release the result so data-ready drops until the next measurement completes.
          Status = VL53L1_clear_interrupt ( &MyDevice_L1 );
          update_status_L1 ( Status );

          l1xDecodeResult ( result, &RangingMeasurementData_L1 );

          // One genuine new result ( a failed clear latches Global_Status_L1, which isOutofRange_L1 reports ).
          sampleCount_L1++;
          lastSampleMs_L1     = millis ( );
          isTofDataNewflag_L1 = true;
          Range_Status_L1     = RangingMeasurementData_L1.RangeStatus;

          const int16_t rangeMm = RangingMeasurementData_L1.RangeMilliMeter;    // mm, may be negative close in

          if ( ( RangingMeasurementData_L1.RangeStatus == 0 ) && ( rangeMm >= L1X_MIN_VALID_MM ) && ( rangeMm < 4500 ) ) {
            NewSensorRange_L1 = ( uint16_t ) rangeMm;    // positive here, so no wrap
            out_of_range_L1   = false;
          } else {
            out_of_range_L1 = true;    // NewSensorRange_L1 keeps the last valid range ( also below L1X_MIN_VALID_MM )
          }
        }
      }
    }
  }
}

bool isTofDataNew_L1 ( void ) {
  return isTofDataNewflag_L1;
}

bool isOutofRange_L1 ( void ) {
  // Out of range also when a latched ST error has stopped ranging for good, or when no new result has
  // arrived for L1X_STALE_MS ( a stall that raises no error ): the consumer then falls back to the baro.
  return out_of_range_L1 || ( Global_Status_L1 != VL53L1_ERROR_NONE ) || ( ( millis ( ) - lastSampleMs_L1 ) > L1X_STALE_MS );
}

#endif

static bool _startNow = true;

bool LaserSensor_L1::init ( VL53L1_DistanceModes _DistanceMode ) {

  // Set the I2C device address for the sensor
  MyDevice_L1x.I2cDevAddr = 0x29;
  // Set the communication type (1 indicates I2C)
  MyDevice_L1x.comms_type = 1;
  // Set the communication speed in kHz
  MyDevice_L1x.comms_speed_khz = 400;

  // Wait until the device has completed its boot process.
  // This is a blocking call that updates _Global_Status_L1x with the result.
  _Global_Status_L1x = VL53L1_WaitDeviceBooted ( &MyDevice_L1x );

  // Check if there was no error during the boot process
  if ( _Global_Status_L1x == VL53L1_ERROR_NONE )
    // Initialize data structures related to the device
    _Global_Status_L1x = VL53L1_DataInit ( &MyDevice_L1x );

  // If data initialization was successful, proceed with static device initialization
  if ( _Global_Status_L1x == VL53L1_ERROR_NONE )
    _Global_Status_L1x = VL53L1_StaticInit ( &MyDevice_L1x );

  // If static initialization was successful, set the distance mode to medium
  if ( _Global_Status_L1x == VL53L1_ERROR_NONE )
    _Global_Status_L1x = VL53L1_SetDistanceMode ( &MyDevice_L1x, _DistanceMode );

  // Return true if all operations were successful, otherwise return false
  if ( _Global_Status_L1x == VL53L1_ERROR_NONE )
    return true;
  return false;
}

void LaserSensor_L1::setAddress ( uint8_t _address ) {
  // Call the VL53L1 API function to set the device address.
  // The address is multiplied by 2 due to the I2C protocol shift requirement.
  VL53L1_SetDeviceAddress ( &MyDevice_L1x, ( _address * 2 ) );
  // Update the internal record of the device's I2C address.
  MyDevice_L1x.I2cDevAddr = _address;
}

bool LaserSensor_L1::startRanging ( void ) {

  if ( _Global_Status_L1x == VL53L1_ERROR_NONE && _startNow ) {
    _Global_Status_L1x = VL53L1_StartMeasurement ( &MyDevice_L1x );
    _startNow          = false;
    Monitor_Println ( "check1", _Global_Status_L1x );
  }
  if ( _Global_Status_L1x == VL53L1_ERROR_NONE ) {
    _Global_Status_L1x = VL53L1_GetRangingMeasurementData ( &MyDevice_L1x, &_RangingMeasurementData_L1x );

    isTofDataNewflag_L1 = true;
    if ( ! _RangingMeasurementData_L1x.RangeStatus ) {

      if ( _RangingMeasurementData_L1x.RangeMilliMeter < 4500 ) {

        _range = _RangingMeasurementData_L1x.RangeMilliMeter;
        return true;
      }
    }
  }
  _range = -1;
  return false;
}

int16_t LaserSensor_L1::getLaserRange ( void ) {
  return _range;
}
