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
 #  Last Modified: Thu, 1st Oct 2026                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-01	AJ	batteryCriticalConfirmed ( ): critical, not provisional     #
 #  2026-09-29	AJ	Fallback fixed 3.1 / 3.0 V, sag 650 mV, count re-level     #
 #  2026-09-29	AJ	Provisional re-level, FSI re-assert, fallback hover sag    #
 #  2026-09-29	AJ	SoC review: default R, R over hops, latch, voltage-only    #
 #  2026-09-28	AJ	SoC: count-led, per-flight R, Vcomp floor, stale sensor    #
 #  2026-09-28	AJ	Plug-in estimate: settled average, LiPo curve, cell count  #
 #  2026-09-28	AJ	Signed shunt average, no auto-gain, us charge counter      #
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
static uint16_t vBatRawSamples [ vBAT_RAW_BUFFER_SIZE ];    // INA219 bus voltage samples ( mV )

// Signed moving average of the raw INA219 shunt reading ( 10 uV LSB ), kept as a running sum
#define vSHUNT_RAW_BUFFER_SIZE 50
static int16_t vShunt10uVSamples [ vSHUNT_RAW_BUFFER_SIZE ];    // raw shunt samples ( 10 uV )
static int32_t vShunt10uVSum   = 0;                             // sum of the stored samples ( 10 uV ), |sum| <= 50 x 32768
static uint8_t vShunt10uVHead  = 0;                             // next write index
static uint8_t vShunt10uVCount = 0;                             // valid samples ( <= vSHUNT_RAW_BUFFER_SIZE )

// uint16_t vbatscaled        = 0;
// uint16_t vbatLatestADC     = 0;    // most recent unsmoothed raw reading from vBatRaw ADC
// uint16_t amperageLatestADC = 0;    // most recent raw reading from current ADC

// #define VBATT_LPF_FREQ             10

// Function getBatteryState
static batteryState_e batteryState;

// Function batteryInit

// Function handleBatteryConnected

// Cell count from the pack voltage: cells = ceil ( mV / 4400 ), 1..3 ( INA219 16 V bus range ).
// 4.20-4.35 V gives 1, a 2S pack ( 6.0-8.4 V ) 2, a 3S pack ( 9.0-12.6 V ) 3.
static constexpr uint32_t VBATT_CELL_DETECT_MV = 4400U;    // highest voltage still counted as one cell ( mV )
static constexpr uint32_t VBATT_MAX_CELLS      = 3U;

// Plug-in estimate: good bus samples averaged after connect before EstBatteryCapacity is set ( 24 x 21 ms ~ 0.5 s )
static constexpr uint8_t VBATT_PLUGIN_SAMPLES = 24U;

// Settle cap: when bus reads keep failing, the estimate is made at the latest this many updateINA219Voltage ( )
// calls after connect ( 96 x 21 ms ~ 2 s ), from the samples collected, or E = 0 with none ( the count warns )
static constexpr uint8_t VBATT_PLUGIN_MAX_CALLS = 96U;

// Pack resistance used to lift the idle plug-in reading to the resting voltage ( mOhm ): board idle ~125 mA,
// bus-basis pack resistance 100-160 mOhm measured on the test packs, so about +15 mV at idle
static constexpr uint32_t BATTERY_REST_R_MOHM = 120U;

static bool batteryEstimateReady = false;    // EstBatteryCapacity set from the plug-in average
static uint8_t plugInSamples     = 0;        // good bus samples since connect ( <= VBATT_PLUGIN_SAMPLES )
static uint8_t plugInCalls       = 0;        // voltage updates since connect, good or failed ( <= VBATT_PLUGIN_MAX_CALLS )
static uint32_t plugInSum_mV     = 0;        // sum of those samples ( mV ), <= 24 x 65535

// 1S LiPo resting-voltage curve ( standard 21-point table ): cell mV -> percent left, linear between points,
// 100 % at or above 4200 mV, 0 % at or below 3270 mV. Descending voltage.
typedef struct {
  uint16_t cell_mV;    // resting cell voltage ( mV )
  uint8_t pct;         // charge left ( % )
} lipoCurvePoint_t;

static constexpr lipoCurvePoint_t lipoRestCurve [ ] = {
  { 4200, 100 }, { 4150, 95 }, { 4110, 90 }, { 4080, 85 }, { 4020, 80 }, { 3980, 75 }, { 3950, 70 },
  { 3910, 65 },  { 3870, 60 }, { 3850, 55 }, { 3840, 50 }, { 3820, 45 }, { 3800, 40 }, { 3790, 35 },
  { 3770, 30 },  { 3750, 25 }, { 3730, 20 }, { 3710, 15 }, { 3690, 10 }, { 3610, 5 },  { 3270, 0 },
};
static constexpr uint8_t LIPO_REST_CURVE_POINTS = static_cast< uint8_t > ( sizeof ( lipoRestCurve ) / sizeof ( lipoRestCurve [ 0 ] ) );
static_assert ( LIPO_REST_CURVE_POINTS == 21, "the LiPo resting curve has 21 points" );

uint8_t batteryCellCount        = 1;    // cell count
uint16_t batteryMaxVoltage      = 0;
uint16_t batteryWarningVoltage  = 0;    // stored warning voltage x cells ( mV ); reported only, no low-battery decision uses it
uint16_t batteryCriticalVoltage = 0;    // stored minimum voltage x cells ( mV ); reported only, no low-battery decision uses it
uint16_t batteryCapacity_mAh    = 0;
uint16_t EstBatteryCapacity     = 0;

// Function handleBatteryDisconnected

// Function updateINA219Voltage
#define VBATT_PRESENT_MV 1000U    // battery present at or above this averaged bus voltage ( mV ), absent below
uint16_t vBat_mV = 0;    // battery ( bus ) voltage in mV, 50-sample average
uint16_t vBatRaw = 0;    // battery voltage in 0.1V steps: vBat_mV / 100, floored

// Stale sensor: consecutive failed INA219 reads; stale after 12 calls without a good read ( 12 x 21 ms = 252 ms )
static constexpr uint8_t INA219_STALE_CALLS = 12U;
static uint8_t busFailCalls                 = 0;    // consecutive failed bus reads ( saturates at 255 )
static uint8_t shuntFailCalls               = 0;    // consecutive failed shunt reads ( saturates at 255 )
uint8_t batterySensorStale                  = 0;    // BATTERY_STALE_VOLTAGE | BATTERY_STALE_CURRENT ( battery.h )

// Function ProcessedINA219Current
// Shunt scale: I ( mA ) = n x 10 uV / R ohm = n x ( 0.01 mV / R ohm ) = n x 0.5 mA at 20 mOhm
static constexpr float INA219_MA_PER_10UV = 0.01f / INA219_SHUNT_RESISTOR;
static_assert ( INA219_MA_PER_10UV > 0.49f && INA219_MA_PER_10UV < 0.51f, "one R020 shunt: 0.5 mA per 10 uV LSB" );

uint16_t mAmpRaw      = 0;    // averaged battery current in mA ( >= 0, rounded )
uint16_t mAmpWithGain = 0;    // equal to mAmpRaw ( the auto-gain was removed ); kept for its readers
uint16_t mAhDrawn     = 0;    // milliampere hours drawn from the battery since start ( saturates at 0xFFFF )
uint16_t mAhRemain    = 0;    // reported mAh left ( BMS_Update ): E - mAhDrawn pulled down by the voltage floor, non-increasing
                              // ( no current sensing: curve fraction of the raw cell voltage x capacity )

// Function updateINA219Current
#define MA_US_PER_MAH 3600000000ULL    // 1 mAh = 1 mA x 3600 s = 3.6e9 mA x us
static uint32_t last_us     = 0;       // timestamp of the previous call ( us )
static uint64_t mA_us_accum = 0;       // charge drawn since start ( mA x us )

// SoC model and warnings ( Pluto Fuel Gauge ). Per cell: pack mV / batteryCellCount.
// Vcomp = ( bus + I x R ) per cell, with R the pack resistance measured once per power-up over the first 35 s of loaded
// flight, and BATTERY_DEFAULT_R_MOHM until then. Loaded flight: armed and mAmpRaw >= 1500 mA.
static constexpr uint32_t BATTERY_DEFAULT_R_MOHM  = 100U;          // R until measured: at the low end of the packs ( 100-157 ), so Vcomp errs low ( early )
static constexpr float BATTERY_R_MIN_MOHM         = 80.0f;         // measured R clamped to this range ( mOhm )
static constexpr float BATTERY_R_MAX_MOHM         = 250.0f;
static constexpr uint16_t BATTERY_LOADED_MIN_MA   = 1500U;         // loaded flight: armed and at least this current ( mA )
static constexpr uint32_t BATTERY_R_WIN_START_US  = 25000000U;     // loaded time at which the R window opens ( us )
static constexpr uint32_t BATTERY_R_WIN_MID_US    = 30000000U;     // window midpoint: charge used is read here
static constexpr uint32_t BATTERY_R_WIN_END_US    = 35000000U;     // window end: R computed
static constexpr uint32_t BATTERY_REST_CAP_CELL_MV = 4180U;        // rest point capped per cell: removes charger surface charge
static constexpr float VCOMP_EMA_TAU_US           = 1000000.0f;    // Vcomp EMA time constant ( 1 s )
static constexpr float VCOMP_WARN_CELL_MV         = 3745.0f;       // warning at or below ( mV per cell )
static constexpr float VCOMP_CRIT_CELL_MV         = 3600.0f;       // critical at or below ( mV per cell )
static constexpr uint32_t COUNT_WARN_PCT          = 15U;           // warning at count remaining <= 15 % of capacity
static constexpr uint32_t COUNT_CRIT_PCT          = 5U;            // critical at count remaining <= 5 % of capacity
static constexpr float VOLT_PULLDOWN_FRACTION     = 0.25f;         // Vcomp pulls the remaining down at <= 25 % on the curve
static constexpr uint32_t BATTERY_DEBOUNCE_US     = 1500000U;      // a condition holds this long before the state steps down
static constexpr uint32_t DEFAULT_R_MAX_COUNT_PCT = 40U;           // default-R voltage test only at count remaining <= 40 %
// No-current fallback ( voltage only ): fixed thresholds on the loaded ( raw bus ) voltage per cell, armed only. On the two
// measured PRIMUS_X2_v1 packs the warning falls at ~13-27 % and the critical at ~5-13 % left. The stored config voltages
// are not used: MSP_SET_VOLTAGE_METER_CONFIG rewrites them with every capacity change from the app.
static constexpr uint32_t FALLBACK_WARN_CELL_MV     = 3100U;      // warning at or below ( mV per cell, loaded )
static constexpr uint32_t FALLBACK_CRIT_CELL_MV     = 3000U;      // critical at or below ( mV per cell, loaded )
static constexpr uint32_t BATTERY_HOVER_SAG_CELL_MV = 650U;       // armed SoC: hover sag per cell ( mV ), measured 4.3 A x 146-169 mOhm = 630-730 mV

// Pack resistance: kept until the next battery connect ( a power-up ), never reset on disarm. The rest point is the
// one before the first arming; the loaded window accumulates across armings ( hops ) until it completes.
uint16_t batteryResistance_mOhm = 0;        // bus-basis pack resistance ( mOhm ), 80..250, 0 = not measured yet
static bool rArmedOnce          = false;    // armed at least once since connect: the rest point is frozen
static bool rRestValid          = false;    // a rest point has been taken
static uint16_t rRest_mV        = 0;        // rest point: vBat_mV, capped at 4180 mV per cell ( mV, pack )
static uint16_t rRest_mA        = 0;        // rest point: mAmpRaw ( mA )
static uint16_t rRestDrawn_mAh  = 0;        // rest point: mAhDrawn ( mAh )
static uint32_t rLoadedUs       = 0;        // loaded time since connect ( us ), saturating; >= window end: done
static uint32_t rSumV_mV        = 0;        // window sums, <= 476 x 65535
static uint32_t rSumI_mA        = 0;
static uint16_t rSamples        = 0;        // window samples ( ~476 in 10 s )
static bool rMidSeen            = false;
static uint16_t rMidDrawn_mAh   = 0;        // mAhDrawn at the window midpoint ( mAh )

// Compensated voltage and reported remaining
static bool vCompCellInit    = false;    // vCompCell_mV holds a value
static float vCompCell_mV    = 0.0f;     // EMA of ( bus + I x R ) / cells ( mV per cell )
static uint16_t mAhPullDown  = 0;        // persistent pull-down of the count by the voltage floor ( mAh )
static bool reportInit       = false;    // reportPrev holds a value since the estimate became ready
static uint16_t reportPrev   = 0;        // previous reported remaining ( mAh ): the report never rises
static float socPrev         = 0.0f;     // previous soc_Fused ( % ), voltage-only mode: no rise while armed
static bool socPrevInit      = false;    // socPrev holds a value

// Warnings: one debounce timer per source ( us ). The voltage timers run only in loaded flight ( 0 outside it ) and
// hold while the bus reading is stale; the count timers always run.
static uint32_t warnVoltUs  = 0;
static uint32_t critVoltUs  = 0;
static uint32_t warnCountUs = 0;
static uint32_t critCountUs = 0;

// A warning or critical step caused by the default-R voltage test alone ( R unknown, no count condition ). Not latched:
// re-levelled to what the count supports on disarm and when R is measured ( count mode only ), and confirmed as soon
// as the count supports the standing level.
static bool alarmProvisional = false;

uint8_t BatteryWarningMode = 0;    // 0 OK, 1 WARNING, 2 CRITICAL ( MSP_ANALOG )

// Function BMS_Update variables
float soc_Fused   = 0;    // reported remaining / capacity ( % ), 0..100, non-increasing once E is ready
                          // ( no current sensing: curve fraction of the raw cell voltage, non-increasing while armed )
uint16_t vBatComp = 0;    // pack mV: Vcomp x cells, R measured or BATTERY_DEFAULT_R_MOHM, I = 0 while the current is stale

batteryState_e getBatteryState ( void ) {
  // Return the current battery state stored in the variable 'batteryState'
  return batteryState;
}

/**
 * @brief True when the battery is critical and the level is confirmed, not provisional.
 *
 * A critical raised by the default-R voltage test alone ( before the pack resistance is measured ) is provisional
 * and can be re-levelled; it sounds the beeper but must not start the auto-land or refuse arming ( mw.cpp ). A
 * critical from the count, from the measured-R voltage test, or on the voltage-only path is confirmed and latched.
 */
bool batteryCriticalConfirmed ( void ) {
  return ( batteryState == BATTERY_CRITICAL ) && ! alarmProvisional;
}

/**
 * @brief Initializes the battery configuration and related parameters.
 *
 * This function sets up the battery configuration by assigning the provided initial configuration to the
 * global battery configuration pointer. It initializes the battery state to indicate absence and prepares
 * the averaging buffers for voltage and current measurements.
 *
 * @param initialBatteryConfig Pointer to the initial battery configuration structure.
 */
void batteryInit ( batteryConfig_t *initialBatteryConfig ) {
  // Assign the initial configuration to the global battery configuration pointer
  batteryConfig = initialBatteryConfig;

  // Initialize battery state and parameters
  batteryState = BATTERY_NOT_PRESENT;    // Set the initial state as battery not present

  ring_avg_u16_init ( &vbatRawAvgRing, vBatRawSamples, vBAT_RAW_BUFFER_SIZE );
  memset ( vShunt10uVSamples, 0, sizeof ( vShunt10uVSamples ) );
  vShunt10uVSum   = 0;
  vShunt10uVHead  = 0;
  vShunt10uVCount = 0;
}

/**
 * @brief Rounds a non-negative float to uint16, saturating at 0 and 0xFFFF.
 */
static uint16_t roundToU16 ( float v ) {
  if ( v <= 0.0f ) {
    return 0U;
  }
  if ( v >= 65535.0f ) {
    return 0xFFFFU;
  }
  return static_cast< uint16_t > ( v + 0.5f );
}

/**
 * @brief a + b, saturating at 0xFFFFFFFF ( time accumulators ).
 */
static inline uint32_t addSatU32 ( uint32_t a, uint32_t b ) {
  return ( a > 0xFFFFFFFFU - b ) ? 0xFFFFFFFFU : a + b;
}

/**
 * @brief Fraction of charge left in a resting 1S LiPo cell, from the lipoRestCurve table.
 *
 * Linear between the table points; 1.0 at or above 4200 mV, 0.0 at or below 3270 mV. Bounded: at most 20
 * table steps. Used by the plug-in estimate, the pack resistance measurement and the voltage floor.
 *
 * @param cell_mV Resting cell voltage in mV.
 * @return Charge left, 0.0 .. 1.0.
 */
static float lipoCellRestFraction ( uint16_t cell_mV ) {
  if ( cell_mV >= lipoRestCurve [ 0 ].cell_mV ) {
    return 1.0f;
  }

  for ( uint8_t i = 1; i < LIPO_REST_CURVE_POINTS; i++ ) {
    const lipoCurvePoint_t &hi = lipoRestCurve [ i - 1 ];
    const lipoCurvePoint_t &lo = lipoRestCurve [ i ];
    if ( cell_mV > lo.cell_mV ) {
      const float span_mV  = static_cast< float > ( hi.cell_mV - lo.cell_mV );    // > 0, table descends
      const float span_pct = static_cast< float > ( hi.pct - lo.pct );
      const float pct      = static_cast< float > ( lo.pct ) + static_cast< float > ( cell_mV - lo.cell_mV ) * span_pct / span_mV;
      return pct * 0.01f;    // % -> fraction
    }
  }

  return 0.0f;    // at or below the last point ( 3270 mV )
}

/**
 * @brief Resting 1S LiPo cell voltage for a fraction of charge left: the inverse of lipoCellRestFraction ( ).
 *
 * Linear between the lipoRestCurve points; 4200 mV at or above 1.0, 3270 mV at or below 0.0. Bounded: at most
 * 20 table steps.
 *
 * @param fraction Charge left, 0.0 .. 1.0.
 * @return Resting cell voltage in mV.
 */
static float lipoCellRestVoltage ( float fraction ) {
  const float pct = fraction * 100.0f;    // fraction -> %
  if ( pct >= static_cast< float > ( lipoRestCurve [ 0 ].pct ) ) {
    return static_cast< float > ( lipoRestCurve [ 0 ].cell_mV );
  }

  for ( uint8_t i = 1; i < LIPO_REST_CURVE_POINTS; i++ ) {
    const lipoCurvePoint_t &hi = lipoRestCurve [ i - 1 ];
    const lipoCurvePoint_t &lo = lipoRestCurve [ i ];
    if ( pct > static_cast< float > ( lo.pct ) ) {
      const float span_mV  = static_cast< float > ( hi.cell_mV - lo.cell_mV );
      const float span_pct = static_cast< float > ( hi.pct - lo.pct );    // > 0, table descends
      return static_cast< float > ( lo.cell_mV ) + ( pct - static_cast< float > ( lo.pct ) ) * span_mV / span_pct;
    }
  }

  return static_cast< float > ( lipoRestCurve [ LIPO_REST_CURVE_POINTS - 1U ].cell_mV );    // 0 % ( 3270 mV )
}

/**
 * @brief Sets the cell count from a pack voltage, then the per-pack warning and critical thresholds from it.
 *
 * cells = ceil ( pack_mV / 4400 ), clamped to 1..3, so 4.20-4.35 V is always one cell and the count is never 0
 * while a battery is present ( frsky.c divides by it ). batteryMaxVoltage stays per cell as before:
 * MSP_VOLTAGE_METER_CONFIG echoes it beside the per-cell min / warning settings.
 *
 * @param pack_mV Averaged bus ( pack ) voltage in mV.
 */
static void setCellCountAndThresholds ( uint16_t pack_mV ) {
  uint32_t cells = ( static_cast< uint32_t > ( pack_mV ) + VBATT_CELL_DETECT_MV - 1U ) / VBATT_CELL_DETECT_MV;    // ceil
  if ( cells < 1U ) {
    cells = 1U;
  } else if ( cells > VBATT_MAX_CELLS ) {
    cells = VBATT_MAX_CELLS;
  }
  batteryCellCount = static_cast< uint8_t > ( cells );

  // Config values are per cell in 0.1 V units; thresholds are per pack in mV, clamped to uint16 ( 3 x 255 x 100 > 65535 )
  const uint32_t crit_mV = cells * batteryConfig->vBatMinVoltage * 100U;
  const uint32_t warn_mV = cells * batteryConfig->vBatWarningVoltage * 100U;
  batteryCriticalVoltage = static_cast< uint16_t > ( crit_mV > 0xFFFFU ? 0xFFFFU : crit_mV );
  batteryWarningVoltage  = static_cast< uint16_t > ( warn_mV > 0xFFFFU ? 0xFFFFU : warn_mV );
}

/**
 * @brief Clears the SoC model's per-pack state: pack resistance, rest point, R window, Vcomp, pull-down, timers.
 *
 * Called on a battery connect ( which also sets the state back to OK, the only way out of a latched warning
 * besides a power-off ). Removing the battery powers the board down, so in normal use this runs once per
 * power-up; on USB power a new pack is a new pack and must not inherit the old one's resistance.
 */
static void batteryModelReset ( void ) {
  batteryResistance_mOhm = 0;
  rArmedOnce             = false;
  rRestValid             = false;
  rLoadedUs              = 0;
  rSumV_mV               = 0;
  rSumI_mA               = 0;
  rSamples               = 0;
  rMidSeen               = false;
  vCompCellInit          = false;
  mAhPullDown            = 0;
  reportInit             = false;
  socPrevInit            = false;
  warnVoltUs             = 0;
  critVoltUs             = 0;
  warnCountUs            = 0;
  critCountUs            = 0;
  alarmProvisional       = false;
}

/**
 * @brief Handles a battery being connected: state, configured values, cell count and thresholds.
 *
 * Non-blocking. The capacity estimate is not made here: updateINA219Voltage ( ) averages the next
 * VBATT_PLUGIN_SAMPLES good bus samples ( ~0.5 s ) and then calls handleBatteryPlugInEstimate ( ). Until then
 * EstBatteryCapacity is 0 and batteryEstimateReady is false, which holds off the low-battery warning and holds
 * soc_Fused / mAhRemain.
 */
static inline void handleBatteryConnected ( ) {
  batteryState = BATTERY_OK;

  batteryCapacity_mAh = batteryConfig->BatteryCapacity;                                        // mAh
  batteryMaxVoltage   = static_cast< uint16_t > ( batteryConfig->vBatMaxVoltage * 100U );    // 0.1 V -> mV, per cell

  // Cell count first, then the thresholds that use it; refreshed from the settled average at the estimate
  setCellCountAndThresholds ( vBat_mV );

  EstBatteryCapacity   = 0;    // set by handleBatteryPlugInEstimate ( )
  batteryEstimateReady = false;
  plugInSamples        = 0;
  plugInCalls          = 0;
  plugInSum_mV         = 0;

  batteryModelReset ( );
}

/**
 * @brief Plug-in capacity estimate from the averaged bus voltage of the first ~0.5 s after connect.
 *
 * The average is lifted to the resting voltage by the idle current x BATTERY_REST_R_MOHM, divided by the cell
 * count and read on the 1S LiPo curve; EstBatteryCapacity = configured capacity x that fraction, rounded,
 * 0 .. capacity ( no wrap below 3.0 V ). Runs once per connect; the estimate is not refreshed before arming.
 * Needs plugInSamples >= 1 ( the caller checks ).
 */
static void handleBatteryPlugInEstimate ( ) {
  const uint16_t avg_mV = static_cast< uint16_t > ( plugInSum_mV / plugInSamples );    // mean of the settle samples ( mV )

  setCellCountAndThresholds ( avg_mV );    // same rule, settled reading

#ifdef INA219_Current
  // Resting pack voltage: loaded average + I x R ( mA x mOhm / 1000 = mV ); mAmpRaw is 0 without current sensing
  const uint32_t rest_mV = static_cast< uint32_t > ( avg_mV ) + ( static_cast< uint32_t > ( mAmpRaw ) * BATTERY_REST_R_MOHM ) / 1000U;
  uint32_t cell_mV       = rest_mV / batteryCellCount;    // cell count >= 1
  if ( cell_mV > 0xFFFFU ) {
    cell_mV = 0xFFFFU;
  }

  const float fraction = lipoCellRestFraction ( static_cast< uint16_t > ( cell_mV ) );    // 0.0 .. 1.0
  float est_mAh        = static_cast< float > ( batteryCapacity_mAh ) * fraction + 0.5f;    // rounded below
  if ( est_mAh > static_cast< float > ( batteryCapacity_mAh ) ) {
    est_mAh = static_cast< float > ( batteryCapacity_mAh );
  }
  EstBatteryCapacity = static_cast< uint16_t > ( est_mAh );    // mAh, 0 .. batteryCapacity_mAh
#endif

  batteryEstimateReady = true;
}

static inline void handleBatteryDisconnected ( ) {
  batteryState           = BATTERY_NOT_PRESENT;
  batteryCellCount       = 1;    // never 0: telemetry/frsky.c divides by it
  batteryWarningVoltage  = 0;
  batteryCriticalVoltage = 0;
  batteryMaxVoltage      = 0;
  batteryCapacity_mAh    = 0;
  EstBatteryCapacity     = 0;
  batteryEstimateReady   = false;
}

/**
 * @brief Sets or clears one batterySensorStale bit.
 */
static inline void setSensorStale ( uint8_t bit, bool stale ) {
  if ( stale ) {
    batterySensorStale = static_cast< uint8_t > ( batterySensorStale | bit );
  } else {
    batterySensorStale = static_cast< uint8_t > ( batterySensorStale & ~bit );
  }
}

/**
 * @brief Updates the battery voltage reading from the INA219 sensor and manages battery connection state.
 *
 * This function reads the INA219 bus voltage in mV, averages it over 50 samples into `vBat_mV` and
 * derives `vBatRaw` ( 0.1 V steps ) from it. A failed read ( I2C error or overflow ) is skipped and the
 * previous average is kept; after INA219_STALE_CALLS failed reads in a row BATTERY_STALE_VOLTAGE is set. It also
 * checks the connection state: connected at vBat_mV >= VBATT_PRESENT_MV, disconnected below it. After a connect,
 * the next VBATT_PLUGIN_SAMPLES good bus samples are summed and the plug-in capacity estimate is made from their
 * mean, once; if they are not in within VBATT_PLUGIN_MAX_CALLS calls the estimate is made from the samples
 * collected, or E = 0 when there are none.
 */
void updateINA219Voltage ( ) {
  uint16_t busMv;    // bus voltage ( mV, 4 mV LSB )
  const bool busOk = INA219_ReadBus_mV ( &busMv );
  if ( busOk ) {
    vBat_mV      = ring_avg_u16_get ( &vbatRawAvgRing, busMv );
    vBatRaw      = static_cast< uint16_t > ( vBat_mV / 100U );    // mV -> 0.1 V, floored
    busFailCalls = 0;
  } else if ( busFailCalls < 0xFFU ) {
    busFailCalls++;
  }
  setSensorStale ( BATTERY_STALE_VOLTAGE, busFailCalls >= INA219_STALE_CALLS );

  // Check if the battery is currently not present but the voltage indicates otherwise
  if ( batteryState == BATTERY_NOT_PRESENT && vBat_mV >= VBATT_PRESENT_MV ) {
    // Handle scenario where a battery has been connected
    handleBatteryConnected ( );
  }
  // Check if the battery is present but the voltage indicates disconnection
  else if ( batteryState != BATTERY_NOT_PRESENT && vBat_mV < VBATT_PRESENT_MV ) {
    // Handle scenario where the battery has been disconnected
    handleBatteryDisconnected ( );
  }
  // Battery present, estimate pending: collect the settle samples ( only samples read after the connect )
  else if ( batteryState != BATTERY_NOT_PRESENT && ! batteryEstimateReady ) {
    if ( plugInCalls < 0xFFU ) {
      plugInCalls++;
    }
    if ( busOk ) {
      plugInSum_mV += busMv;
      plugInSamples++;
    }

    if ( plugInSamples >= VBATT_PLUGIN_SAMPLES ) {
      handleBatteryPlugInEstimate ( );
    } else if ( plugInCalls >= VBATT_PLUGIN_MAX_CALLS ) {
      // Settle cap: bus reads keep failing
      if ( plugInSamples >= 1U ) {
        handleBatteryPlugInEstimate ( );    // from the samples collected
      } else {
        EstBatteryCapacity   = 0;    // nothing to estimate from: count remaining 0, the count warning fires
        batteryEstimateReady = true;
      }
    }
  }
}

/**
 * @brief Adds one raw shunt sample to the signed 50-sample running sum.
 *
 * Bounded work: the oldest sample is subtracted and the new one added, no loop over the buffer.
 *
 * @param sample10uV Raw INA219 shunt reading, signed, 10 uV LSB.
 */
static inline void shuntAvgPush ( int16_t sample10uV ) {
  if ( vShunt10uVCount == vSHUNT_RAW_BUFFER_SIZE ) {
    vShunt10uVSum -= vShunt10uVSamples [ vShunt10uVHead ];    // drop the sample being overwritten
  } else {
    vShunt10uVCount++;
  }

  vShunt10uVSamples [ vShunt10uVHead ] = sample10uV;
  vShunt10uVSum += sample10uV;

  vShunt10uVHead++;
  if ( vShunt10uVHead >= vSHUNT_RAW_BUFFER_SIZE ) vShunt10uVHead = 0;
}

/**
 * @brief Reads the INA219 shunt, updates the 50-sample average and derives the battery current.
 *
 * The raw signed shunt reading ( 10 uV LSB ) is averaged with no rounding; the average is converted to mA
 * ( INA219_MA_PER_10UV, 0.5 mA per 10 uV at 20 mOhm ), a negative average is clamped to 0 once, after
 * averaging, and the result is clamped to 0xFFFF. No gain is applied: the INA219 is trusted.
 * On a failed read the sample is skipped and mAmpRaw / mAmpWithGain keep the last average; after
 * INA219_STALE_CALLS failed reads in a row BATTERY_STALE_CURRENT is set.
 */
static inline void ProcessedINA219Current ( void ) {
  int16_t shunt10uV;    // raw shunt reading ( 10 uV, signed )
  if ( ! INA219_ReadShunt_10uV ( &shunt10uV ) ) {
    if ( shuntFailCalls < 0xFFU ) {
      shuntFailCalls++;
    }
    setSensorStale ( BATTERY_STALE_CURRENT, shuntFailCalls >= INA219_STALE_CALLS );
    return;    // I2C error: keep the last good average
  }
  shuntFailCalls = 0;
  setSensorStale ( BATTERY_STALE_CURRENT, false );

  shuntAvgPush ( shunt10uV );    // vShunt10uVCount >= 1 from here on

  // Average in 10 uV units ( |sum| <= 1.6e6, exact in float )
  float avg10uV = static_cast< float > ( vShunt10uVSum ) / static_cast< float > ( vShunt10uVCount );
  if ( avg10uV < 0.0f ) {
    avg10uV = 0.0f;    // negative ( reverse or offset ) current clamped once, after averaging
  }

  float mA = avg10uV * INA219_MA_PER_10UV;    // 10 uV -> mA
  if ( mA > 65535.0f ) {
    mA = 65535.0f;
  }

  mAmpRaw      = static_cast< uint16_t > ( mA + 0.5f );    // mA, rounded to nearest
  mAmpWithGain = mAmpRaw;                                  // no auto-gain
}

/**
 * @brief Updates the INA219 current measurements and the charge counter.
 *
 * Every call reads one shunt sample ( skipped on an I2C error ) and integrates the averaged current over
 * the full time since the previous call, in mA x us, so no sub-millisecond remainder is lost and a failed
 * read does not drop time ( the last good average is used ). With the current reading stale the last good
 * current is still integrated while armed ( counts on the safe side ) but not while disarmed. mAhDrawn
 * saturates at 0xFFFF. mAhRemain is set by BMS_Update ( ).
 *
 * @param nowUs Current timestamp in microseconds.
 * @param armed True while armed: a stale current is integrated only then.
 */
void updateINA219Current ( uint32_t nowUs, bool armed ) {
  static bool init = false;

  ProcessedINA219Current ( );

  if ( ! init ) {
    init        = true;
    last_us     = nowUs;
    mA_us_accum = 0;
    mAhDrawn    = 0;
    return;
  }

  const uint32_t dtUs = nowUs - last_us;    // wrap-safe ( us )
  last_us             = nowUs;

  const bool currentStale = ( batterySensorStale & BATTERY_STALE_CURRENT ) != 0U;
  if ( ! currentStale || armed ) {
    mA_us_accum += static_cast< uint64_t > ( mAmpWithGain ) * static_cast< uint64_t > ( dtUs );    // mA x us
  }

  const uint64_t mAh64 = mA_us_accum / MA_US_PER_MAH;    // mA x us -> mAh, floored
  mAhDrawn             = ( mAh64 > 0xFFFFULL ) ? static_cast< uint16_t > ( 0xFFFFU ) : static_cast< uint16_t > ( mAh64 );
}

/**
 * @brief True when the count-led model applies: current sensing is built in and enabled.
 *
 * Without INA219_Current the plug-in estimate is not made ( EstBatteryCapacity stays 0 ), so a count test would
 * warn at once; without FEATURE_INA219_CBAT nothing is counted. When false, BMS_Update ( ) uses the voltage-only
 * rule on the raw bus voltage.
 */
static inline bool batteryCountAvailable ( void ) {
#ifdef INA219_Current
  return feature ( FEATURE_INA219_CBAT );
#else
  return false;
#endif
}

/**
 * @brief Compensated cell voltage: ( bus + I x R ) / cells.
 *
 * @param bus_mV Averaged bus ( pack ) voltage in mV.
 * @param current_mA Averaged current in mA.
 * @param r_mOhm Pack resistance in mOhm ( bus basis ).
 * @param cells Cell count, >= 1.
 * @return mV per cell.
 */
static float vCompCellFrom ( uint16_t bus_mV, uint16_t current_mA, uint32_t r_mOhm, uint8_t cells ) {
  const float pack_mV = static_cast< float > ( bus_mV ) + static_cast< float > ( current_mA ) * static_cast< float > ( r_mOhm ) * 0.001f;    // mA x mOhm / 1000 = mV
  return pack_mV / static_cast< float > ( cells );
}

/**
 * @brief Pack resistance from a rest point and a loaded point, less the curve's OCV drop for the charge used.
 *
 * f0 = curve ( rest cell mV ), f1 = f0 - used / capacity ( 0..1 ), drop = ( OCV ( f0 ) - OCV ( f1 ) ) x cells;
 * R = ( rest_mV - drop - load_mV ) / ( load_mA - rest_mA ). The rest current is subtracted, so the rest reading
 * need not be a true open-circuit voltage ( its own I x R cancels ).
 *
 * @param r_mOhm Out: R in mOhm ( unclamped, may be <= 0 for a bad measurement ).
 * @return False when R cannot be computed ( capacity 0, load current not above the rest current ).
 */
static bool batteryResistanceFromPoints ( uint16_t rest_mV, uint16_t rest_mA, uint16_t load_mV, uint16_t load_mA, uint16_t used_mAh, uint16_t capacity_mAh, uint8_t cells, float *r_mOhm ) {
  if ( capacity_mAh == 0U || cells == 0U || load_mA <= rest_mA ) {
    return false;
  }

  const float cellsF  = static_cast< float > ( cells );
  const float f0      = lipoCellRestFraction ( roundToU16 ( static_cast< float > ( rest_mV ) / cellsF ) );
  const float f1      = constrainf ( f0 - static_cast< float > ( used_mAh ) / static_cast< float > ( capacity_mAh ), 0.0f, 1.0f );
  const float drop_mV = ( lipoCellRestVoltage ( f0 ) - lipoCellRestVoltage ( f1 ) ) * cellsF;    // pack OCV drop, >= 0
  const float num_mV  = static_cast< float > ( rest_mV ) - drop_mV - static_cast< float > ( load_mV );
  const float den_mA  = static_cast< float > ( load_mA - rest_mA );    // > 0

  *r_mOhm = num_mV * 1000.0f / den_mA;    // mV / mA = Ohm, x 1000 = mOhm
  return true;
}

/**
 * @brief Remaining charge the compensated voltage gives, when it is low enough to pull the count down.
 *
 * @param vCompCell Compensated cell voltage in mV.
 * @param capacity_mAh Configured capacity in mAh.
 * @param remain_mAh Out: curve fraction x capacity, floored ( mAh ), set only when returning true.
 * @return True when the curve fraction is at or below VOLT_PULLDOWN_FRACTION ( 25 % ).
 */
static bool voltageFloorRemain ( float vCompCell, uint16_t capacity_mAh, uint16_t *remain_mAh ) {
  const float fraction = lipoCellRestFraction ( roundToU16 ( vCompCell ) );
  if ( fraction > VOLT_PULLDOWN_FRACTION ) {
    return false;
  }
  *remain_mAh = static_cast< uint16_t > ( fraction * static_cast< float > ( capacity_mAh ) );    // <= capacity / 4
  return true;
}

/**
 * @brief True when remain_mAh is at or below pct % of capacity_mAh ( integer, no rounding ).
 */
static inline bool countAtOrBelowPct ( uint16_t remain_mAh, uint16_t capacity_mAh, uint32_t pct ) {
  return static_cast< uint32_t > ( remain_mAh ) * 100U <= static_cast< uint32_t > ( capacity_mAh ) * pct;
}

/**
 * @brief Debounce step: adds dtUs while the condition holds, back to 0 when it does not ( saturating, us ).
 */
static inline uint32_t debounceStep ( uint32_t heldUs, bool cond, uint32_t dtUs ) {
  return cond ? addSatU32 ( heldUs, dtUs ) : 0U;
}

/**
 * @brief Measures the pack resistance once per power-up, over the first 35 s of loaded flight.
 *
 * Rest point: every disarmed update with fresh readings before the first arming since connect ( vBat_mV capped at
 * 4180 mV per cell, mAmpRaw, mAhDrawn ). It freezes at the first arming, so a snapshot taken right after a landing
 * ( pack still recovering ) never replaces it. Loaded time counts the real elapsed time of armed updates with
 * mAmpRaw >= 1500 mA and fresh readings, accumulated across armings ( hops ): from 25 s to 35 s of it vBat_mV and
 * mAmpRaw are summed, mAhDrawn is taken at 30 s ( the midpoint ), and at 35 s R is computed, clamped to
 * 80..250 mOhm and kept until the next battery connect. A stale update pauses the window. One attempt per
 * power-up: if R cannot be computed it stays 0 and BATTERY_DEFAULT_R_MOHM is used. Bounded work: no loops.
 *
 * @param dtUs Time since the previous BMS update ( us ).
 * @param armed True while armed.
 * @param stale True when the voltage or the current reading is stale.
 */
static void batteryResistanceUpdate ( uint32_t dtUs, bool armed, bool stale ) {
  if ( batteryResistance_mOhm != 0U || rLoadedUs >= BATTERY_R_WIN_END_US ) {
    return;    // measured, or this power-up's window is over
  }

  if ( ! armed ) {
    if ( ! rArmedOnce && ! stale ) {
      // Surface charge: a pack right off the charger reads 4.20 V or more per cell at rest, which makes R read high
      // ( a late warning ); a rested full pack reads 4.16-4.18 V. The cap removes only that excess; if it clips a
      // true 4.20 V, R reads ~5 mOhm low ( the safe side ).
      const uint32_t cap_mV = BATTERY_REST_CAP_CELL_MV * batteryCellCount;    // <= 3 x 4180
      rRest_mV              = ( static_cast< uint32_t > ( vBat_mV ) > cap_mV ) ? static_cast< uint16_t > ( cap_mV ) : vBat_mV;
      rRest_mA              = mAmpRaw;
      rRestDrawn_mAh        = mAhDrawn;
      rRestValid            = true;
    }
    return;
  }

  rArmedOnce = true;    // the rest point is frozen from here on
  if ( ! rRestValid || stale || mAmpRaw < BATTERY_LOADED_MIN_MA ) {
    return;    // no rest point, paused, or not loaded: the window waits
  }

  rLoadedUs = addSatU32 ( rLoadedUs, dtUs );
  if ( rLoadedUs < BATTERY_R_WIN_START_US ) {
    return;
  }

  if ( ! rMidSeen && rLoadedUs >= BATTERY_R_WIN_MID_US ) {
    rMidDrawn_mAh = mAhDrawn;
    rMidSeen      = true;
  }
  rSumV_mV += vBat_mV;
  rSumI_mA += mAmpRaw;
  rSamples++;

  if ( rLoadedUs < BATTERY_R_WIN_END_US ) {
    return;
  }

  // Window complete ( once per power-up: rLoadedUs stays at or past the end ); rSamples >= 1, rMidSeen
  const uint16_t load_mV = static_cast< uint16_t > ( rSumV_mV / rSamples );
  const uint16_t load_mA = static_cast< uint16_t > ( rSumI_mA / rSamples );
  if ( load_mA < BATTERY_LOADED_MIN_MA ) {
    return;    // every window sample is >= 1500 mA, so this only guards the design rule
  }
  const uint16_t used_mAh = ( rMidDrawn_mAh > rRestDrawn_mAh ) ? static_cast< uint16_t > ( rMidDrawn_mAh - rRestDrawn_mAh ) : 0U;

  float r_mOhm;
  if ( batteryResistanceFromPoints ( rRest_mV, rRest_mA, load_mV, load_mA, used_mAh, batteryCapacity_mAh, batteryCellCount, &r_mOhm ) ) {
    batteryResistance_mOhm = roundToU16 ( constrainf ( r_mOhm, BATTERY_R_MIN_MOHM, BATTERY_R_MAX_MOHM ) );
  }
}

/**
 * @brief Battery state machine: OK, WARNING, CRITICAL.
 *
 * Steps down on a debounced condition. OK -> WARNING also needs the plug-in estimate and the user failsafe enable;
 * a critical condition met from OK goes through WARNING in the same update, with both steps' actions. It never
 * steps back up by itself: only handleBatteryConnected ( ) ( a new pack, normally a power-up ) sets OK again, and
 * BMS_Update ( ) re-levels a provisional alarm ( batteryApplyLevel ( ) ). Each state repeats its arming flag call,
 * beeper and BatteryWarningMode ( 0 / 1 / 2 ) every update, and WARNING / CRITICAL re-assert their
 * FSI flag every update ( mwDisarm ( ) resets LowBattery_inFlight on every disarm ).
 *
 * @param warnLow A warning condition held for BATTERY_DEBOUNCE_US.
 * @param critLow A critical condition held for BATTERY_DEBOUNCE_US.
 */
static void updateBatteryState ( bool warnLow, bool critLow ) {

  switch ( batteryState ) {

    case BATTERY_OK:
      ENABLE_ARMING_FLAG ( OK_TO_ARM );
      BatteryWarningMode = 0;
      // No warning before the plug-in estimate is in ( ~0.5 s after connect, EstBatteryCapacity still 0 )
      if ( batteryEstimateReady && ( warnLow || critLow ) && fsInFlightLowBattery ) {
        batteryState = BATTERY_WARNING;
        set_FSI ( Low_battery );
        beeper ( BEEPER_BAT_LOW );
        BatteryWarningMode = 1;

        if ( critLow ) {    // both at once: the WARNING -> CRITICAL step in the same update
          batteryState = BATTERY_CRITICAL;
          set_FSI ( LowBattery_inFlight );
          reset_FSI ( Low_battery );
          beeper ( BEEPER_BAT_CRIT_LOW );
          BatteryWarningMode = 2;
        }
      }
      break;

    case BATTERY_WARNING:
      DISABLE_ARMING_FLAG ( PREVENT_ARMING );
      if ( critLow ) {
        batteryState = BATTERY_CRITICAL;
        set_FSI ( LowBattery_inFlight );
        reset_FSI ( Low_battery );
        beeper ( BEEPER_BAT_CRIT_LOW );
        BatteryWarningMode = 2;
      } else {
        set_FSI ( Low_battery );    // re-assert every update
        beeper ( BEEPER_BAT_LOW );
        BatteryWarningMode = 1;
      }
      break;

    case BATTERY_CRITICAL:
      DISABLE_ARMING_FLAG ( PREVENT_ARMING );
      set_FSI ( LowBattery_inFlight );    // re-assert every update: mwDisarm ( ) resets it on each disarm
      beeper ( BEEPER_BAT_CRIT_LOW );
      BatteryWarningMode = 2;
      break;

    case BATTERY_NOT_PRESENT:
      break;
  }
}

/**
 * @brief The battery level the given conditions support: CRITICAL, else WARNING, else OK.
 *
 * @param warnNow A warning condition holds now ( not debounced ).
 * @param critNow A critical condition holds now ( not debounced ).
 */
static batteryState_e batterySupportedLevel ( bool warnNow, bool critNow ) {
  if ( critNow ) {
    return BATTERY_CRITICAL;
  }
  return warnNow ? BATTERY_WARNING : BATTERY_OK;
}

/**
 * @brief Sets the battery state to a level with that level's FSI flags and BatteryWarningMode.
 *
 * Used only to re-level a provisional alarm. WARNING: Low_battery set, LowBattery_inFlight reset, mode 1.
 * CRITICAL: LowBattery_inFlight set, Low_battery reset, mode 2. OK: both reset, mode 0. The state machine's per-state
 * beeper and arming flag calls follow in the same update.
 *
 * @param level BATTERY_OK, BATTERY_WARNING or BATTERY_CRITICAL.
 */
static void batteryApplyLevel ( batteryState_e level ) {
  batteryState = level;
  if ( level == BATTERY_CRITICAL ) {
    set_FSI ( LowBattery_inFlight );
    reset_FSI ( Low_battery );
    BatteryWarningMode = 2;
  } else if ( level == BATTERY_WARNING ) {
    set_FSI ( Low_battery );
    reset_FSI ( LowBattery_inFlight );
    BatteryWarningMode = 1;
  } else {
    reset_FSI ( Low_battery );
    reset_FSI ( LowBattery_inFlight );
    BatteryWarningMode = 0;
  }
}

/**
 * @brief Updates the Battery Management System (BMS): pack resistance, compensated voltage, remaining, warnings.
 *
 * Called every 21 ms ( mw.cpp, BMS_UpdateInterval ); every time constant uses the real time between calls.
 * - R: batteryResistanceUpdate ( ), once per power-up; BATTERY_DEFAULT_R_MOHM ( 100 ) until measured. On the update
 *   R is measured, the Vcomp EMA is re-seeded with it and the voltage debounce timers restart.
 * - vBatComp: Vcomp x cells, Vcomp = 1 s EMA of ( bus + I x R ) / cells, I = 0 while the current is stale ( reads
 *   low, errs early ). Held while the bus reading is stale.
 * - Nothing below runs, and soc_Fused / mAhRemain are held, until the plug-in estimate is ready.
 * - With the count ( batteryCountAvailable ( ) ): remaining = E - mAhDrawn, pulled down ( persistently ) to
 *   curve ( Vcomp ) x capacity once that is <= 25 % and lower, only with a measured R; never rises;
 *   soc_Fused = remaining / capacity. Warning at Vcomp <= 3745 mV/cell or count remaining <= 15 %, critical at
 *   3600 mV/cell or 5 %. The voltage test runs in loaded flight ( armed, mAmpRaw >= 1500 mA ); with R unknown only
 *   while the count remaining is also <= 40 %, and an alarm it alone raises is provisional: re-levelled to what the
 *   count supports on disarm and when R is measured; the measured-R voltage test then starts its own 1.5 s debounce.
 *   Count and measured-R alarms latch until power-off.
 * - Without the count ( voltage only ): soc_Fused = curve fraction of the raw cell voltage, + 650 mV hover sag while
 *   armed ( no rise while armed ), mAhRemain = that fraction x capacity. Warning at the raw cell voltage <= 3100 mV,
 *   critical at <= 3000 mV ( fixed loaded thresholds, not the stored config ), armed only.
 * - Outside its window a voltage timer stays at 0. Every condition must hold 1.5 s.
 *
 * @param now Current time in microseconds ( currentTime ).
 * @param vbatTs Unused ( kept for the signature ).
 * @param ibatTs Unused ( kept for the signature ).
 * @param armed Boolean indicating whether the system is armed.
 * @param throttle_us Unused: the throttle-sag model was removed.
 */
void BMS_Update ( uint32_t now, uint32_t vbatTs, uint32_t ibatTs, bool armed, int throttle_us ) {
  ( void ) vbatTs;
  ( void ) ibatTs;
  ( void ) throttle_us;

  static bool timeInit   = false;
  static uint32_t lastUs = 0;
  const uint32_t dtUs    = timeInit ? ( now - lastUs ) : 0U;    // wrap-safe ( us )
  timeInit               = true;
  lastUs                 = now;

  const bool voltStale = ( batterySensorStale & BATTERY_STALE_VOLTAGE ) != 0U;
  const bool currStale = ( batterySensorStale & BATTERY_STALE_CURRENT ) != 0U;
  const bool present   = ( batteryState != BATTERY_NOT_PRESENT );
  const uint8_t cells  = batteryCellCount;    // >= 1

  // 1. Pack resistance ( rest point before the first arming, loaded window across armings )
  const bool rWasKnown = ( batteryResistance_mOhm != 0U );
  if ( present ) {
    batteryResistanceUpdate ( dtUs, armed, voltStale || currStale );
  }
  const bool rKnown        = ( batteryResistance_mOhm != 0U );
  const bool rJustMeasured = rKnown && ! rWasKnown;    // this update measured R
  const uint32_t r_mOhm    = rKnown ? batteryResistance_mOhm : BATTERY_DEFAULT_R_MOHM;

  // 2. Compensated voltage ( I = 0 while the current is stale: no compensation, reads low ). Re-seeded, not smoothed,
  //    on the update R is measured, so the tests use the measured-R value at once.
  if ( ! voltStale ) {
    const uint16_t i_mA = currStale ? static_cast< uint16_t > ( 0U ) : mAmpRaw;
    const float raw     = vCompCellFrom ( vBat_mV, i_mA, r_mOhm, cells );
    if ( vCompCellInit && ! rJustMeasured ) {
      const float dtF = static_cast< float > ( dtUs );
      vCompCell_mV += ( dtF / ( VCOMP_EMA_TAU_US + dtF ) ) * ( raw - vCompCell_mV );    // EMA, tau 1 s: alpha = dt / ( tau + dt )
    } else {
      vCompCell_mV  = raw;
      vCompCellInit = true;
    }
    vBatComp = roundToU16 ( vCompCell_mV * static_cast< float > ( cells ) );    // mV, pack
  }
  const bool vCompValid = vCompCellInit && ! voltStale;

  // Before the plug-in estimate: hold soc_Fused and mAhRemain, no warning
  if ( ! present || ! batteryEstimateReady ) {
    warnVoltUs  = 0;
    critVoltUs  = 0;
    warnCountUs = 0;
    critCountUs = 0;
    updateBatteryState ( false, false );
    return;
  }

  const uint16_t capacity = batteryCapacity_mAh;
  const bool countMode    = batteryCountAvailable ( );

  if ( countMode ) {
    const bool loaded = armed && ( mAmpRaw >= BATTERY_LOADED_MIN_MA );    // loaded flight

    // 3. Remaining ( count-led ) and SoC. The voltage pull-down needs a measured R: the default R can read ~200 mV
    //    per cell low on a high-R pack, and a pull-down is permanent.
    const uint16_t countRemain = ( EstBatteryCapacity > mAhDrawn ) ? static_cast< uint16_t > ( EstBatteryCapacity - mAhDrawn ) : 0U;

    uint16_t reported   = ( countRemain > mAhPullDown ) ? static_cast< uint16_t > ( countRemain - mAhPullDown ) : 0U;
    uint16_t voltRemain = 0;
    if ( rKnown && loaded && vCompValid && voltageFloorRemain ( vCompCell_mV, capacity, &voltRemain ) && voltRemain < reported ) {
      mAhPullDown = static_cast< uint16_t > ( countRemain - voltRemain );    // countRemain >= reported > voltRemain
      reported    = voltRemain;
    }
    if ( reportInit && reported > reportPrev ) {
      reported = reportPrev;    // never rises in this power-up
    }
    reportPrev = reported;
    reportInit = true;

    mAhRemain = reported;
    soc_Fused = ( capacity > 0U ) ? constrainf ( static_cast< float > ( reported ) * 100.0f / static_cast< float > ( capacity ), 0.0f, 100.0f ) : 0.0f;

    // 4. Conditions. Voltage: in loaded flight, and with R unknown ( default R ) only at count remaining <= 40 %;
    //    timer 0 outside that, held while the bus is stale, restarted when R is measured.
    //    Count: E - drawn ( not the pulled-down value ), always.
    const bool warnCountNow = countAtOrBelowPct ( countRemain, capacity, COUNT_WARN_PCT );
    const bool critCountNow = countAtOrBelowPct ( countRemain, capacity, COUNT_CRIT_PCT );
    const bool voltActive   = loaded && ( rKnown || countAtOrBelowPct ( countRemain, capacity, DEFAULT_R_MAX_COUNT_PCT ) );
    if ( ! voltActive || rJustMeasured ) {
      warnVoltUs = 0;
      critVoltUs = 0;
    } else if ( vCompValid ) {
      warnVoltUs = debounceStep ( warnVoltUs, vCompCell_mV <= VCOMP_WARN_CELL_MV, dtUs );
      critVoltUs = debounceStep ( critVoltUs, vCompCell_mV <= VCOMP_CRIT_CELL_MV, dtUs );
    }
    warnCountUs = debounceStep ( warnCountUs, warnCountNow, dtUs );
    critCountUs = debounceStep ( critCountUs, critCountNow, dtUs );

    // 5. Re-level a provisional alarm ( raised by the default-R voltage test alone ).
    if ( alarmProvisional ) {
      if ( rJustMeasured || ! armed ) {
        // R measured, or disarmed: the level the count supports ( count-supported or OK, so no longer provisional ).
        // With R measured, the measured-R voltage test ( timers zeroed above ) raises a latched alarm through the
        // normal 1.5 s debounce if its condition holds.
        batteryApplyLevel ( batterySupportedLevel ( warnCountNow, critCountNow ) );
        alarmProvisional = false;
      }
    }
  } else {
    // Voltage only ( no current sensing ): the raw bus voltage per cell
    const uint32_t rawCell_mV = static_cast< uint32_t > ( vBat_mV ) / cells;    // mV per cell

    // 3. SoC and remaining from the curve; held while the bus is stale. Armed, the raw voltage is a loaded one: add
    //    the typical hover sag before reading the resting curve.
    if ( ! voltStale ) {
      uint32_t socCell_mV = armed ? ( rawCell_mV + BATTERY_HOVER_SAG_CELL_MV ) : rawCell_mV;
      if ( socCell_mV > 0xFFFFU ) {
        socCell_mV = 0xFFFFU;
      }
      float soc = lipoCellRestFraction ( static_cast< uint16_t > ( socCell_mV ) ) * 100.0f;    // %
      if ( armed && socPrevInit && soc > socPrev ) {
        soc = socPrev;    // no rise while armed
      }
      socPrev     = soc;
      socPrevInit = true;
      soc_Fused   = soc;
      mAhRemain   = static_cast< uint16_t > ( soc * 0.01f * static_cast< float > ( capacity ) );    // mAh, floored; 0 when capacity is 0
    }

    // 4. Conditions: armed only ( the raw voltage is a loaded voltage in flight ), timer held while the bus is stale
    if ( ! armed ) {
      warnVoltUs = 0;
      critVoltUs = 0;
    } else if ( ! voltStale ) {
      warnVoltUs = debounceStep ( warnVoltUs, rawCell_mV <= FALLBACK_WARN_CELL_MV, dtUs );
      critVoltUs = debounceStep ( critVoltUs, rawCell_mV <= FALLBACK_CRIT_CELL_MV, dtUs );
    }
    warnCountUs = 0;
    critCountUs = 0;
  }

  const bool warnLow = ( warnVoltUs >= BATTERY_DEBOUNCE_US ) || ( warnCountUs >= BATTERY_DEBOUNCE_US );
  const bool critLow = ( critVoltUs >= BATTERY_DEBOUNCE_US ) || ( critCountUs >= BATTERY_DEBOUNCE_US );

  const batteryState_e prevState = batteryState;
  updateBatteryState ( warnLow, critLow );

  // A step ( always down: the state machine never steps up ) is provisional when taken in count mode with R unknown
  // and not caused by the count condition of the level stepped to ( the default-R voltage test alone ); a
  // count-caused or measured-R step latches.
  // A standing provisional level becomes confirmed as soon as the count supports it, also without a step ( the
  // state is already there ): otherwise a real count critical behind a provisional one would never start the
  // auto-land if R is never measured.
  const bool countSupports = ( batteryState == BATTERY_CRITICAL ) ? ( critCountUs >= BATTERY_DEBOUNCE_US ) : ( warnCountUs >= BATTERY_DEBOUNCE_US );
  if ( batteryState != prevState ) {
    alarmProvisional = countMode && ! rKnown && ! countSupports;
  } else if ( alarmProvisional && countSupports ) {
    alarmProvisional = false;
  }
}
