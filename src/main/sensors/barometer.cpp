/*
 * This file is part of Cleanflight and Magis.
 *
 * Cleanflight and Magis are free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight and Magis are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "drivers/light_led.h"
#include "drivers/gpio.h"

#include "common/maths.h"

#include "drivers/barometer.h"

#include "drivers/system.h"
#include "config/config.h"
#include "config/runtime_config.h"

#include "sensors/barometer.h"
#include "io/rc_controls.h"    // rcCommand [ THROTTLE ] for the compensation below

#define UPDATE_FREQUENCY_10HZ ( 1000 * 101.5 )

baro_t baro;                  // barometer access functions
uint16_t calibratingB = 0;    // baro calibration = get new ground pressure value
float baroPressure    = 0;
float baroTemperature = 0;
float BaroAlt         = 0;

static uint32_t pressureSum   = 0;
uint32_t temperatureSum       = 0;
static uint32_t pressureCount = 0;
uint32_t temperatureCount     = 0;

uint32_t T1 = 0;
uint32_t P1 = 0;

static uint32_t _last_update;
static int32_t avgTemp  = 0;
static int32_t avgPress = 0;

int16_t baroState = 0;

#ifdef BARO

static int32_t baroGroundAltitude = 0;
float baroGroundPressure          = 0;
float baroGroundTemperature       = 0;
static uint32_t baroPressureSum   = 0;

static bool _updated = false;

static barometerConfig_t *barometerConfig;

void useBarometerConfig ( barometerConfig_t *barometerConfigToUse ) {
  barometerConfig = barometerConfigToUse;
}

bool isBaroCalibrationComplete ( void ) {
  return calibratingB == 0;
}

void baroSetCalibrationCycles ( uint16_t calibrationCyclesRequired ) {
  calibratingB = calibrationCyclesRequired;
}

static bool baroReady = false;

  #define PRESSURE_SAMPLES_MEDIAN 3

  #define PRESSURE_SAMPLE_COUNT   ( barometerConfig->baro_sample_count - 1 )

typedef enum {
  BAROMETER_NEEDS_SAMPLES = 0,
  BAROMETER_NEEDS_CALCULATION,
  BAROMETER_NEEDS_PROCESSING
} barometerState_e;

bool isBaroReady ( void ) {
  return baroReady;
}

void icp10111BaroInit ( void ) {
  baro.measurment_start ( NORMAL );
}

void baroInit ( void ) {

  #if defined( PRIMUSX2 ) || defined( PRIMUS_X2_v1 ) || defined( PRIMUS_V5 )

  icp10111BaroInit ( );

  #else

  #endif
}

void apmBaroUpdate ( uint32_t currentTime ) {
  static uint32_t baroDeadline  = 0;
  static barometerState_e state = BAROMETER_NEEDS_SAMPLES;

  if ( ( int32_t ) ( currentTime - baroDeadline ) < 0 )
    return;

  baroDeadline = 0;
  switch ( state ) {
    case BAROMETER_NEEDS_SAMPLES:
      T1 = baro.get_ut ( );

      if ( T1 != 0 ) {
        temperatureSum += T1;

        temperatureCount++;

        if ( temperatureCount == 32 ) {

          // we have summed 32 values. This only happens
          // when we stop reading the barometer for a long time
          // (more than 1.2 seconds)
          temperatureSum >>= 1;
          temperatureCount = 16;
        }
      }

      baroState++;

      baro.start_up ( );
      state = BAROMETER_NEEDS_CALCULATION;
      baroDeadline += baro.up_delay;
      break;

    case BAROMETER_NEEDS_CALCULATION:
      P1 = baro.get_up ( );

      if ( P1 != 0 ) {
        pressureSum += P1;

        pressureCount++;

        if ( pressureCount == 128 ) {

          // we have summed 32 values. This only happens
          // when we stop reading the barometer for a long time
          // (more than 1.2 seconds)
          pressureSum >>= 1;
          pressureCount = 64;
        }
      }

      _updated = true;
      baroState++;

      if ( baroState == 5 ) {

        //  state = BAROMETER_NEEDS_PROCESSING;

        baro.start_ut ( );
        baroState = 0;
        state     = BAROMETER_NEEDS_SAMPLES;

      }

      else {
        baro.start_up ( );
      }

      baroDeadline += baro.ut_delay;

      break;

    case BAROMETER_NEEDS_PROCESSING:

      baro.start_ut ( );
      baroState = 0;
      state     = BAROMETER_NEEDS_SAMPLES;

      break;
  }
  baroDeadline += micros ( );    // make sure deadline is set after calling baro callbacks
}

void apmBaroRead ( uint32_t currentTime ) {

  static uint32_t previousTime;
  uint32_t dTime;
  dTime = currentTime - previousTime;

  if ( dTime < UPDATE_FREQUENCY_10HZ )
    return;

  previousTime = currentTime;

  bool updated = _updated;
  int T, P;

  if ( updated ) {
    T = temperatureSum / temperatureCount;
    P = pressureSum / pressureCount;

    if ( T == 0 ) {
      T = avgTemp;

    }

    else {
      avgTemp = T;
    }

    if ( P == 0 ) {
      P = avgPress;
    }

    else

    {

      avgPress = P;
    }

    _updated = false;

    pressureSum      = 0;
    temperatureSum   = 0;
    pressureCount    = 0;
    temperatureCount = 0;
  }

  baro.calculate ( &baroPressure, &baroTemperature, P, T );

  if ( updated ) {
    _last_update = millis ( );
  }
}

void icp10111BaroUpdate ( uint32_t currentTime ) {

  uint32_t ctime = currentTime / 1000;

  // Ensure a measurement is always running
  static bool measurementStarted = false;
  if ( ! measurementStarted ) {
    baro.measurment_start ( NORMAL );
    measurementStarted = true;
  }

  if ( baro.read ( ctime, &baroPressure, &baroTemperature ) ) {
    _last_update = millis ( );
    // Start the next measurement immediately after successful read
    baro.measurment_start ( NORMAL );
  }
}

void baroUpdate ( uint32_t currentTime ) {

  #if defined( PRIMUSX2 ) || defined( PRIMUS_X2_v1 ) || defined( PRIMUS_V5 )

  icp10111BaroUpdate ( currentTime );

  #else

  apmBaroUpdate ( currentTime );
  apmBaroRead ( currentTime );

  #endif
}

float getBaroPressure ( )

{
  return baroPressure;
}

float getBaroTemperature ( )
{
  return baroTemperature;
}

uint32_t getBaroLastUpdate ( ) {

  return _last_update;
}

float apmBaroCalculateAltitude ( void ) {

  // calculates height from ground via baro readings
  // see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
  float scaling = ( float ) baroGroundPressure / ( float ) baroPressure;
  float temp    = ( baroGroundTemperature + 27315 ) / 100;

  BaroAlt = logf ( scaling ) * temp * 29.271267f * 100;

  return BaroAlt;
}

/*
 * Pressure to altitude ( cm ), ISA standard atmosphere at a fixed 288.15 K, as in
 * INAV. No temperature term on purpose: the ICP-10111 reports self-heating die
 * temperature, not air temperature, so using it made the altitude scale vary
 * flight to flight. A fixed scale is repeatable; thermal drift is corrected
 * separately in baroCompensationPa ( ).
 */
static float pressureToAltitude ( const float pressure ) {
  return ( 1.0f - powf ( pressure / 101325.0f, 0.190295f ) ) * 4433000.0f;
}

/*
 * powf ( ) is not cheap, and the ground pressure only moves when the reference
 * is recalibrated - so convert it once and reuse until it actually changes.
 */
static float baroGroundAltitudeCm      = 0.0f;
static float baroGroundPressureCached  = 0.0f;

static float getBaroGroundAltitude ( void ) {

  if ( baroGroundPressure != baroGroundPressureCached ) {
    baroGroundPressureCached = baroGroundPressure;
    baroGroundAltitudeCm     = pressureToAltitude ( baroGroundPressure );
  }

  return baroGroundAltitudeCm;
}

/*
 * Zero upkeep. The boot-time ground reference goes stale as the die self-heats
 * before takeoff ( ~13 Pa / ~110 cm measured ). While disarmed the craft is on
 * the ground, so the zero tracks the reading ( IIR, to reject pressure noise )
 * and freezes on arm. Kept as a separate offset, not by rewriting
 * baroGroundPressure, as INAV does. In-flight drift is baroCompensationPa ( ).
 */
#define BARO_ZERO_TRACK_ALPHA 0.05f    // ~20-sample average at the baro rate

static float baroAltitudeOffset = 0.0f;
static bool baroOffsetSeeded    = false;

static void baroUpdateZero ( const float rawAltitude ) {

  // Frozen while armed - this is the datum the flight is measured against.
  if ( ARMING_FLAG ( ARMED ) ) {
    return;
  }

  if ( ! baroOffsetSeeded ) {
    baroAltitudeOffset = rawAltitude;
    baroOffsetSeeded   = true;
    return;
  }

  baroAltitudeOffset += ( rawAltitude - baroAltitudeOffset ) * BARO_ZERO_TRACK_ALPHA;
}

float getBaroZeroOffset ( void ) {
  return baroAltitudeOffset;
}

/*
 * Throttle and temperature compensation. The sensed pressure falls with:
 *   throttle : 8.6 Pa per 1000 counts ( FC sits in the rotor inflow )
 *   temp     : ~2.3 Pa per degC of die temperature ( measured -2.17 to -2.42 on
 *              PRIMUS_V5 ). Set to 2.1, below the mean, so any error sinks
 *              gently rather than climbs.
 * Both are relative to the arm instant ( zero on the first armed sample ) and
 * clamped so a bad coefficient or reading cannot move the datum more than ~2 m.
 * A 4 min cold-start hover used ~70 % of the 25 Pa clamp: if longer cold flights
 * sink late, raise the clamp to 30-35 Pa rather than lowering the coefficient.
 * Measured on one airframe; re-check on a new frame. Detail and data:
 * docs/fw-development-reference/active-development/altitude-hold/TESTING.md
 */
#define BARO_COMP_THROTTLE_PA_PER_COUNT 0.0086f
#define BARO_COMP_TEMP_PA_PER_DEGC      2.10f
#define BARO_COMP_LIMIT_PA              25.0f

static float baroCompThrottleRef = 0.0f;
static float baroCompTempRef     = 0.0f;
static bool baroCompLatched      = false;

static float baroCompensationPa ( void ) {

  if ( ! ARMING_FLAG ( ARMED ) ) {
    baroCompLatched = false;
    return 0.0f;
  }

  const float thr  = ( float ) rcCommand [ THROTTLE ];
  const float temp = getBaroTemperature ( );

  if ( ! baroCompLatched ) {
    baroCompThrottleRef = thr;
    baroCompTempRef     = temp;
    baroCompLatched     = true;
    return 0.0f;
  }

  // Sensed pressure falls with both, so add back what they took.
  const float correction = BARO_COMP_THROTTLE_PA_PER_COUNT * ( thr - baroCompThrottleRef )
                           + BARO_COMP_TEMP_PA_PER_DEGC * ( temp - baroCompTempRef );

  return constrainf ( correction, -BARO_COMP_LIMIT_PA, BARO_COMP_LIMIT_PA );
}

float icp10111BaroCalculateAltitude ( void ) {

  if ( baroPressure <= 0.0f || baroGroundPressure <= 0.0f ) {
    return 0.0f;    // Return 0 altitude if readings are invalid
  }

  // Height above the ground reference: both sides converted through the same
  // fixed curve, so everything common to them cancels.
  const float rawAltitude = pressureToAltitude ( baroPressure + baroCompensationPa ( ) ) - getBaroGroundAltitude ( );

  baroUpdateZero ( rawAltitude );

  BaroAlt = rawAltitude - baroAltitudeOffset;

  return BaroAlt;
}

float baroCalculateAltitude ( void ) {

  #if defined( PRIMUSX2 ) || defined( PRIMUS_X2_v1 ) || defined( PRIMUS_V5 )

  return icp10111BaroCalculateAltitude ( );

  #else

  return apmBaroCalculateAltitude ( );

  #endif
}

void apmBaroCalibrationRead ( ) {
  bool updated = _updated;
  uint32_t T, P;

  if ( updated ) {
    T = temperatureSum / temperatureCount;
    P = pressureSum / pressureCount;

    _updated = false;

    pressureSum      = 0;
    temperatureSum   = 0;
    pressureCount    = 0;
    temperatureCount = 0;
  }

  baro.calculate ( &baroPressure, &baroTemperature, P, T );

  if ( updated ) {
    _last_update = millis ( );
  }
}

void performBaroCalibrationCycle ( void ) {
  baroGroundPressure -= baroGroundPressure / 8;
  baroGroundPressure += baroPressureSum / PRESSURE_SAMPLE_COUNT;
  baroGroundAltitude = ( 1.0f - powf ( ( baroGroundPressure / 8 ) / 101325.0f, 0.190295f ) ) * 4433000.0f;

  calibratingB--;
}

void apmBaroCalibrate ( void ) {

  baroGroundPressure    = 0;
  baroGroundTemperature = 0;

  int i = 0, j = 0;
  while ( baroGroundPressure == 0 ) {    // Dump old values
    for ( i = 0; i < 35; i++ ) {
      apmBaroUpdate ( micros ( ) );
      delay ( 10 );
    }
    apmBaroCalibrationRead ( );
    baroGroundPressure    = getBaroPressure ( );
    baroGroundTemperature = getBaroTemperature ( );
  }

  baroGroundPressure    = 0;
  baroGroundTemperature = 0;
  // Start gathering new values
  for ( j = 0; j < 15; j++ ) {
    for ( i = 0; i < 5; i++ ) {
      apmBaroUpdate ( micros ( ) );
      delay ( 10 );
    }

    apmBaroCalibrationRead ( );
    baroGroundPressure += getBaroPressure ( );
    baroGroundTemperature += getBaroTemperature ( );
  }
  baroGroundPressure /= 15;
  baroGroundTemperature /= 15;
}

void icp10111BaroCalibrate ( void ) {

  baroGroundPressure    = 0;
  baroGroundTemperature = 0;

  // Start sensor in very accurate mode
  // baro.measurment_start ( VERY_ACCURATE );

  int i, j;
  int max_attempts = 100;    // Prevent infinite loop
  int attempts     = 0;

  // Dump old values - ensure sensor stabilizes
  while ( baroGroundPressure == 0 && attempts < max_attempts ) {
    for ( i = 0; i < 35; i++ ) {
      icp10111BaroUpdate ( micros ( ) );
      delay ( 10 );
    }
    baroGroundPressure    = getBaroPressure ( );
    baroGroundTemperature = getBaroTemperature ( );
    attempts++;
  }

  // If no valid readings were obtained, return (fail-safe)
  if ( baroGroundPressure == 0 ) {
    return;
  }

  // Reset values before averaging new readings
  baroGroundPressure    = 0.0f;
  baroGroundTemperature = 0.0f;

  // Collect & average new sensor data
  for ( j = 0; j < 15; j++ ) {
    for ( i = 0; i < 10; i++ ) {
      icp10111BaroUpdate ( micros ( ) );
      delay ( 10 );
    }
    baroGroundPressure += getBaroPressure ( );
    baroGroundTemperature += getBaroTemperature ( );
  }

  // Average the readings
  baroGroundPressure /= 15.0f;
  baroGroundTemperature /= 15.0f;
}

void baroCalibrate ( void ) {

  #if defined( PRIMUSX2 ) || defined( PRIMUS_X2_v1 ) || defined( PRIMUS_V5 )

  icp10111BaroCalibrate ( );

  #else

  apmBaroCalibrate ( );

  #endif
}
// Add this function near the existing baroCalibrate() function
void baroResetGroundLevel ( void ) {
  // Quick ground level reset (single reading)
  baroGroundPressure    = getBaroPressure ( );
  baroGroundTemperature = getBaroTemperature ( );
}
// Note if you want to use the function below add #define BARO before #include "sensors/barometer.h"
//  Checks if barometer has excessive drift during startup, returns true if drift exceeds threshold
bool checkBaroDriftDuringStartup ( void ) {
  static float baroReferenceAltitude = 0.0f;     // Reference altitude used as baseline for drift calculations
  const uint8_t numReadings          = 15;       // Number of readings to collect for each drift calculation
  const float DRIFT_THRESHOLD        = 20.0f;    // Maximum allowed drift in centimeters (20 cm threshold)

  static float readings [ 15 ];      // Array to store barometer readings for analysis
  static uint8_t index       = 0;    // Current position in the readings array
  static uint8_t initialized = 0;    // Flag to track if reference altitude is established

  float currentAltitude = icp10111BaroCalculateAltitude ( );    // Get current barometer altitude reading
  static bool firstCall = true;                                 // Flag to identify first execution of this function

  if ( firstCall ) {                            // Special handling for first function call
    baroReferenceAltitude = currentAltitude;    // Set reference to first reading
    firstCall             = false;              // Clear first-call flag to prevent re-entry
    return false;                               // No drift detected during first call
  }

  if ( ! initialized ) {                       // Initialization phase - collect readings for reference
    readings [ index++ ] = currentAltitude;    // Store reading and increment index
    if ( index >= numReadings ) {              // Check if we've collected enough readings
      float sum = 0.0f;                        // Calculate sum of all collected readings
      for ( uint8_t i = 0; i < numReadings; ++i ) {
        sum += readings [ i ];
      }
      baroReferenceAltitude = sum / numReadings;    // Set reference to average of readings
      initialized           = 1;                    // Mark initialization as complete
      index                 = 0;                    // Reset index for drift analysis
    }
    return false;    // No drift during initialization
  }

  readings [ index++ ] = currentAltitude;    // Store new reading in array for monitoring
  if ( index >= numReadings ) {              // Check if we have enough readings for analysis
    float maxDrift = 0.0f;                   // Track maximum deviation from reference
    for ( uint8_t i = 0; i < numReadings; ++i ) {
      float drift = fabsf ( readings [ i ] - baroReferenceAltitude );    // Calculate deviation
      if ( drift > maxDrift ) {
        maxDrift = drift;    // Update maximum if this reading has larger deviation
      }
    }

    index = 0;                                // Reset index for next batch
    return ( maxDrift > DRIFT_THRESHOLD );    // Return true if drift exceeds threshold
  }

  return false;    // Not enough readings collected yet
}
#endif /* BARO */
