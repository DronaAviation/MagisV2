/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\flight\altitudehold.cpp                                    #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 24th Sep 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  24-09-2026	AJ	VL53L1X runs the shared laser fusion; guarded return.        #
 #  24-09-2026	AJ	Laser fusion reads the sensor via per-sensor accessors.      #
*******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "drivers/gpio.h"
#include "drivers/ranging_vl53l0x.h"
#include "drivers/ranging_vl53l1x.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/escservo.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "config/runtime_config.h"

#include "command/command.h"
#include "flight/acrobats.h"
#include "altitudehold.h"

// Kalman Filter Structure
typedef struct {
  float Q;    // Process noise covariance
  float R;    // Measurement noise covariance
  float X;    // Estimated value
  float P;    // Estimation error covariance
  float K;    // Kalman gain
} KalmanFilter;

// Global Kalman filter instances
static KalmanFilter altHoldFilter;
static KalmanFilter velHoldFilter;

// Distance below max_altitude where limitAltitude ( ) stops accepting climb.
// It has no velocity term, so this is the stopping distance: keep it at least
// the overshoot at the fastest climb flown, or the ceiling gets breached.
#define ALT_CEILING_MARGIN_CM 25

int16_t max_altitude              = -1;
int16_t althold_throttle          = 0;
int32_t errorVelocityI            = 0;
int32_t altHoldThrottleAdjustment = 0;
int32_t AltHold;
int32_t vario           = 0;    // variometer in cm/s
int32_t setVelocity     = 0;
int32_t calculatedError = 10;
int32_t VelocityZ;
int32_t baroAlt_offset_print = 0;
int32_t PositionZ;
uint32_t baro_last_update;
bool AltRstRequired = 1;

// int32_t altest;

static barometerConfig_t *barometerConfig;
static pidProfile_t *pidProfile;
static rcControlsConfig_t *rcControlsConfig;
static escAndServoConfig_t *escAndServoConfig;
barometerConfig_t *barometerConfig_tmp;

int QUEUE_MAX_LENGTH = 15;

static float buff [ 15 ];
static int16_t head      = 0;
static int16_t rear      = -1;
static int16_t itemCount = 0;
// The VL53L0X and the VL53L1X both sit on I2C1 at 0x29, so only one can be fitted; with both
// defined both laser paths would also write the same estimator state.
#if defined( LASER_TOF ) && defined( LASER_TOF_L1x )
  #error "LASER_TOF ( VL53L0X ) and LASER_TOF_L1x ( VL53L1X ) are both defined: define only the laser that is fitted"
#endif
#ifdef LASER_ALT
// Complementary-filter time constant, s. One value for the laser and the baro path, so a
// source change does not also change the filter gains ( tof-althold-fusion task 3 ).
  #define ALT_EST_TAU_S 1.5f
// Laser samples taken above this tilt are rejected ( slant range, e.g. during a flip ).
  #define ALT_TOF_MAX_TILT_DECIDEG 250
// Laser <-> baro handover ( tof-althold-fusion task 5 ): the laser hands over to the baro above
// ALT_TOF_HANDOVER_UP_CM and takes back below ALT_TOF_HANDOVER_DOWN_CM; the band stops it
// chattering. The edges follow the sensor's reach: per-sensor block below.
// No good laser sample for ALT_TOF_DROPOUT_MS ( dropout, tilt, sensor silent ) hands over to the
// baro. Shorter out-of-range or tilt gaps coast on the accelerometer; a silent sensor keeps its last
// reading, which keeps correcting until the timeout. Set from the sample period: per-sensor block.
// ALT_TOF_RETURN_SAMPLES consecutive good samples before the laser takes back over: below the lower
// edge after a climb above the band, below the upper edge after a dropout or tilt handover ( a hover
// between the edges must not stay on the baro for good ). A sample count: per-sensor block.
// The laser reading is advanced by VelocityZ times ALT_TOF_IIR_LAG_S ( the lag of the driver's range
// filter ) for the baro offset and the return shift ( the object test advances the estimate
// instead ), so speed is not baked into either. Per-sensor block.
// Object under the craft ( task 6 ). A raw laser sample this far from the estimate is a change of
// surface ( hand, box, table edge ), not motion: the estimate holds on the baro for the hold-off,
// then re-bases to the new surface once the laser is steady ( ALT_TOF_STEADY_SAMPLES, per sensor ),
// and the setpoint flies back to the old clearance on the goal profile. Nearer than half the step
// for 3 samples cancels it.
  #define ALT_TOF_STEP_CM          30.0f
  #define ALT_TOF_STEP_HOLD_MS     2500
  #define ALT_TOF_STEADY_CM        10.0f    // raw samples within this of each other count as steady
// Window test ( task 14 ). The wide VL53L0X cone turns an edge into a ramp of ~0.5-0.8 s, which
// the estimate absorbs sample by sample ( a slowly slid box, or any edge crossed while moving ).
// So compare the raw laser's change over the last 0.5 s with the change of _position_base_z,
// the accelerometer-integrated position. The laser's position ( k1 ) correction does not move it;
// its velocity ( k2 ) correction still reaches it, weakly, so the window reads a little less than
// the true surface change.
// A mismatch above ALT_TOF_STEP_CM is a change of surface; above ALT_TOF_SUSPECT_CM the
// baro-offset average pauses so an edge cannot leak into it. The sample ring holds
// ALT_TOF_RING_LEN samples ( per sensor ).
  #define ALT_TOF_WINDOW_MS        500
  #define ALT_TOF_WINDOW_MIN_MS    400     // shortest span the test uses after a reset
  #define ALT_TOF_SUSPECT_CM       15.0f
// Once the window passes ALT_TOF_SUSPECT_CM the reference is frozen at the start of the edge, the
// laser correction pauses ( coast on the accelerometer ) and the mismatch keeps adding up, so an
// edge spread over more than the window still reaches ALT_TOF_STEP_CM. If it has not after this
// long, it is a slope: the laser correction resumes and the craft follows it.
  #define ALT_TOF_SUSPECT_MAX_MS   1000
// At that timeout a mismatch above this is an edge ( hold-off, or the baro above the band );
// at or below it, a slope that is followed. Keeps margin under the 30 cm per-sample residual.
  #define ALT_TOF_SUSPECT_ESCALATE_CM 20.0f
// No window-based suspicion for this long after becoming airborne: lift-off reads up to ~22 cm.
  #define ALT_TOF_AIRBORNE_GRACE_MS 1000
// Time constant of the baro-minus-laser offset average kept while on the laser, s.
  #define ALT_BARO_OFFSET_TAU_S    2.0f

// Per-sensor constants and driver accessors ( vl53l1x-althold-parity task 5 ). The laser fusion in
// checkReading ( ) reads the sensor only through these, so one body can serve either laser:
//   tofNew ( )         a new result is waiting              tofClearNew ( )  mark it consumed
//   tofOutOfRange ( )  no valid reading ( driver's test )   tofReseed ( )    restart the range filter
//   tofFiltCm ( )      filtered range, cm, before tilt      tofRawCm ( )     unfiltered range, cm
// Constants that are sample counts, or follow the sample period or the driver filter, live here.
  #if defined( LASER_TOF )
// VL53L0X: a sample every ~33 ms. NewSensorRange is the driver's LASER_LPS 0.1 IIR of the range,
// RangingMeasurementData.RangeMilliMeter the unfiltered sample.
    #define ALT_TOF_HANDOVER_UP_CM   160.0f    // it dropped out at ~172 cm on the test floor
    #define ALT_TOF_HANDOVER_DOWN_CM 140.0f
    #define ALT_TOF_DROPOUT_MS       120       // 3 missed samples at 33 ms + the 10 ms estimator tick ( 100 tripped on 2 )
    #define ALT_TOF_RETURN_SAMPLES   3
    #define ALT_TOF_IIR_LAG_S        0.3f      // LASER_LPS 0.1 at 33 ms for a ramp: 0.033 * 0.9 / 0.1 s
    #define ALT_TOF_STEADY_SAMPLES   15        // 0.5 s at 33 ms
    #define ALT_TOF_RING_LEN         24        // > 0.66 s of samples at 33 ms
    #define ALT_TOF_OFFSET_DT_MAX_S  0.05f     // longest step one baro-offset update weights, s
    #define tofNew()        isTofDataNew ( )
    #define tofClearNew()   ( isTofDataNewflag = false )
    #define tofOutOfRange() isOutofRange ( )
    #define tofReseed()     tofRequestReseed ( )
    #define tofFiltCm()     ( ( float ) NewSensorRange / 10.0f )
    #define tofRawCm()      ( ( float ) RangingMeasurementData.RangeMilliMeter / 10.0f )
  #elif defined( LASER_TOF_L1x )
// VL53L1X: a result every L1X_SAMPLE_PERIOD_MS ( 50 ms ) and no driver filter, so NewSensorRange_L1
// ( last valid range, mm ) is both the filtered and the raw value, the lag is 0 and reseed does
// nothing. isOutofRange_L1 ( ) is also true on a latched error, a result older than 3 periods
// + 10 ms, or a range under the driver's L1X_MIN_VALID_MM ( a covered window reads 0-10 mm as
// valid ). Comments in the shared body that mention the driver IIR apply to the L0X only.
    #define ALT_TOF_HANDOVER_UP_CM   160.0f    // Medium mode on the flying floor: 100 % valid to 180 cm ( log-5 )
    #define ALT_TOF_HANDOVER_DOWN_CM 140.0f
    // 2 missed samples at the measured 51-53 ms period, plus ~14 ms poll quantisation and the 10 ms estimator
    // tick ( the L0X 120 ms is 3 x 33 + 21 )
    #define ALT_TOF_DROPOUT_MS       ( 3 * L1X_SAMPLE_PERIOD_MS + 35 )                              // 185 ms
    #define ALT_TOF_RETURN_SAMPLES   3
    #define ALT_TOF_IIR_LAG_S        0.0f
    #define ALT_TOF_STEADY_SAMPLES   ( 500 / L1X_SAMPLE_PERIOD_MS )                                 // 10, 0.5 s
    #define ALT_TOF_RING_LEN         ( ( 660 + L1X_SAMPLE_PERIOD_MS - 1 ) / L1X_SAMPLE_PERIOD_MS )  // 14, >= 0.66 s
    #define ALT_TOF_OFFSET_DT_MAX_S  ( 1.5f * ( float ) L1X_SAMPLE_PERIOD_MS * 0.001f )              // 1.5 periods, s ( L0X 0.05 s ~ 1.5 x 33 ms )
// Baro -> laser return guard ( L1x only ). The object tests all need the laser as the source, so on
// the baro nothing checks what the laser sees: ceiling-fan blades read a valid 56 cm for 4 samples
// with the craft at 250 cm ( log-5 ). While armed and not in the pre-take-off ground idle ( in flight
// and landing ), a return sample counts only if the laser is within ALT_TOF_RETURN_AGREE_CM of the
// baro-path estimate _position_z. That estimate
// already follows Baro_Height - baro_offset ( the frozen offset carries the constant +15..22 cm baro
// vs laser offset ), so the difference is the return frame shift itself: the baro drift since the
// handover plus any surface change. Allowance over the 30 cm object step: the baro drift during a
// baro leg and the estimate's residual baro noise ( raw sample sd 13 cm, smoothed by the filter ).
// Way out: when the disagreement ( laser minus estimate ) stays within ALT_TOF_RETURN_STEADY_CM for
// ALT_TOF_STEP_HOLD_MS, it is a new floor ( take-off from a table, baro drift over the bound ), and the
// return is taken with the normal frame shift; any gap restarts that timer.
    #define ALT_TOF_RETURN_BARO_CM   20.0f
    #define ALT_TOF_RETURN_AGREE_CM  ( ALT_TOF_STEP_CM + ALT_TOF_RETURN_BARO_CM )    // 50 cm
    // Steady band of the guard's exit: the baro-path estimate swings 20-40 cm while the craft climbs or
    // descends ( log-12 ), so the 10 cm object band restarted the 2.5 s timer for ~10 s. Fan blades and passing
    // objects still cannot pass: they are intermittent, and every gap restarts the run.
    #define ALT_TOF_RETURN_STEADY_CM 25.0f
    #define tofNew()        isTofDataNew_L1 ( )
    #define tofClearNew()   ( isTofDataNewflag_L1 = false )
    #define tofOutOfRange() isOutofRange_L1 ( )
    #define tofReseed()     ( ( void ) 0 )
    #define tofFiltCm()     ( ( float ) NewSensorRange_L1 / 10.0f )
    #define tofRawCm()      ( ( float ) NewSensorRange_L1 / 10.0f )
static_assert ( ALT_TOF_STEADY_SAMPLES >= 1 && ALT_TOF_STEADY_SAMPLES <= 255, "steadyCount is a saturating uint8_t" );
static_assert ( ALT_TOF_RING_LEN * L1X_SAMPLE_PERIOD_MS > ALT_TOF_WINDOW_MS, "the ring must span the step window" );
static_assert ( ALT_TOF_DROPOUT_MS >= L1X_STALE_MS, "dropout must not be shorter than the driver's stale time" );
  #else
    #error "LASER_ALT needs a laser: define LASER_TOF ( VL53L0X ) or LASER_TOF_L1x ( VL53L1X )"
  #endif
static_assert ( ALT_TOF_RING_LEN > 0 && ALT_TOF_RING_LEN <= 255, "the window ring is indexed with uint8_t" );
float _time_constant_z = ALT_EST_TAU_S;
#else
float _time_constant_z = 2.0f;
#endif
float accZ_tmp;
static float accZ_old = 0.0f;

int16_t first_reads               = 0;
int16_t first_velocity_reads      = 0;
int16_t ctr                       = 0;
static int32_t last_hist_position = 0;

void setAltitude ( float new_altitude );

// Kalman Filter Functions
void kalmanFilterInit ( KalmanFilter *filter, float Q, float R, float initialValue ) {
  filter->Q = Q;
  filter->R = R;
  filter->X = initialValue;
  filter->P = 1.0;    // Initial estimation error covariance
  filter->K = 0.0;    // Initial Kalman gain
}

float kalmanFilterUpdate ( KalmanFilter *filter, float measurement ) {
  // Prediction update
  filter->P = filter->P + filter->Q;

  // Measurement update
  filter->K = filter->P / ( filter->P + filter->R );
  filter->X = filter->X + filter->K * ( measurement - filter->X );
  filter->P = ( 1 - filter->K ) * filter->P;

  return filter->X;
}

float _k1_z;    // gain for vertical position correction
float _k2_z;    // gain for vertical velocity correction
float _k3_z;    // gain for vertical accelerometer offset correction

// general variables
float _position_base_z;          // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float _position_correction_z;    // sum of corrections to _position_base from delayed 1st order samples in cm
float _accel_correction_hbf_z;
float _velocity_z;          // latest velocity estimate (integrated from accelerometer values) in cm/s
float _position_error_z;    // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
float _position_z;          // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float accel_ef_z;

// temp vr

float _time_constant_z1 = 2.0f;

float _k1_z1;    // gain for vertical position correction
float _k2_z1;    // gain for vertical velocity correction
float _k3_z1;    // gain for vertical accelerometer offset correction

int32_t VelocityZ1 = 0;
int32_t EstAlt1    = 0;

uint32_t baro_last_update1;

int16_t first_reads1               = 0;
static int32_t last_hist_position1 = 0;

float _position_base_z1;          // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float _position_correction_z1;    // sum of corrections to _position_base from delayed 1st order samples in cm
float _accel_correction_hbf_z1;
float _velocity_z1;          // latest velocity estimate (integrated from accelerometer values) in cm/s
float _position_error_z1;    // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
float _position_z1;          // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)

float ToF_Height  = 0.0f;
float Baro_Height = 0.0f;
float fused       = 0.0f;
float filtered    = 0.0f;

int32_t altholdDebug          = 0;
int32_t altholdDebug1         = 0;
int32_t altholdDebug2         = 0;
int32_t altholdDebug3         = 0;
int32_t altholdDebug4         = 0;
int32_t altholdDebug5         = 0;
int32_t altholdDebug6         = 0;
int32_t altholdDebug7         = 0;
int32_t altholdDebug8         = 0;
int32_t altholdDebug9         = 0;
int32_t velControlDebug [ 3 ] = { 0 };

void configureAltitudeHold ( pidProfile_t *initialPidProfile, barometerConfig_t *intialBarometerConfig, rcControlsConfig_t *initialRcControlsConfig, escAndServoConfig_t *initialEscAndServoConfig ) {
  pidProfile          = initialPidProfile;
  barometerConfig     = intialBarometerConfig;
  rcControlsConfig    = initialRcControlsConfig;
  barometerConfig_tmp = barometerConfig;
  escAndServoConfig   = initialEscAndServoConfig;

  // Initialize Kalman filters
  kalmanFilterInit ( &altHoldFilter, 0.01, 0.5, 0.0 );    // Lower Q, higher R for altitude
  kalmanFilterInit ( &velHoldFilter, 0.1, 1.0, 0.0 );     // Higher Q, higher R for velocity
}

#if defined( BARO ) || defined( LASER_ALT )

int16_t initialThrottleHold_test;
int16_t debug_e1;

static int16_t initialThrottleHold;
static int32_t EstAlt = 0;    // in cm

  #define BARO_UPDATE_FREQUENCY_40HZ ( 1000 * 25 )
  #define UPDATE_FREQUENCY           ( 1000 * 10 )    // 100Hz

  #define DEGREES_80_IN_DECIDEGREES  800

/*
 * Hover-trim offload. With initialThrottleHold pinned at 1500, the whole hover
 * trim lives in errorVelocityI, clamped at +/- 300 counts, so hover throttle
 * could not go above 1800 and the craft sank as the battery sagged.
 *
 * While settled ( armed, stick centred, setpoint fixed, |VelocityZ| small ) the
 * trim moves into initialThrottleHold one count at a time. Commanded throttle
 * is unchanged: the baseline gains what the integrator loses, and
 * altHoldThrottleAdjustment is corrected here too, since it only updates at
 * 100 Hz. Never offload mid-climb - a transient would stick in the baseline.
 */
  #define ALT_TRIM_OFFLOAD_MS  20    // at most one count this often
  #define ALT_TRIM_OFFLOAD_VEL 10    // cm/s below which the craft counts as settled

/*
 * Setpoint shaping ( ArduPilot / DJI style ). The throttle stick never takes the
 * position loop out of the chain: it sets a climb rate that moves AltHold, and
 * the position loop keeps tracking the moving target with that rate fed
 * forward. The rate is ramped, so climbs and descents start and stop without a
 * step, and centring the stick lets the target coast to a stop instead of
 * snapping it to wherever EstAlt happens to be ( which overshot ).
 *
 * A new AltHold written from outside ( take-off, setAltitude ( ), MSP ) is not
 * jumped to. It becomes a goal, and the target travels there on a profile:
 * ramp up, cruise at the command rate, and brake at ALT_GOAL_DECEL_CMSS just in
 * time to stop on it. Stepping AltHold instead left the P = 1 position loop to
 * close the gap, and its demand shrinks with the gap, so the last 40 cm of a
 * 120 cm take-off crawled ( ~4.3 s total ). Moving the stick cancels the goal.
 *
 * The final velocity setpoint is clamped and slewed as well.
 */
  #define ALT_MAX_CLIMB_CMS    40     // cm/s, full stick up
  #define ALT_MAX_DESCENT_CMS  30     // cm/s, full stick down
  #define ALT_CMD_MAX_CLIMB_CMS   60  // cm/s, cruise up to a commanded goal ( take-off )
  #define ALT_CMD_MAX_DESCENT_CMS 30  // cm/s, cruise down to a commanded goal
  #define ALT_GOAL_DECEL_CMSS  80     // cm/s^2, braking into a goal ( below the ramp, so the ramp can follow it )
  #define ALT_STICK_FULL_TRAVEL 500   // stick counts from centre to end
  #define ALT_STICK_ACCEL_CMSS 100    // cm/s^2, ramp of the profile rate
  #define ALT_VEL_ACCEL_CMSS   150    // cm/s^2, slew of the final velocity setpoint
  #define ALT_TARGET_LEASH_CM  50     // target stops advancing this far from EstAlt

/*
 * Landing keeps its own descent profile, not the stick limits. land ( ) ramps
 * landThrottle 1300 -> 1150 and it is flown as ( landThrottle - 1500 ) / 4, so
 * -50 -> -87 cm/s, as before setpoint shaping. Near the floor the baro drifts
 * low in the craft's own downwash; at the stick's 10-20 cm/s that drift alone
 * met the descent demand, so the craft hovered a few cm up with EstAlt still
 * falling and the touchdown test ( descent stopped ) never fired.
 */
  #define ALT_LAND_MAX_DESCENT_CMS 100    // cm/s, descent clamp while isLanding

static float altRate           = 0.0f;     // ramped profile rate moving the target, cm/s
static float altVelTarget      = 0.0f;     // slewed velocity setpoint, cm/s
static float altTarget         = 0.0f;     // AltHold with the fraction kept, cm
static float altGoal           = 0.0f;     // commanded altitude the target is travelling to, cm
static bool altGoalActive      = false;
static bool altHoldGroundIdle  = false;    // armed on the throttle stick, motors held at idle
static int32_t altPreFlipAltHold = 0;      // AltHold when a flip was sent, flown back to after it, cm

// Drop all setpoint shaping state and pin the target to the current estimate.
static void resetAltSetpoint ( void ) {
  altRate       = 0.0f;
  altVelTarget  = 0.0f;
  altGoalActive = false;
  AltHold       = EstAlt;
  altTarget     = ( float ) EstAlt;
}

static int32_t shapedVelocitySetpoint ( float ctrlDt );

/*
 * Flip ( acrobats.cpp ) keeps ALT_HOLD on: its DEACTIVATE_RC_MODE ( BOXBARO ) is
 * undone on the next RX frame while the app holds AUX3. It drives
 * rcData [ THROTTLE ] itself ( 2000 in ASCEND and HOLD ) and ASCEND waits for
 * VelocityZ >= 100 cm/s before it starts the rotation. Through setpoint shaping
 * full stick is 40 cm/s, so ASCEND timed out after 2.2 s and never flipped
 * ( flip-althold-regression, log-1.txt ). While a flip runs, the controller flies
 * the raw rate path it had before setpoint shaping instead.
 */
  #define ALT_FLIP_MAX_CLIMB_CMS   120    // cm/s, ( 2000 - 1500 ) / 4 = 125, clamped as before shaping
  #define ALT_FLIP_MAX_DESCENT_CMS 100    // cm/s
  #define ALT_FLIP_EXIT_IGNORE_MS  100    // ms, throttle ignored after a flip ( RX refresh <= 20 ms )
  #define ALT_FLIP_RETURN_MIN_CM   20     // cm, pre-flip AltHold below this is treated as on the ground: no return
  #define ALT_FLIP_EXIT_I_HOLD_MS  500    // ms, velocity integrator held after a flip while its climb is braked

static bool flipActive ( void ) {
  #ifdef ENABLE_ACROBAT
  return flipState != 0;
  #else
  return false;
  #endif
}

// Pre-shaping controller: outside the deadband the rate is fed straight to the
// velocity loop with no cap, ramp or slew; inside it the position loop holds
// AltHold. The shaping state follows; the hand-back on exit is in
// calculateAltHoldThrottleAdjustment ( ). cm/s.
static int32_t flipVelocitySetpoint ( void ) {
  int32_t setVel;

  if ( setVelocity != 0 ) {
    AltHold = EstAlt;
    setVel  = setVelocity;
  } else {
    const int32_t error = constrain ( AltHold - EstAlt, -500, 500 );
    calculatedError     = error;
    altholdDebug8       = error;
    setVel              = constrain ( ( pidProfile->P8 [ PIDALT ] * error / 128 ), -300, +300 );
  }

  altTarget     = ( float ) AltHold;
  altRate       = 0.0f;
  altGoalActive = false;
  altVelTarget  = ( float ) setVel;
  return setVel;
}

static void offloadHoverTrim ( void ) {

  static uint32_t lastOffload = 0;

  if ( ! ARMING_FLAG ( ARMED ) || altHoldGroundIdle || setVelocity != 0 || altRate != 0.0f || altGoalActive )
    return;

  if ( ABS ( VelocityZ ) > ALT_TRIM_OFFLOAD_VEL )
    return;

  uint32_t now = millis ( );
  if ( ( now - lastOffload ) < ALT_TRIM_OFFLOAD_MS )
    return;

  lastOffload = now;

  int32_t trim = errorVelocityI / 8192;
  if ( trim == 0 )
    return;

  int32_t step     = ( trim > 0 ) ? 1 : -1;
  int32_t baseline = ( int32_t ) initialThrottleHold + step;

  // Never walk the baseline outside what the ESCs can be commanded anyway.
  if ( baseline < escAndServoConfig->minthrottle || baseline > escAndServoConfig->maxthrottle )
    return;

  initialThrottleHold       = ( int16_t ) baseline;
  errorVelocityI           -= step * 8192;
  altHoldThrottleAdjustment -= step;
}

static void applyMultirotorAltHold ( void ) {
  static uint8_t isAltHoldChanged = 0;
  static int16_t throttle_history = 0;
  static int16_t sensitivity_inv  = 6;

  if ( rcControlsConfig->alt_hold_fast_change ) {
    if ( ABS ( rcData [ THROTTLE ] - initialThrottleHold ) > rcControlsConfig->alt_hold_deadband ) {
      isAltHoldChanged       = 1;
      rcCommand [ THROTTLE ] = throttle_history + constrain ( ( rcData [ THROTTLE ] - initialThrottleHold ) / sensitivity_inv, -50, 80 );
    } else {
      if ( isAltHoldChanged ) {
        AltHold          = EstAlt;
        isAltHoldChanged = 0;
        // Do NOT reset errorVelocityI here: it holds the hover trim, so clearing
        // it drops the throttle and the craft falls. Reset only on BARO entry /
        // disarm.
        // errorVelocityI   = 0;
      }
      rcCommand [ THROTTLE ] = constrain ( initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle );
    }
    throttle_history = rcCommand [ THROTTLE ];
  } else {
    // Stick sets the climb rate that moves AltHold ( see setpoint shaping ). The
    // rate starts at zero on the deadband edge, so leaving the deadband is not a
    // step, and reaches the maximum at full stick.
    const int32_t deflection = rcData [ THROTTLE ] - 1500;
    const int32_t deadband   = rcControlsConfig->alt_hold_deadband;

    // flip ( ) writes 2000 into rcData [ THROTTLE ] on its last tick as well, and
    // it stays there until the next RX frame. Read as stick it would cancel the
    // return-to-height goal set on flip exit, so ignore the throttle briefly.
    static bool flipSeen          = false;
    static uint32_t flipEndedAtMs = 0;
    if ( flipActive ( ) ) {
      flipSeen = true;
    } else if ( flipSeen ) {
      flipSeen      = false;
      flipEndedAtMs = millis ( );
    }
    const bool flipExitGrace = ( flipEndedAtMs != 0 ) && ( ( millis ( ) - flipEndedAtMs ) < ALT_FLIP_EXIT_IGNORE_MS );

    if ( isLanding ) {
      // rcData [ THROTTLE ] holds landThrottle here ( mw.cpp ).
      setVelocity = constrain ( deflection / 4, -ALT_LAND_MAX_DESCENT_CMS, 0 );
    } else if ( flipExitGrace ) {
      setVelocity = 0;
    } else if ( flipActive ( ) ) {
      // rcData [ THROTTLE ] is written by flip ( ), not the pilot: raw rate as
      // before setpoint shaping ( see flipVelocitySetpoint ( ) ).
      setVelocity = ( ABS ( deflection ) > deadband ) ? constrain ( deflection / 4, -ALT_FLIP_MAX_DESCENT_CMS, ALT_FLIP_MAX_CLIMB_CMS ) : 0;
    } else if ( ABS ( deflection ) > deadband ) {
      const int32_t maxRate = ( deflection > 0 ) ? ALT_MAX_CLIMB_CMS : ALT_MAX_DESCENT_CMS;
      const int32_t rate    = constrain ( ( ABS ( deflection ) - deadband ) * maxRate / ( ALT_STICK_FULL_TRAVEL - deadband ), 0, maxRate );
      setVelocity           = ( deflection > 0 ) ? rate : -rate;
    } else {
      setVelocity = 0;
      offloadHoverTrim ( );
    }
    rcCommand [ THROTTLE ] = constrain ( initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle );
  }

  if ( isThrottleStickArmed && rcData [ THROTTLE ] <= 1500 ) {
    rcCommand [ THROTTLE ] = 1000;
    altHoldGroundIdle      = true;
  } else {
    isThrottleStickArmed = false;
    altHoldGroundIdle    = false;
  }

  altholdDebug  = AltHold;
  altholdDebug1 = altHoldThrottleAdjustment;
  altholdDebug2 = initialThrottleHold;
  altholdDebug3 = isAltHoldChanged;
  altholdDebug4 = rcCommand [ THROTTLE ];
}

static void applyFixedWingAltHold ( airplaneConfig_t *airplaneConfig ) {
  rcCommand [ PITCH ] += altHoldThrottleAdjustment * airplaneConfig->fixedwing_althold_dir;
}

void applyAltHold ( airplaneConfig_t *airplaneConfig ) {
  if ( STATE ( FIXED_WING ) ) {
    applyFixedWingAltHold ( airplaneConfig );
  } else {
    applyMultirotorAltHold ( );
  }
}

void updateAltHoldState ( void ) {
  if ( ! IS_RC_MODE_ACTIVE ( BOXBARO ) ) {
    DISABLE_FLIGHT_MODE ( BARO_MODE );
    // applyAltHold ( ) no longer runs, so nothing else clears the stick inputs.
    setVelocity       = 0;
    altHoldGroundIdle = false;
    return;
  }

  if ( ! FLIGHT_MODE ( BARO_MODE ) ) {
    ENABLE_FLIGHT_MODE ( BARO_MODE );
    // baroResetGroundLevel(); // Reset ground level for barometer
    resetAltSetpoint ( );
    initialThrottleHold       = 1500;
    errorVelocityI            = 0;
    altHoldThrottleAdjustment = 0;
  }
  initialThrottleHold_test = initialThrottleHold;
  debug_e1                 = rcCommand [ THROTTLE ];
}



bool isThrustFacingDownwards ( rollAndPitchInclination_t *inclination ) {
  return ABS ( inclination->values.rollDeciDegrees ) < DEGREES_80_IN_DECIDEGREES && ABS ( inclination->values.pitchDeciDegrees ) < DEGREES_80_IN_DECIDEGREES;
}

int16_t calculateTiltAngle ( rollAndPitchInclination_t *inclination ) {
  return MAX ( ABS ( inclination->values.rollDeciDegrees ), ABS ( inclination->values.pitchDeciDegrees ) );
}

int32_t calculateAltHoldThrottleAdjustment ( int32_t velocity_z, float accZ_tmp, float accZ_old, float ctrlDt ) {
  int32_t result = 0;
  int32_t error;
  int32_t setVel;

  // Flip hand-back ( flip-althold-regression, log-2.txt ). HOLD asks for 120 cm/s
  // while the craft falls out of the rotation, so the velocity integrator winds
  // up, and the flip's last setpoint ( 120 ) was left for the slew to walk down:
  // together they drove a flyaway into the ceiling. On exit, restart from a zero
  // setpoint with the hover trim from before the flip, and fly back to the
  // altitude ALT_HOLD was holding when the flip was sent: HOLD's fixed 1.5 s of
  // full throttle ends the flip 40-120 cm high ( log-3.txt, log-4.txt ). Writing
  // AltHold makes it a goal, flown on the goal profile like take-off; the stick
  // cancels it. Checked ahead of the tilt test so a flip that ends tilted is
  // still handed back.
  static bool altFlipWasActive       = false;
  static int32_t altPreFlipVelocityI = 0;
  static bool altPreFlipReturn       = false;
  static uint32_t altFlipExitAtMs    = 0;
  const bool flipping                = flipActive ( );
  if ( flipping && ! altFlipWasActive ) {
    // Without BARO_MODE the integrator is chasing an AltHold nothing actuates;
    // save what BARO entry would reset it to instead, and hold where the flip
    // ends ( a user-code flip can start with ALT_HOLD off, command.cpp only
    // checks MAG_MODE ).
    altPreFlipVelocityI = FLIGHT_MODE ( BARO_MODE ) ? errorVelocityI : 0;
    // Only return to a height the craft was flying at: a flip sent while armed on
    // the floor ( idle-held, or AltHold still at the ground datum ) would fly back
    // down to the floor with the motors running, so hold where the flip ends.
    altPreFlipReturn    = FLIGHT_MODE ( BARO_MODE ) && ! altHoldGroundIdle && AltHold >= ALT_FLIP_RETURN_MIN_CM;
    altPreFlipAltHold   = AltHold;
  } else if ( ! flipping && altFlipWasActive ) {
    resetAltSetpoint ( );
    errorVelocityI  = altPreFlipVelocityI;
    altFlipExitAtMs = millis ( );
    if ( altPreFlipReturn ) {
      AltHold = altPreFlipAltHold;
    }
  }
  altFlipWasActive = flipping;

  if ( ! isThrustFacingDownwards ( &inclination ) ) {
    return result;
  }

  // Disarmed, or armed on the throttle stick with the motors held at idle: the
  // craft is on the ground, so hold the whole controller in reset. Otherwise the
  // integrator winds down against a craft that cannot move and the take-off is
  // late and sluggish.
  if ( ! ARMING_FLAG ( ARMED ) || altHoldGroundIdle ) {
    resetAltSetpoint ( );
    errorVelocityI  = 0;
    altFlipExitAtMs = 0;    // no post-flip integrator hold after a re-arm
    return result;
  }

  ctrlDt = constrainf ( ctrlDt, 0.0f, 0.05f );

  setVel = flipActive ( ) ? flipVelocitySetpoint ( ) : shapedVelocitySetpoint ( ctrlDt );

  error         = setVel - velocity_z;
  altholdDebug9 = error;
  result        = constrain ( ( pidProfile->P8 [ PIDVEL ] * error / 32 ), -300, +300 );

  velControlDebug [ 0 ] = result;

  // The flip ends climbing at ~120 cm/s; braking that from a near-zero setpoint
  // drove the integrator to -20 counts in 0.4 s, and once the climb stopped that
  // trim dropped the craft 15-46 cm below the target ( log-5.txt ). Hold the
  // restored pre-flip trim while the P term ( at its clamp ) brakes the climb.
  const bool holdFlipExitI = ( altFlipExitAtMs != 0 ) && ( ( millis ( ) - altFlipExitAtMs ) < ALT_FLIP_EXIT_I_HOLD_MS );

  if ( ARMING_FLAG ( ARMED ) ) {
    if ( ! holdFlipExitI ) {
      errorVelocityI += ( pidProfile->I8 [ PIDVEL ] * error );
    }
  } else {
    errorVelocityI = 0;
  }

  errorVelocityI = constrain ( errorVelocityI, -( 8192 * 300 ), ( 8192 * 300 ) );
  result += errorVelocityI / 8192;

  velControlDebug [ 1 ] = errorVelocityI / 8192;

  result -= constrain ( pidProfile->D8 [ PIDVEL ] * ( accZ_tmp + accZ_old ) / 512, -150, 150 );
  velControlDebug [ 2 ] = constrain ( pidProfile->D8 [ PIDVEL ] * ( accZ_tmp + accZ_old ) / 512, -150, 150 );

  return result;
}

// Velocity setpoint from setpoint shaping: stick rate or goal profile moving the
// target, fed forward with the position loop, clamped and slewed. cm/s.
static int32_t shapedVelocitySetpoint ( float ctrlDt ) {
  int32_t error;

  // AltHold written elsewhere ( take-off, setAltitude ( ), MSP ) becomes the goal;
  // the target stays where it is and travels there below.
  if ( AltHold != lrintf ( altTarget ) ) {
    altGoal       = ( float ) AltHold;
    altGoalActive = true;
  }
  if ( setVelocity != 0 ) {
    altGoalActive = false;    // the stick ( or landing ) takes over
  }

  // Rate the target should move at: the stick's, or the goal profile's - the
  // fastest rate that can still brake to a stop on the goal, capped at cruise.
  float profileRate = ( float ) setVelocity;
  if ( altGoalActive ) {
    const float remaining = altGoal - altTarget;
    const float stopRate  = sqrtf ( 2.0f * ALT_GOAL_DECEL_CMSS * fabsf ( remaining ) );
    if ( remaining > 0.0f ) {
      profileRate = fminf ( stopRate, ( float ) ALT_CMD_MAX_CLIMB_CMS );
    } else {
      profileRate = -fminf ( stopRate, ( float ) ALT_CMD_MAX_DESCENT_CMS );
    }
  }

  const float rateStep = ALT_STICK_ACCEL_CMSS * ctrlDt;
  altRate += constrainf ( profileRate - altRate, -rateStep, rateStep );

  // Move the target. The leash stops it running away from a craft that cannot
  // keep up ( still on the ground, weak battery ); the rate is kept, so it resumes
  // as soon as the craft catches up.
  const float nextTarget = altTarget + altRate * ctrlDt;
  if ( ! ( ( altRate > 0.0f && nextTarget > ( float ) ( EstAlt + ALT_TARGET_LEASH_CM ) ) || ( altRate < 0.0f && nextTarget < ( float ) ( EstAlt - ALT_TARGET_LEASH_CM ) ) ) ) {
    altTarget = nextTarget;
  }

  // Arrived: land exactly on the goal once the profile has braked.
  if ( altGoalActive && fabsf ( altGoal - altTarget ) < 1.0f && fabsf ( altRate ) <= 2.0f * rateStep ) {
    altTarget     = altGoal;
    altRate       = 0.0f;
    altGoalActive = false;
  }
  AltHold = lrintf ( altTarget );

  error = constrain ( AltHold - EstAlt, -500, 500 );
  // No deadband on the position error: applyDeadband ( ) subtracts, so a 5 cm
  // band took 5 cm off every error ( a 10 cm sag read as 5 ) and caused a limit
  // cycle. EstAlt is already smooth ( ~1 cm between samples ), so none is needed.
  calculatedError = error;
  altholdDebug8   = error;

  // Profile rate fed forward plus the position loop, clamped and slewed.
  const float maxClimb   = altGoalActive ? ( float ) ALT_CMD_MAX_CLIMB_CMS : ( float ) ALT_MAX_CLIMB_CMS;
  const float maxDescent = isLanding ? ( float ) ALT_LAND_MAX_DESCENT_CMS : ( altGoalActive ? ( float ) ALT_CMD_MAX_DESCENT_CMS : ( float ) ALT_MAX_DESCENT_CMS );
  float velDemand        = altRate + ( float ) pidProfile->P8 [ PIDALT ] * ( float ) error / 128.0f;
  velDemand              = constrainf ( velDemand, -maxDescent, maxClimb );
  const float velStep = ALT_VEL_ACCEL_CMSS * ctrlDt;
  altVelTarget       += constrainf ( velDemand - altVelTarget, -velStep, velStep );
  return lrintf ( altVelTarget );
}

int16_t accalttemp;
float Temp;

void calculateEstimatedAltitude ( uint32_t currentTime ) {
  static uint32_t previousTime;
  uint32_t dTime;
  int32_t baroVel;
  float dt;
  float vel_acc;
  int32_t vel_tmp;
  float accZ_tmp;

  static float accZ_old = 0.0f;
  static float vel      = 0.0f;
  static float accAlt   = 0.0f;
  static int32_t lastBaroAlt;

  dTime = currentTime - previousTime;
  if ( dTime < BARO_UPDATE_FREQUENCY_40HZ )
    return;

  previousTime = currentTime;

  #ifdef BARO
  if ( ! isBaroCalibrationComplete ( ) ) {
    performBaroCalibrationCycle ( );
    vel    = 0;
    accAlt = 0;
  }
  #else
  BaroAlt = 0;
  #endif



  dt = accTimeSum * 1e-6f;

  if ( accSumCount ) {
    accZ_tmp = ( float ) accSum [ 2 ] / ( float ) accSumCount;
  } else {
    accZ_tmp = 0;
  }
  vel_acc = accZ_tmp * accVelScale * ( float ) accTimeSum;

  accAlt += ( vel_acc * 0.5f ) * dt + vel * dt;
  accalttemp = lrintf ( 100 * accAlt );
  accAlt     = accAlt * barometerConfig->baro_cf_alt + ( float ) BaroAlt * ( 1.0f - barometerConfig->baro_cf_alt );
  vel += vel_acc;

  #ifdef DEBUG_ALT_HOLD
  debug [ 1 ] = accSum [ 2 ] / accSumCount;
  debug [ 2 ] = vel;
  debug [ 3 ] = accAlt;
  #endif

  imuResetAccelerationSum ( 1 );

  #ifdef BARO
  if ( ! isBaroCalibrationComplete ( ) ) {
    return;
  }
  #endif

  EstAlt = accAlt;

  baroVel     = ( BaroAlt - lastBaroAlt ) * 1000000.0f / dTime;
  lastBaroAlt = BaroAlt;

  baroVel = constrain ( baroVel, -1500, 1500 );
  baroVel = applyDeadband ( baroVel, 10 );

  vel     = vel * barometerConfig->baro_cf_vel + baroVel * ( 1.0f - barometerConfig->baro_cf_vel );
  vel_tmp = lrintf ( vel );

  vario = applyDeadband ( vel_tmp, 5 );

  // Update Kalman filters with new sensor readings
  float filteredVel = kalmanFilterUpdate ( &velHoldFilter, vel_tmp );
  float filteredAlt = kalmanFilterUpdate ( &altHoldFilter, EstAlt );

  altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment ( vel_tmp, accZ_tmp, accZ_old, ( float ) dTime * 1e-6f );

  Temp                         = pidProfile->I8 [ PIDALT ];
  barometerConfig->baro_cf_alt = 1 - Temp / 1000;
  accZ_old                     = accZ_tmp;

  // Update global variables with filtered values
  EstAlt = filteredAlt;
  vel    = filteredVel;
  // altest=EstAlt;
}

/* queue implementation */

void addHistPositionBaseEstZ ( float position ) {
  if ( itemCount < QUEUE_MAX_LENGTH ) {
    rear++;
    if ( rear >= QUEUE_MAX_LENGTH ) {
      rear = 0;
    }
    buff [ rear ] = position;
    itemCount++;
  } else {
    if ( ++rear == QUEUE_MAX_LENGTH ) {
      rear          = 0;
      buff [ rear ] = position;
      head++;
    } else {
      buff [ rear ] = position;
      head++;
      if ( head == QUEUE_MAX_LENGTH ) {
        head = 0;
      }
    }
  }
}

float getFrontHistPositionBaseEstZ ( ) {
  float return_value = buff [ head ];
  head++;
  if ( head == QUEUE_MAX_LENGTH ) {
    head = 0;
  }
  itemCount--;
  return return_value;
}

bool isPositionBaseQueueIsFull ( ) {
  return itemCount == QUEUE_MAX_LENGTH;
}

/* using ArduPilots Third Order Compilmentary filter */

void apmCalculateEstimatedAltitude ( uint32_t currentTime ) {
  static uint32_t previousTime;
  float dt       = ( currentTime - previousTime ) / 1000000.0f;
  uint32_t dTime = currentTime - previousTime;

  if ( dTime < UPDATE_FREQUENCY )
    return;

  previousTime = currentTime;

  if ( dTime > 2 * UPDATE_FREQUENCY ) {
    imuResetAccelerationSum ( 1 );
  }

  if ( AltRstRequired && ! ARMING_FLAG ( ARMED ) )
    AltRst ( );

  #if defined( BARO ) && ! ( defined( LASER_ALT ) )
  checkBaro ( );
  #else
  checkReading ( );
  #endif

  if ( accSumCount > 0 ) {
    accel_ef_z = ( float ) accSum [ 2 ] / ( float ) accSumCount;
  } else {
    accel_ef_z = 0;
  }

  accZ_tmp   = accel_ef_z;
  accel_ef_z = constrainf ( accel_ef_z * accVelScale, -800, 800 );

  imuResetAccelerationSum ( 1 );

  if ( first_velocity_reads <= 5 ) {
    first_velocity_reads++;
    return;
  }

  _accel_correction_hbf_z += _position_error_z * _k3_z * dt;
  _velocity_z += _position_error_z * _k2_z * dt;
  _position_correction_z += _position_error_z * _k1_z * dt;

  float velocity_increase_z = ( accel_ef_z + _accel_correction_hbf_z ) * dt;
  _position_base_z += ( _velocity_z + velocity_increase_z * 0.5f ) * dt;
  _position_z = _position_base_z + _position_correction_z;
  _velocity_z += velocity_increase_z;

  VelocityZ = lrintf ( _velocity_z );
  EstAlt    = lrintf ( _position_z );

  addHistPositionBaseEstZ ( _position_base_z );

  // Apply Kalman filtering to APM estimates
  float filteredVelocityZ = kalmanFilterUpdate ( &velHoldFilter, VelocityZ );
  float filteredEstAlt    = kalmanFilterUpdate ( &altHoldFilter, EstAlt );

  // Update global variables with filtered values
  VelocityZ = lrintf ( filteredVelocityZ );
  EstAlt    = lrintf ( filteredEstAlt );

  altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment ( VelocityZ, accZ_tmp, accZ_old, dt );
  accZ_old                  = accZ_tmp;
  vario                     = applyDeadband ( VelocityZ, 5 );
  if ( abs ( VelocityZ ) > 200 )
    AltRstRequired = 1;
}

  #ifdef LASER_ALT
static bool altSourceLaser = true;    // estimator corrected by the laser ( true ) or the baro
static bool altStepPending = false;   // object step seen, holding on the baro ( task 6 )

uint8_t altHoldSource ( void ) {
  return altStepPending ? 2 : ( altSourceLaser ? 1 : 0 );
}

// Recent raw laser samples with the inertial position at the same instant ( task 14 ).
// The helpers below serve either laser ( the shared fusion body in checkReading ( ) ).
static float tofRingRawCm [ ALT_TOF_RING_LEN ];
static float tofRingBaseCm [ ALT_TOF_RING_LEN ];
static uint32_t tofRingMs [ ALT_TOF_RING_LEN ];
static uint8_t tofRingHead  = 0;    // next slot to write
static uint8_t tofRingCount = 0;

static void tofWindowReset ( void ) {
  tofRingCount = 0;
}

static void tofWindowPush ( uint32_t nowMs, float rawCm, float baseCm ) {
  tofRingRawCm [ tofRingHead ]  = rawCm;
  tofRingBaseCm [ tofRingHead ] = baseCm;
  tofRingMs [ tofRingHead ]     = nowMs;
  tofRingHead                   = ( uint8_t ) ( ( tofRingHead + 1 ) % ALT_TOF_RING_LEN );
  if ( tofRingCount < ALT_TOF_RING_LEN ) {
    tofRingCount++;
  }
}

// Laser change minus inertial change since the newest stored sample at least ALT_TOF_WINDOW_MS old
// ( or the oldest one, if it is at least ALT_TOF_WINDOW_MIN_MS old ). False when the ring is too short.
static bool tofWindowMismatch ( uint32_t nowMs, float rawCm, float baseCm, float *mismatchCm, float *refRawCm, float *refBaseCm ) {
  bool found = false;
  uint8_t pick = 0;
  for ( uint8_t k = 0; k < tofRingCount; k++ ) {    // oldest to newest
    const uint8_t idx = ( uint8_t ) ( ( tofRingHead + ALT_TOF_RING_LEN - tofRingCount + k ) % ALT_TOF_RING_LEN );
    const uint32_t ageMs = nowMs - tofRingMs [ idx ];
    if ( ageMs >= ALT_TOF_WINDOW_MS || ( k == 0 && ageMs >= ALT_TOF_WINDOW_MIN_MS ) ) {
      pick  = idx;
      found = true;
    }
  }
  if ( found ) {
    *mismatchCm = ( rawCm - tofRingRawCm [ pick ] ) - ( baseCm - tofRingBaseCm [ pick ] );
    *refRawCm   = tofRingRawCm [ pick ];
    *refBaseCm  = tofRingBaseCm [ pick ];
  }
  return found;
}

// Move the whole altitude frame by deltaCm: filter state, its delayed history, the smoother and
// the setpoint. Every controller error is a difference of these, so none of them changes and a
// reference change ( baro -> laser ) does not move the aircraft ( task 5 ).
static void altShiftFrame ( float deltaCm ) {
  const bool holdTracksTarget = ( AltHold == lrintf ( altTarget ) );
  _position_base_z += deltaCm;
  _position_z      += deltaCm;
  for ( int i = 0; i < QUEUE_MAX_LENGTH; i++ ) {
    buff [ i ] += deltaCm;
  }
  altHoldFilter.X += deltaCm;
  EstAlt           = lrintf ( altHoldFilter.X );
  altTarget       += deltaCm;
  altGoal         += deltaCm;
  if ( holdTracksTarget ) {
    AltHold = lrintf ( altTarget );    // keep them equal, or shaping reads a new goal
  } else {
    AltHold += lrintf ( deltaCm );
  }
  altPreFlipAltHold += lrintf ( deltaCm );    // a post-flip return goal stays in the same frame
  tofWindowReset ( );                         // stored inertial positions are in the old frame
}

void checkReading ( ) {
  uint32_t baro_update_time;
  float baroDt = 0.0f;              // s since the previous baro sample, 0 when none is new
  float tilt   = 0;                 // rad
  static float baro_offset = 0.0f;  // cm, filtered baro altitude minus laser height, updated on the laser
    #if defined( LASER_TOF ) || defined( LASER_TOF_L1x )
  static bool tofTiltOk    = true;  // last laser sample taken within ALT_TOF_MAX_TILT_DECIDEG
    #endif

  baro_update_time = getBaroLastUpdate ( );
  if ( baro_update_time != baro_last_update ) {
    baroDt           = ( float ) ( baro_update_time - baro_last_update ) * 0.001f;
    Baro_Height      = baroCalculateAltitude ( );
    filtered         = ( 0.75f * filtered ) + ( ( 1 - 0.75f ) * Baro_Height );
    baro_last_update = baro_update_time;
  }
    #if defined( LASER_TOF ) || defined( LASER_TOF_L1x )    // the shared fusion body, for either laser
  const uint32_t nowMs = millis ( );
  static uint32_t lastGoodTofMs  = 0;       // time of the last usable laser sample
  static uint8_t returnCount     = 0;       // consecutive usable samples below the lower edge
  static bool offsetSeeded       = false;
  static uint32_t lastOffsetMs   = 0;
  static bool leftOnDropout      = false;   // last handover was a dropout / tilt, not a climb
  static uint32_t stepStartMs    = 0;
  static float steadyRefCm       = 0.0f;
  static uint8_t steadyCount     = 0;
  static uint8_t cancelCount     = 0;
  static bool tofSuspect         = false;   // laser and inertial disagree: coast, keep the edge out
  static bool windowReady        = false;   // the window has enough history ( false after a reset )
  static uint32_t suspectStartMs = 0;
  static float suspectRefRawCm   = 0.0f;
  static float suspectRefBaseCm  = 0.0f;
  static float suspectMismatchCm = 0.0f;
  static bool suspectEscalate    = false;   // timed out as an edge: start the hold-off next sample
  static float lastRawCm         = 0.0f;
  static bool wasAirborne        = false;
  static uint32_t airborneSinceMs = 0;
  bool newGoodSample             = false;
  float rawCm                    = 0.0f;    // this sample before the driver IIR, tilt-corrected

  if ( tofNew ( ) && ( ! tofOutOfRange ( ) ) ) {
    tofClearNew ( );
    // The old gate compared radians with 25, so it never rejected a tilted sample.
    const int16_t tiltDeciDeg = calculateTiltAngle ( &inclination );
    tofTiltOk                 = tiltDeciDeg < ALT_TOF_MAX_TILT_DECIDEG;
    if ( ! tofTiltOk ) {
      tofReseed ( );    // keep slant-range samples out of the driver's IIR history
      tofWindowReset ( );
    } else {
      tilt       = degreesToRadians ( ( int16_t ) ( tiltDeciDeg / 10 ) );
      ToF_Height = tofFiltCm ( ) * cos_approx ( tilt );
      rawCm      = tofRawCm ( ) * cos_approx ( tilt );
      if ( ToF_Height > 0.0f ) {
        newGoodSample = true;
      }
    }
  }
  const bool tofUsable = tofTiltOk && ( ! tofOutOfRange ( ) ) && ToF_Height > 0.0f;
  // Laser height advanced past the driver IIR's lag, for pairing with the baro or the estimate.
  const float tofNowCm = ToF_Height + ( float ) VelocityZ * ALT_TOF_IIR_LAG_S;

  // Object under the craft ( task 6 ): compare each raw sample with the estimate. Off while
  // disarmed, in the pre-take-off ground idle or landing ( a landing on the laser keeps following
  // it to touchdown ), and outside the laser band ( the handover owns that ).
  const bool airborne = ARMING_FLAG ( ARMED ) && ( ! altHoldGroundIdle ) && ( ! isLanding );
  if ( airborne && ( ! wasAirborne ) ) {
    airborneSinceMs = nowMs;
  }
  wasAirborne          = airborne;
  const bool windowArmed = airborne && ( nowMs - airborneSinceMs ) >= ALT_TOF_AIRBORNE_GRACE_MS;
  float windowMismatchCm = 0.0f;
  bool haveWindow        = false;
  if ( newGoodSample ) {
    float refRawCm  = 0.0f;
    float refBaseCm = 0.0f;
    haveWindow      = tofWindowMismatch ( nowMs, rawCm, _position_base_z, &windowMismatchCm, &refRawCm, &refBaseCm );
    tofWindowPush ( nowMs, rawCm, _position_base_z );
    windowReady = haveWindow;
    lastRawCm   = rawCm;
    if ( ! windowArmed ) {
      haveWindow = false;    // lift-off: the window may not start a suspicion or a hold-off yet
    }
    if ( tofSuspect ) {
      // Keep adding up against the reference frozen when the edge began.
      suspectMismatchCm = ( rawCm - suspectRefRawCm ) - ( _position_base_z - suspectRefBaseCm );
    } else if ( haveWindow && fabsf ( windowMismatchCm ) > ALT_TOF_SUSPECT_CM && altSourceLaser && airborne && ( ! altStepPending ) ) {
      tofSuspect        = true;
      suspectStartMs    = nowMs;
      suspectRefRawCm   = refRawCm;
      suspectRefBaseCm  = refBaseCm;
      suspectMismatchCm = windowMismatchCm;
    }
  }
  if ( tofSuspect ) {
    if ( ( ! altSourceLaser ) || ( ! airborne ) || altStepPending ) {
      tofSuspect      = false;
      suspectEscalate = false;
    } else if ( ( ! suspectEscalate ) && fabsf ( suspectMismatchCm ) < 0.5f * ALT_TOF_SUSPECT_CM ) {
      tofSuspect = false;    // laser and inertial agree again
    } else if ( ( ! suspectEscalate ) && ( nowMs - suspectStartMs ) > ALT_TOF_SUSPECT_MAX_MS ) {
      if ( fabsf ( suspectMismatchCm ) <= ALT_TOF_SUSPECT_ESCALATE_CM ) {
        tofSuspect = false;    // a slope, not an edge: follow it
      } else if ( lastRawCm >= ALT_TOF_HANDOVER_UP_CM ) {
        tofSuspect     = false;    // an edge that ends above the band: straight to the baro
        altSourceLaser = false;
        leftOnDropout  = true;
      } else {
        suspectEscalate = true;    // an edge: the next sample starts the hold-off
      }
    } else if ( suspectEscalate && ( nowMs - suspectStartMs ) > ALT_TOF_SUSPECT_MAX_MS + ALT_TOF_DROPOUT_MS ) {
      tofSuspect      = false;    // no usable sample came to start it: hand to the baro
      suspectEscalate = false;
      altSourceLaser  = false;
      leftOnDropout   = true;
    }
  }
  if ( ( ! altSourceLaser ) || ( ! airborne ) ) {
    if ( altStepPending && altSourceLaser && isLanding && tofUsable ) {
      // Landing began during a hold-off: take the new surface now ( frame shift, the craft does
      // not move ) rather than hand the whole step to correctedWithTof ( ), which would slow the
      // descent and delay touchdown.
      const float landDeltaCm = tofNowCm - _position_z;
      altShiftFrame ( landDeltaCm );
      baro_offset -= landDeltaCm;
    }
    altStepPending = false;
  } else if ( newGoodSample && rawCm < ALT_TOF_HANDOVER_UP_CM ) {
    // The estimate follows the IIR-lagged laser, so it trails the raw reading by VelocityZ times
    // the lag in steady motion ( up to ~30 cm in a landing or flip climb ); compare against the
    // estimate advanced by that lag, so only a change of surface counts.
    const float expectedCm = _position_z + ( float ) VelocityZ * ALT_TOF_IIR_LAG_S;
    const float residCm    = rawCm - expectedCm;
    if ( ! altStepPending ) {
      if ( fabsf ( residCm ) > ALT_TOF_STEP_CM || ( haveWindow && fabsf ( windowMismatchCm ) > ALT_TOF_STEP_CM )
           || ( tofSuspect && ( fabsf ( suspectMismatchCm ) > ALT_TOF_STEP_CM || suspectEscalate ) ) ) {
        altStepPending  = true;
        tofSuspect      = false;
        suspectEscalate = false;
        stepStartMs    = nowMs;
        steadyRefCm    = rawCm;
        steadyCount    = 0;
        cancelCount    = 0;
        tofReseed ( );    // the driver IIR follows the new surface at once
        tofWindowReset ( );
      }
    } else {
      if ( fabsf ( residCm ) < 0.5f * ALT_TOF_STEP_CM ) {
        cancelCount++;
      } else {
        cancelCount = 0;
      }
      if ( fabsf ( rawCm - steadyRefCm ) <= ALT_TOF_STEADY_CM ) {
        if ( steadyCount < 255 ) {
          steadyCount++;
        }
      } else {
        steadyRefCm = rawCm;
        steadyCount = 0;
      }
      if ( cancelCount >= ALT_TOF_RETURN_SAMPLES ) {
        altStepPending = false;    // the surface came back: nothing to do
        tofReseed ( );
        tofWindowReset ( );
      } else if ( ( nowMs - stepStartMs ) >= ALT_TOF_STEP_HOLD_MS && steadyCount >= ALT_TOF_STEADY_SAMPLES ) {
        // Still there and steady: re-base the estimate to the new surface without moving the
        // aircraft, then give the old setpoint back as a goal so it climbs over the object ( or
        // descends after it is gone ) on the goal profile.
        const int32_t holdBefore = AltHold;
        const float deltaCm      = rawCm - expectedCm;
        altShiftFrame ( deltaCm );
        baro_offset -= deltaCm;    // the offset moves with the frame ( baro minus the new surface )
        // An active goal ( take-off, flip return, MSP ) keeps its end point over the old surface.
        AltHold        = altGoalActive ? lrintf ( altGoal - deltaCm ) : holdBefore;
        altPreFlipAltHold -= lrintf ( deltaCm );    // a flip return also keeps its clearance
        ToF_Height     = rawCm;
        altStepPending = false;
        tofReseed ( );
      }
    }
  }

    #ifdef LASER_TOF_L1x
  // Baro -> laser return guard ( see ALT_TOF_RETURN_AGREE_CM ). A return candidate far from the
  // baro-path estimate is held off, unless the disagreement ( laser minus estimate ) stays within
  // ALT_TOF_RETURN_STEADY_CM for ALT_TOF_STEP_HOLD_MS: then it is a new floor ( take-off from a table, a long
  // baro drift ), not fan blades or a passing object, and the return is taken with the normal shift.
  // The disagreement, not the raw reading, is tested, so the craft may climb, descend or land meanwhile.
  static bool retSteadyActive      = false;
  static float retSteadyRefCm      = 0.0f;    // cm, disagreement of the first candidate of the steady run
  static uint32_t retSteadyStartMs = 0;
  bool returnHeld                  = false;   // disagreeing return candidate, not yet steady long enough
  const float returnDisagreeCm     = tofNowCm - _position_z;    // cm, laser minus baro-path estimate
  if ( ( ! altSourceLaser ) && newGoodSample && ToF_Height < ( leftOnDropout ? ALT_TOF_HANDOVER_UP_CM : ALT_TOF_HANDOVER_DOWN_CM )
       && ARMING_FLAG ( ARMED ) && ( ! altHoldGroundIdle ) && fabsf ( returnDisagreeCm ) > ALT_TOF_RETURN_AGREE_CM ) {
    if ( retSteadyActive && fabsf ( returnDisagreeCm - retSteadyRefCm ) <= ALT_TOF_RETURN_STEADY_CM ) {
      returnHeld = ( nowMs - retSteadyStartMs ) < ALT_TOF_STEP_HOLD_MS;
    } else {
      retSteadyActive  = true;    // first candidate, or not steady: restart the run
      retSteadyRefCm   = returnDisagreeCm;
      retSteadyStartMs = nowMs;
      returnHeld       = true;
    }
    if ( ! returnHeld ) {
      returnCount      = ALT_TOF_RETURN_SAMPLES - 1;    // steady long enough: return on this sample
      retSteadyActive  = false;
    }
  } else if ( newGoodSample || ( ! tofUsable ) || altSourceLaser ) {
    retSteadyActive = false;    // a gap, an agreeing or out-of-band sample, or on the laser: restart
  }
    #endif

  if ( altSourceLaser ) {
    if ( newGoodSample ) {
      lastGoodTofMs = nowMs;
    }
    if ( newGoodSample && ( ! altStepPending ) && ( ! tofSuspect ) && windowReady ) {
      // Slow average of baro minus laser, so the offset frozen at a handover is not one
      // noisy baro sample ( 13 cm sd sample to sample on this board ).
      const float sampleOffset = Baro_Height - tofNowCm;
      if ( ! offsetSeeded ) {
        baro_offset  = sampleOffset;
        offsetSeeded = true;
      } else {
        // At most one sample period, so the first sample after a pause gets a normal weight.
        const float dtS = fminf ( ( float ) ( nowMs - lastOffsetMs ) * 0.001f, ALT_TOF_OFFSET_DT_MAX_S );
        baro_offset += ( sampleOffset - baro_offset ) * ( dtS / ( ALT_BARO_OFFSET_TAU_S + dtS ) );
      }
      lastOffsetMs = nowMs;
    }

    if ( tofUsable && ToF_Height >= ALT_TOF_HANDOVER_UP_CM ) {
      altSourceLaser = false;    // above the band: hand over, offset frozen
      leftOnDropout  = false;
    } else if ( ( nowMs - lastGoodTofMs ) > ALT_TOF_DROPOUT_MS ) {
      altSourceLaser = false;    // no usable sample for too long: hand over, offset frozen
      leftOnDropout  = true;
    }
      #ifdef LASER_TOF_L1x
  } else if ( returnHeld ) {
    returnCount = 0;    // disagrees with the estimate and not yet steady: not the floor ( yet )
      #endif
  } else if ( newGoodSample && ToF_Height < ( leftOnDropout ? ALT_TOF_HANDOVER_UP_CM : ALT_TOF_HANDOVER_DOWN_CM ) ) {
    if ( ++returnCount >= ALT_TOF_RETURN_SAMPLES ) {
      // Back on the laser: move the whole altitude frame by the baro drift so the numbers
      // change and the aircraft does not ( pilot's choice, task 5 ).
      const float returnDeltaCm = tofNowCm - _position_z;
      altShiftFrame ( returnDeltaCm );
      baro_offset -= returnDeltaCm;    // what the baro drifted by is now in the offset
      altSourceLaser = true;
      lastGoodTofMs  = nowMs;
      lastOffsetMs   = nowMs;    // the offset average carries on from its frozen value
    }
  } else if ( newGoodSample || ( ! tofUsable ) ) {
    returnCount = 0;
  }
  if ( altSourceLaser ) {
    returnCount = 0;
  }

  if ( altSourceLaser && ( ! altStepPending ) ) {
    if ( tofUsable && ( ! tofSuspect ) ) {
      correctedWithTof ( ToF_Height );
    } else {
      _position_error_z = 0.0f;    // short gap or suspected edge: coast on the accelerometer
    }
  } else {
    correctedWithBaro ( Baro_Height - baro_offset, baroDt );
  }
    #endif
}
  #endif

void checkBaro ( ) {
  uint32_t baro_update_time;
  baro_update_time = getBaroLastUpdate ( );
  if ( baro_update_time != baro_last_update ) {
    const float dt = ( float ) ( baro_update_time - baro_last_update ) * 0.001f;
    correctedWithBaro ( baroCalculateAltitude ( ), dt );
    baro_last_update = baro_update_time;
  }
}

void correctedWithBaro ( float baroAlt, float dt ) {
  altholdDebug5 = baroAlt;
  if ( dt > 0.5f ) {
    return;
  }

  float hist_position_base_z;
  if ( isPositionBaseQueueIsFull ( ) ) {
    hist_position_base_z = getFrontHistPositionBaseEstZ ( );
  } else {
    hist_position_base_z = _position_base_z;
  }
  _position_error_z = baroAlt - ( hist_position_base_z + _position_correction_z );

  #ifdef LASER_ALT
  // One time constant for both sources: switching 1.5 s <-> 2 s at every laser/baro change
  // disturbed the estimate. Tilted laser samples are already rejected in checkReading ( ).
  if ( _time_constant_z != ALT_EST_TAU_S ) {
    _time_constant_z = ALT_EST_TAU_S;
    updateGains ( );
  }
  #else
  // Deci-degrees: 300 = 30 deg ( the old 30 meant 3 deg, so the slow ~15 s filter
  // ran all the time and the estimate lagged high in descents ). 30 deg is above
  // the 20 deg max_angle_inclination, so this only trips in aggressive flight.
  if ( ABS ( inclination_generalised.values.rollDeciDegrees ) > 300 || ABS ( inclination_generalised.values.pitchDeciDegrees ) > 300 ) {
    _time_constant_z = 5;
    updateGains ( );
  } else {
    _time_constant_z = 2;
    updateGains ( );
  }
  #endif
}

  #ifdef LASER_ALT
void correctedWithTof ( float tofHeightCm ) {
  if ( first_reads == 0 ) {
    setAltitude ( tofHeightCm );
    first_reads++;
  }
  // Error against the filter's own position, not the Kalman-smoothed, integer EstAlt,
  // so no smoothed output is fed back into the filter.
  _position_error_z = tofHeightCm - _position_z;
  if ( _time_constant_z != ALT_EST_TAU_S ) {
    _time_constant_z = ALT_EST_TAU_S;
    updateGains ( );
  }
}
  #endif

void updateGains ( ) {
  if ( _time_constant_z == 0.0f ) {
    _k1_z = _k2_z = _k3_z = 0.0f;
  } else {
    _k1_z = 3.0f / _time_constant_z;
    _k2_z = 3.0f / ( _time_constant_z * _time_constant_z );
    _k3_z = 1.0f / ( _time_constant_z * _time_constant_z * _time_constant_z );
  }
}

void updateGains1 ( ) {
  if ( _time_constant_z1 == 0.0f ) {
    _k1_z1 = _k2_z1 = _k3_z1 = 0.0f;
  } else {
    _k1_z1 = 3.0f / _time_constant_z1;
    _k2_z1 = 3.0f / ( _time_constant_z1 * _time_constant_z1 );
    _k3_z1 = 1.0f / ( _time_constant_z1 * _time_constant_z1 * _time_constant_z1 );
  }
}

void setAltitude ( float new_altitude ) {
  _position_base_z       = new_altitude;
  _position_correction_z = 0;
  _position_z            = new_altitude;
  last_hist_position     = 0;
  imuResetAccelerationSum ( 1 );
}

int32_t altitudeHoldGetEstimatedAltitude ( void ) {
  return EstAlt;
}

int32_t getSetVelocity ( void ) {
  return setVelocity;
}

int32_t getSetAltitude ( void ) {
  return AltHold;
}

void AltRst ( void ) {
  _velocity_z = 0;
  imuResetAccelerationSum ( 1 );
  AltRstRequired            = 0;
  initialThrottleHold       = 1500;
  errorVelocityI            = 0;
  altHoldThrottleAdjustment = 0;

  // Reset Kalman filters
  kalmanFilterInit ( &altHoldFilter, 1.0, 0.10, 0.0 );
  kalmanFilterInit ( &velHoldFilter, 1.0, 0.10, 0.0 );
}

float getTimeConstant ( ) {
  return _time_constant_z;
}

#endif

void setAltitude ( int32_t altitude ) {
  AltHold = altitude;
}

void setRelativeAltitude ( int32_t altitude ) {
  AltHold = EstAlt + altitude;
}

int32_t getEstAltitude ( ) {
  return EstAlt;
  // altest=EstAlt;
}

int32_t getEstVelocity ( ) {
  return VelocityZ;
}

int32_t getEstAltitude1 ( ) {
  return EstAlt1;
}

int32_t getEstVelocity1 ( ) {
  return VelocityZ1;
}

bool limitAltitude ( ) {
  if ( max_altitude != -1 && IS_RC_MODE_ACTIVE ( BOXBARO ) ) {
    if ( EstAlt >= ( max_altitude - ALT_CEILING_MARGIN_CM ) ) {
      return true;
    }
  }
  return false;
}