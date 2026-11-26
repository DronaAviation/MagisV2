# Set desired rate API - Test code

[README](README.md) · [TASKS](TASKS.md) · [TESTING](TESTING.md)

The `PlutoPilot.cpp` used for the T16 / B30 runs ( task 1 pre-fix, task 6 post-fix ). It is not committed: the commit
restores the stock `PlutoPilot.cpp`. To re-run, copy the code below over `PlutoPilot.cpp`, build, and follow
[TESTING.md](TESTING.md). Set `TEST_SET_RATES` to `false` for the control run.

```cpp
// Do not remove the include below
#include "PlutoPilot.h"

/* ------------------------------------------------------------------------------------------------
 * TEMPORARY TEST CODE: set-desired-rate-api, task 1 ( API test T16 / B30 ).
 * Remove before release ( flagged in the topic README ). Kept in the topic's TEST_CODE.md.
 *
 * setDesiredRate ( ) is called in plutoInit ( ) and then at fixed times after Developer Mode is
 * first switched on ( the timeline survives Wi-Fi restarts of the user code ):
 *   S0   0 s  nothing new ( values from plutoInit: roll 40, pitch 50, yaw 10 )
 *   S1   5 s  set roll 40, pitch 50, yaw 10 from plutoLoop ( disarmed )
 *   S2  15 s  set out-of-range roll 300, pitch -5, yaw 400 ( clamp check ); skipped while armed, so an
 *             early arm cannot fly the clamped yaw 255 ( ~8.6x the stock yaw stick )
 *   S3  20 s  set roll 40, pitch 50, yaw 10 again ( disarmed ): arm only after "STEP 3"
 *   S4  45 s  set roll 30, pitch 40, yaw 15 ( armed, hovering in ANGLE mode ), then hold
 * Stock rates are roll / pitch 75 ( after the disarmed reset ), yaw 0. Yaw scales the yaw stick in
 * every mode by ( rate + 20 ) / 32: 10 / 15 give 1.5x / 1.75x the stock yaw stick. Roll / pitch act
 * only outside ANGLE mode ( RATE / HORIZON, flip ).
 * Set TEST_SET_RATES to false for the control run ( no setDesiredRate call at all ).
 *
 * Log, 10 Hz ( default user loop ), ~110 B per tick, inside the ~130 B budget:
 *   t   millis ( )
 *   gR / gP / gY  getDesiredRate ( AG_ROLL / AG_PITCH / AG_YAW )
 *   lR / lP / lY  live currentControlRateProfile->rates [ ] ( what the PID really uses )
 *   A   1 = armed           F   flipState ( 0 = no flip )
 * Markers on their own tick: "LOOP START" ( onLoopStart, also after a Wi-Fi restart ),
 * "STEP <n>" ( the step's setDesiredRate calls ran on that tick ), "STEP 2 SKIPPED ARMED".
 * ---------------------------------------------------------------------------------------------- */

#include "io/rc_controls.h"    // controlRateConfig_t, read-only for the log

extern "C" {
extern controlRateConfig_t *currentControlRateProfile;
extern uint32_t             flipState;
}

static const bool     TEST_SET_RATES = true;    // false = control run
static const uint8_t  RATE_STEPS     = 5;
static const uint8_t  RATE_STEP_RANGE = 2;    // the out-of-range step: disarmed only
static const uint32_t stepStartMs [ RATE_STEPS ] = { 0, 5000, 15000, 20000, 45000 };
static const int32_t  stepRoll [ RATE_STEPS ]    = { 0, 40, 300, 40, 30 };
static const int32_t  stepPitch [ RATE_STEPS ]   = { 0, 50, -5, 50, 40 };
static const int32_t  stepYaw [ RATE_STEPS ]     = { 0, 10, 400, 10, 15 };

static bool     rateSeqStarted = false;
static uint32_t rateSeqStartMs = 0;
static uint8_t  rateStepDone   = 0;      // steps 1..4 run once each; step 0 is plutoInit
static bool     loopStartMark  = false;

static void setRates ( int32_t roll, int32_t pitch, int32_t yaw ) {
  if ( ! TEST_SET_RATES ) return;
  setDesiredRate ( AG_ROLL, roll );
  setDesiredRate ( AG_PITCH, pitch );
  setDesiredRate ( AG_YAW, yaw );
}

/**
 * Configures Pluto's receiver mode.
 * AUX channel configurations for ELRS:
 * ARM mode      : Rx_AUX1, range 1300 to 2100 (2-pos switch)
 * ANGLE mode    : Rx_AUX2, range 1300 to 2100 (3-pos switch: mid+high = ANGLE, low = ACRO)
 * MAG mode      : Rx_AUX3, range 1500 to 2100
 * DEV mode      : Rx_AUX4, range 1500 to 2100
 * ALT HOLD / THROTTLE mode : Rx_AUX5, range 1500 to 2100
 *                            (2-pos switch: low = THROTTLE mode, high = ALT HOLD mode)
 */
void plutoRxConfig ( void ) {
  // Receiver mode: Uncomment one line matching your setup.
  Receiver_Mode ( Rx_ESP );    // Onboard ESP
  // Receiver_Mode ( Rx_CAM );    // WiFi CAMERA
  // Receiver_Mode ( Rx_PPM );    // PPM based
  // Receiver_Mode ( Rx_ELRS );      // ExpressLRS (CRSF) on USART1
}

// The setup function is called once at Pluto's hardware startup
void plutoInit ( void ) {
  setRates ( 40, 50, 10 );    // S0: set once at start-up, while disarmed
}

// The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart ( void ) {
  if ( ! rateSeqStarted ) {
    rateSeqStarted = true;
    rateSeqStartMs = millis ( );
  }
  loopStartMark = true;
}

// The loop function is called in an endless loop
void plutoLoop ( void ) {
  if ( loopStartMark ) {
    Monitor_Println ( "LOOP START" );
    loopStartMark = false;
    return;
  }

  uint32_t elapsed = millis ( ) - rateSeqStartMs;
  uint8_t  next    = ( uint8_t ) ( rateStepDone + 1 );

  if ( next < RATE_STEPS && elapsed >= stepStartMs [ next ] ) {
    // The out-of-range step clamps yaw to 255 ( ~8.6x the stock yaw stick in every mode ), so it only runs
    // on the ground: an early arm skips it instead of flying it.
    if ( next == RATE_STEP_RANGE && FlightStatus_Check ( FS_ARMED ) ) {
      rateStepDone = next;
      Monitor_Println ( "STEP 2 SKIPPED ARMED" );
      return;
    }
    setRates ( stepRoll [ next ], stepPitch [ next ], stepYaw [ next ] );
    rateStepDone = next;
    Monitor_Println ( "STEP ", ( int ) next );
    return;
  }

  Monitor_Print ( "t:", ( int ) millis ( ) );
  Monitor_Print ( " gR:", ( int ) getDesiredRate ( AG_ROLL ) );
  Monitor_Print ( " gP:", ( int ) getDesiredRate ( AG_PITCH ) );
  Monitor_Print ( " gY:", ( int ) getDesiredRate ( AG_YAW ) );
  Monitor_Print ( " lR:", ( int ) currentControlRateProfile->rates [ FD_ROLL ] );
  Monitor_Print ( " lP:", ( int ) currentControlRateProfile->rates [ FD_PITCH ] );
  Monitor_Print ( " lY:", ( int ) currentControlRateProfile->rates [ FD_YAW ] );
  Monitor_Print ( " A:", FlightStatus_Check ( FS_ARMED ) ? 1 : 0 );
  Monitor_Println ( " F:", ( int ) flipState );
}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  // nothing: the timeline keeps running across Developer Mode restarts
}
```
