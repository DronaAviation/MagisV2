# Set desired angle roll / pitch - Test code

[README](README.md) · [TASKS](TASKS.md) · [TESTING](TESTING.md)

The `PlutoPilot.cpp` used for log-1 ( pre-fix ) and log-2 ( post-fix ), kept here because it was never committed: the
commit restored the stock `PlutoPilot.cpp`. To re-run the T11 tilt sequence, copy the code below over `PlutoPilot.cpp`,
build, and follow the procedure in [TESTING.md](TESTING.md). It tilts the drone automatically whenever Developer Mode is
switched on, so fly it only for this test.

```cpp
// Do not remove the include below
#include "PlutoPilot.h"

/* ------------------------------------------------------------------------------------------------
 * TEMPORARY TEST CODE: set-desired-angle-roll-pitch, task 1 ( API test T11 / B4 ).
 * Remove before release ( flagged in the topic README ).
 *
 * When Developer Mode turns on, an automatic sequence commands small roll / pitch tilts with
 * setDesiredAngle ( ), one call per step ( so the fixed firmware is also tested for "hold until
 * replaced" ): 2 s level, roll +5 deg, level, roll -5 deg, level, pitch +5 deg, level, pitch -5 deg,
 * level, 1 s each, then DONE ( level ). Fly in ANGLE mode ( ALT_HOLD ), hovering, before Dev Mode on.
 *
 * Log, 10 Hz ( default user loop ), ~101 B per tick ( worst case ~105 B, inside the ~130 B budget ):
 *   t  millis ( )               n  plutoLoop tick count
 *   s  step index ( 9 = done )  cR / cP  commanded roll / pitch, deci-degrees
 *   R / P  Estimate_Get ( Angle ) roll / pitch, deci-degrees ( read after the setter call )
 *   H  heading, degrees
 * Markers on their own tick: "TILT START" ( a new sequence ), "TILT RESUME" ( a Wi-Fi restart of
 * the user code within 3 s: the sequence keeps its timeline ), "TILT DONE".
 * ---------------------------------------------------------------------------------------------- */

typedef struct {
  uint16_t durationMs;
  int16_t  roll;     // deci-degrees
  int16_t  pitch;    // deci-degrees
} tiltStep_t;

static const tiltStep_t tiltSeq [ ] = {
  { 2000, 0, 0 },      // 0 level
  { 1000, 50, 0 },     // 1 roll right 5 deg
  { 1000, 0, 0 },      // 2 level
  { 1000, -50, 0 },    // 3 roll left 5 deg
  { 1000, 0, 0 },      // 4 level
  { 1000, 0, 50 },     // 5 pitch +5 deg
  { 1000, 0, 0 },      // 6 level
  { 1000, 0, -50 },    // 7 pitch -5 deg
  { 1000, 0, 0 },      // 8 level
};

static const uint8_t  TILT_STEPS            = ( uint8_t ) ( sizeof ( tiltSeq ) / sizeof ( tiltSeq [ 0 ] ) );
// A Wi-Fi stall of 1-2 s is normal and the RC-loss landing starts at ~2.5 s, so a gap under 3 s is
// treated as a flicker and the sequence resumes; a longer one starts a new sequence.
static const uint32_t TILT_RESTART_GAP_MS   = 3000;
static const uint8_t  TILT_STEP_NONE        = 0xFF;

static bool     tiltSeqRunning  = false;
static uint32_t tiltSeqStartMs  = 0;
static uint32_t tiltLastStopMs  = 0;
static uint8_t  tiltApplied     = TILT_STEP_NONE;    // step whose command was last sent
static uint8_t  tiltMarker      = 0;                 // 1 START, 2 RESUME, 3 DONE: printed on its own tick
static uint32_t tiltTick        = 0;
static int16_t  tiltCmdRoll     = 0;
static int16_t  tiltCmdPitch    = 0;

// Step index for a time since the sequence start; TILT_STEPS once the sequence is over.
static uint8_t tiltStepAt ( uint32_t elapsedMs ) {
  uint32_t end = 0;
  for ( uint8_t i = 0; i < TILT_STEPS; i++ ) {
    end += tiltSeq [ i ].durationMs;
    if ( elapsedMs < end ) return i;
  }
  return TILT_STEPS;
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
  // Add your hardware initialization code here
}

// The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart ( void ) {
  uint32_t now = millis ( );

  if ( ! tiltSeqRunning || ( now - tiltLastStopMs ) > TILT_RESTART_GAP_MS ) {
    tiltSeqRunning = true;
    tiltSeqStartMs = now;
    tiltApplied    = TILT_STEP_NONE;
    tiltTick       = 0;
    tiltMarker     = 1;
  } else if ( tiltMarker == 0 ) {
    // Wi-Fi restart of the user code: keep the timeline and the last command. The firmware keeps the
    // held tilt through a flicker ( the dev AUX channel holds its value, so devSwitchOn stays true and
    // the clear in userCode ( ) does not fire ), so the step does not need re-sending. A marker still
    // waiting to be printed ( DONE ) is kept, so the restart cannot swallow it.
    tiltMarker = 2;
  }
}

// The loop function is called in an endless loop
void plutoLoop ( void ) {
  tiltTick++;

  if ( tiltMarker != 0 ) {
    Monitor_Println ( tiltMarker == 1 ? "TILT START" : ( tiltMarker == 2 ? "TILT RESUME" : "TILT DONE" ) );
    tiltMarker = 0;
    return;
  }

  uint32_t now  = millis ( );
  uint8_t  step = tiltStepAt ( now - tiltSeqStartMs );

  if ( step != tiltApplied ) {
    tiltCmdRoll  = ( step < TILT_STEPS ) ? tiltSeq [ step ].roll : ( int16_t ) 0;
    tiltCmdPitch = ( step < TILT_STEPS ) ? tiltSeq [ step ].pitch : ( int16_t ) 0;
    setDesiredAngle ( AG_ROLL, tiltCmdRoll );
    setDesiredAngle ( AG_PITCH, tiltCmdPitch );
    tiltApplied = step;
    if ( step >= TILT_STEPS ) tiltMarker = 3;
  }

  Monitor_Print ( "t:", ( int ) now );
  Monitor_Print ( " n:", ( int ) tiltTick );
  Monitor_Print ( " s:", ( int ) step );
  Monitor_Print ( " cR:", ( int ) tiltCmdRoll );
  Monitor_Print ( " cP:", ( int ) tiltCmdPitch );
  Monitor_Print ( " R:", ( int ) Estimate_Get ( Angle, AG_ROLL ) );
  Monitor_Print ( " P:", ( int ) Estimate_Get ( Angle, AG_PITCH ) );
  Monitor_Println ( " H:", ( int ) Estimate_Get ( Angle, AG_YAW ) );
}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  tiltLastStopMs = millis ( );
}

```
