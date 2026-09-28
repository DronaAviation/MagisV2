// Do not remove the include below
#include "PlutoPilot.h"

// TEMPORARY battery-capacity-estimate diagnostic: remove before release ( topic task 9 ).
extern "C" {
extern uint16_t mAmpRaw;              // averaged shunt current, mA ( no gain since task 3 )
extern uint16_t vBatComp;             // compensated battery voltage ( mV ): bus + I x R once R is measured
extern float soc_Fused;               // state of charge: reported remaining / capacity ( % )
extern int16_t motor [];              // motor commands ( us )
extern uint8_t batteryCellCount;      // cells estimated at plug-in
extern uint16_t batteryResistance_mOhm;    // pack resistance measured in flight ( mOhm ), 0 until measured ( battery-soc-fix task 5 )
extern int16_t motor_disarmed [];     // what the mixer sends to the motors while disarmed ( motor test path )
}

// Bench motor sequence for the current-ratio test ( task 12 ): with Dev Mode on and the drone DISARMED, hold the
// four flight motors at idle, 25, 50, 75 and 100 % for BENCH_STEP_S each, then stop. Runs once per Dev Mode start.
// PROPS OFF. Set BENCH_MOTOR_SEQUENCE to 0 before any flight test.
#define BENCH_MOTOR_SEQUENCE 1
#define BENCH_STEP_S         10
#if BENCH_MOTOR_SEQUENCE
  #warning "BENCH_MOTOR_SEQUENCE is on: Dev Mode while disarmed spins the motors. Props off. Set to 0 before flying."
static const int16_t benchLevels []= { 1000, 1250, 1500, 1750, 2000, 1000 };    // last entry: stop
static const int benchLevelCount    = ( int ) ( sizeof ( benchLevels ) / sizeof ( benchLevels [ 0 ] ) );
static bool benchRun                = false;
static int benchPhase               = 0;
static uint32_t benchPhaseStartMs   = 0;

static void benchMotorsSet ( int16_t pwm ) {
  for ( int i = 0; i < 4; i++ ) motor_disarmed [ i ] = pwm;
}

// Returns the phase index to log ( -1 when the sequence is not running ).
static int benchMotorsTick ( void ) {
  if ( ! benchRun ) return -1;
  if ( FlightStatus_Check ( FS_ARMED ) ) {    // never drive motor_disarmed while armed: abort the sequence
    benchMotorsSet ( 1000 );
    benchRun = false;
    return -1;
  }
  const uint32_t nowMs = millis ( );
  if ( benchPhase == 0 && benchPhaseStartMs == 0 ) {
    benchPhaseStartMs = nowMs;
    benchMotorsSet ( benchLevels [ 0 ] );
  } else if ( nowMs - benchPhaseStartMs >= ( uint32_t ) BENCH_STEP_S * 1000U ) {
    benchPhase++;
    benchPhaseStartMs = nowMs;
    if ( benchPhase >= benchLevelCount ) {    // sequence over: motors stopped by the last level
      benchMotorsSet ( 1000 );
      benchRun = false;
      return -1;
    }
  }
  benchMotorsSet ( benchLevels [ benchPhase ] );    // every tick, so no other writer can hold a different level
  return benchPhase;
}
#endif    // BENCH_MOTOR_SEQUENCE

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
static bool printCapacityLine = false;    // onLoopStart and the first plutoLoop share one tick: keep them apart

void onLoopStart ( void ) {
  // do your one time stuffs here
  printCapacityLine = true;
#if BENCH_MOTOR_SEQUENCE
  benchRun          = ! FlightStatus_Check ( FS_ARMED );
  benchPhase        = 0;
  benchPhaseStartMs = 0;
#endif
}

// The loop function is called in an endless loop
void plutoLoop ( void ) {
  if ( printCapacityLine ) {
    printCapacityLine = false;
    Monitor_Print ( "E:", Bms_Get ( Estimated_Capacity ) );
    Monitor_Print ( " Cap:", Bms_Get ( Battery_Capicity ) );
    Monitor_Println ( " Cells:", batteryCellCount );
    return;
  }

  const int motorAvg = ( motor [ 0 ] + motor [ 1 ] + motor [ 2 ] + motor [ 3 ] ) / 4;
#if BENCH_MOTOR_SEQUENCE
  const int benchPh = benchMotorsTick ( );
#else
  const int benchPh = -1;
#endif

  Monitor_Print ( "t:", static_cast< int > ( millis ( ) ) );
#if BENCH_MOTOR_SEQUENCE
  Monitor_Print ( " Ph:", benchPh );    // bench motor phase, -1 when idle
#else
  ( void ) benchPh;
#endif
  Monitor_Print ( " V:", Bms_Get ( Voltage ) );    // exact mV since API 1.4.0 ( task 6 )
  Monitor_Print ( " Vc:", vBatComp );
  Monitor_Print ( " I:", mAmpRaw );
  Monitor_Print ( " R:", batteryResistance_mOhm );    // task 5: pack resistance ( mOhm ), 0 until measured
  Monitor_Print ( " D:", Bms_Get ( mAh_Consumed ) );
  Monitor_Print ( " S:", static_cast< int > ( soc_Fused ) );
  Monitor_Print ( " L:", Bms_Get ( Warning_Level ) );    // task 8: 0 OK, 1 low, 2 critical
  Monitor_Print ( " M:", motorAvg );
  Monitor_Println ( " Arm:", FlightStatus_Check ( FS_ARMED ) ? 1 : 0 );
}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  // do your cleanup stuffs here
#if BENCH_MOTOR_SEQUENCE
  benchMotorsSet ( 1000 );    // Dev Mode off or link lost: motors stop
  benchRun = false;
#endif
}