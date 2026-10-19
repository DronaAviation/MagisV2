// Do not remove the include below
#include "PlutoPilot.h"

/*
 * cg-offset-takeoff-tilt, task 2 - diagnostic log for the off-centre payload lean. Sends no commands: take off,
 * descend and land from the app as usual ( TESTING.md ). Diagnostic only: restore the template PlutoPilot.cpp
 * before committing.
 *
 * Log, one line per user tick ( ~115 B, budget 130 ):
 *   t   millis ( )
 *   F   flags, read as 4 digits: armed, ALT HOLD, isLanding, isThrottleStickArmed ( ARM-switch ground idle )
 *   Ir  roll rate integrator, axisPID units ( pidGetGyroITerm, ±256 )
 *   Ip  pitch rate integrator, axisPID units ( a payload at the front needs a steady pitch torque here )
 *   Rs  pidResetErrorGyro ( ) calls since power-on: steps at every arm and on every RX frame with the
 *       raw throttle below mincheck 1100
 *   Tp  pilot throttle stick rcDataPilot, us
 *   R   roll angle, decidegrees              P   pitch angle, decidegrees
 *   Ze  estimated altitude, cm
 *   S   printed on its own tick when onLoopStart ( ) ran: ms since power-on, with A:1 if armed
 *       ( a Dev Mode restart on a Wi-Fi flicker shows here too )
 */

extern "C" {
uint32_t millis ( void );
int16_t  pidGetGyroITerm ( uint8_t axis );    // flight/pid.h
uint16_t pidGetGyroResetCount ( void );       // flight/pid.h
extern bool    isThrottleStickArmed;          // io/rc_controls.h: set only by the ARM-switch arming path
extern int16_t rcDataPilot [ ];               // rx/rx.h: the pilot's sticks, before user overrides
}
extern bool isLanding;    // command/command.h

static bool     startMarker = false;
static uint32_t startAt     = 0;
static bool     startArmed  = false;

void plutoRxConfig ( void ) {
  Receiver_Mode ( Rx_ESP );    // Onboard ESP
}

void plutoInit ( void ) {
}

void onLoopStart ( void ) {
  startAt     = millis ( );
  startArmed  = FlightStatus_Check ( FS_ARMED );
  startMarker = true;
}

void plutoLoop ( void ) {
  if ( startMarker ) {
    startMarker = false;
    Monitor_Print ( "S:", ( int ) startAt );
    Monitor_Println ( " A:", startArmed ? 1 : 0 );
    return;
  }

  int flags = ( FlightStatus_Check ( FS_ARMED ) ? 1000 : 0 ) + ( FlightMode_Check ( ATLTITUDEHOLD ) ? 100 : 0 )
              + ( isLanding ? 10 : 0 ) + ( isThrottleStickArmed ? 1 : 0 );

  Monitor_Print ( "t:", ( int ) millis ( ) );
  Monitor_Print ( " F:", flags );
  Monitor_Print ( " Ir:", ( int ) pidGetGyroITerm ( 0 ) );    // ROLL
  Monitor_Print ( " Ip:", ( int ) pidGetGyroITerm ( 1 ) );    // PITCH
  Monitor_Print ( " Rs:", ( int ) pidGetGyroResetCount ( ) );
  Monitor_Print ( " Tp:", ( int ) rcDataPilot [ RC_THROTTLE ] );
  Monitor_Print ( " R:", ( int ) Estimate_Get ( Angle, AG_ROLL ) );
  Monitor_Print ( " P:", ( int ) Estimate_Get ( Angle, AG_PITCH ) );
  Monitor_Println ( " Ze:", ( int ) Estimate_Get ( Position, Z ) );
}

void onLoopFinish ( void ) {
}
