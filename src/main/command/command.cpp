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

#include "platform.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/runtime_config.h"

#include "drivers/system.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"

#include "io/display.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/acrobats.h"
#include "flight/posEstimate.h"
#include "flight/posControl.h"

#include "command/command.h"

#include "API/FC-Config.h"
#include "API/Scheduler-Timer.h"
#include "mw.h"

uint8_t current_command = 0;
uint8_t command_status  = 2;

bool isLanding          = false;
bool isArmed            = false;
bool setTakeOffAlt      = false;
bool setTakeOffThrottle = false;
bool setLandTimer       = true;
bool setTakeOffTimer    = true;
bool isTookOff          = false;
bool isTakeOffHeightSet = false;

uint32_t landStartTime;
uint32_t takeOffLoopTime;
int32_t takeOffThrottle = 950;
int8_t checkVelocity    = -8;

uint16_t takeOffHeight = 120;
uint16_t landThrottle  = 1300; //! NEW - 1200 old value
bool isUserLandCommand = false;

Interval takeoffTimer;
Interval posSetTimer;

void takeOff ( ) {

  if ( command_status != FINISHED ) {

    if ( ARMING_FLAG ( ARMED ) ) {

      //            if(takeoffTimer.set(1000, false)){

      current_command = NONE;
      command_status  = FINISHED;
      isTookOff       = true;

      //            }

    } else {
      if ( IS_RC_MODE_ACTIVE ( BOXARM ) ) {

        pidResetErrorAngle ( );
        pidResetErrorGyro ( );
        mwArm ( );
        takeoffTimer.reset ( );
        posSetTimer.reset ( );

#ifdef LASER_ALT
        if ( ! isTakeOffHeightSet ) {
          setDesiredPosition ( Z, takeOffHeight );
          isTakeOffHeightSet = true;
        }
#else
        if ( ! isTakeOffHeightSet ) {
          DesiredPosition_setRelative ( Z, takeOffHeight );
          isTakeOffHeightSet = true;
        }
#endif
      }
    }
  }
}
/*
 * Landing is open loop - the craft is flown down on a commanded throttle and
 * the touchdown is inferred, since there is no downward rangefinder in the
 * default build. A stopped descent on its own cannot tell "resting on the
 * floor" apart from "held up by ground effect": in both cases thrust balances
 * weight and the vertical velocity is zero.
 *
 * Prop size decides whether that ambiguity bites. Thrust augmentation near a
 * surface goes roughly as 1 / ( 1 - ( R / 4z )^2 ), so at 3 cm a 110 mm rotor
 * gains about 27 % against roughly 5 % for a 55 mm one - enough to turn a
 * fixed 1300 from a descent throttle into a hover throttle a few centimetres
 * off the ground. The craft then parks there and the old velocity-only test
 * read the arrest as a landing and disarmed in mid-air.
 *
 * So the descent throttle is bled down for the whole descent, and an arrest is
 * only believed once the ramp has passed below a throttle at which hovering is
 * impossible. The ramp doubles as the probe that separates the two states: in
 * ground effect less thrust means the descent resumes, while on the floor
 * nothing happens. Still not descending at LAND_CONFIRM_THROTTLE therefore
 * means something other than the rotors is carrying the weight.
 *
 * The ramp also removes the need for the old two-second blanking window - at
 * land entry the throttle is far above LAND_CONFIRM_THROTTLE, so the "velocity
 * is already zero because we have not started moving yet" case cannot fire.
 */
#define LAND_RAMP_RATE        40      // throttle counts shed per second
#define LAND_THROTTLE_MIN     1150    // motors near idle - nothing hovers here
#define LAND_CONFIRM_THROTTLE 1200    // below any plausible in-ground-effect hover
#define LAND_SETTLE_MS        300     // an arrest must persist this long to count
#define LAND_IMPACT_ACC       6000    // ~1.46 G with acc_1G = 4096 - firm contact
#define LAND_VEL_STOP         ( -8 )  // cm/s - slower descent counts as arrested
#define LAND_TIMEOUT_MS       30000   // backstop if nothing else ever fires

static uint16_t landThrottleStart = 1300;
static uint32_t landSettleStart   = 0;

static void finishLanding ( void ) {

  command_status = FINISHED;

  mwDisarm ( );

  isLanding         = false;
  setLandTimer      = true;
  landSettleStart   = 0;
  isUserLandCommand = false;
}

void land ( ) {

  if ( command_status == FINISHED )
    return;

  isLanding = true;

  if ( setLandTimer ) {

    landStartTime = millis ( );
    // mwDisarm ( ) restores landThrottle to its default, so the value standing
    // at land entry - the default, or whatever Command_Land ( ) asked for - is
    // the top of the ramp.
    landThrottleStart = landThrottle;
    landSettleStart   = 0;
    setLandTimer      = false;
  }

  uint32_t elapsed = millis ( ) - landStartTime;

  // Bleed the descent throttle down, so ground effect can delay the descent but
  // never stop it. Never ramp up past what was asked for, and never below idle.
  int32_t rampFloor = ( ( int32_t ) landThrottleStart < LAND_THROTTLE_MIN ) ? ( int32_t ) landThrottleStart : LAND_THROTTLE_MIN;
  int32_t ramped    = ( int32_t ) landThrottleStart - ( int32_t ) ( ( LAND_RAMP_RATE * elapsed ) / 1000 );

  landThrottle = ( uint16_t ) constrain ( ramped, rampFloor, ( int32_t ) landThrottleStart );

  if ( elapsed >= LAND_TIMEOUT_MS ) {
    finishLanding ( );
    return;
  }

  // Firm contact needs no corroboration.
  if ( ABS ( accADC [ 2 ] ) > LAND_IMPACT_ACC ) {
    finishLanding ( );
    return;
  }

  // Confirmed touchdown: the descent has stopped AND the ramp has already gone
  // below a throttle that could have held the craft up. The settle timer throws
  // out momentary arrests from gusts or baro noise.
  bool arrested   = ( getEstVelocity ( ) > LAND_VEL_STOP );
  bool conclusive = ( landThrottle <= LAND_CONFIRM_THROTTLE );

  if ( arrested && conclusive ) {

    if ( landSettleStart == 0 ) {
      landSettleStart = millis ( );
    } else if ( ( millis ( ) - landSettleStart ) >= LAND_SETTLE_MS ) {
      finishLanding ( );
    }

  } else {
    landSettleStart = 0;
  }
}

void executeCommand ( ) {

  switch ( current_command ) {

    case NONE:

      break;

    case TAKE_OFF:

      takeOff ( );

      break;

    case LAND:

      land ( );
      break;

    case B_FLIP:

      if ( flipState == 0 && FLIGHT_MODE ( MAG_MODE ) ) {

        flipDirection = 0;
        flipState     = 1;
        flipStartTime = millis ( );
      }
      current_command = NONE;
      command_status  = FINISHED;

      break;

    case F_FLIP:

      break;

    case R_FLIP:

      break;

    case L_FLIP:

      break;

    default:
      break;
  }
}

void updateCommandStatus ( ) {

  if ( current_command != NONE && command_status == FINISHED ) {

    current_command = NONE;

    command_status = FINISHED;
  }
}
