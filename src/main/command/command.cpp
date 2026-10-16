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

#define TAKEOFF_ARMED_WINDOW_MS 3000    // a take-off on an armed drone counts as from the floor this long after arming
#define TAKEOFF_REFUSED_DROP_MS 2000    // ARM switch on but mwArm ( ) refusing this long: the take-off is dropped

bool takeOffFromUser = false;

static bool     takeOffRefused   = false;    // the ARM switch is on and mwArm ( ) refuses
static uint32_t takeOffRefusedAt = 0;        // ms, when the refusal started

// Every end of a take-off ( done, dropped ) goes through here, so nothing of it carries over to the next.
static void endTakeOff ( void ) {
  current_command = NONE;
  command_status  = FINISHED;
  takeOffFromUser = false;
  takeOffRefused  = false;
}

// A queued take-off waits for the ARM switch. One that user code queued must not outlive Dev Mode, or it
// would arm the drone at the next ARM-on with no user code running. The app's take-off is left alone.
void dropUserTakeOff ( void ) {
  if ( current_command == TAKE_OFF && takeOffFromUser ) {
    endTakeOff ( );
  }
}

// Climb goal of a take-off. Laser: an absolute height above the floor. Baro: relative to the current height.
// Only with the stick centred: a stick rate cancels the goal at once and the once-per-arm flag would be
// spent on nothing. A ground idle is released first, or its reset wipes the goal ( altitudehold.cpp ).
static void setTakeOffGoal ( void ) {
  if ( ! altHoldStickCentred ( ) ) {
    return;
  }
  altHoldReleaseGroundIdle ( );
  if ( altLaserFusion ( ) ) {
    setDesiredPosition ( Z, takeOffHeight );
  } else {
    DesiredPosition_setRelative ( Z, takeOffHeight );
  }
  isTakeOffHeightSet = true;
}

// Armed before the take-off ran ( Command_Arm, the ARM switch ): still on the floor just after arming, or
// while the ARM-switch ground idle holds the motors. arm_time is set on the arming edge ( mw.cpp ).
static bool takeOffFromFloor ( void ) {
  return ( millis ( ) - arm_time ) < TAKEOFF_ARMED_WINDOW_MS || isThrottleStickArmed;
}

// ARM switch on and arming refused: start the timer, and drop the take-off once it has run
// TAKEOFF_REFUSED_DROP_MS, so a take-off that cannot arm ( calibrating, low battery ) does not wait to arm the
// drone later. With the battery OK, BMS_Update ( ) re-enables OK_TO_ARM every update, so arming is allowed
// again right after a landing: a re-sent take-off then re-arms at once ( documented, not blocked ).
static void takeOffRefusedTick ( void ) {
  if ( ! takeOffRefused ) {
    takeOffRefused   = true;
    takeOffRefusedAt = millis ( );
  } else if ( ( millis ( ) - takeOffRefusedAt ) >= TAKEOFF_REFUSED_DROP_MS ) {
    endTakeOff ( );
  }
}

void takeOff ( ) {

  if ( command_status != FINISHED ) {

    if ( ARMING_FLAG ( ARMED ) ) {

      // Already armed: set the goal here, once per arm, if the drone is still on the floor. In the air
      // the take-off sets no goal.
      if ( ! isTakeOffHeightSet && takeOffFromFloor ( ) ) {
        setTakeOffGoal ( );
      }

      //            if(takeoffTimer.set(1000, false)){

      endTakeOff ( );
      isTookOff = true;

      //            }

    } else {
      // Arm only with the stick centred: off centre no goal can be set, and the drone would arm with
      // nothing to climb to and follow the stick. Until then it waits, as for the ARM switch.
      if ( IS_RC_MODE_ACTIVE ( BOXARM ) && altHoldStickCentred ( ) ) {

        pidResetErrorAngle ( );
        pidResetErrorGyro ( );
        mwArm ( );
        takeoffTimer.reset ( );
        posSetTimer.reset ( );

        if ( ARMING_FLAG ( ARMED ) ) {
          takeOffRefused = false;
          // mwArm ( ) can refuse ( calibration, battery ). A goal set while disarmed is wiped, and the
          // flag would then block the goal once a later attempt arms.
          if ( ! isTakeOffHeightSet ) {
            setTakeOffGoal ( );
          }
        } else {
          takeOffRefusedTick ( );
        }
      } else if ( IS_RC_MODE_ACTIVE ( BOXARM ) && ! ARMING_FLAG ( OK_TO_ARM ) ) {
        // Off centre, but arming is refused anyway ( OK_TO_ARM is off ): still a refusal, so moving the
        // stick cannot keep the take-off alive past the drop.
        takeOffRefusedTick ( );
      } else {
        takeOffRefused = false;    // waiting for the ARM switch or a centred stick: not a refusal
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
 * land entry the throttle is normally far above LAND_CONFIRM_THROTTLE, so the
 * "velocity is already zero because we have not started moving yet" case cannot
 * fire. Command_Land ( speed ) can start the ramp at or below it ( 105, the
 * default, gives 1200 ), so an arrest is not believed before the ramp has run
 * LAND_CONFIRM_MIN_MS either: by then a real descent has built up ( about
 * -45 cm/s after 0.6 s in flight logs ).
 */
#define LAND_RAMP_RATE        40      // throttle counts shed per second
#define LAND_THROTTLE_MIN     1150    // motors near idle - nothing hovers here
#define LAND_CONFIRM_THROTTLE 1200    // below any plausible in-ground-effect hover
#define LAND_SETTLE_MS        300     // an arrest must persist this long to count
#define LAND_CONFIRM_MIN_MS   1000    // no arrest counts before the ramp has run this long
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
  // below a throttle that could have held the craft up, and has run long enough
  // for a descent to start. The settle timer throws out momentary arrests from
  // gusts or baro noise.
  bool arrested   = ( getEstVelocity ( ) > LAND_VEL_STOP );
  bool conclusive = ( landThrottle <= LAND_CONFIRM_THROTTLE ) && ( elapsed >= LAND_CONFIRM_MIN_MS );

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

  // Any end of a take-off that skips endTakeOff ( ) must not leave the refusal timer running for the next
  // one: an app command or a flip replaces it, and mwDisarm ( ) sets FINISHED but leaves TAKE_OFF in place.
  if ( current_command != TAKE_OFF || command_status == FINISHED ) {
    takeOffRefused = false;
  }

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
