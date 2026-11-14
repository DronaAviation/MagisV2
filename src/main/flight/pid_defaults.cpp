/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  SPDX-FileCopyrightText: 2026 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\flight\pid_defaults.cpp                                    #
 #  Created Date: Sat, 14th Nov 2026                                           #
 #  Brief: Firmware default PID gains ( P8 / I8 / D8 ), the single source      #
 #         shared by the config reset and setDefault_PIDProfile ( ).           #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sat, 14th Nov 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  14-11-2026	AJ	Moved out of resetPidProfile ( ) in config.cpp, values    #
 #              	unchanged ( set-default-pid-profile ).                     #
*******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/axis.h"

#include "flight/pid.h"

// Default gains for every pidIndex_e slot. Only the integer gains live here:
// the controller type, the float gains, the cut frequencies and the GTUNE limits
// stay in resetPidProfile ( ), because an API gain reset must not touch them.
void pidProfileDefaultGains ( pidProfile_t *pidProfile ) {
  pidProfile->P8 [ PIDROLL ]  = 40;     // 38; //40; //40 10 30
  pidProfile->I8 [ PIDROLL ]  = 10;     // 35; //30;
  pidProfile->D8 [ PIDROLL ]  = 30;     // 23;
  pidProfile->P8 [ PIDPITCH ] = 40;     // 38; //40; //40 10 30
  pidProfile->I8 [ PIDPITCH ] = 10;     // 35; //30;
  pidProfile->D8 [ PIDPITCH ] = 30;     // 23;
  pidProfile->P8 [ PIDYAW ]   = 150;    // 85;
  pidProfile->I8 [ PIDYAW ]   = 70;     // 45;
  pidProfile->D8 [ PIDYAW ]   = 50;     // 0;
  // Outer altitude loop is P-only; PIDVEL holds the integrator ( an I here would
  // wind up against throttle saturation ). P8 = 128 is unity: a 10 cm error asks
  // for 10 cm/s ( setVel = P8 * error / 128 ). The old 100 left small sags inside
  // the VelocityZ noise, uncorrected.
  pidProfile->P8 [ PIDALT ]   = 128;
  pidProfile->I8 [ PIDALT ]   = 0;
  pidProfile->D8 [ PIDALT ]   = 30;    // 0;

  pidProfile->P8 [ PIDPOS ] = 80;    // POSHOLD_P * 100;
  pidProfile->I8 [ PIDPOS ] = 0;     // POSHOLD_I * 100;
  pidProfile->D8 [ PIDPOS ] = 0;

  pidProfile->P8 [ PIDPOSR ] = 30;    // POSHOLD_RATE_P * 10;
  pidProfile->I8 [ PIDPOSR ] = 2;     // POSHOLD_RATE_I * 100;
  pidProfile->D8 [ PIDPOSR ] = 5;     // POSHOLD_RATE_D * 1000;

  pidProfile->P8 [ PIDNAVR ] = 25;    // NAV_P * 10;
  pidProfile->I8 [ PIDNAVR ] = 33;    // NAV_I * 100;
  pidProfile->D8 [ PIDNAVR ] = 83;    // NAV_D * 1000;

  pidProfile->P8 [ PIDLEVEL ] = 40;    // 90;
  pidProfile->I8 [ PIDLEVEL ] = 10;
  pidProfile->D8 [ PIDLEVEL ] = 100;
  pidProfile->P8 [ PIDMAG ]   = 40;
  pidProfile->P8 [ PIDVEL ]   = 120;    // 120  //40
  pidProfile->I8 [ PIDVEL ]   = 45;     // 45    //20
  pidProfile->D8 [ PIDVEL ]   = 1;      // 1      //0

  pidProfile->P8 [ PIDUSER ] = 60;
  pidProfile->I8 [ PIDUSER ] = 30;
  pidProfile->D8 [ PIDUSER ] = 5;
}
