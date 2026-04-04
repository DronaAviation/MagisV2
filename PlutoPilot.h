/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\PlutoPilot.h                                           #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Fri, 4th Apr 2026                                           #
 #  Modified By: techsavvyomi                                                  #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-04-04	techsavvyomi	Added RGB-LED.h include for WS2812B API        #
*******************************************************************************/

#ifndef _PlutoPilot_H_
#define _PlutoPilot_H_

#include "src/main/API/RxConfig.h"
#include "src/main/API/Peripherals.h"
#include "src/main/API/Status-LED.h"
#include "src/main/API/Motor.h"
#include "src/main/API/BMS.h"
#include "src/main/API/FC-Data.h"
#include "src/main/API/RC-Interface.h"
#include "src/main/API/FC-Control.h"
#include "src/main/API/FC-Config.h"
#include "src/main/API/Scheduler-Timer.h"
#include "src/main/API/Debugging.h"
#include "src/main/API/Serial-IO.h"
#include "src/main/API/XRanging.h"
#include "src/main/API/RGB-LED.h"

void plutoRxConfig ( void );

void plutoInit ( void );

void onLoopStart ( void );

void plutoLoop ( void );

void onLoopFinish ( void );

#endif
