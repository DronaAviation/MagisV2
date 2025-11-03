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
 #  Last Modified: Sun, 7th Sep 2025                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
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

void plutoRxConfig ( void );

void plutoInit ( void );

void onLoopStart ( void );

void plutoLoop ( void );

void onLoopFinish ( void );

#endif
