/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                               #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\tof_dev_area.cpp                                   #
 #  Created Date: Mon, 19th Oct 2026                                           #
 #  Brief: The one RAM area shared by the VL53L0X and VL53L1X device tables    #
 #         ( see tof_dev_area.h ).                                             #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 19th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-19	AJ	Created ( vl53l1x-xranging-api t16 )                       #
*******************************************************************************/

#include "platform.h"

#include "drivers/tof_dev_area.h"

// No initializer: zero-filled .bss at startup, no constructor ( -nostartfiles ).
union tofDevArea_u tofDevArea;
