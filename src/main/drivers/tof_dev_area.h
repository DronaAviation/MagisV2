/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                               #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\tof_dev_area.h                                     #
 #  Created Date: Mon, 19th Oct 2026                                           #
 #  Brief: One RAM area for the ST device structs of both laser families: the  #
 #         VL53L0X table ( tof_manager ) and the VL53L1X table                 #
 #         ( tof_manager_l1x ). Only the latched family runs in a boot.        #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 19th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-10-19	AJ	Created ( vl53l1x-xranging-api t16 )                       #
*******************************************************************************/

#pragma once

// Both ST device types in one header: the two APIs use different prefixes ( VL53L0X_ / VL53L1_ ) and their
// headers define no common macro, so they coexist ( no redefinition diagnostic in either include order ).
#include "vl53l0x_platform.h"             // VL53L0X_Dev_t
#include "vl53l1_platform_user_data.h"    // VL53L1_Dev_t

#include "drivers/tof_manager.h"    // TOF_COUNT

#ifdef __cplusplus
extern "C" {
#endif

// Only one laser family runs per boot ( rangingFamily ( ), API/API-Utils.h ): VL53L0X uses .l0x, VL53L1X uses
// .l1x, NONE and BOTH use neither. Each manager reads and writes only its own member, so no member is ever read
// through the other's type, and the running family's member is still the startup-zeroed .bss it assumes.
// Every access sits behind a successful bring-up of that family ( tofManagerBringUp / tofL1xBringUp refuse any
// other family ); the polls and start-sides touch an entry only while its status is OK, which only that bring-up sets.
union tofDevArea_u {
  VL53L0X_Dev_t l0x [ TOF_COUNT ];    // tof_manager.cpp only
  VL53L1_Dev_t l1x [ TOF_COUNT ];     // tof_manager_l1x.cpp only
};

extern union tofDevArea_u tofDevArea;    // drivers/tof_dev_area.cpp ( .bss )

#ifdef __cplusplus
}
#endif
