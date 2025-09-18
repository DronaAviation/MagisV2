/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 MechAsh (j.mechash@gmail.com)                 #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\Motor.h                                                #
 #  Created Date: Tue, 20th May 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 18th Sep 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#ifndef MOTOR_API_H
#define MOTOR_API_H

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef enum motor_direction {
  CLOCK_WISE = 0,
  ANTICLOCK_WISE
} motor_direction_e;

typedef enum bidirectional_motor {
  M1,
  M2,
  M5,
  M6,
  M7,
  M8
} bidirectional_motor_e;




extern bool usingMotorAPI;

void Motor_Init ( bidirectional_motor_e _motor );
void Motor_Set ( bidirectional_motor_e _motor, int16_t pwmValue );
void Motor_SetDir ( bidirectional_motor_e motor, motor_direction_e direction );

#ifdef __cplusplus
}
#endif

#endif
