/*******************************************************************************
 #  Copyright (c) 2025 DRONA AVIATION                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2-MechAsh-Dev                                               #
 #  File: \RxConfig.h                                                          #
 #  Created Date: Tue, 16th Jan 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 29th Apr 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#ifndef RX_CONFIG_H
#define RX_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
typedef enum {
  Rx_ESP,
  Rx_PPM,
  Rx_CAM
} rx_mode_e;

typedef enum {
  Mode_ARM,
  Mode_ANGLE,
  Mode_BARO,
  Mode_MAG,
  Mode_HEADFREE
} flight_mode;

typedef enum {
  Rx_AUX1 = 4,
  Rx_AUX2,
  Rx_AUX3,
  Rx_AUX4,
  Rx_AUX5,
} rx_channel_e;

extern uint8_t DevModeAUX;
extern uint16_t DevModeMinRange;
extern uint16_t DevModeMaxRange;

void Receiver_Mode ( rx_mode_e rxMode );
void Receiver_Config_Arm ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
void Receiver_Config_Mode_Angle ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
void Receiver_Config_Mode_Baro ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
void Receiver_Config_Mode_Mag ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
void Receiver_Config_Mode_HeadFree ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
void Receiver_Config_Mode_Dev ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );

void STM_PB3_ESP_IO14_Init ( void );

#ifdef __cplusplus
}
#endif
#endif
