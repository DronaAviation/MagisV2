/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\Serial-IO-I2C.cpp                                  #
 #  Created Date: Sun, 7th Sep 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 18th Sep 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include "platform.h"

#include "drivers/bus_i2c.h"

#include "API/Serial-IO.h"


bool I2C_Read ( uint8_t device_add, uint8_t reg, uint8_t &value ) {
  return i2cRead ( device_add, reg, 1, &value );
}


int16_t I2C_Read ( uint8_t device_add, uint8_t reg, uint32_t length, uint8_t *buffer ) {
  return i2cRead ( device_add, reg, length, buffer );
}


bool I2C_Write ( uint8_t device_add, uint8_t reg, uint8_t data ) {
  return i2cWrite ( device_add, reg, data );
}


bool I2C_Write ( uint8_t device_add, uint8_t reg, uint32_t length, uint8_t *data ) {
  return i2cWriteBuffer ( device_add, reg, length, data );
}
