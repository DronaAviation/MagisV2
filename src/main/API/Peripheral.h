/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\Peripheral.h                                           #
 #  Created Date: Thu, 8th May 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sat, 6th Sep 2025                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#ifndef API_PERIPHERAL_H
#define API_PERIPHERAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cstdint>
#include "Specifiers.h"

#ifdef __cplusplus
extern "C" {
#endif

extern bool gpioReset;
extern bool changeAdress;





class I2C_P {
 public:
  bool read ( uint8_t device_add, uint8_t reg, uint8_t &value );

  int16_t read ( uint8_t device_add, uint8_t reg, uint32_t length, uint8_t *buffer );

  bool write ( uint8_t device_add, uint8_t reg, uint8_t data );

  bool write ( uint8_t device_add, uint8_t reg, uint32_t length, uint8_t *data );
};



/*
 MODE 	CPOL	CPHA			Data Captured 		Output
  0		Low		0 i.e.edge1		Rising edge			Falling	edge
  1		Low		1 i.e.edge2		Falling	edge		Rising edge
  2		High	0				Falling	edge		Rising edge
  3		High	1				Rising edge			Falling	edge
*/

typedef enum SPImode_s {
  MODE0 = 0,    //<SPI mode 1
  MODE1,        //<SPI mode 2
  MODE2,        //<SPI mode 3
  MODE3         //<SPI mode 4
} SPImode_t;

typedef enum SPIfirst_bit_s {
  LSBFIRST = 0,    //<Specifies that data transfer starts from LSB bit.
  MSBFIRST         //<Specifies that data transfer starts from MSB bit.
} SPIfirst_bit_t;

class SPI_P {
 public:
  void init ( );

  void init ( SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit );

  void enable ( void );

  void disable ( void );

  uint8_t read ( uint8_t register_address );

  void read ( uint8_t register_address, int16_t length, uint8_t *buffer );

  void write ( uint8_t register_address, uint8_t data );
};



extern I2C_P I2C;

extern SPI_P SPI;


#ifdef __cplusplus
}
#endif


#endif
