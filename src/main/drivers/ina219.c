/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\ina219.c                                           #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 6th Oct 2025                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#include "ina219.h"

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/system.h"
#include "drivers/bus_i2c.h"

bool INA219_RegWrite ( uint8_t reg, uint16_t val ) {
  uint8_t buffer [ 2 ] = { ( uint8_t ) ( val >> 8 ), ( uint8_t ) ( val & 0xFF ) };
  return i2cWriteBuffer ( INA219_I2C_ADDRESS, reg, 2, buffer );
}

uint16_t INA219_RegRead ( uint8_t reg ) {
  uint8_t buffer [ 2 ];
  if ( i2cRead ( INA219_I2C_ADDRESS, reg, 2, buffer ) ) {
    return ( uint16_t ) ( buffer [ 0 ] << 8 ) | buffer [ 1 ];
  }
  return 0xFFFF;    // Error case
}

bool INA219_Config ( uint16_t RST, uint16_t BRNG, uint16_t PG, uint16_t BADC, uint16_t SADC, uint16_t MODE ) {
  uint16_t config = RST | BRNG | PG | BADC | SADC | MODE;
  return INA219_RegWrite ( INA219_REG_CONFIG, config );
}

bool INA219_Init ( void ) {
  // INA219_Config ( INA219_CONFIG_RST_0, INA219_CONFIG_BRNG_16V, INA219_CONFIG_GAIN_4, INA219_CONFIG_BADC ( INA219_CONFIG_xADC_12B ), INA219_CONFIG_SADC ( INA219_CONFIG_xADC_12B ), INA219_CONFIG_MODE ( INA219_CONFIG_MODE_SHUNT_BUS_CNT ) );
  return INA219_Config ( INA219_CONFIG_RST_0, INA219_CONFIG_BRNG_16V, INA219_CONFIG_GAIN_4, INA219_CONFIG_BADC ( INA219_CONFIG_xADC_12B ), INA219_CONFIG_SADC ( INA219_CONFIG_xADC_12B ), INA219_CONFIG_MODE ( INA219_CONFIG_MODE_SHUNT_BUS_CNT ) );
}

uint16_t bus_voltage ( void ) {
  uint16_t busVoltageReg = INA219_RegRead ( INA219_REG_BUSVOLTAGE );
  if ( busVoltageReg & 0x01 ) return 0xFFFF;
  return ( ( busVoltageReg >> 3 ) * 4 ) / 100;    // Now returning in millivolts
}

int16_t shunt_voltage ( void ) {
  uint16_t raw      = INA219_RegRead ( INA219_REG_SHUNTVOLTAGE );
  int16_t signedRaw = ( int16_t ) raw;    // sign-extend
  // LSB = 10 µV → return in mV
  return ( signedRaw * 10 ) / 1000;
}

/* void configureINA219 ( ) {
  uint8_t configRegister = INA219_REG_CONFIG;
  uint16_t configValue   = Ina219Config;

  // Split the 16-bit configuration value into two bytes
  uint8_t configData [ 2 ];
  configData [ 0 ] = ( configValue >> 8 ) & 0xFF;    // MSB
  configData [ 1 ] = configValue & 0xFF;             // LSB

  // Send the configuration to the INA219
  bool success = i2cWriteBuffer ( INA219_I2C_ADDRESS, configRegister, 2, configData );

  if ( success ) {
    // Configuration was successful
  } else {
    // Handle error
  }
} */