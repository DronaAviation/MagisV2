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
 #  Last Modified: Mon, 28th Sep 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-09-28	AJ	Raw bus mV / shunt 10 uV reads with a valid flag           #
 #  2026-09-28	AJ	Removed bus_voltage / shunt_voltage shims                  #
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

#define INA219_BUS_OVF_BIT 0x0001U    // Bus voltage register bit 0: math overflow

// Reads one 16-bit register ( MSB first ). Returns false on an I2C error; *val is untouched then.
static bool ina219ReadReg ( uint8_t reg, uint16_t *val ) {
  uint8_t buffer [ 2 ];
  if ( ! i2cRead ( INA219_I2C_ADDRESS, reg, 2, buffer ) ) return false;
  *val = ( uint16_t ) ( ( ( uint16_t ) buffer [ 0 ] << 8 ) | buffer [ 1 ] );
  return true;
}

bool INA219_Config ( uint16_t RST, uint16_t BRNG, uint16_t PG, uint16_t BADC, uint16_t SADC, uint16_t MODE ) {
  uint16_t config = RST | BRNG | PG | BADC | SADC | MODE;
  return INA219_RegWrite ( INA219_REG_CONFIG, config );
}

bool INA219_Init ( void ) {
  // INA219_Config ( INA219_CONFIG_RST_0, INA219_CONFIG_BRNG_16V, INA219_CONFIG_GAIN_4, INA219_CONFIG_BADC ( INA219_CONFIG_xADC_12B ), INA219_CONFIG_SADC ( INA219_CONFIG_xADC_12B ), INA219_CONFIG_MODE ( INA219_CONFIG_MODE_SHUNT_BUS_CNT ) );
  return INA219_Config ( INA219_CONFIG_RST_0, INA219_CONFIG_BRNG_16V, INA219_CONFIG_GAIN_4, INA219_CONFIG_BADC ( INA219_CONFIG_xADC_12B ), INA219_CONFIG_SADC ( INA219_CONFIG_xADC_12B ), INA219_CONFIG_MODE ( INA219_CONFIG_MODE_SHUNT_BUS_CNT ) );
}

bool INA219_ReadBus_mV ( uint16_t *busMv ) {
  uint16_t reg;
  if ( ! ina219ReadReg ( INA219_REG_BUSVOLTAGE, &reg ) ) return false;
  if ( reg & INA219_BUS_OVF_BIT ) return false;
  // Bits 15..3 are the reading, LSB 4 mV; max 8191 * 4 = 32764 mV fits uint16_t
  *busMv = ( uint16_t ) ( ( uint32_t ) ( reg >> 3 ) * 4U );
  return true;
}

bool INA219_ReadShunt_10uV ( int16_t *shunt10uV ) {
  uint16_t reg;
  if ( ! ina219ReadReg ( INA219_REG_SHUNTVOLTAGE, &reg ) ) return false;
  // The part sign-extends the two's complement reading to 16 bits; LSB 10 uV
  *shunt10uV = ( int16_t ) reg;
  return true;
}
