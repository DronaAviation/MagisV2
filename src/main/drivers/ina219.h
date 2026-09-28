/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\ina219.h                                           #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 28th Sep 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-09-28	AJ	Added INA219_ReadBus_mV / INA219_ReadShunt_10uV            #
 #  2026-09-28	AJ	Removed bus_voltage / shunt_voltage shims                  #
*******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define INA219_I2C_ADDRESS        0x40    //< Default I2C address
#define INA219_SHUNT_RESISTOR     0.02f     //< Default shunt resistor in Ohm

#define INA219_REG_CONFIG         0x00    //< Config register
#define INA219_REG_SHUNTVOLTAGE   0x01    //< Shunt/voltage register
#define INA219_REG_BUSVOLTAGE     0x02    //< Bus voltage register
#define INA219_REG_POWER          0x03    //< Power register
#define INA219_REG_CURRENT        0x04    //< Current register
#define INA219_REG_CALIBRATION    0x05    //< Calibration register

#define INA219_CONFIG_RST_0       ( 0 << 15 )
#define INA219_CONFIG_RST_1       ( 1 << 15 )

#define INA219_CONFIG_BRNG_16V    ( 0 << 13 )    //< Bus voltage range 16V
#define INA219_CONFIG_BRNG_32V    ( 1 << 13 )    //< Bus voltage range 32V

#define INA219_CONFIG_GAIN_1      ( 0 << 11 )    //< ±40 mV
#define INA219_CONFIG_GAIN_2      ( 1 << 11 )    //< ±80 mV
#define INA219_CONFIG_GAIN_4      ( 2 << 11 )    //< ±160 mV
#define INA219_CONFIG_GAIN_8      ( 3 << 11 )    //< ±320 mV

#define INA219_CONFIG_BADC( adc ) ( adc << 7 )    //< Bus ADC mask and shift

#define INA219_CONFIG_SADC( adc ) ( adc << 3 )    //< Shunt ADC mask and shift

#define INA219_CONFIG_xADC_9B     0     //< 9 bit
#define INA219_CONFIG_xADC_10B    1     //< 10 bit
#define INA219_CONFIG_xADC_11B    2     //< 11 bit
#define INA219_CONFIG_xADC_12B    3     //< 12 bit
#define INA219_CONFIG_xADC_2S     9     //< 2 samples
#define INA219_CONFIG_xADC_4S     10    //< 4 samples
#define INA219_CONFIG_xADC_8S     11    //< 8 samples
#define INA219_CONFIG_xADC_16S    12    //< 16 samples
#define INA219_CONFIG_xADC_32S    13    //< 32 samples
#define INA219_CONFIG_xADC_64S    14    //< 64 samples
#define INA219_CONFIG_xADC_128S   15    //< 128 samples

//< Operating Mode
#define INA219_CONFIG_MODE( mode )       ( mode << 0 )    //< Config mode mask and shift
#define INA219_CONFIG_MODE_POWER_DOWN    0                //< Power-Down
#define INA219_CONFIG_MODE_SHUNT_TRG     1                //< Shunt voltage, triggered
#define INA219_CONFIG_MODE_BUS_TRG       2                //< Bus voltage, triggered
#define INA219_CONFIG_MODE_SHUNT_BUS_TRG 3                //< Shunt and bus voltage, triggered
#define INA219_CONFIG_MODE_ADC_OFF       4                //< ADC off (disabled)
#define INA219_CONFIG_MODE_SHUNT_CNT     5                //< Shunt voltage, continuous
#define INA219_CONFIG_MODE_BUS_CNT       6                //< Bus voltage, continuous
#define INA219_CONFIG_MODE_SHUNT_BUS_CNT 7

// #define INA219_SHUNT_RESISTOR_MILLIOHM   0.002f    // 2 mΩ for PE1206FRE470R02L

/*!
 * \brief Default config register value
 * \details
 *    Shunt and bus:
 *      BADC: +/-320 mV
 *      Continuous conversion
 *      532 us conversion time
 */
// #define REG_CONFIG_VALUE 0x399F

// class INA219 {
//  private:
//   void clear ( );

//   uint8_t _i2cAddress;     //< I2C address
//   uint8_t _i2cStatus;
//   float _shuntResistor;    //< Shunt resistor in Ohm
//   bool _powerDown;         //< Power down state

//  public:
//   INA219 ( uint8_t i2cAddress = INA219_I2C_ADDRESS, float shuntResistor = INA219_SHUNT_RESISTOR );

//   // INA219 functions
//   bool begin ( );
//   bool powerDown ( );
//   bool powerUp ( );
//   bool read ( );

//   // Debug functions
//   void dumpRegisters ( Stream *serial );

//   // I2C register access
//   void registerWrite ( uint8_t reg, uint16_t val );
//   uint16_t registerRead ( uint8_t reg );
//   uint8_t getI2CStatus ( );

//   float busVoltage;      //< Bus voltage in V
//   float shuntVoltage;    //< Shunt voltage in mV
//   float current;         //< Current in mA
//   float power;           //< Power in mW
//   bool overflow;         //< Overflow
//   bool available;        //< Successful conversion
// };

bool INA219_Init ( void );

/*!
 * \brief Bus voltage in mV ( register bits 15..3 x 4 mV LSB, 0..32764 mV ).
 * \return false on an I2C error or when the OVF bit ( bit 0 ) is set; *busMv is untouched then.
 */
bool INA219_ReadBus_mV ( uint16_t *busMv );

/*!
 * \brief Shunt voltage, signed, in the native 10 uV LSB ( sign-extended register ).
 *        With PGA /4 ( +/-160 mV ) the range is +/-16000 ( x 10 uV ).
 * \return false on an I2C error; *shunt10uV is untouched then. 0xFFFF ( -10 uV ) is a valid
 *         reading, which is why the error is reported separately.
 */
bool INA219_ReadShunt_10uV ( int16_t *shunt10uV );

#ifdef __cplusplus
}
#endif
