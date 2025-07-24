/*
 * This file is part of Cleanflight and Magis.
 *
 * Cleanflight and Magis are free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight and Magis are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "barometer.h"

#include "gpio.h"
#include "system.h"
#include "bus_i2c.h"

#include "build_config.h"
#include "config/runtime_config.h"
#include "drivers/light_led.h"
#include "drivers/system.h"

#include "barometer_icp10111.h"

// MS5611, Standard address 0x77
#define MS5611_ADDR  0x77

#define CMD_RESET    0x1E    // ADC reset command
#define CMD_ADC_READ 0x00    // ADC read command
#define CMD_ADC_CONV 0x40    // ADC conversion command
#define CMD_ADC_D1   0x00    // ADC D1 conversion
#define CMD_ADC_D2   0x10    // ADC D2 conversion
#define CMD_ADC_256  0x00    // ADC OSR=256
#define CMD_ADC_512  0x02    // ADC OSR=512
#define CMD_ADC_1024 0x04    // ADC OSR=1024
#define CMD_ADC_2048 0x06    // ADC OSR=2048
#define CMD_ADC_4096 0x08    // ADC OSR=4096
#define CMD_PROM_RD  0xA0    // Prom read command
#define PROM_NB      8

static bool icp10111_read ( uint32_t currentTime, float *pressure, float *temperature );
void icp10111_calculate ( float *pressure, float *temperature, uint32_t raw_p, uint16_t raw_t );
static uint32_t icp10111_measureStart ( mmode mode );

uint32_t baroRead = 9230;

float _scal [ 4 ];
uint16_t _raw_t;
uint32_t _raw_p;
float _temperature_C;
float _pressure_Pa;
uint32_t _meas_start;
uint32_t _meas_duration;
bool _data_ready;

#define ICP_I2C_ID       0x63

#define ICP_CMD_READ_ID  0xefc8
#define ICP_CMD_SET_ADDR 0xc595
#define ICP_CMD_READ_OTP 0xc7f7
#define ICP_CMD_MEAS_LP  0x609c
#define ICP_CMD_MEAS_N   0x6825
#define ICP_CMD_MEAS_LN  0x70df
#define ICP_CMD_MEAS_ULN 0x7866

// constants for presure calculation
const float _pcal [ 3 ]   = { 45000.0f, 80000.0f, 105000.0f };
const float _lut_lower    = 3.5f * static_cast< float > ( 0x100000 );     // 1<<20
const float _lut_upper    = 11.5f * static_cast< float > ( 0x100000 );    // 1<<20
const float _quadr_factor = 1.0f / 16777216.0f;
const float _offst_factor = 2048.0f;

void sendCommand ( int16_t command ) {

  // i2
}

bool icp10111Detect ( baro_t *baro ) {
  uint8_t command [ 2 ];
  uint8_t buf [ 3 ];
  uint16_t sensorID;
  bool ack = false;

  delay ( 10 );    // Allow sensor to power up

  // Send device ID command
  command [ 0 ] = ( 0xEFC8 >> 8 ) & 0xFF;
  command [ 1 ] = 0xEFC8 & 0xFF;

  ack = i2cWriteBufferwithoutregister ( 0x63, 2, command );
  if ( ! ack ) {
    return false;    // I2C write failed
  }

  delay ( 1 );

  // Read response
  ack = i2cReadwithoutregister ( 0x63, 2, buf );
  if ( ! ack ) {
    return false;    // I2C read failed
  }

  // Combine received bytes into sensor ID
  sensorID = ( buf [ 0 ] << 8 ) | buf [ 1 ];

  if ( ( sensorID & 0x03F ) != 0x08 ) {
    return false;    // Sensor ID mismatch
  }

  // Read sensor calibration data (OTP values)
  uint8_t otpCommand [ 5 ] = {
    ( ICP_CMD_SET_ADDR >> 8 ) & 0xFF,
    ICP_CMD_SET_ADDR & 0xFF,
    0x00, 0x66, 0x9C
  };

  ack = i2cWriteBufferwithoutregister ( 0x63, 5, otpCommand );
  if ( ! ack ) {
    return false;    // I2C write failed
  }

  for ( int i = 0; i < 4; i++ ) {
    command [ 0 ] = ( ICP_CMD_READ_OTP >> 8 ) & 0xFF;
    command [ 1 ] = ICP_CMD_READ_OTP & 0xFF;

    ack = i2cWriteBufferwithoutregister ( 0x63, 2, command );
    if ( ! ack ) {
      return false;
    }

    delay ( 1 );

    ack = i2cReadwithoutregister ( 0x63, 2, buf );
    if ( ! ack ) {
      return false;
    }

    _scal [ i ] = ( buf [ 0 ] << 8 ) | buf [ 1 ];
  }

  baro->measurment_start = icp10111_measureStart;
  baro->read             = icp10111_read;

  _pressure_Pa   = 0.0f;
  _temperature_C = 0.0f;
  _meas_duration = 98;
  _meas_start    = millis ( );
  _data_ready    = false;

  // Start the first measurement immediately after detection
  icp10111_measureStart(VERY_ACCURATE);

  return true;
}

static uint32_t icp10111_measureStart ( mmode mode ) {
  uint16_t cmd;
  uint8_t command [ 2 ];
  switch ( mode ) {
    case FAST:
      cmd            = ICP_CMD_MEAS_LP;
      _meas_duration = 3;
      break;
    case ACCURATE:
      cmd            = ICP_CMD_MEAS_LN;
      _meas_duration = 24;
      break;
    case VERY_ACCURATE:
      cmd            = ICP_CMD_MEAS_ULN;
      _meas_duration = 98;
      break;
    case NORMAL:
    default:
      cmd            = ICP_CMD_MEAS_N;
      _meas_duration = 7;
      break;
  }
  //  _sendCommand(cmd);

  command [ 0 ] = ( cmd >> 8 ) & 0xff;
  command [ 1 ] = cmd & 0xff;

  i2cWriteBufferwithoutregister ( 0x63, 2, command );
  // baroRead++;
  _data_ready = false;
  _meas_start = millis ( );
  return _meas_duration;
}

void icp10111_calculate ( float *pressure, float *temperature, uint32_t raw_p, uint16_t raw_t ) {
  if ( ! pressure || ! temperature ) {
    return;    // Prevent NULL pointer errors
  }

  // Calculate temperature in Celsius
  _temperature_C = -45.0f + ( 175.0f / 65536.0f ) * ( float ) raw_t;

  // calculate pressure
  float t = ( float ) raw_t - 32768.0f;    // Ensure floating point calculation

  float s1 = _lut_lower + ( ( float ) _scal [ 0 ] * t * t ) * _quadr_factor;
  float s2 = _offst_factor * ( float ) _scal [ 3 ] + ( ( float ) _scal [ 1 ] * t * t ) * _quadr_factor;
  float s3 = _lut_upper + ( ( float ) _scal [ 2 ] * t * t ) * _quadr_factor;

  float denominator = ( s3 * ( _pcal [ 0 ] - _pcal [ 1 ] ) + s1 * ( _pcal [ 1 ] - _pcal [ 2 ] ) + s2 * ( _pcal [ 2 ] - _pcal [ 0 ] ) );

  // Avoid division by zero
  if ( denominator == 0.0f ) {
    *pressure    = 0.0f;
    *temperature = _temperature_C;
    return;
  }

  float c = ( s1 * s2 * ( _pcal [ 0 ] - _pcal [ 1 ] ) + s2 * s3 * ( _pcal [ 1 ] - _pcal [ 2 ] ) + s3 * s1 * ( _pcal [ 2 ] - _pcal [ 0 ] ) ) / denominator;

  float a = ( _pcal [ 0 ] * s1 - _pcal [ 1 ] * s2 - ( _pcal [ 1 ] - _pcal [ 0 ] ) * c ) / ( s1 - s2 );
  float b = ( _pcal [ 0 ] - a ) * ( s1 + c );

  _pressure_Pa = a + b / ( c + ( float ) raw_p );

  *pressure    = _pressure_Pa;
  *temperature = _temperature_C;
}

static bool icp10111_read ( uint32_t currentTime, float *pressure, float *temperature ) {
// icp10111_measureStart(VERY_ACCURATE);
  if ( _data_ready ) {
    return true;    // Data is already processed, no need to read again
  }

  // Check if the measurement time has elapsed
  if ( ( currentTime - _meas_start ) < _meas_duration ) {
    return false;    // Data is not ready yet
  }

  uint8_t res_buf [ 9 ];    // Buffer to store raw sensor data

  // Read 9 bytes from the sensor
  if ( ! i2cReadwithoutregister ( 0x63, 9, res_buf ) ) {
    return false;    // I2C read failed
  }

  // Extract temperature (first two bytes)
  _raw_t = ( ( uint16_t ) res_buf [ 0 ] << 8 ) | res_buf [ 1 ];

  // Extract pressure (bytes 3, 4, 6)
  _raw_p = ( ( uint32_t ) res_buf [ 3 ] << 16 ) | ( ( uint32_t ) res_buf [ 4 ] << 8 ) | ( uint32_t ) res_buf [ 6 ];

  // Compute temperature and pressure
  icp10111_calculate ( pressure, temperature, _raw_p, _raw_t );

  // Mark data as processed
  _data_ready = true;

  return true;
}
