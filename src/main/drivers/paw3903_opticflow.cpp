/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\paw3903_opticflow.cpp                              #
 #  Created Date: Thu, 6th Nov 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 6th Nov 2025                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include "platform.h"

#include "API/Debugging.h"
#include "drivers/system.h"
#include "drivers/paw3903_opticflow.h"
#include "drivers/bridge_sc18is602b.h"

static inline uint8_t paw3903_read_reg ( uint8_t reg ) {
  uint8_t tx [ 2 ] = { ( uint8_t ) ( reg & ~0x80u ), 0x00 };    // addr + dummy
  uint8_t rx [ 2 ] = { 0 };
  if ( ! sc18_spiTransfer ( SS0, tx, 2, rx ) ) return 0;    // bridge clocks 2 bytes
  return rx [ 1 ];                                          // data arrives on 2nd byte
}

static inline bool paw3903_write_reg ( uint8_t reg, uint8_t val ) {
  uint8_t tx [ 2 ] = { ( uint8_t ) ( reg | 0x80u ), val };    // write bit set
  uint8_t rx [ 2 ];                                           // ignored
  return sc18_spiTransfer ( SS0, tx, 2, rx );
}

bool paw3903_spi_setup ( void ) {
  // MSB first, SPI Mode 3 (CPOL=1, CPHA=1), ~461 kHz (safe for bring-up)
  sc18_address_cfg ( true, true, true );
  return sc18_configureSPI ( false, SC18IS601B_SPIMODE_3, SC18IS601B_SPICLK_461_kHz );
}

bool paw3903_check_id ( uint8_t *product_id, uint8_t *revision_id ) {
  uint8_t pid = paw3903_read_reg ( PAW3903_REG_Product_ID );
  uint8_t rid = paw3903_read_reg ( PAW3903_REG_Revision_ID );

  if ( product_id ) *product_id = pid;
  if ( revision_id ) *revision_id = rid;

  return ( pid == 0x49 ) && ( rid == 0x01 );
}

bool paw3903_init ( void ) {
  if ( ! paw3903_spi_setup ( ) )
    return false;
  delay ( 50 );
  uint8_t product_id, revision_id = 0;
  return paw3903_check_id ( &product_id, &revision_id );
}

bool paw3903_set_mode ( PAW3903_OperationMode_t mode ) {
  if ( mode > PAW3903_MODE_SUPER_LOW_LIGHT ) return false;    // valid range: 0–2

  return paw3903_write_reg ( PAW3903_REG_LightMode, mode );
}

PAW3903_OperationMode_t paw3903_get_mode ( void ) {
  return ( PAW3903_OperationMode_t ) paw3903_read_reg ( PAW3903_REG_LightMode );
}

bool paw3903_power_up_reset ( void ) {
  return paw3903_write_reg ( PAW3903_REG_PowerUpReset, 0x5A );
}

bool paw3903_shutdown ( void ) {
  return paw3903_write_reg ( PAW3903_REG_Shutdown, 0x00 );
}

uint8_t paw3903_read_motion ( void ) {
  return paw3903_read_reg ( PAW3903_REG_Motion );
}

uint8_t paw3903_read_squal ( void ) {
  return paw3903_read_reg ( PAW3903_REG_Squal );
}
uint8_t paw3903_read_observation ( void ) {
  return paw3903_read_reg ( PAW3903_REG_Observation );
}

uint16_t paw3903_read_shutter ( void ) {
  uint8_t low  = paw3903_read_reg ( PAW3903_REG_Shutter_Lower );
  uint8_t high = paw3903_read_reg ( PAW3903_REG_Shutter_Upper );
  return ( ( uint16_t ) high << 8 ) | low;
}

bool paw3903_read_motion_burst ( PAW3903_Data &out ) {
  uint8_t tx [ 10 ] = { PAW3903_REG_Motion_Burst & ~0x80u };    // READ command for 0x16
  uint8_t rx [ 10 ] = { 0 };

  // Read 9 bytes from burst
  if ( ! sc18_spiTransfer ( SS0, tx, 9, rx ) )
    return false;

  out.motion      = rx [ 1 ];
  out.deltaX      = ( int16_t ) ( ( rx [ 3 ] << 8 ) | rx [ 2 ] );
  out.deltaY      = ( int16_t ) ( ( rx [ 5 ] << 8 ) | rx [ 4 ] );
  out.squal       = rx [ 6 ];
  out.shutter     = ( uint16_t ) ( ( rx [ 8 ] << 8 ) | rx [ 7 ] );
  out.observation = rx [ 9 ];

  return true;
}

// bool paw3903_read_motion_burst ( PAW3903_Data &out, bool debug ) {
//   // We'll read 9 bytes worth of burst data (motion + dx_l/h + dy_l/h + squal + shutter_l/h + obs)
//   uint8_t tx [ 10 ] = { PAW3903_REG_Motion_Burst & ~0x80u, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//   uint8_t rx [ 10 ] = { 0 };

//   // Do SPI transfer. If your bridge needs a tiny delay between sending address and read,
//   // consider adding a few us delay before sc18_spiTransfer or using two transfers.
//   if ( ! sc18_spiTransfer ( SS0, tx, 9, rx ) ) return false;

//   // Raw dump for diagnostics (enable by passing debug=true)
//   if ( debug ) {
//     Monitor_Println ( "RAW RX:", 0 );
//     for ( int i = 0; i < 9; ++i ) {
//       Monitor_Print ( "rx[", i );
//       Monitor_Print ( "] = ", rx [ i ] );
//     }
//   }

//   // Determine the index offset:
//   // Some bridges return the first meaningful byte at rx[0], others at rx[1].
//   // We'll choose offset so that the motion byte has the high-motion-bit if set.
//   int idx = 1;    // default
//   if ( ( rx [ 1 ] & 0x80 ) || ( rx [ 1 ] != 0 ) ) {
//     // If rx[1] looks like a motion byte (either has bit7 or non-zero), prefer offset=1.
//     idx = 1;
//   } else if ( ( rx [ 0 ] & 0x80 ) || ( rx [ 0 ] != 0 ) ) {
//     // otherwise if rx[0] looks meaningful, use offset 0
//     idx = 0;
//   } else {
//     // If both look zero, still try offset=1 first (common).
//     idx = 1;
//   }

//   uint8_t motion = rx [ idx + 0 ];
//   uint8_t dx_l   = rx [ idx + 1 ];
//   uint8_t dx_h   = rx [ idx + 2 ];
//   uint8_t dy_l   = rx [ idx + 3 ];
//   uint8_t dy_h   = rx [ idx + 4 ];
//   uint8_t squal  = rx [ idx + 5 ];
//   uint8_t sh_l   = rx [ idx + 6 ];
//   uint8_t sh_h   = rx [ idx + 7 ];
//   uint8_t obs    = rx [ idx + 8 ];

//   // If debug, show interpreted bytes
//   if ( debug ) {
//     Monitor_Println ( "Interpreted burst:", 0 );
//     Monitor_Println ( "motion:", motion );
//     Monitor_Println ( "dx_l:", dx_l );
//     Monitor_Println ( "dx_h:", dx_h );
//     Monitor_Println ( "dy_l:", dy_l );
//     Monitor_Println ( "dy_h:", dy_h );
//     Monitor_Println ( "squal:", squal );
//     Monitor_Println ( "shutter hi:", sh_h );
//     Monitor_Println ( "shutter lo:", sh_l );
//     Monitor_Println ( "obs:", obs );
//   }

//   // If the motion flag is not set, deltas are not meaningful — return zeros.
//   // motion bit7 (0x80) is the common "motion occurred" flag.
//   if ( ( motion & 0x80 ) == 0 ) {
//     out.motion      = motion;
//     out.deltaX      = 0;
//     out.deltaY      = 0;
//     out.squal       = squal;
//     out.shutter     = ( uint16_t ) sh_h << 8 | sh_l;
//     out.observation = obs;
//     if ( debug ) Monitor_Println ( "No new motion (motion bit clear) — deltas zeroed", 0 );
//     return true;
//   }

//   // Combine into signed 16-bit values (two's complement).
//   out.deltaX      = ( int16_t ) ( ( ( uint16_t ) dx_h << 8 ) | dx_l );
//   out.deltaY      = ( int16_t ) ( ( ( uint16_t ) dy_h << 8 ) | dy_l );
//   out.motion      = motion;
//   out.squal       = squal;
//   out.shutter     = ( uint16_t ) sh_h << 8 | sh_l;
//   out.observation = obs;

//   // Final sanity: if squal very low, deltas may be invalid -> ignore
//   if ( out.squal < 8 ) {    // threshold depends on your surface; tune as needed
//     if ( debug ) Monitor_Println ( "Low SQUAL -> ignoring deltas", 0 );
//     out.deltaX = 0;
//     out.deltaY = 0;
//   }

//   return true;
// }

bool paw3903_set_resolution ( PAW3903_ResolutionCPI_t res ) {
  return paw3903_write_reg ( PAW3903_REG_Resolution, ( uint8_t ) res );
}

PAW3903_ResolutionCPI_t paw3903_get_resolution ( void ) {
  return ( PAW3903_ResolutionCPI_t ) paw3903_read_reg ( PAW3903_REG_Resolution );
}

bool paw3903_set_orientation ( PAW3903_Orientation_t orient ) {
  return paw3903_write_reg ( PAW3903_REG_Orientation, ( uint8_t ) orient );
}

PAW3903_Orientation_t paw3903_get_orientation ( void ) {
  return ( PAW3903_Orientation_t ) paw3903_read_reg ( PAW3903_REG_Orientation );
}

SC18IS601B_GPIO Motion_sc18GPIO;
peripheral_gpio_pin_e Motion_GPIO;
uint8_t Motion_Gpio = 0x00;

bool paw3903_config_motion_pin ( SC18IS601B_GPIO sc18GPIO  ) {
  Motion_sc18GPIO = sc18GPIO;
  if ( ! sc18_enableGPIO ( Motion_sc18GPIO, true ) )
    return false;
  Motion_Gpio = 0x01;
  return sc18_setupGPIO ( Motion_sc18GPIO, SC18IS601B_GPIO_MODE_INPUT_ONLY );
}

void paw3903_config_motion_pin ( peripheral_gpio_pin_e GPIO ) {
  Motion_GPIO = GPIO;
  Peripheral_Init ( Motion_GPIO, INPUT );
  Motion_Gpio = 0x02;
}

bool paw3903_read_motion_pin ( void ) {
  if ( Motion_Gpio == 0x01 ) {
    return sc18_readGPIO ( Motion_sc18GPIO );
  } else if ( Motion_Gpio == 0x02 ) {
    return Peripheral_Read ( Motion_GPIO );
  }
  return false;
}