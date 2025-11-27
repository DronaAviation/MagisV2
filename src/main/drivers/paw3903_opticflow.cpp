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
 #  Last Modified: Thu, 27th Nov 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include "platform.h"
#include "build_config.h"
#include "drivers/system.h"
#include "drivers/paw3903_opticflow.h"
#include "drivers/bridge_sc18is602b.h"
#include "drivers/bus_spi.h"
#include "drivers/gpio.h"

#ifdef PAW3903_SPI

  // Define constants for the time delay between sending an address and receiving the first data byte.
  // tSRAD: time between sending address and first data byte
  #define PAW_TSRAD_US  35    // Safe delay in microseconds, range: 35..50 µs
  #define PAW_TSRAD_US2 2     // Alternative value for safe delay in microseconds, range: 35..50 µs
  #define PAW_TSRAD_US3 5     // Another alternative value for safe delay in microseconds, range: 35..50 µs

  // Macros to control the SPI chip select (NSS) line for enabling and disabling SPI communication.
  #define DISABLE_SPI GPIO_SetBits ( SPI2_GPIO, SPI2_NSS_PIN )      // Set NSS high to disable SPI
  #define ENABLE_SPI  GPIO_ResetBits ( SPI2_GPIO, SPI2_NSS_PIN )    // Reset NSS low to enable SPI

/**
 * @brief Initializes and configures the SPI2 interface for the PAW3903 sensor.
 *
 * This function sets up the clocks, resets, and GPIOs necessary for using
 * SPI2 with the PAW3903 sensor. It configures SPI2 with settings suitable
 * for safe bring-up, including full-duplex communication, master mode,
 * 8-bit data size, and SPI Mode 3 (CPOL=High, CPHA=2Edge). The prescaler
 * is set to provide a typical speed of 1-2 MHz. The NSS pin is configured
 * as a software-controlled chip select, initially driven high to remain inactive.
 *
 * @return Returns true indicating that the SPI2 setup process is complete.
 */
bool paw3903_spi_setup ( ) {
  // Structure to configure SPI settings
  SPI_InitTypeDef spi;

  /* -- Enable clocks & reset for SPI2 peripheral -- */
  // Enable the clock for SPI2 on the APB1 bus
  RCC_APB1PeriphClockCmd ( RCC_APB1Periph_SPI2, ENABLE );

  // Reset SPI2 to ensure it is in a known state
  RCC_APB1PeriphResetCmd ( RCC_APB1Periph_SPI2, ENABLE );
  RCC_APB1PeriphResetCmd ( RCC_APB1Periph_SPI2, DISABLE );

  // Structure to configure GPIO settings
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the clock for the GPIO port used by SPI2 */
  RCC_AHBPeriphClockCmd ( SPI2_GPIO_PERIPHERAL, ENABLE );

  /* Configure alternate function mapping for SPI2 pins (PB13/PB14/PB15 -> AF5) */
  GPIO_PinAFConfig ( SPI2_GPIO, SPI2_SCK_PIN_SOURCE, GPIO_AF_5 );
  GPIO_PinAFConfig ( SPI2_GPIO, SPI2_MISO_PIN_SOURCE, GPIO_AF_5 );
  GPIO_PinAFConfig ( SPI2_GPIO, SPI2_MOSI_PIN_SOURCE, GPIO_AF_5 );

  /* Configure SCK and MOSI pins as alternate function push-pull outputs,
     and MISO pin as alternate function input with pull-down resistor for stability */
  GPIO_InitStructure.GPIO_Pin   = SPI2_SCK_PIN | SPI2_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init ( SPI2_GPIO, &GPIO_InitStructure );

  GPIO_InitStructure.GPIO_Pin  = SPI2_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;    // Prevent floating MISO
  GPIO_Init ( SPI2_GPIO, &GPIO_InitStructure );

  /* Configure NSS (chip select) as a plain GPIO for software-controlled CS.
     Set as push-pull output, idle high */
  GPIO_InitStructure.GPIO_Pin   = SPI2_NSS_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init ( SPI2_GPIO, &GPIO_InitStructure );

  /* Drive chip select (CS) line high (inactive) */
  GPIO_SetBits ( SPI2_GPIO, SPI2_NSS_PIN );

  /* -- Initialize the SPI peripheral -- */
  // De-initialize the SPI2 to ensure a clean start
  SPI_I2S_DeInit ( SPI2 );

  // Set up SPI configuration
  spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;    // Use full duplex communication
  spi.SPI_Mode      = SPI_Mode_Master;                    // Configure as master device
  spi.SPI_DataSize  = SPI_DataSize_8b;                    // Use 8-bit data size

  // Configure SPI mode: CPOL = High, CPHA = 2Edge (MODE3)
  spi.SPI_CPOL = SPI_CPOL_High;
  spi.SPI_CPHA = SPI_CPHA_2Edge;

  /* Safe bring-up speed: Prescaler x32 (APB/32 -> ~1-2 MHz typical) */
  spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;

  spi.SPI_NSS           = SPI_NSS_Soft;        // Software management of NSS
  spi.SPI_FirstBit      = SPI_FirstBit_MSB;    // Transmit MSB first
  spi.SPI_CRCPolynomial = 7;                   // CRC polynomial for error checking

  /* Ensure RX FIFO threshold is configured for 8-bit reads */
  SPI_RxFIFOThresholdConfig ( SPI2, SPI_RxFIFOThreshold_QF );

  // Initialize SPI2 with the specified settings
  SPI_Init ( SPI2, &spi );
  // Enable the SPI2 peripheral
  SPI_Cmd ( SPI2, ENABLE );

  /* Ensure chip select (CS) line is idle high after setup */
  GPIO_SetBits ( SPI2_GPIO, SPI2_NSS_PIN );
  return true;    // Indicate successful setup
}

/**
 * @brief Reads a single register from the PAW3903 sensor.
 *
 * This function reads a single byte from the specified register of the PAW3903
 * sensor using SPI communication. The read operation will return 0 if it fails.
 *
 * @param reg The register address to read from. The most significant bit should be 0
 *            to indicate a read operation.
 * @return uint8_t The value read from the register or 0 on failure.
 */
static inline uint8_t paw3903_read_reg ( uint8_t reg ) {
  uint8_t v;    // Variable to store the value read from the register
  // Ensure the read bit is set to 0 by clearing the MSB (bit 7)
  reg &= ~0x80u;
  ENABLE_SPI;    // Enable SPI communication by setting the Chip Select (CS) low
  // Delay to provide a guard time between CS activation and SCK (Clock) signal
  delayMicroseconds ( PAW_TSRAD_US3 );
  // Send the register address over SPI
  spiTransferByte ( SPI2, reg );
  // Wait for the required delay to allow the internal logic to present the data
  delayMicroseconds ( PAW_TSRAD_US );
  // Clock out one byte and read the response explicitly
  v = spiTransferByte ( SPI2, 0x00 );
  // Short delay after reading, then release the CS line
  delayMicroseconds ( PAW_TSRAD_US2 );
  DISABLE_SPI;    // Disable SPI communication by releasing the Chip Select (CS)
  return v;       // Return the value read from the register
}

/**
 * @brief Reads multiple bytes from the PAW3903 sensor in burst mode.
 *
 * This function performs a burst read operation from the specified register of
 * the PAW3903 sensor. It reads `len` bytes into the provided buffer `buf`.
 * Ensure that `buf` is large enough to hold `len` bytes. The most significant bit
 * of `reg` should be 0 to indicate a read operation.
 *
 * @param reg The register address to start reading from.
 * @param len The number of bytes to read.
 * @param buf A pointer to the buffer where the read data will be stored. Must be
 *            large enough to hold `len` bytes.
 */
static inline void paw3903_read_reg2 ( uint8_t reg, int len, uint8_t *buf ) {
  // Ensure the read bit is set to 0 by clearing the MSB (bit 7)
  reg &= ~0x80u;
  ENABLE_SPI;    // Enable SPI communication by setting the Chip Select (CS) low
  // Delay to provide a guard time between CS activation and SCK (Clock) signal
  delayMicroseconds ( PAW_TSRAD_US3 );
  // Send the register address over SPI
  spiTransferByte ( SPI2, reg );
  // Wait for the required delay to allow the internal logic to present the data
  delayMicroseconds ( PAW_TSRAD_US );
  // Loop through the buffer length to read multiple bytes
  for ( int i = 0; i < len; ++i )
    buf [ i ] = spiTransferByte ( SPI2, 0x00 );    // Read each byte and store it in the buffer
  // Short delay after reading, then release the CS line
  delayMicroseconds ( PAW_TSRAD_US2 );
  DISABLE_SPI;    // Disable SPI communication by releasing the Chip Select (CS)
}

/**
 * @brief Writes a single byte to a specified register of the PAW3903 sensor.
 *
 * This function writes a single byte `val` to the specified register `reg`
 * of the PAW3903 sensor. The most significant bit of `reg` should be 1 to
 * indicate a write operation.
 *
 * @param reg The register address to write to. The most significant bit is set
 *            to indicate a write operation.
 * @param val The value to write into the register.
 * @return Returns true if the write operation is successful.
 */
static inline bool paw3903_write_reg ( uint8_t reg, uint8_t val ) {
  // Set the write bit by setting the MSB (bit 7)
  reg |= 0x80u;
  ENABLE_SPI;    // Enable SPI communication by setting the Chip Select (CS) low
  // Delay to ensure a proper setup time between CS activation and SCK (Clock) signal
  delayMicroseconds ( PAW_TSRAD_US3 );
  // Send the register address with the write bit set over SPI
  spiTransferByte ( SPI2, reg );
  // Short delay before sending the data byte
  delayMicroseconds ( PAW_TSRAD_US2 );
  // Send the data byte to be written to the register
  spiTransferByte ( SPI2, val );
  // Short delay after writing, then release the CS line
  delayMicroseconds ( PAW_TSRAD_US2 );
  DISABLE_SPI;    // Disable SPI communication by releasing the Chip Select (CS)
  return true;    // Return true indicating the write operation was successful
}

#endif

#ifdef PAW3903_SC18
/**
 * @brief Reads a single byte from a specified register of the PAW3903 sensor.
 *
 * This function reads and returns a single byte from the specified register
 * `reg` of the PAW3903 sensor. The most significant bit of `reg` should be 0
 * to indicate a read operation. The function uses a dummy byte during the
 * SPI transfer.
 *
 * @param reg The register address to read from. The most significant bit is
 *            cleared to indicate a read operation.
 * @return Returns the value read from the register. Returns 0 if the SPI
 *         transfer fails.
 */
static inline uint8_t paw3903_read_reg ( uint8_t reg ) {
  uint8_t tx [ 2 ] = { ( uint8_t ) ( reg & ~0x80u ), 0x00 };    // addr + dummy
  uint8_t rx [ 2 ] = { 0 };
  if ( ! sc18_spiTransfer ( SS0, tx, 2, rx ) ) return 0;    // bridge clocks 2 bytes
  return rx [ 1 ];                                          // data arrives on 2nd byte
}

/**
 * @brief Writes a single byte to a specified register of the PAW3903 sensor.
 *
 * This function writes a single byte `val` to the specified register `reg`
 * of the PAW3903 sensor. The most significant bit of `reg` is set to 1 to
 * indicate a write operation. The function performs the SPI transfer and
 * returns true if successful.
 *
 * @param reg The register address to write to. The most significant bit is
 *            set to indicate a write operation.
 * @param val The value to be written to the register.
 * @return Returns true if the SPI transfer is successful, false otherwise.
 */
static inline bool paw3903_write_reg ( uint8_t reg, uint8_t val ) {
  uint8_t tx [ 2 ] = { ( uint8_t ) ( reg | 0x80u ), val };    // write bit set
  uint8_t rx [ 2 ];                                           // ignored
  return sc18_spiTransfer ( SS0, tx, 2, rx );
}

/**
 * @brief Configures the SPI interface for the PAW3903 sensor.
 *
 * This function sets up the SPI communication parameters for the PAW3903
 * sensor. It configures the SPI to operate with most significant bit first,
 * SPI Mode 3 (CPOL=1, CPHA=1), and a clock frequency of approximately 461 kHz.
 * These settings are suitable for initial bring-up of the sensor.
 *
 * @return Returns true if the SPI configuration is successful, false otherwise.
 */
bool paw3903_spi_setup ( void ) {
  // MSB first, SPI Mode 3 (CPOL=1, CPHA=1), ~461 kHz (safe for bring-up)
  sc18_address_cfg ( true, true, true );
  return sc18_configureSPI ( false, SC18IS601B_SPIMODE_3, SC18IS601B_SPICLK_461_kHz );
}

#endif

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
#ifdef PAW3903_SC18
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
#endif
#ifdef PAW3903_SPI
  uint8_t b [ 9 ];
  paw3903_read_reg2 ( PAW3903_REG_Motion_Burst, 9, b );    // uses paw3903_read_reg_spi_buf

  // Layout: b0=motion, b1=dx_l, b2=dx_h, b3=dy_l, b4=dy_h, b5=squal, b6=sh_l, b7=sh_h, b8=obs
  out.motion      = b [ 0 ];
  out.deltaX      = ( int16_t ) ( ( ( uint16_t ) b [ 2 ] << 8 ) | b [ 1 ] );
  out.deltaY      = ( int16_t ) ( ( ( uint16_t ) b [ 4 ] << 8 ) | b [ 3 ] );
  out.squal       = b [ 5 ];
  out.shutter     = ( uint16_t ) b [ 7 ] << 8 | b [ 6 ];
  out.observation = b [ 8 ];

  // If motion bit not set, deltas are not meaningful
  if ( ( out.motion & 0x80 ) == 0 ) {
    out.deltaX = 0;
    out.deltaY = 0;
  }
  return true;
#endif
}

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

bool paw3903_config_motion_pin ( SC18IS601B_GPIO sc18GPIO ) {
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