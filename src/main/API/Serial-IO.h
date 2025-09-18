/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\Serial-IO.h                                            #
 #  Created Date: Sat, 6th Sep 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 18th Sep 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#ifndef SERIAL_IO_H
#define SERIAL_IO_H

#include <stdint.h>

// Enumeration for UART baud rates.
// Each enumerator represents a standard baud rate value that can be used to configure UART communication.
typedef enum {
  BAUD_RATE_4800,      // Baud rate of 4800 bits per second.
  BAUD_RATE_9600,      // Baud rate of 9600 bits per second.
  BAUD_RATE_14400,     // Baud rate of 14400 bits per second.
  BAUD_RATE_19200,     // Baud rate of 19200 bits per second.
  BAUD_RATE_38400,     // Baud rate of 38400 bits per second.
  BAUD_RATE_57600,     // Baud rate of 57600 bits per second.
  BAUD_RATE_115200,    // Baud rate of 115200 bits per second.
  BAUD_RATE_128000,    // Baud rate of 128000 bits per second.
  BAUD_RATE_256000     // Baud rate of 256000 bits per second.
} UART_Baud_Rate_e;

// Enumeration for UART ports.
// This enumeration is used to specify which UART port to interact with in the application.
typedef enum {
  UART2    // Represents the second UART port, often used for specific communication tasks.
} UART_Port_e;

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

/**
 * @brief Initializes the specified UART port with the given baud rate.
 *
 * @param PORT The UART port to initialize (e.g., UART2).
 * @param BAUD The desired baud rate for the UART communication.
 */
void Uart_Init ( UART_Port_e PORT, UART_Baud_Rate_e BAUD );

/**
 * @brief Reads an 8-bit value from the specified UART port.
 *
 * @param PORT The UART port to read from.
 * @return uint8_t The 8-bit data read from the UART port.
 */
uint8_t Uart_Read8 ( UART_Port_e PORT );

/**
 * @brief Reads a 16-bit value from the specified UART port.
 *
 * @param PORT The UART port to read from.
 * @return uint16_t The 16-bit data read from the UART port.
 */
uint16_t Uart_Read16 ( UART_Port_e PORT );

/**
 * @brief Reads a 32-bit value from the specified UART port.
 *
 * @param PORT The UART port to read from.
 * @return uint32_t The 32-bit data read from the UART port.
 */
uint32_t Uart_Read32 ( UART_Port_e PORT );

/**
 * @brief Writes an 8-bit data value to the specified UART port.
 *
 * @param PORT The UART port to write to.
 * @param data The 8-bit data to write.
 */
void Uart_Write ( UART_Port_e PORT, uint8_t data );

/**
 * @brief Writes a string to the specified UART port.
 *
 * @param PORT The UART port to write to.
 * @param str The null-terminated string to write.
 */
void Uart_Write ( UART_Port_e PORT, const char *str );

/**
 * @brief Writes a sequence of bytes to the specified UART port.
 *
 * @param PORT The UART port to write to.
 * @param data The pointer to the byte array to write.
 * @param length The number of bytes to write.
 */
void Uart_Write ( UART_Port_e PORT, uint8_t *data, uint16_t length );

/**
 * @brief Checks if there are bytes waiting to be read in the receive buffer of the specified UART port.
 *
 * @param PORT The UART port to check.
 * @return true If there are bytes waiting to be read.
 * @return false If there are no bytes waiting to be read.
 */
bool Uart_rxBytesWaiting ( UART_Port_e PORT );

/**
 * @brief Checks if there is space available in the transmit buffer of the specified UART port.
 *
 * @param PORT The UART port to check.
 * @return true If there is space available for writing.
 * @return false If there is no space available for writing.
 */
bool Uart_txBytesFree ( UART_Port_e PORT );

/**
 * @brief Reads a single byte from a specified register of an I2C device.
 *
 * This function initiates an I2C read operation for a single byte from the given
 * register address of the specified I2C device. The read byte is stored in the
 * provided reference variable.
 *
 * @param device_add The I2C address of the device.
 * @param reg The register address to read from.
 * @param value Reference to a variable where the read byte will be stored.
 * @return Returns true if the read operation is successful, otherwise false.
 */
bool I2C_Read ( uint8_t device_add, uint8_t reg, uint8_t &value );

/**
 * @brief Reads multiple bytes from a specified register of an I2C device.
 *
 * This function performs an I2C read operation to fetch multiple bytes starting
 * from the given register address of the specified I2C device. The data is stored
 * in the provided buffer.
 *
 * @param device_add The I2C address of the device.
 * @param reg The register address to start reading from.
 * @param length Number of bytes to read.
 * @param buffer Pointer to a buffer where the read data will be stored.
 * @return Returns the number of bytes successfully read or a negative error code.
 */
int16_t I2C_Read ( uint8_t device_add, uint8_t reg, uint32_t length, uint8_t *buffer );

/**
 * @brief Writes a single byte to a specified register of an I2C device.
 *
 * This function initiates an I2C write operation to send a single byte to the given
 * register address of the specified I2C device.
 *
 * @param device_add The I2C address of the device.
 * @param reg The register address to write to.
 * @param data The byte of data to write.
 * @return Returns true if the write operation is successful, otherwise false.
 */
bool I2C_Write ( uint8_t device_add, uint8_t reg, uint8_t data );

/**
 * @brief Writes multiple bytes to a specified register of an I2C device.
 *
 * This function performs an I2C write operation to send multiple bytes starting
 * from the given register address of the specified I2C device. The data is taken
 * from the provided buffer.
 *
 * @param device_add The I2C address of the device.
 * @param reg The register address to start writing to.
 * @param length Number of bytes to write.
 * @param data Pointer to a buffer containing the data to be written.
 * @return Returns true if the write operation is successful, otherwise false.
 */
bool I2C_Write ( uint8_t device_add, uint8_t reg, uint32_t length, uint8_t *data );

/**
 * @brief Initializes the SPI interface on SPI2 with default settings.
 *
 * This function sets up the SPI2 peripheral with default initialization
 * parameters, preparing it for communication. It calls a lower-level
 * initialization function to configure the hardware.
 */
void SPI_Init ( );

/**
 * @brief Initializes the SPI interface on SPI2 with specified settings.
 *
 * This function configures the SPI2 peripheral according to the provided
 * settings for mode, speed, and data bit order. It sets up the direction,
 * mode, data size, clock polarity and phase based on the chosen mode:
 * MODE0, MODE1, MODE2, or MODE3. The baud rate prescaler is determined by
 * the `speed` parameter. The first transmitted bit is set according to the
 * `bit` parameter (LSBFIRST or MSBFIRST). Finally, it enables the SPI
 * interface and drives NSS high to disable any connected SPI device.
 *
 * @param mode The SPI mode (MODE0, MODE1, MODE2, MODE3).
 * @param speed The desired communication speed in Hz.
 * @param bit The bit order (LSBFIRST or MSBFIRST).
 */
void SPI_Init ( SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit );

/**
 * @brief Enables the SPI interface.
 *
 * This function activates the SPI peripheral, allowing it to begin
 * communication operations. It sets the necessary hardware or software
 * flags to turn on the SPI module.
 */
void SPI_Enable ( void );

/**
 * @brief Disables the SPI interface.
 *
 * This function deactivates the SPI peripheral, stopping any ongoing
 * communication operations. It clears the necessary hardware or software
 * flags to turn off the SPI module.
 */
void SPI_Disable ( void );

/**
 * @brief Reads a byte from a specified SPI register.
 *
 * This function communicates with an SPI device to read a single byte of data
 * from a given register address. It handles enabling and disabling the SPI
 * interface, as well as managing necessary delays for proper synchronization.
 *
 * @param register_address The address of the register to read from.
 * @return The value read from the specified register.
 */
uint8_t SPI_Read ( uint8_t register_address );

/**
 * @brief Reads data from an SPI device register into a buffer.
 *
 * This function initiates an SPI read operation by sending the
 * provided register address (after clearing its most significant bit)
 * to the SPI device. It then reads the specified length of data bytes
 * into the given buffer.
 *
 * @param register_address The address of the register to read from.
 *        The MSB is cleared before use.
 * @param length The number of bytes to read into the buffer.
 * @param buffer A pointer to the buffer where the read data will be stored.
 */
void SPI_Read ( uint8_t register_address, int16_t length, uint8_t *buffer );

/**
 * @brief Sends data to a specified register via SPI.
 *
 * This function writes a byte of data to a specific register
 * on the SPI device. It first sets the most significant bit
 * of the register address to indicate a write operation, and
 * then sends the register address followed by the data byte
 * over the SPI bus with appropriate delays.
 *
 * @param register_address The address of the register to write to.
 * @param data The data byte to be written to the register.
 */
void SPI_Write ( uint8_t register_address, uint8_t data );

#endif