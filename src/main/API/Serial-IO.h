/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 MechAsh (j.mechash@gmail.com)                 #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
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
 #  Last Modified: Sat, 6th Sep 2025                                           #
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
  BAUD_RATE_4800,   // Baud rate of 4800 bits per second.
  BAUD_RATE_9600,   // Baud rate of 9600 bits per second.
  BAUD_RATE_14400,  // Baud rate of 14400 bits per second.
  BAUD_RATE_19200,  // Baud rate of 19200 bits per second.
  BAUD_RATE_38400,  // Baud rate of 38400 bits per second.
  BAUD_RATE_57600,  // Baud rate of 57600 bits per second.
  BAUD_RATE_115200, // Baud rate of 115200 bits per second.
  BAUD_RATE_128000, // Baud rate of 128000 bits per second.
  BAUD_RATE_256000  // Baud rate of 256000 bits per second.
} UART_Baud_Rate_e;

// Enumeration for UART ports.
// This enumeration is used to specify which UART port to interact with in the application.
typedef enum {
  UART2  // Represents the second UART port, often used for specific communication tasks.
} UART_Port_e;

/**
 * @brief Initializes the specified UART port with the given baud rate.
 *
 * @param PORT The UART port to initialize (e.g., UART2).
 * @param BAUD The desired baud rate for the UART communication.
 */
void Uart_init ( UART_Port_e PORT, UART_Baud_Rate_e BAUD );

/**
 * @brief Reads an 8-bit value from the specified UART port.
 *
 * @param PORT The UART port to read from.
 * @return uint8_t The 8-bit data read from the UART port.
 */
uint8_t Uart_read8 ( UART_Port_e PORT );

/**
 * @brief Reads a 16-bit value from the specified UART port.
 *
 * @param PORT The UART port to read from.
 * @return uint16_t The 16-bit data read from the UART port.
 */
uint16_t Uart_read16 ( UART_Port_e PORT );

/**
 * @brief Reads a 32-bit value from the specified UART port.
 *
 * @param PORT The UART port to read from.
 * @return uint32_t The 32-bit data read from the UART port.
 */
uint32_t Uart_read32 ( UART_Port_e PORT );

/**
 * @brief Writes an 8-bit data value to the specified UART port.
 *
 * @param PORT The UART port to write to.
 * @param data The 8-bit data to write.
 */
void Uart_write ( UART_Port_e PORT, uint8_t data );

/**
 * @brief Writes a string to the specified UART port.
 *
 * @param PORT The UART port to write to.
 * @param str The null-terminated string to write.
 */
void Uart_write ( UART_Port_e PORT, const char *str );

/**
 * @brief Writes a sequence of bytes to the specified UART port.
 *
 * @param PORT The UART port to write to.
 * @param data The pointer to the byte array to write.
 * @param length The number of bytes to write.
 */
void Uart_write ( UART_Port_e PORT, uint8_t *data, uint16_t length );

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

#endif