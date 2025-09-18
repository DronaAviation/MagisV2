/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\Serial-IO-Uart.cpp                                 #
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

#include "platform.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "io/serial.h"

#include "API/Serial-IO.h"

#include <cstddef>

static serialPort_t *uart2;


/**
 * @brief Retrieves the numerical baud rate value corresponding to a given enumeration constant.
 *
 * This function takes a `UART_Baud_Rate_e` enumerator as input and returns the actual baud rate value as an unsigned integer.
 *
 * @param BAUD The enumerator representing the desired baud rate.
 * @return The numerical value of the baud rate in bits per second.
 */
static uint32_t getBaud ( UART_Baud_Rate_e BAUD ) {
  switch ( BAUD ) {
    case BAUD_RATE_4800:
      return 4800;
    case BAUD_RATE_9600:
      return 9600;
    case BAUD_RATE_14400:
      return 14400;
    case BAUD_RATE_19200:
      return 19200;
    case BAUD_RATE_38400:
      return 38400;
    case BAUD_RATE_57600:
      return 57600;
    case BAUD_RATE_115200:
      return 115200;
    case BAUD_RATE_128000:
      return 128000;
    case BAUD_RATE_256000:
      return 256000;
  }
}

void Uart_Init ( UART_Port_e PORT, UART_Baud_Rate_e BAUD ) {
  if ( PORT == UART2 ) {
    uart2 = openSerialPort ( SERIAL_PORT_USART2, FUNCTION_UNIBUS, NULL, getBaud ( BAUD ), MODE_RXTX, SERIAL_NOT_INVERTED );
  }
}

uint8_t Uart_Read8 ( UART_Port_e PORT ) {
  if ( PORT == UART2 ) {
    return serialRead ( uart2 ) & 0xff;
  }
}

uint16_t Uart_Read16 ( UART_Port_e PORT ) {
  uint16_t t = Uart_Read8 ( PORT );
  t += ( uint16_t ) Uart_Read8 ( PORT ) << 8;
  return t;
}

uint32_t Uart_Read32 ( UART_Port_e PORT ) {
  uint32_t t = Uart_Read16 ( PORT );
  t += ( uint32_t ) Uart_Read16 ( PORT ) << 16;
  return t;
}

void Uart_Write ( UART_Port_e PORT, uint8_t data ) {
  if ( PORT == UART2 ) {
    serialWrite ( uart2, data );
  }
}

void Uart_Write ( UART_Port_e PORT, const char *str ) {
  if ( PORT == UART2 ) {
    serialPrint ( uart2, str );
  }
}

void Uart_Write ( UART_Port_e PORT, uint8_t *data, uint16_t length ) {
  if ( PORT == UART2 ) {
    while ( length-- ) {
      serialWrite ( uart2, *data );
      data++;
    }
  }
}

bool Uart_rxBytesWaiting ( UART_Port_e PORT ) {
  if ( PORT == UART2 ) {
    return serialRxBytesWaiting ( uart2 );
  }
}

bool Uart_txBytesFree ( UART_Port_e PORT ) {
  if ( PORT == UART2 ) {
    return serialTxBytesFree ( uart2 );
  }
}
