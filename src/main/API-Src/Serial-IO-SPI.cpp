/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\Serial-IO-SPI                                      #
 #  Created Date: Sun, 7th Sep 2025                                            #
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

#include <stddef.h>

#include "drivers/system.h"
#include "drivers/bus_spi.h"

#include "API/Serial-IO.h"

#define DISABLE_SPI GPIO_SetBits ( GPIOB, GPIO_Pin_5 )
#define ENABLE_SPI  GPIO_ResetBits ( GPIOB, GPIO_Pin_5 )


/**
 * @brief Determines the SPI prescaler value based on the desired speed.
 *
 * This function returns the appropriate SPI baud rate prescaler constant 
 * corresponding to the given speed in kHz. The prescaler is used to set 
 * the SPI clock division factor.
 *
 * @param speed Desired SPI speed in kHz.
 * @return Returns the SPI baud rate prescaler constant corresponding to the specified speed.
 */
uint16_t getPrescaler ( uint16_t speed ) {
  uint16_t prescaler = 0;
  switch ( speed ) {
    case 140:
      prescaler = SPI_BaudRatePrescaler_256;
      break;

    case 281:
      prescaler = SPI_BaudRatePrescaler_128;
      break;

    case 562:
      prescaler = SPI_BaudRatePrescaler_64;
      break;

    case 1125:
      prescaler = SPI_BaudRatePrescaler_32;
      break;

    case 2250:
      prescaler = SPI_BaudRatePrescaler_16;
      break;

    case 4500:
      prescaler = SPI_BaudRatePrescaler_8;
      break;

    case 9000:
      prescaler = SPI_BaudRatePrescaler_4;
      break;

    case 18000:
      prescaler = SPI_BaudRatePrescaler_2;
      break;
  }
  return prescaler;
}


void SPI_init ( ) {
  spiInit ( SPI2 );
}


void SPI_Init ( SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit ) {
  uint16_t prescaler = getPrescaler ( speed );
  SPI_InitTypeDef spi;

  SPI_I2S_DeInit ( SPI2 );

  spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi.SPI_Mode      = SPI_Mode_Master;
  spi.SPI_DataSize  = SPI_DataSize_8b;
  switch ( mode ) {
    case MODE0:
      spi.SPI_CPOL = SPI_CPOL_Low;
      spi.SPI_CPHA = SPI_CPHA_1Edge;
      break;

    case MODE1:
      spi.SPI_CPOL = SPI_CPOL_Low;
      spi.SPI_CPHA = SPI_CPHA_2Edge;
      break;

    case MODE2:
      spi.SPI_CPOL = SPI_CPOL_High;
      spi.SPI_CPHA = SPI_CPHA_1Edge;
      break;

    case MODE3:
      spi.SPI_CPOL = SPI_CPOL_High;
      spi.SPI_CPHA = SPI_CPHA_2Edge;
      break;
  }
  spi.SPI_NSS               = SPI_NSS_Soft;
  spi.SPI_BaudRatePrescaler = prescaler;
  if ( ( bit == LSBFIRST ) ) {
    spi.SPI_FirstBit = SPI_FirstBit_LSB;
  } else if ( ( bit == MSBFIRST ) ) {
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
  }
  spi.SPI_CRCPolynomial = 7;

#ifdef STM32F303xC
  // Configure for 8-bit reads.
  SPI_RxFIFOThresholdConfig ( SPI2, SPI_RxFIFOThreshold_QF );
#endif
  SPI_Init ( SPI2, &spi );
  SPI_Cmd ( SPI2, ENABLE );

  // Drive NSS high to disable connected SPI device.
  DISABLE_SPI;
}

void SPI_Enable ( void ) {
  ENABLE_SPI;
}

void SPI_Disable ( void ) {
  DISABLE_SPI;
}

uint8_t SPI_Read ( uint8_t register_address ) {

  uint8_t value = 0;

  register_address &= ~0x80u;

  ENABLE_SPI;

  delayMicroseconds ( 50 );

  spiTransferByte ( SPI2, register_address );

  delayMicroseconds ( 50 );

  spiTransfer ( SPI2, &value, NULL, 1 );

  delayMicroseconds ( 50 );

  DISABLE_SPI;

  delayMicroseconds ( 200 );

  return value;
}

void SPI_Read ( uint8_t register_address, int16_t length, uint8_t *buffer ) {

  register_address &= ~0x80u;

  ENABLE_SPI;

  delayMicroseconds ( 50 );

  spiTransferByte ( SPI2, register_address );

  delayMicroseconds ( 50 );

  spiTransfer ( SPI2, buffer, NULL, length );

  delayMicroseconds ( 50 );

  DISABLE_SPI;

  delayMicroseconds ( 200 );
}

void SPI_Write ( uint8_t register_address, uint8_t data ) {

  register_address |= 0x80u;

  ENABLE_SPI;

  delayMicroseconds ( 50 );

  spiTransferByte ( SPI2, register_address );

  delayMicroseconds ( 50 );

  spiTransferByte ( SPI2, data );

  delayMicroseconds ( 50 );

  DISABLE_SPI;

  delayMicroseconds ( 200 );
}
