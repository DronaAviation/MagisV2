/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\Peripheral.h                                           #
 #  Created Date: Thu, 8th May 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 8th May 2025                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#ifndef API_PERIPHERAL_H
#define API_PERIPHERAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cstdint>
#include "Specifiers.h"

#ifdef __cplusplus
extern "C" {
#endif

extern bool gpioReset;
extern bool changeAdress;

typedef enum {
  INPUT,
  INPUT_PU,
  INPUT_PD,
  OUTPUT,
} GPIO_Mode_e;

typedef enum {
  STATE_LOW,
  STATE_HIGH,
  STATE_TOGGLE
} GPIO_State_e;

class GPIO_P {
 public:
  void init ( unibus_e pin_number, GPIO_Mode_e mode );

  bool read ( unibus_e pin_number );

  void write ( unibus_e pin_number, GPIO_State_e STATE );
};

typedef enum {
  UART2,
  UART3
} UART_Port_e;

typedef enum {
  BAUD_RATE_4800,
  BAUD_RATE_9600,
  BAUD_RATE_14400,
  BAUD_RATE_19200,
  BAUD_RATE_38400,
  BAUD_RATE_57600,
  BAUD_RATE_115200,
  BAUD_RATE_128000,
  BAUD_RATE_256000
} UART_Baud_Rate_e;

class UART_P {
 public:
  void init ( UART_Port_e PORT, UART_Baud_Rate_e BAUD );

  uint8_t read8 ( UART_Port_e PORT );

  uint16_t read16 ( UART_Port_e PORT );

  uint32_t read32 ( UART_Port_e PORT );

  void write ( UART_Port_e PORT, uint8_t data );

  void write ( UART_Port_e PORT, const char *str );

  void write ( UART_Port_e PORT, uint8_t *data, uint16_t length );

  bool rxBytesWaiting ( UART_Port_e PORT );

  bool txBytesFree ( UART_Port_e PORT );
};

class I2C_P {
 public:
  bool read ( uint8_t device_add, uint8_t reg, uint8_t &value );

  int16_t read ( uint8_t device_add, uint8_t reg, uint32_t length, uint8_t *buffer );

  bool write ( uint8_t device_add, uint8_t reg, uint8_t data );

  bool write ( uint8_t device_add, uint8_t reg, uint32_t length, uint8_t *data );
};

class PWM_P {

 public:
  void init ( unibus_e pin_number, uint16_t pwmRate );

  void write ( unibus_e pin_number, uint16_t pwmValue );
};

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

class SPI_P {
 public:
  void init ( );

  void init ( SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit );

  void enable ( void );

  void disable ( void );

  uint8_t read ( uint8_t register_address );

  void read ( uint8_t register_address, int16_t length, uint8_t *buffer );

  void write ( uint8_t register_address, uint8_t data );
};

extern GPIO_P GPIO;
extern UART_P UART;
extern I2C_P I2C;
extern PWM_P PWM;
extern SPI_P SPI;

typedef enum peripheral_gpio {
  GPIO_1,
  GPIO_2,
  GPIO_3,
  GPIO_4,
  GPIO_5,
  GPIO_6,
  GPIO_7,
  GPIO_8,
  GPIO_9,
  GPIO_10,
  GPIO_11,
  GPIO_12,
  GPIO_13,
  GPIO_14,
  GPIO_15
} peripheral_gpio_pin;

typedef enum peripheral_adc {
  ADC_1,
  ADC_2,
  ADC_3,
  ADC_4,
  ADC_5,
  ADC_6,
  ADC_7,
  ADC_8,
  ADC_9,
  ADC_10,
  ADC_11
} peripheral_adc_pin;

typedef enum peripheral_adc_channel {
  ADC2_IN12,
  ADC4_IN5,
  ADC4_IN4,
  ADC3_IN5,
  ADC4_IN3,
  ADC2_IN1,
  ADC2_IN2,
  ADC1_IN4,
  ADC1_IN3,
  ADC2_IN4,
  ADC3_IN1
} Peripheral_ADC_Channel;

typedef enum peripheral_pwm {
  PWM_1,
  PWM_2,
  PWM_3,
  PWM_4,
  PWM_5,
  PWM_6,
  PWM_7,
  PWM_8,
  PWM_9,
  PWM_10
} peripheral_pwm_pin;

#define ADC_CHANNEL_COUNT 11

extern bool _isAdcEnable [ ADC_CHANNEL_COUNT ];
extern uint8_t _adcDmaIndex [ ADC_CHANNEL_COUNT ];

#define ADC1_CHANNEL_COUNT 2
#define ADC2_CHANNEL_COUNT 4
#define ADC3_CHANNEL_COUNT 2
#define ADC4_CHANNEL_COUNT 3

extern volatile uint16_t _adc1Values [ ADC1_CHANNEL_COUNT ];
extern volatile uint16_t _adc2Values [ ADC2_CHANNEL_COUNT ];
extern volatile uint16_t _adc3Values [ ADC3_CHANNEL_COUNT ];
extern volatile uint16_t _adc4Values [ ADC4_CHANNEL_COUNT ];

class Peripheral_M {
 public:
  void init ( peripheral_adc_pin _pin );
  uint16_t read ( peripheral_adc_pin _adc_pin );
};

void APIAdcInit ( void );

extern Peripheral_M Peripheral;

#ifdef __cplusplus
}
#endif
#endif
