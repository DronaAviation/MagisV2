/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\Peripheral.cpp                                         #
 #  Created Date: Thu, 8th May 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sat, 6th Sep 2025                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include "Peripheral.h"

#include "platform.h"
#include "build_config.h"
#include "common/utils.h"
#include "common/atomic.h"

#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"
#include "drivers/timer_stm32f30x.h"
#include "drivers/timer_impl.h"
#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "sensors/sensors.h"

#include "io/serial.h"
#include "io/display.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/gps_conversion.h"
#include "flight/navigation.h"

#include "config/config.h"
#include "config/runtime_config.h"

// #include "Control.h"
#include "XRanging.h"
#include "API/Scheduler-Timer.h"

#include "API-Utils.h"


#define DISABLE_SPI       GPIO_SetBits(GPIOB,   GPIO_Pin_5)
#define ENABLE_SPI        GPIO_ResetBits(GPIOB, GPIO_Pin_5)

bool gpioReset = false;
bool changeAdress = false;

pwmOutputPort_t* pwm[11];




portMode_t mapped_mode;
portOptions_t mapped_options;



bool I2C_P::read(uint8_t device_add, uint8_t reg,uint8_t &value)
{

    return i2cRead(device_add, reg, 1, &value);
}



int16_t I2C_P::read(uint8_t device_add, uint8_t reg, uint32_t length,uint8_t* buffer)
{

    return i2cRead(device_add, reg, length, buffer);
}

bool I2C_P::write(uint8_t device_add, uint8_t reg, uint8_t data)
{

    return i2cWrite(device_add, reg, data);
}

bool I2C_P::write(uint8_t device_add, uint8_t reg, uint32_t length, uint8_t* data)
{

    return i2cWriteBuffer(device_add, reg, length, data);
}



uint16_t getPrescaler(uint16_t speed)
{
    uint16_t prescaler = 0;
    switch (speed) {
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


void SPI_P::init()
{
    spiInit (SPI2);
}

void SPI_P::init(SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit)
{
    uint16_t prescaler = getPrescaler(speed);
    SPI_InitTypeDef spi;

    SPI_I2S_DeInit (SPI2);

    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    switch (mode) {
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
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = prescaler;
    if ((bit == LSBFIRST)) {
        spi.SPI_FirstBit = SPI_FirstBit_LSB;
    } else if ((bit == MSBFIRST)) {
        spi.SPI_FirstBit = SPI_FirstBit_MSB;
    }
    spi.SPI_CRCPolynomial = 7;

#ifdef STM32F303xC
    // Configure for 8-bit reads.
    SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);
#endif
    SPI_Init(SPI2, &spi);
    SPI_Cmd(SPI2, ENABLE);

    // Drive NSS high to disable connected SPI device.
    DISABLE_SPI;
}


void SPI_P::enable(void)
{
    ENABLE_SPI;
}

void SPI_P::disable(void)
{
    DISABLE_SPI;
}

uint8_t SPI_P::read(uint8_t register_address)
{

       uint8_t value=0;


      register_address &=  ~0x80u;

	//    opticFlowAddress=spi.Read(0x4E, 1);



	    ENABLE_SPI;

	//    GPIO.write(Pin14, STATE_LOW);

	    delayMicroseconds(50);


	    spiTransferByte(SPI2, register_address);

	    delayMicroseconds(50);

	    spiTransfer(SPI2, &value, NULL, 1);

	    delayMicroseconds(50);

	//    GPIO.write(Pin14, STATE_HIGH);

	    DISABLE_SPI;

	    delayMicroseconds(200);


        return value;

}

void SPI_P::read(uint8_t register_address, int16_t length,uint8_t* buffer)
{


      register_address &=  ~0x80u;

    //    opticFlowAddress=spi.Read(0x4E, 1);



        ENABLE_SPI;

    //    GPIO.write(Pin14, STATE_LOW);

        delayMicroseconds(50);


        spiTransferByte(SPI2, register_address);

        delayMicroseconds(50);

        spiTransfer(SPI2, buffer, NULL, length);

        delayMicroseconds(50);

    //    GPIO.write(Pin14, STATE_HIGH);

        DISABLE_SPI;

        delayMicroseconds(200);




}


void SPI_P::write( uint8_t register_address, uint8_t data)
{
//    ENABLE_SPI;
//    spiTransferByte(SPI2, register_address);
//    spiTransferByte(SPI2, data);
//    DISABLE_SPI;

       register_address  |= 0x80u;

	   ENABLE_SPI;

//	    GPIO.write(Pin14, STATE_LOW);

	    delayMicroseconds(50);

	    spiTransferByte(SPI2, register_address);

	    delayMicroseconds(50);


  	   spiTransferByte(SPI2, data);

	    delayMicroseconds(50);


//	    GPIO.write(Pin14, STATE_HIGH);

	    DISABLE_SPI;

	    delayMicroseconds(200);


}


I2C_P I2C;
SPI_P SPI;


