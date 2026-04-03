/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\bridge_sc18is602b.cpp                              #
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

#include "bridge_sc18is602b.h"
#include "platform.h"
#include "drivers/system.h"
#include "common/maths.h"

#include "bus_i2c.h"

uint8_t resetPin   = -1; /* reset pin */
uint8_t address    = 0;  /* address of the module */
uint8_t gpioEnable = 0;  /* last value for GPIO enable */
uint8_t gpioConfig = 0;  /* last value for GPIO configuration */
uint8_t gpioWrite  = 0;  /* last value for GPIO write */


void sc18_address_cfg(peripheral_gpio_pin_e _resetPin, bool a0, bool a1, bool a2) {
    // calculate the module's address here.
    // last 3 bit are the value of the address pin
    resetPin = (uint8_t)_resetPin;
    address  = 0B0101000 | (a2 << 2) | (a1 << 1) | (a0);
}



void sc18_address_cfg(bool a0, bool a1, bool a2) {
    // calculate the module's address here.
    // last 3 bit are the value of the address pin
    address = 0B0101000 | (a2 << 2) | (a1 << 1) | (a0);
}


/**
 * @brief Sends a command and data to an I2C device.
 *
 * This function writes a command byte followed by a sequence of data bytes 
 * to an I2C device specified by its address. The function constructs a buffer
 * containing the command and data, then sends it via the I2C interface.
 *
 * @param _address  The 7-bit I2C address of the target device.
 * @param _cmdByte  The command byte to be sent first.
 * @param _data     Pointer to the array of data bytes to be sent after the command.
 * @param _len      The number of data bytes in the _data array.
 * @return `true` if the write operation was successful, otherwise `false`.
 */
bool sc18_i2c_write(uint8_t _address, uint8_t _cmdByte, const uint8_t *_data, size_t _len) {
    uint8_t buf[256];
    buf[0] = _cmdByte;
    memcpy(&buf[1], _data, _len);
    return i2cWriteBufferwithoutregister(_address, _len + 1, buf);
}

/**
 * @brief Reads data from an I2C device into a buffer.
 *
 * This function attempts to read a specified number of bytes from an I2C device
 * at the given address, storing the data in the provided buffer. It repeatedly 
 * tries to read until data is successfully received, similar to a blocking 
 * behavior seen in Arduino's while loop.
 *
 * @param _address  The 7-bit I2C address of the target device.
 * @param _readBuf  Pointer to the buffer where the read data will be stored.
 * @param _len      The number of bytes to read from the device.
 * @return The number of bytes successfully read and stored in _readBuf.
 */
size_t sc18_i2c_read(uint8_t _address, uint8_t *_readBuf, size_t _len) {
    uint8_t received = 0;

    // Keep trying until data is received (like Arduino's while loop)
    while ((received = i2cReadwithoutregister(_address, (uint8_t)_len, _readBuf)) == 0);

    return received;
}



void sc18_reset() {
    if (resetPin != -1) {
        Peripheral_Init((peripheral_gpio_pin_e)resetPin, OUTPUT);
        // RESET is low active, LOW
        // Generate a high-to-low-to-high transition
        // must be at least 50ns long (t_sa). 1ms is enough.
        Peripheral_Write((peripheral_gpio_pin_e)resetPin, STATE_HIGH);
        delay(1);
        Peripheral_Write((peripheral_gpio_pin_e)resetPin, STATE_LOW);
        delay(1);
        Peripheral_Write((peripheral_gpio_pin_e)resetPin, STATE_HIGH);
    }
}



bool sc18_enableGPIO(SC18IS601B_GPIO _pin, bool enable) {
    // sanity check
    if (_pin < SS0 || _pin > SS3)
        return false;
    // enable this GPIO while leaving the others untouched.
    bitWrite(gpioEnable, _pin, enable);
    // Send the new enable configuration
    return sc18_i2c_write(address, SC18IS601B_GPIO_ENABLE_CMD, &gpioEnable, sizeof(gpioEnable));
}


bool sc18_setupGPIO(SC18IS601B_GPIO _pin, SC18IS601B_GPIOPinMode mode) {
    // sanity check
    if (_pin < SS0 || _pin > SS3)
        return false;

    // Cast the enum back to the bits
    // mode is a 2-bit wide bitfield
    uint8_t modeAsBitfield = (uint8_t) mode;

    // write 2 the bits into our last config value
    // refer to table 10 in the datasheet
    bitWrite(gpioConfig, 2 * _pin, modeAsBitfield & 1);
    bitWrite(gpioConfig, 2 * _pin + 1, modeAsBitfield >> 1);

    return sc18_i2c_write(address, SC18IS601B_GPIO_CONFIGURATION_CMD, &gpioConfig, sizeof(gpioConfig));
}


bool sc18_writeGPIO(SC18IS601B_GPIO _pin, bool _val) {
    if (_pin < SS0 || _pin > SS3)
        return false;
    bitWrite(gpioWrite, _pin, _val);
    return sc18_i2c_write(address, SC18IS601B_GPIO_WRITE_CMD, &gpioWrite, sizeof(gpioWrite));
}



bool sc18_readGPIO(int _pin) {
    if (_pin < SS0 || _pin > SS3)
        return false;

    // Refer chapter 7.1.9: issue a read command.
    // This will cause the storage of 1 byte in the data buffer.
    if (!sc18_i2c_write(address, SC18IS601B_GPIO_READ_CMD, nullptr, 0))
        return false;

    // Now try to read the buffer.
    uint8_t gpioReadBuf = 0;
    size_t readBytes = sc18_i2c_read(address, &gpioReadBuf, sizeof(gpioReadBuf));

    if (readBytes == 0)
        return false;

    // Return the bit at the needed position.
    return bitRead(gpioReadBuf, _pin);
}



bool sc18_setLowPowerMode() {
    return sc18_i2c_write(address, SC18IS601B_IDLE_CMD, nullptr, 0);
}



bool sc18_clearInterrupt() {
    return sc18_i2c_write(address, SC18IS601B_CLEAR_INTERRUPT_CMD, nullptr, 0);
}


bool sc18_spiTransfer(SC18IS601B_GPIO slaveNum, const uint8_t *txData, size_t txLen, uint8_t *readBuf) {
    // sanity check
    if (slaveNum < SS0 || slaveNum > SS3)
        return false;

    // Overly long data?
    if (txLen > SC18IS601B_DATABUFFER_DEPTH)
        return false;

    // the function ID will have the lower 4 bits set to the
    // activated slave selects. We use only 1 at a time here.
    uint8_t functionID = (1 << slaveNum);
    // transmit our TX buffer
    if (!sc18_i2c_write(address, functionID, txData, txLen))
        return false;
    // read in the data that came from MISO
    return sc18_i2c_read(address, readBuf, txLen);
}


uint8_t sc18_spiTransfer(SC18IS601B_GPIO slaveNum, uint8_t txByte) {
    uint8_t readBuf = 0;
    sc18_spiTransfer(slaveNum, &txByte, 1, &readBuf);
    return readBuf;
}



bool sc18_configureSPI(bool lsbFirst, SC18IS601B_SPI_Mode spiMode, SC18IS601B_SPI_Speed clockSpeed) {
    // sanity check on parameters
    if (spiMode > SC18IS601B_SPIMODE_3)
        return false;
    uint8_t clk = (uint8_t)((uint8_t)(clockSpeed) & 0B11);

    // see chapter 7.1.5
    uint8_t configByte = (lsbFirst << 5) | (spiMode << 2) | clk;
    return sc18_i2c_write(address, SC18IS601B_CONFIG_SPI_CMD, &configByte, sizeof(configByte));
}
