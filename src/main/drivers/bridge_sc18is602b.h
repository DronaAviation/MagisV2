/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\bridge_sc18is602b.h                                #
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

#ifndef sc18_H
#define sc18_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "API/Peripherals.h"
/* Describes the pin modes a pin of the sc18 can be in */
enum SC18IS601B_GPIOPinMode {
  SC18IS601B_GPIO_MODE_QUASI_BIDIRECTIONAL = 0B00,
  SC18IS601B_GPIO_MODE_PUSH_PULL           = 0B01,
  SC18IS601B_GPIO_MODE_INPUT_ONLY          = 0B10,
  SC18IS601B_GPIO_MODE_OPEN_DRAIN          = 0B11
};

/* Describes the possible SPI speeds */
enum SC18IS601B_SPI_Speed {
  SC18IS601B_SPICLK_1843_kHz = 0B00, /* 1.8 MBit/s */
  SC18IS601B_SPICLK_461_kHz  = 0B01, /* 461 kbit/s */
  SC18IS601B_SPICLK_115_kHz  = 0B10, /* 115 kbit/s */
  SC18IS601B_SPICLK_58_kHz   = 0B11  /* 58 kbit/s */
};

/* Describes the possible SPI modes */
enum SC18IS601B_SPI_Mode {
  SC18IS601B_SPIMODE_0 = 0B00, /* CPOL: 0  CPHA: 0 */
  SC18IS601B_SPIMODE_1 = 0B01, /* CPOL: 0  CPHA: 1 */
  SC18IS601B_SPIMODE_2 = 0B10, /* CPOL: 1  CPHA: 0 */
  SC18IS601B_SPIMODE_3 = 0B11  /* CPOL: 1  CPHA: 1 */
};

enum SC18IS601B_GPIO : uint8_t {
  SS0 = 0,
  SS1 = 1,
  SS2 = 2,
  SS3 = 3
};

/* Function IDs */
#define SC18IS601B_CONFIG_SPI_CMD         0xF0
#define SC18IS601B_CLEAR_INTERRUPT_CMD    0xF1
#define SC18IS601B_IDLE_CMD               0xF2
#define SC18IS601B_GPIO_WRITE_CMD         0xF4
#define SC18IS601B_GPIO_READ_CMD          0xF5
#define SC18IS601B_GPIO_ENABLE_CMD        0xF6
#define SC18IS601B_GPIO_CONFIGURATION_CMD 0xF7

#define SC18IS601B_DATABUFFER_DEPTH       200

/**
 * @brief Configures the address of the SC18 module.
 *
 * The function calculates and sets the address for the SC18 module based on
 * the provided address pins. It combines a fixed base address with the state
 * of three address pins (a0, a1, a2) to form the complete module's address.
 *
 * @param _resetPin  Reset pin associated with the peripheral GPIO.
 * @param a0         Boolean value representing the state of address pin A0.
 * @param a1         Boolean value representing the state of address pin A1.
 * @param a2         Boolean value representing the state of address pin A2.
 */
void sc18_address_cfg ( peripheral_gpio_pin_e _resetPin, bool a0, bool a1, bool a2 );
/**
 * @brief Configures the address of the SC18 module.
 *
 * The function calculates and sets the address for the SC18 module by
 * combining a fixed base address with the state of three address pins
 * (a0, a1, a2) to form the complete module's address.
 *
 * @param a0  Boolean value representing the state of address pin A0.
 * @param a1  Boolean value representing the state of address pin A1.
 * @param a2  Boolean value representing the state of address pin A2.
 */
void sc18_address_cfg ( bool a0, bool a1, bool a2 );

/**
 * @brief Resets a peripheral device using a GPIO pin.
 *
 * This function resets a peripheral device by toggling the reset pin. It
 * initializes the pin as an output and then generates a high-to-low-to-high
 * transition to trigger a reset. The low state must last for at least 50ns,
 * but this implementation uses a delay of 1ms to ensure reliability.
 *
 * The function checks if the reset pin is valid (not equal to -1) before
 * attempting to perform the reset sequence.
 */
void sc18_reset ( );

/**
 * @brief Enables or disables a specified GPIO pin on the SC18IS601B device.
 *
 * This function checks if the specified GPIO pin is valid and then enables
 * or disables it based on the `enable` parameter. The function modifies only
 * the specified pin's state, leaving other GPIO configurations unchanged.
 * After updating the internal configuration, it sends the new settings to
 * the SC18IS601B device using I2C communication.
 *
 * @param _pin The GPIO pin to be enabled or disabled, should be in range [SS0, SS3].
 * @param enable A boolean indicating whether to enable (true) or disable (false) the pin.
 * @return True if the operation was successful, otherwise false.
 */
bool sc18_enableGPIO ( SC18IS601B_GPIO _pin, bool enable );

/**
 * @brief Configures the mode of a specified GPIO pin on the SC18IS601B device.
 *
 * This function validates the specified GPIO pin and then sets its mode
 * according to the provided `mode` parameter. The mode is represented as a
 * 2-bit wide bitfield, which is integrated into the existing configuration
 * without altering other pin settings. Once configured, the updated settings
 * are communicated to the SC18IS601B device via I2C.
 *
 * @param _pin The GPIO pin to be configured, should be in range [SS0, SS3].
 * @param mode The desired mode for the GPIO pin, defined by the SC18IS601B_GPIOPinMode enum.
 * @return True if the operation was successful, otherwise false.
 */
bool sc18_setupGPIO ( SC18IS601B_GPIO _pin, SC18IS601B_GPIOPinMode mode );

/**
 * @brief Writes a digital value to a specified GPIO pin on the SC18IS601B device.
 *
 * This function checks the validity of the specified GPIO pin and writes
 * a digital value to it. The updated GPIO value is then sent to the
 * SC18IS601B device through an I2C write operation.
 *
 * @param _pin The GPIO pin to which the value will be written, should be in range [SS0, SS3].
 * @param _val The digital value to write to the GPIO pin (true for high, false for low).
 * @return True if the operation was successful, otherwise false.
 */
bool sc18_writeGPIO ( SC18IS601B_GPIO _pin, bool value );

/**
 * @brief Reads the digital value from a specified GPIO pin on the SC18IS601B device.
 *
 * This function checks the validity of the specified GPIO pin and issues a read command
 * to the SC18IS601B device. The function retrieves the stored data from the device's buffer,
 * reads the desired GPIO pin state, and returns its value.
 *
 * @param _pin The GPIO pin from which the value will be read, should be in range [SS0, SS3].
 * @return The digital value at the specified GPIO pin (true for high, false for low), or
 *         false if the operation fails.
 */
bool sc18_readGPIO ( SC18IS601B_GPIO _pin );

/**
 * @brief Sets the SC18IS601B device into low power (idle) mode.
 *
 * This function sends an idle command to the SC18IS601B device to transition it
 * into a low power state. It utilizes the I2C communication protocol to issue
 * the command and returns whether the operation was successful.
 *
 * @return True if the low power mode command is successfully sent, false otherwise.
 */
bool sc18_setLowPowerMode ( );

/**
 * @brief Clears the interrupt on the SC18IS601B device.
 *
 * This function sends a command to clear any active interrupts on the
 * SC18IS601B device using the I2C communication protocol. It returns
 * a boolean indicating whether the command was successfully sent.
 *
 * @return True if the interrupt clear command is successfully sent, false otherwise.
 */
bool sc18_clearInterrupt ( );

/**
 * @brief Configures the SPI settings for the SC18IS601B device.
 *
 * This function sets up the SPI configuration for the SC18IS601B
 * including the bit order, mode, and clock speed. It performs a sanity
 * check on the `spiMode` parameter to ensure it is within valid range.
 *
 * @param lsbFirst If true, data is transferred least significant bit first; otherwise, most significant bit first.
 * @param spiMode The SPI mode (0-3) setting for clock polarity and phase.
 * @param clockSpeed The SPI clock speed setting.
 *
 * @return True if the configuration was successfully written; false if an invalid parameter was provided.
 */
bool sc18_configureSPI ( bool lsbFirst, SC18IS601B_SPI_Mode spiMode, SC18IS601B_SPI_Speed clockSpeed );

/**
 * @brief Performs an SPI transfer with the SC18IS601B device.
 *
 * This function initiates an SPI data transfer on the specified slave
 * select line of the SC18IS601B device. It sends data from a transmit
 * buffer and reads incoming data into a receive buffer. The function
 * returns a boolean indicating the success of the operation.
 *
 * @param slaveNum The GPIO pin used as the slave select (SS0 to SS3).
 * @param txData Pointer to the data buffer to be transmitted.
 * @param txLen Length of the data to be transmitted.
 * @param readBuf Pointer to the buffer where received data will be stored.
 *
 * @return True if the SPI transfer is successful, false otherwise.
 */
bool sc18_spiTransfer ( SC18IS601B_GPIO slaveNum, const uint8_t *txData, size_t txLen, uint8_t *readBuf );

/**
 * @brief Performs an SPI byte transfer with the SC18IS601B device.
 *
 * This function conducts an SPI data transfer of a single byte on the
 * specified slave select line of the SC18IS601B device. It sends a byte
 * and returns the byte received from the slave during the transfer.
 *
 * @param slaveNum The GPIO pin used as the slave select (SS0 to SS3).
 * @param txByte The byte to be transmitted.
 *
 * @return The byte received from the slave during the SPI transfer.
 */
uint8_t sc18_spiTransfer ( SC18IS601B_GPIO slaveNum, uint8_t txByte );

#endif