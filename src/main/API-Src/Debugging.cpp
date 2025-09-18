/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\Debugging.cpp                                      #
 #  Created Date: Sun, 24th Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sun, 24th Aug 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"

#include "drivers/display_ug2864hsweg01.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/system.h"

#include "blackbox/blackbox.h"

#include "io/serial_msp.h"

#include "API/API-Utils.h"
#include "API/Debugging.h"

static uint8_t checksum;

// Function to serialize a string for debugging purposes.
void serializeString ( const char *msg ) {
  // Iterate over each character in the string.
  for ( uint8_t i = 0; i < strlen ( msg ); i++ ) {
    // Serialize each character using serialize8Debug.
    serialize8Debug ( msg [ i ] );

    // Update the checksum by XORing it with the current character.
    checksum ^= msg [ i ];
  }
}

// Function to debug print a message string.
void debugPrint ( const char *msg ) {

  // Initialize the checksum to zero for error detection.
  checksum = 0;

  // Serialize special debug start markers '$' and 'D'.
  serialize8Debug ( '$' );
  serialize8Debug ( 'D' );

  // Serialize the length of the message.
  serialize8Debug ( strlen ( msg ) );

  // Update checksum with the length of the message.
  checksum ^= strlen ( msg );

  // Call serializeString to handle the serialization of the message itself.
  serializeString ( msg );

  // Serialize the final checksum for error detection.
  serialize8Debug ( checksum );
}

// Function to debug print a message and a floating-point number with specified precision.
void debugPrint ( const char *msg, double number, uint8_t digits ) {

  // Initialize checksum variable used for error detection.
  checksum = 0;

  // Serialize special debug start markers '$' and 'D'.
  serialize8Debug ( '$' );
  serialize8Debug ( 'D' );

  // Check if the number is NaN (Not a Number).
  if ( isnan ( number ) ) {
    const char *err   = "not a number";
    uint8_t data_size = strlen ( msg ) + strlen ( err );
    serialize8Debug ( data_size );
    checksum ^= data_size;
    serializeString ( msg );
    serialize8Debug ( '\t' );
    checksum ^= '\t';
    serializeString ( err );
    serialize8Debug ( checksum );
    return;
  }

  // Check if the number is infinite.
  if ( isinf ( number ) ) {
    const char *err   = "infinite";
    uint8_t data_size = ( uint8_t ) ( strlen ( msg ) + strlen ( err ) );
    serialize8Debug ( data_size );
    checksum ^= data_size;
    serializeString ( msg );
    serialize8Debug ( '\t' );
    checksum ^= '\t';
    serializeString ( err );
    serialize8Debug ( checksum );
    return;
  }

  // Check for overflow conditions.
  if ( number > ( double ) 4294967040.0 || number < ( double ) -4294967040.0 ) {
    const char *err   = "overflow";
    uint8_t data_size = strlen ( msg ) + strlen ( err ) + 1;
    serialize8Debug ( data_size );
    checksum ^= data_size;
    serializeString ( msg );
    serialize8Debug ( '\t' );
    checksum ^= '\t';
    serializeString ( err );
    serialize8Debug ( checksum );
    return;
  }

  // Determine if the number is negative and convert it to positive if so.
  bool isNumNeg = false;
  if ( number < ( double ) 0.0 ) {
    number   = -number;
    isNumNeg = true;
  }

  // Adjust the number for rounding based on the specified digit precision.
  double rounding = 0.5;
  for ( uint8_t i = 0; i < digits; ++i )
    rounding /= ( double ) 10.0;
  number += rounding;

  // Extract the integer part of the number.
  uint32_t int_part = ( uint32_t ) number;
  double remainder  = number - ( double ) int_part;

  // Buffer to hold the string representation of the integer part.
  char buf [ 8 * sizeof ( int_part ) + 1 ];
  char *str = &buf [ sizeof ( buf ) - 1 ];

  // Null-terminate the string initially.
  *str = '\0';

  // Convert the integer part to a string in reverse order.
  do {
    char digit = int_part % 10;
    int_part /= 10;
    *--str = digit < 10 ? digit + '0' : digit + 'A' - 10;
  } while ( int_part );

  // Calculate the total data size including message, integer part, sign, decimal point, and fractional part.
  uint8_t data_size = ( uint8_t ) strlen ( msg ) + ( uint8_t ) strlen ( str )
                      + ( isNumNeg ? 1 : 0 ) + ( digits > 0 ? ( digits + 1 ) : 0 ) + 1;

  // Serialize the data size.
  serialize8Debug ( data_size );

  // Update checksum with the data size.
  checksum ^= data_size;

  // Serialize the message string.
  serializeString ( msg );

  // Serialize a tab character as a delimiter between message and number.
  serialize8Debug ( '\t' );
  checksum ^= '\t';

  // If the number is negative, serialize the negative sign.
  if ( isNumNeg ) {
    serialize8Debug ( '-' );
    checksum ^= '-';
  }

  // Serialize the string representation of the integer part.
  serializeString ( str );

  // If there are digits to display after the decimal point, handle them.
  if ( digits > 0 ) {
    // Serialize the decimal point.
    serialize8Debug ( '.' );
    checksum ^= '.';
  }

  // Serialize each digit of the fractional part.
  while ( digits-- > 0 ) {
    remainder *= ( double ) 10.0;
    uint8_t toPrint = ( uint8_t ) ( remainder );
    toPrint         = toPrint < 10 ? toPrint + '0' : toPrint + 'A' - 10;
    serialize8Debug ( toPrint );
    checksum ^= toPrint;
    remainder -= toPrint;
  }

  // Serialize the final checksum for error detection.
  serialize8Debug ( checksum );
}

// Function to debug print a message and an integer number.
void debugPrint ( const char *msg, int number ) {

  // Initialize checksum variable used for error detection.
  checksum = 0;

  // Serialize special debug start markers '$' and 'D'.
  serialize8Debug ( '$' );
  serialize8Debug ( 'D' );

  // Determine if the number is negative and convert it to positive if so.
  bool isNumNeg = false;
  if ( number < 0 ) {
    number   = -number;
    isNumNeg = true;
  }

  // Buffer to hold the string representation of the number.
  char buf [ 8 * sizeof ( number ) + 1 ];

  // Pointer to the end of the buffer.
  char *str = &buf [ sizeof ( buf ) - 1 ];

  // Null-terminate the string initially.
  *str = '\0';

  // Convert the number to a string in reverse order.
  do {
    char digit = number % 10;                                // Get the last digit.
    number /= 10;                                            // Remove the last digit.
    *--str = digit < 10 ? digit + '0' : digit + 'A' - 10;    // Convert digit to character.
  } while ( number );

  // Calculate the total data size including message, number, sign, and delimiter.
  uint8_t data_size = ( uint8_t ) strlen ( msg ) + ( uint8_t ) strlen ( str )
                      + ( isNumNeg ? 1 : 0 ) + 1;

  // Serialize the data size.
  serialize8Debug ( data_size );

  // Update checksum with the data size.
  checksum ^= data_size;

  // Serialize the message string.
  serializeString ( msg );

  // Serialize a tab character as a delimiter between message and number.
  serialize8Debug ( '\t' );

  // Update checksum with the tab character.
  checksum ^= '\t';

  // If the number is negative, serialize the negative sign.
  if ( isNumNeg ) {
    serialize8Debug ( '-' );
    checksum ^= '-';
  }

  // Serialize the string representation of the number.
  serializeString ( str );

  // Serialize the final checksum for error detection.
  serialize8Debug ( checksum );
}

// Function to print a message to the debug monitor.
void Monitor_Print ( const char *msg ) {
  // Call the debugPrint function to output the message.
  debugPrint ( msg );
}

// Function to print a tag and an integer to the debug monitor.
void Monitor_Print ( const char *tag, int number ) {
  // Call the debugPrint function to output the tag and integer.
  debugPrint ( tag, number );
}

// Function to print a tag and a double with specified precision to the debug monitor.
void Monitor_Print ( const char *tag, double number, uint8_t precision ) {
  // Constrain the precision value between 0 and 7.
  precision = constrain ( precision, 0, 7 );
  // Call the debugPrint function to output the tag, number, and precision.
  debugPrint ( tag, number, precision );
}

// Function to print a message followed by a new line to the debug monitor.
void Monitor_Println ( const char *msg ) {
  // Call the debugPrint function to output the message.
  debugPrint ( msg );
  // Print a newline character.
  debugPrint ( "\n" );
}

// Function to print a tag and an integer followed by a new line to the debug monitor.
void Monitor_Println ( const char *tag, int number ) {
  // Call the debugPrint function to output the tag and integer.
  debugPrint ( tag, number );
  // Print a newline character.
  debugPrint ( "\n" );
}

// Function to print a tag and a double with specified precision followed by a new line to the debug monitor.
void Monitor_Println ( const char *tag, double number, uint8_t precision ) {
  // Constrain the precision value between 0 and 7.
  precision = constrain ( precision, 0, 7 );
  // Call the debugPrint function to output the tag, number, and precision.
  debugPrint ( tag, number, precision );
  // Print a newline character.
  debugPrint ( "\n" );
}

// Function to initialize the OLED display.
void Oled_Init ( void ) {

  // Set the flag to enable the OLED display.
  OledEnable = true;
}

// Function to clear the OLED display.
void Oled_Clear ( void ) {

  // Check if the OLED display is enabled (OledEnable is true).
  if ( OledEnable ) {

    // Clear the OLED display quickly using an I2C command.
    i2c_OLED_clear_display_quick ( );
  }
}

// Function to print a string on an OLED display at a specified column and row.
void Oled_Print ( uint8_t col, uint8_t row, const char *string ) {

  // Check if the column is within the range [0, 20] and the row is within the range [1, 6],
  // and also ensure that the OLED display is enabled (OledEnable is true).
  if ( ( col >= 0 && col <= 20 ) && ( row >= 1 && row <= 6 ) && OledEnable ) {

    // Set the cursor position on the OLED display to the specified column and row.
    i2c_OLED_set_xy ( col, row );

    // Send the string to be displayed on the OLED screen.
    i2c_OLED_send_string ( string );
  }
}

#ifdef BLACKBOX
void BlackBox_setVar ( char *varName, int32_t &reference ) {
  // Calls the function to set the user - defined field in the Black Box
  // with the given variable name and its associated integer reference.
  setUserBlackBoxField ( varName, reference );
}
#endif