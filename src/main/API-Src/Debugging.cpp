  /******************************************************************************
   #  SPDX-License-Identifier: GPL-3.0-or-later                                 #
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
  #  Last Modified: Sun, 15th Nov 2026                                          #
  #  Modified By: AJ                                                            #
  #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
  #  HISTORY:                                                                   #
  #  Date      	By	Comments                                                    #
  #  ----------   ----------------  ------------------------------------------  #
  #  2025-09-17   Omkar Dandekar     Removed Oled functionality,                #      
                                      Check out Oled.h / Oled.cpp               #
  #  2026-11-15   AJ                 Double print: digits after the first       #
                                      decimal no longer print 0 ( B9 );         #
                                      precision clamped to 0..4; strings cut    #
                                      at 255 and frame lengths corrected;       #
                                      INT_MIN prints correctly                  #
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

  // Most digits after the decimal point the double overloads print ( a larger request is clamped ).
  #define MONITOR_MAX_PRECISION 4

  // Largest payload the one-byte length field of a debug frame can carry. Strings are cut
  // so that the whole payload ( tag, tab and number ) fits; the number is never cut.
  #define MONITOR_MAX_PAYLOAD 255

  static uint8_t checksum;

  // Length of a string, counted only up to max characters ( a NULL string counts as empty ).
  // Bounded so that a long string can neither wrap the length byte nor stall the loop.
  static uint8_t boundedLength ( const char *s, uint8_t max ) {
    uint8_t len = 0;
    if ( s != NULL ) {
      while ( len < max && s [ len ] != '\0' ) {
        len++;
      }
    }
    return len;
  }

  // Function to serialize the first len characters of a string for debugging purposes.
  static void serializeString ( const char *msg, uint8_t len ) {
    // Iterate over each character in the string.
    for ( uint8_t i = 0; i < len; i++ ) {
      // Serialize each character using serialize8Debug.
      serialize8Debug ( msg [ i ] );

      // Update the checksum by XORing it with the current character.
      checksum ^= msg [ i ];
    }
  }

  // Send one frame whose payload is "tag <tab> text", the tag cut so that the payload fits.
  static void debugPrintTagText ( const char *msg, const char *text ) {
    uint8_t textLen   = boundedLength ( text, MONITOR_MAX_PAYLOAD - 1 );
    uint8_t msgLen    = boundedLength ( msg, ( uint8_t ) ( MONITOR_MAX_PAYLOAD - 1 - textLen ) );
    uint8_t data_size = ( uint8_t ) ( msgLen + 1 + textLen );    // + 1: the tab

    serialize8Debug ( data_size );
    checksum ^= data_size;
    serializeString ( msg, msgLen );
    serialize8Debug ( '\t' );
    checksum ^= '\t';
    serializeString ( text, textLen );
    serialize8Debug ( checksum );
  }

  // Function to debug print a message string.
  void debugPrint ( const char *msg ) {

    // Initialize the checksum to zero for error detection.
    checksum = 0;

    // Serialize special debug start markers '$' and 'D'.
    serialize8Debug ( '$' );
    serialize8Debug ( 'D' );

    // Length of the message, cut at MONITOR_MAX_PAYLOAD characters.
    uint8_t msgLen = boundedLength ( msg, MONITOR_MAX_PAYLOAD );

    // Serialize the length of the message.
    serialize8Debug ( msgLen );

    // Update checksum with the length of the message.
    checksum ^= msgLen;

    // Call serializeString to handle the serialization of the message itself.
    serializeString ( msg, msgLen );

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

    // Check if the number is NaN (Not a Number). The length counts the tab ( it did not before ).
    if ( isnan ( number ) ) {
      debugPrintTagText ( msg, "not a number" );
      return;
    }

    // Check if the number is infinite.
    if ( isinf ( number ) ) {
      debugPrintTagText ( msg, "infinite" );
      return;
    }

    // Check for overflow conditions.
    if ( number > ( double ) 4294967040.0 || number < ( double ) -4294967040.0 ) {
      debugPrintTagText ( msg, "overflow" );
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

    // Length of the number: sign, integer part ( 10 digits at most ), decimal point and fractional part.
    uint8_t strLen = boundedLength ( str, ( uint8_t ) ( sizeof ( buf ) - 1 ) );
    uint8_t numLen = ( uint8_t ) ( ( isNumNeg ? 1 : 0 ) + strLen + ( digits > 0 ? ( digits + 1 ) : 0 ) );

    // The message is cut so that message, tab and number fit in the one-byte length field.
    uint8_t msgLen = boundedLength ( msg, ( uint8_t ) ( MONITOR_MAX_PAYLOAD - 1 - numLen ) );

    // Calculate the total data size including message, tab and number.
    uint8_t data_size = ( uint8_t ) ( msgLen + 1 + numLen );

    // Serialize the data size.
    serialize8Debug ( data_size );

    // Update checksum with the data size.
    checksum ^= data_size;

    // Serialize the message string.
    serializeString ( msg, msgLen );

    // Serialize a tab character as a delimiter between message and number.
    serialize8Debug ( '\t' );
    checksum ^= '\t';

    // If the number is negative, serialize the negative sign.
    if ( isNumNeg ) {
      serialize8Debug ( '-' );
      checksum ^= '-';
    }

    // Serialize the string representation of the integer part.
    serializeString ( str, strLen );

    // If there are digits to display after the decimal point, handle them.
    if ( digits > 0 ) {
      // Serialize the decimal point.
      serialize8Debug ( '.' );
      checksum ^= '.';
    }

    // Serialize each digit of the fractional part.
    // The digit value ( 0-9 ) is taken off the remainder before it is turned into ASCII:
    // subtracting the character ( '0' + d ) made the remainder negative, and every later
    // digit then printed as 0 ( API audit B9: 3.14159 @ 3 printed 3.100 ).
    while ( digits-- > 0 ) {
      remainder *= ( double ) 10.0;
      uint8_t digit = ( uint8_t ) ( remainder );
      if ( digit > 9 ) {
        digit = 9;    // remainder < 1 keeps this at 9 or less; guard only
      }
      remainder -= ( double ) digit;
      uint8_t toPrint = ( uint8_t ) ( '0' + digit );
      serialize8Debug ( toPrint );
      checksum ^= toPrint;
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

    // Determine if the number is negative and take its magnitude in an unsigned type.
    // Negating in unsigned arithmetic is defined for every value: -INT_MIN overflowed an int
    // and stayed negative, so its digits printed as garbage characters ( D3 ).
    static_assert ( sizeof ( int ) <= sizeof ( uint32_t ), "magnitude must hold every int" );
    bool isNumNeg      = false;
    uint32_t magnitude = ( uint32_t ) number;
    if ( number < 0 ) {
      magnitude = 0u - magnitude;
      isNumNeg  = true;
    }

    // Buffer to hold the string representation of the number.
    char buf [ 8 * sizeof ( magnitude ) + 1 ];

    // Pointer to the end of the buffer.
    char *str = &buf [ sizeof ( buf ) - 1 ];

    // Null-terminate the string initially.
    *str = '\0';

    // Convert the number to a string in reverse order.
    do {
      uint8_t digit = ( uint8_t ) ( magnitude % 10u );    // Get the last digit ( 0-9 ).
      magnitude /= 10u;                                    // Remove the last digit.
      *--str = ( char ) ( '0' + digit );                   // Convert digit to character.
    } while ( magnitude );

    // Length of the number: sign and digits ( 10 digits at most ).
    uint8_t strLen = boundedLength ( str, ( uint8_t ) ( sizeof ( buf ) - 1 ) );
    uint8_t numLen = ( uint8_t ) ( ( isNumNeg ? 1 : 0 ) + strLen );

    // The message is cut so that message, tab and number fit in the one-byte length field.
    uint8_t msgLen = boundedLength ( msg, ( uint8_t ) ( MONITOR_MAX_PAYLOAD - 1 - numLen ) );

    // Calculate the total data size including message, tab and number.
    uint8_t data_size = ( uint8_t ) ( msgLen + 1 + numLen );

    // Serialize the data size.
    serialize8Debug ( data_size );

    // Update checksum with the data size.
    checksum ^= data_size;

    // Serialize the message string.
    serializeString ( msg, msgLen );

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
    serializeString ( str, strLen );

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
    // Constrain the precision value between 0 and MONITOR_MAX_PRECISION.
    precision = constrain ( precision, 0, MONITOR_MAX_PRECISION );
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
    // Constrain the precision value between 0 and MONITOR_MAX_PRECISION.
    precision = constrain ( precision, 0, MONITOR_MAX_PRECISION );
    // Call the debugPrint function to output the tag, number, and precision.
    debugPrint ( tag, number, precision );
    // Print a newline character.
    debugPrint ( "\n" );
  }


  #ifdef BLACKBOX
  void BlackBox_setVar ( char *varName, int32_t &reference ) {
    // Calls the function to set the user - defined field in the Black Box
    // with the given variable name and its associated integer reference.
    setUserBlackBoxField ( varName, reference );
  }
  #endif