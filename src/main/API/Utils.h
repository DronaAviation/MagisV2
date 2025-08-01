/*
 * This file is part of Magis.
 *
 * Magis is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Magis is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


class Graph_P {
 public:
  void red ( double value, uint8_t precision = 4 );

  void green ( double value, uint8_t precision = 4 );

  void blue ( double value, uint8_t precision = 4 );
};

class Interval {

 private:
  uint32_t time;
  uint32_t loopTime;
  bool repeat;

 public:
  bool set ( uint32_t time, bool repeat );    // time is in milliseconds
  void reset ( void );
  bool check ( );
};

class Monitor_P {

 public:
  void print ( const char *msg );

  void print ( const char *tag, int number );

  void print ( const char *tag, double number, uint8_t precision );

  void println ( const char *msg );

  void println ( const char *tag, int number );

  void println ( const char *tag, double number, uint8_t precision );
};

class Oled_P {

 public:
  void init ( void );
  void clear ( void );
  void print ( uint8_t col, uint8_t row, const char *string );
};

uint32_t micros ( void );

uint32_t millis ( void );

extern Graph_P Graph;
extern Monitor_P Monitor;
extern Oled_P Oled;

#ifdef __cplusplus
}
#endif
