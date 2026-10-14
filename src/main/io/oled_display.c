/*******************************************************************************
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: oled_display.c                                                       #
 #  Created Date: Sat, 25th Jan 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Wed, 14th Oct 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2025-09-XX   Omkar Dandekar     Introduced OLED ownership control via      #
 #                                 oledMode (SYSTEM / USER). System rendering  #
 #                                 is now disabled when OLED is owned by user  #
 #                                 code, enabling safe custom UI, graphics,    #
 #                                 and PlutoBlocks-based OLED usage without    #
 #                                 interfering with flight telemetry.          #
 #  2026-10-10   AJ    System telemetry drawn into a RAM framebuffer; every    #
 #                     panel write goes through a diff flush of at most 16     #
 #                     bytes per loop ( was 45-60 ms of blocking I2C every     #
 #                     200 ms while armed: 5 Hz twitch in flight ).            #
 *******************************************************************************/
#include "oled_display.h"

#include <string.h>

#include "platform.h"

#include "common/printf.h"

#include "common/maths.h"
#include "common/axis.h"

#include "config/runtime_config.h"

#include "drivers/sensor.h"
#include "drivers/display_ug2864hsweg01.h"
#include "drivers/system.h"
#include "drivers/accgyro.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "io/rc_controls.h"

#include "rx/rx.h"
#include "version.h"

/* ============================================================================
 * GLOBAL FLAGS & VARIABLES
 * ============================================================================ */

bool OledEnable = false;

// Buffer for storing text to be displayed on the OLED
char buffer [ 20 ];

// Pointer to a message string, initialized to NULL (no message)
const char *message = NULL;

// Timing variables
unsigned long updatedMillis            = 0;    // Stores current time in milliseconds
unsigned long lastOledStartUpMillis    = 0;    // Time when OLED startup was initiated
unsigned long lastOledClearMillis      = 0;    // Unused in this code snippet
unsigned long OldOledDisplayDataMillis = 0;    // Last time display data was updated

// Constants for timing intervals
const int OledStartUpTime = 2000;    // Duration for the OLED startup screen
const int OledClearTime   = 500;     // Unused in this code snippet

// Stopwatch timing variables
unsigned long SwStartMillis, SwElapsedMillis;

// State flags
bool devmode            = false;    // Development mode flag
bool StopWatchRunning   = false;    // Tracks if the stopwatch is running
bool OledInitStatus     = false;    // Status of OLED initialization
bool devModeLastState   = false;    // Previous state of development mode
bool OledStartupPageEnd = false;    // Indicates if the startup page has ended

// Blinking control variables
int blink_at      = 2;    // Blink interval
int blink_counter = 0;    // Counter for blinking logic

/**
 * @brief Current OLED ownership mode.
 */
oled_mode_e oledMode = OLED_MODE_SYSTEM;

/**
 * @brief Shadow framebuffer: what the panel is showing right now.
 *
 * Every write to the panel after start-up goes through oledFlushStep ( ), which
 * keeps this in step with the hardware. A direct hardware clear must zero it.
 */
static uint8_t oledShadowBuffer [ 1024 ];

/**
 * @brief System telemetry framebuffer. The system screen is drawn here ( no I2C )
 * and reaches the panel through the same flush as user frames.
 */
static uint8_t oledSystemBuffer [ 1024 ];

/**
 * @brief Frame the flush is sending: oledSystemBuffer, or the user's buffer
 * after Oled_display_Update ( ).
 */
static uint8_t *oledActiveBuffer = oledSystemBuffer;

static bool oledDirty        = false;    // active frame may differ from the panel
static uint8_t oledFlushPage = 0;        // page the flush is working on ( 0-7 )
static uint8_t oledFailScore = 0;        // leaky count of failed I2C runs

/**
 * Data bytes sent per flush step. I2C1 is polled and shared with the IMU, baro and
 * mag, so each step blocks loop ( ) between the IMU read and the PID: 16 bytes plus
 * the page / column commands is about 0.6 ms at 400 kHz. A full 1024-byte screen
 * takes about 64 loops ( ~225 ms at looptime 3500 us ).
 */
#define OLED_FLUSH_MAX_BYTES 16

// Failed runs add OLED_FAIL_COST to a score, good runs take 1 off. At OLED_FAIL_LIMIT the
// panel is treated as gone and the OLED is switched off, so neither a dead display ( 3
// failures in a row ) nor a flaky one ( more than 1 run in 5 failing ) keeps holding up
// the loop on I2C timeouts and bus resets.
#define OLED_FAIL_COST  4
#define OLED_FAIL_LIMIT 12

#define JOY_RADIUS      18
#define LEFT_JOY_X      32
#define RIGHT_JOY_X     96
#define JOY_Y           32
#define STICK_RADIUS    3

/* ============================================================================
 * FRAME FLUSH ( RATE-LIMITED, DIFF-BASED )
 * ============================================================================ */

/**
 * @brief The panel was cleared directly: the shadow must say so too.
 */
static void oledPanelCleared ( void ) {
  memset ( oledShadowBuffer, 0, sizeof ( oledShadowBuffer ) );
  oledDirty = true;
}

/**
 * @brief Send at most one run of changed bytes ( <= OLED_FLUSH_MAX_BYTES ).
 *
 * Called once per loop. Finds the next byte of the active frame that differs from
 * the shadow, sends it together with any changed bytes up to the budget, and
 * returns. Unchanged bytes inside the run are re-sent, which is cheaper than a new
 * transaction. When a full pass finds no difference the frame is on the panel and
 * the flush goes idle until the next frame is marked dirty.
 */
static void oledFlushStep ( void ) {
  if ( ! oledDirty || ! oledActiveBuffer ) return;

  for ( uint8_t n = 0; n < 8; n++ ) {
    uint16_t base = ( uint16_t ) ( oledFlushPage * SCREEN_WIDTH );

    for ( uint8_t col = 0; col < SCREEN_WIDTH; col++ ) {
      if ( oledActiveBuffer [ base + col ] == oledShadowBuffer [ base + col ] ) continue;

      uint8_t last = col;
      for ( uint8_t c = col; c < SCREEN_WIDTH && ( uint8_t ) ( c - col ) < OLED_FLUSH_MAX_BYTES; c++ ) {
        if ( oledActiveBuffer [ base + c ] != oledShadowBuffer [ base + c ] ) last = c;
      }
      uint8_t len = ( uint8_t ) ( last - col + 1 );

      if ( i2c_OLED_send_run ( oledFlushPage, col, &oledActiveBuffer [ base + col ], len ) ) {
        memcpy ( &oledShadowBuffer [ base + col ], &oledActiveBuffer [ base + col ], len );
        if ( oledFailScore > 0 ) oledFailScore--;
      } else {
        oledFailScore = ( uint8_t ) ( oledFailScore + OLED_FAIL_COST );
        if ( oledFailScore >= OLED_FAIL_LIMIT ) {
          OledInitStatus = false;    // display not answering reliably: stop using the bus for it
        }
      }
      return;    // one run per loop
    }

    oledFlushPage = ( uint8_t ) ( ( oledFlushPage + 1 ) & 0x07 );
  }

  oledDirty = false;    // the whole frame is on the panel
}

/**
 * @brief Draw text into the system framebuffer on the 21 x 8 character grid.
 *
 * Same glyph layout as i2c_OLED_send_char ( 5 font columns + 1 blank ), but into
 * RAM; the flush sends it. Text past the right edge is cut off.
 */
static void sysText ( uint8_t col, uint8_t row, const char *text ) {
  if ( row > 7 || ! text ) return;

  uint16_t x   = ( uint16_t ) ( col * CHARACTER_WIDTH_TOTAL );
  uint8_t *dst = &oledSystemBuffer [ row * SCREEN_WIDTH ];

  while ( *text && x + CHARACTER_WIDTH_TOTAL <= SCREEN_WIDTH ) {
    uint8_t ch = ( uint8_t ) *text;
    if ( ch < 32 || ch > 166 ) ch = ' ';    // font table covers 32-166 ( incl. bargraph glyphs )

    for ( uint8_t i = 0; i < FONT_WIDTH; i++ ) {
      dst [ x + i ] = multiWiiFont [ ch - 32 ][ i ];
    }
    dst [ x + FONT_WIDTH ] = 0;    // the gap

    x = ( uint16_t ) ( x + CHARACTER_WIDTH_TOTAL );
    text++;
  }
}

/* ============================================================================
 * STARTUP & SYSTEM RENDERING (FIRMWARE TELEMETRY)
 * ============================================================================ */

/**
 * @brief Initializes the OLED display.
 */
void OledStartUpInit ( void ) {
  OledInitStatus        = ug2864hsweg01InitI2C ( );    // Initialize OLED via I2C
  lastOledStartUpMillis = millis ( );                  // Record the start time
}

/**
 * @brief Displays the startup page on the OLED.
 */
void OledStartUpPage ( void ) {
  if ( OledInitStatus ) {
    updatedMillis = millis ( );                                         // Update current time
    if ( updatedMillis - lastOledStartUpMillis < OledStartUpTime ) {    // Check if startup time hasn't elapsed
      i2c_OLED_set_xy ( 7, 0 );
      tfp_sprintf ( buffer, "%s", FwName );    // Display firmware name
      i2c_OLED_send_string ( buffer );
      i2c_OLED_set_xy ( 2, 2 );
      tfp_sprintf ( buffer, "FC: %s", targetName );    // Display target name
      i2c_OLED_send_string ( buffer );
      i2c_OLED_set_xy ( 0, 4 );
      tfp_sprintf ( buffer, "FW:%s | API:%s", FwVersion, ApiVersion );    // Display firmware and API version
      i2c_OLED_send_string ( buffer );
      i2c_OLED_set_xy ( 1, 5 );
      tfp_sprintf ( buffer, "Build : %s", buildDate );    // Display build date
      i2c_OLED_send_string ( buffer );
      i2c_OLED_set_xy ( 2, 7 );
      i2c_OLED_send_string ( "By DRONA AVIATION" );    // Display author information
      OledStartupPageEnd = true;                       // Mark that startup page has been shown
    } else if ( OledStartupPageEnd ) {                 // Clear the display after showing the startup page
      i2c_OLED_clear_display_quick ( );                // disarmed, at boot: the one direct clear left
      oledPanelCleared ( );
      OledStartupPageEnd = false;
    }
  }
}

/**
 * @brief Displays the battery voltage on the OLED.
 */
static void dispVoltage ( void ) {
  tfp_sprintf ( buffer, "| %d.%d", ( int ) ( vBatRaw / 10 ), ( int ) ( vBatRaw % 10 ) );    // Format voltage
  sysText ( 16, 0, buffer );
}

/**
 * @brief Displays whether the device is in development mode.
 */
static void dispDevMode ( void ) {
  sysText ( 0, 0, devmode ? "DEV |" : " X  |" );    // Display DEV if in development mode
}

/**
 * @brief Displays the connection status of the receiver.
 */
static void dispRxConnection ( void ) {
  static bool rx_blink = false;
  rx_blink             = ! rx_blink;    // toggle every call

  bool connected = rxIsReceivingSignal ( ) || ppmIsRecievingSignal ( );
  sysText ( 0, 7, connected ? ( rx_blink ? "RX *|" : "RX  |" ) : " X  |" );    // blink asterisk when connected
}

/**
 * @brief Displays arm status.
 */
static void dispArmData ( void ) {
  sysText ( 16, 7, IS_RC_MODE_ACTIVE ( BOXARM ) ? "| ARM" : "|  X " );    // Display ARM if armed
}

/**
 * @brief Displays the flight status on the OLED.
 */
static void dispFlightStatus ( void ) {
  // Check if it's time to update the message
  if ( blink_counter == blink_at ) {
    if ( status_FSI ( Accel_Gyro_Calibration ) )
      message = " ACC CAL ";    // Accelerometer and Gyroscope Calibration
    else if ( status_FSI ( Mag_Calibration ) )
      message = " MAG CAL ";    // Magnetometer Calibration
    else if ( status_FSI ( LowBattery_inFlight ) || status_FSI ( Low_battery ) )
      message = " LOW BAT ";    // Low Battery Warning
    else if ( status_FSI ( Signal_loss ) || ! ( rxIsReceivingSignal ( ) || ppmIsRecievingSignal ( ) ) )
      message = " RX LOSS ";    // Signal Loss Warning
    else if ( status_FSI ( Crash ) )
      message = "  CRASH  ";    // Crash Detected
    else if ( status_FSI ( Not_ok_to_arm ) )
      message = "NOT READY";    // Not Ready to Arm
    else if ( status_FSI ( Ok_to_arm ) )
      message = "  READY  ";    // Ready to Arm
    else if ( status_FSI ( Armed ) )
      message = "IN FLIGHT";    // Armed and In Flight
  } else if ( blink_counter == 5 ) {
    // Reset message for blinking effect
    message       = "         ";
    blink_counter = 0;
  }

  // Only update display if there is a message
  if ( message ) {
    sysText ( 6, 0, message );
  }

  blink_counter++;
}

/**
 * @brief Manages and displays the stopwatch on the OLED.
 */
static void dispStopWatch ( void ) {
  unsigned long currentMillis = millis ( );    // Get current time

  // Check if RC mode is active
  if ( IS_RC_MODE_ACTIVE ( BOXARM ) ) {
    if ( ! StopWatchRunning ) {
      // Start the stopwatch
      SwElapsedMillis = 0;    // Reset elapsed time
      if ( status_FSI ( Armed ) ) {
        StopWatchRunning = true;
        SwStartMillis    = currentMillis;
      }
    } else {
      // Update elapsed time
      SwElapsedMillis = currentMillis - SwStartMillis;
    }
  } else if ( StopWatchRunning ) {
    // Stop the stopwatch
    StopWatchRunning = false;
    SwElapsedMillis  = currentMillis - SwStartMillis;
  }

  // Calculate minutes and seconds from elapsed milliseconds
  unsigned long totalSeconds = SwElapsedMillis / 1000;    // Convert millis to seconds
  unsigned long minutes      = totalSeconds / 60;
  unsigned long seconds      = totalSeconds % 60;

  // Display minutes with leading zero on OLED
  tfp_sprintf ( buffer, "%02lu :", minutes );
  sysText ( 7, 7, buffer );

  // Display seconds with leading zero on OLED
  tfp_sprintf ( buffer, "%02lu", seconds );
  sysText ( 12, 7, buffer );
}

/**
 * @brief Displays navigation-related information on the OLED screen.
 */
void OledDisplayNav ( void ) {
  // Ensure startup page has ended before displaying navigation information
  if ( ! OledStartupPageEnd ) {
    dispDevMode ( );
    dispFlightStatus ( );
    dispVoltage ( );
    dispRxConnection ( );
    dispStopWatch ( );
    dispArmData ( );
  }
}

/**
 * @brief Displays system data on the OLED screen.
 */
void OledDisplaySystemData ( void ) {
  // Check if not in dev mode and startup page has ended
  if ( ! devmode && ! OledStartupPageEnd ) {
    // Display pitch value
    tfp_sprintf ( buffer, "P : %d   ", ( int ) ( inclination.values.pitchDeciDegrees ) / 10 );
    sysText ( 1, 2, buffer );

    // Display roll value
    tfp_sprintf ( buffer, "R : %d   ", ( int ) ( inclination.values.rollDeciDegrees ) / 10 );
    sysText ( 1, 3, buffer );

    // Display yaw value
    tfp_sprintf ( buffer, "Y : %d   ", ( int ) ( heading ) );
    sysText ( 1, 4, buffer );

    // Display RC channel pitch
    tfp_sprintf ( buffer, "p : %d   ", ( int ) ( rcData [ 1 ] ) );
    sysText ( 11, 2, buffer );

    // Display RC channel roll
    tfp_sprintf ( buffer, "R : %d   ", ( int ) ( rcData [ 0 ] ) );
    sysText ( 11, 3, buffer );

    // Display RC channel yaw
    tfp_sprintf ( buffer, "Y : %d   ", ( int ) ( rcData [ 2 ] ) );
    sysText ( 11, 4, buffer );

    // Display RC channel throttle
    tfp_sprintf ( buffer, "T : %d   ", ( int ) ( rcData [ 3 ] ) );
    sysText ( 11, 5, buffer );
  }
}

/**
 * @brief Per-loop OLED entry point ( called from userCode ( ) every loop ).
 *
 * In System mode the telemetry is redrawn into oledSystemBuffer every 200 ms ( RAM
 * only ). In both modes one bounded flush step then moves the active frame to the
 * panel, so the I2C cost per loop stays at one run ( OLED_FLUSH_MAX_BYTES ) whether
 * armed or not.
 */
void OledDisplayData ( void ) {
  if ( ! OledInitStatus ) return;
  if ( OledStartupPageEnd ) return;    // the splash page owns the panel for its 2 s

  if ( oledMode == OLED_MODE_SYSTEM ) {
    updatedMillis = millis ( );
    if ( updatedMillis - OldOledDisplayDataMillis >= 200 ) {
      // Dev mode hides the P/R/Y/RC block: start from a blank frame when it toggles
      if ( devModeLastState != devmode ) {
        memset ( oledSystemBuffer, 0, sizeof ( oledSystemBuffer ) );
        devModeLastState = devmode;
      }
      OledDisplayNav ( );
      OledDisplaySystemData ( );
      oledActiveBuffer         = oledSystemBuffer;
      oledDirty                = true;
      OldOledDisplayDataMillis = updatedMillis;
    }
  }

  oledFlushStep ( );
}

/* ============================================================================
 * MODE & OWNERSHIP CONTROL
 * ============================================================================ */

/**
 * @brief Return OLED ownership to system firmware.
 */
void Oled_SystemMode ( void ) {
  oledMode = OLED_MODE_SYSTEM;

  // Start from a blank frame and redraw the telemetry on the next loop; the flush
  // replaces whatever user code left on the panel, armed or not.
  memset ( oledSystemBuffer, 0, sizeof ( oledSystemBuffer ) );
  oledActiveBuffer         = oledSystemBuffer;
  oledDirty                = true;
  OldOledDisplayDataMillis = millis ( ) - 200U;
}

/**
 * @brief Grant exclusive OLED ownership to user code.
 *
 * The panel is blanked through the flush until the user's first Oled_Update ( ).
 */
void Oled_UserMode ( void ) {
  oledMode = OLED_MODE_USER;
  memset ( oledSystemBuffer, 0, sizeof ( oledSystemBuffer ) );
  oledActiveBuffer = oledSystemBuffer;
  oledDirty        = true;
}

/**
 * @brief Check if OLED is in USER mode.
 */
bool Oled_IsUserMode ( void ) {
  return oledMode == OLED_MODE_USER;
}

/**
 * @brief Check if OLED is in SYSTEM mode.
 */
bool Oled_IsSystemMode ( void ) {
  return oledMode == OLED_MODE_SYSTEM;
}

/* ============================================================================
 * HARDWARE / BUFFER CONTROL
 * ============================================================================ */

/**
 * @brief Clear OLED display.
 */
void Oled_display_Clear ( void ) {
  if ( ! OledInitStatus ) return;    // OLED absent or switched off: no bus traffic at all
  if ( OledStartupPageEnd ) return;
  if ( ARMING_FLAG ( ARMED ) ) return;

  i2c_OLED_clear_display_quick ( );
  oledPanelCleared ( );
}

/**
 * @brief Queue a user framebuffer for the panel.
 *
 * Nothing is sent here: the frame reaches the panel through oledFlushStep ( ), a
 * bounded run per loop, so the screen catches up over a few loops ( a full screen
 * in ~225 ms ). The buffer must stay valid; Oled.cpp passes its static buffer.
 */
void Oled_display_Update ( uint8_t *buf ) {
  if ( oledMode != OLED_MODE_USER ) return;
  if ( ! buf ) return;

  oledActiveBuffer = buf;
  oledDirty        = true;
}

/**
 * @brief Print text using OLED text-grid (SYSTEM mode only).
 *
 * Drawn into the system framebuffer; the flush sends it.
 */
void Oled_Print ( uint8_t col, uint8_t row, const char *text ) {
  // if ( ! OledEnable ) return;
  if ( oledMode != OLED_MODE_SYSTEM ) return;
  if ( ! text ) return;

  if ( col > 20 || row < 1 || row > 6 ) return;

  sysText ( col, row, text );
  oledDirty = true;
}

/* ============================================================================
 * BASIC DRAWING PRIMITIVES
 * ============================================================================ */

/**
 * @brief Set or clear a single pixel in the framebuffer.
 */
void Oled_DrawPixel ( uint8_t *buf, int16_t x, int16_t y, bool on ) {
  if ( ! buf ) return;
  if ( x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT ) return;

  uint16_t index = ( uint16_t ) ( x + ( y >> 3 ) * SCREEN_WIDTH );
  uint8_t mask   = ( uint8_t ) ( 1U << ( y & 7 ) );

  if ( on )
    buf [ index ] |= mask;
  else
    buf [ index ] &= ~mask;
}

void Oled_DrawHLine ( uint8_t *buf, int16_t x, int16_t y, int16_t length, bool on ) {
  for ( int16_t i = 0; i < length; i++ ) {
    Oled_DrawPixel ( buf, x + i, y, on );
  }
}

void Oled_DrawVLine ( uint8_t *buf, int16_t x, int16_t y, int16_t length, bool on ) {
  for ( int16_t i = 0; i < length; i++ ) {
    Oled_DrawPixel ( buf, x, y + i, on );
  }
}

/**
 * @brief Draw a line using Bresenham’s algorithm.
 */
void Oled_DrawLine ( uint8_t *buf, int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool on ) {
  int16_t dx  = ( int16_t ) ABS ( x1 - x0 );
  int16_t dy  = ( int16_t ) ABS ( y1 - y0 );
  int16_t sx  = ( x0 < x1 ) ? 1 : -1;
  int16_t sy  = ( y0 < y1 ) ? 1 : -1;
  int16_t err = dx - dy;

  while ( 1 ) {
    Oled_DrawPixel ( buf, x0, y0, on );
    if ( x0 == x1 && y0 == y1 ) break;

    int16_t e2 = ( int16_t ) ( err << 1 );
    if ( e2 > -dy ) {
      err -= dy;
      x0 += sx;
    }
    if ( e2 < dx ) {
      err += dx;
      y0 += sy;
    }
  }
}

/**
 * @brief Draw text into framebuffer at pixel coordinates.
 */
void Oled_DrawTextColor ( uint8_t *screen, int16_t x, int16_t y, const char *text, bool on ) {
  if ( ! screen || ! text ) return;

  while ( *text ) {
    uint8_t ch = ( uint8_t ) *text;
    if ( ch < 32 ) ch = 32;    // unprintable → space
    uint8_t idx = ch - 32;

    // Draw 5 columns of the glyph
    for ( int16_t col = 0; col < 5; col++ ) {
      uint8_t bits = multiWiiFont [ idx ][ col ];
      for ( int16_t row = 0; row < 7; row++ ) {
        if ( bits & ( 1 << row ) ) {
          Oled_DrawPixel ( screen, x + col, y + row, on );
        }
      }
    }
    // 6th column = gap (caller-provided background)

    x += 6;                      // advance cursor
    if ( x > 127 - 5 ) break;    // stop at screen edge
    text++;
  }
}

void Oled_DrawText ( uint8_t *screen, int16_t x, int16_t y, const char *text ) {
  Oled_DrawTextColor ( screen, x, y, text, true );
}

/**
 * @brief Draw an integer number into framebuffer.
 */
void Oled_DrawNumber ( uint8_t *screen, int16_t x, int16_t y, int16_t number ) {
  char buf [ 12 ];    // enough for -128 to 127
  uint8_t i = 0;
  bool neg  = false;

  if ( number < 0 ) {
    neg    = true;
    number = ( int16_t ) ( -number );
  }

  // Build digits in reverse
  do {
    buf [ i++ ] = ( char ) ( '0' + ( number % 10 ) );
    number /= 10;
  } while ( number > 0 );

  if ( neg ) buf [ i++ ] = '-';

  // Reverse into output
  char out [ 12 ];
  for ( uint8_t j = 0; j < i; j++ ) {
    out [ j ] = buf [ i - 1 - j ];
  }
  out [ i ] = '\0';

  Oled_DrawText ( screen, x, y, out );
}

/* ============================================================================
 * SHAPES (OUTLINE & FILLED)
 * ============================================================================ */

void Oled_DrawRect ( uint8_t *buf, int16_t x, int16_t y, int16_t w, int16_t h, bool on ) {
  Oled_DrawHLine ( buf, x, y, w, on );
  Oled_DrawHLine ( buf, x, y + h - 1, w, on );
  Oled_DrawVLine ( buf, x, y, h, on );
  Oled_DrawVLine ( buf, x + w - 1, y, h, on );
}

void Oled_FillRect ( uint8_t *buf, int16_t x, int16_t y, int16_t w, int16_t h, bool on ) {
  for ( int16_t i = 0; i < h; i++ ) {
    Oled_DrawHLine ( buf, x, y + i, w, on );
  }
}

/**
 * @brief Helper: draw quarter-circle arcs for rounded rect corners.
 */
static void drawCornerArcs ( uint8_t *buf, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, bool on ) {
  int16_t cx, cy;
  int16_t px = r, py = 0, err = 0;

  while ( px >= py ) {
    // Top-right corner
    cx = x + w - 1 - r;
    cy = y + r;
    Oled_DrawPixel ( buf, cx + px, cy - py, on );
    Oled_DrawPixel ( buf, cx + py, cy - px, on );
    // Top-left corner
    cx = x + r;
    cy = y + r;
    Oled_DrawPixel ( buf, cx - px, cy - py, on );
    Oled_DrawPixel ( buf, cx - py, cy - px, on );
    // Bottom-right corner
    cx = x + w - 1 - r;
    cy = y + h - 1 - r;
    Oled_DrawPixel ( buf, cx + px, cy + py, on );
    Oled_DrawPixel ( buf, cx + py, cy + px, on );
    // Bottom-left corner
    cx = x + r;
    cy = y + h - 1 - r;
    Oled_DrawPixel ( buf, cx - px, cy + py, on );
    Oled_DrawPixel ( buf, cx - py, cy + px, on );

    py++;
    err += 1 + 2 * py;
    if ( 2 * ( err - px ) + 1 > 0 ) {
      px--;
      err += 1 - 2 * px;
    }
  }
}

void Oled_DrawRoundedRect ( uint8_t *buf, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, bool on ) {
  if ( r <= 0 ) {
    Oled_DrawRect ( buf, x, y, w, h, on );
    return;
  }
  if ( r > w / 2 ) r = w / 2;
  if ( r > h / 2 ) r = h / 2;

  // Straight edges
  Oled_DrawHLine ( buf, x + r, y, w - 2 * r, on );
  Oled_DrawHLine ( buf, x + r, y + h - 1, w - 2 * r, on );
  Oled_DrawVLine ( buf, x, y + r, h - 2 * r, on );
  Oled_DrawVLine ( buf, x + w - 1, y + r, h - 2 * r, on );

  // Corner arcs
  drawCornerArcs ( buf, x, y, w, h, r, on );
}

void Oled_FillRoundedRect ( uint8_t *buf, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, bool on ) {
  if ( r <= 0 || r > w / 2 || r > h / 2 ) {
    Oled_FillRect ( buf, x, y, w, h, on );
    return;
  }
  // Fill center rectangle
  Oled_FillRect ( buf, x + r, y, w - 2 * r, h, on );
  // Fill left and right side strips
  Oled_FillRect ( buf, x, y + r, r, h - 2 * r, on );
  Oled_FillRect ( buf, x + w - r, y + r, r, h - 2 * r, on );
  // Fill corner circles
  int16_t px = r, py = 0, err = 0;
  while ( px >= py ) {
    Oled_DrawHLine ( buf, x + r - px, y + r - py, px, on );            // top-left
    Oled_DrawHLine ( buf, x + w - r, y + r - py, px, on );             // top-right
    Oled_DrawHLine ( buf, x + r - px, y + h - 1 - r + py, px, on );    // bottom-left
    Oled_DrawHLine ( buf, x + w - r, y + h - 1 - r + py, px, on );     // bottom-right
    Oled_DrawHLine ( buf, x + r - py, y + r - px, py, on );
    Oled_DrawHLine ( buf, x + w - r, y + r - px, py, on );
    Oled_DrawHLine ( buf, x + r - py, y + h - 1 - r + px, py, on );
    Oled_DrawHLine ( buf, x + w - r, y + h - 1 - r + px, py, on );

    py++;
    err += 1 + 2 * py;
    if ( 2 * ( err - px ) + 1 > 0 ) {
      px--;
      err += 1 - 2 * px;
    }
  }
}

void Oled_DrawCircle ( uint8_t *buf, int16_t cx, int16_t cy, int16_t r, bool on ) {
  int16_t x = r, y = 0, err = 0;

  while ( x >= y ) {
    Oled_DrawPixel ( buf, cx + x, cy + y, on );
    Oled_DrawPixel ( buf, cx + y, cy + x, on );
    Oled_DrawPixel ( buf, cx - y, cy + x, on );
    Oled_DrawPixel ( buf, cx - x, cy + y, on );
    Oled_DrawPixel ( buf, cx - x, cy - y, on );
    Oled_DrawPixel ( buf, cx - y, cy - x, on );
    Oled_DrawPixel ( buf, cx + y, cy - x, on );
    Oled_DrawPixel ( buf, cx + x, cy - y, on );

    y++;
    err += 1 + 2 * y;
    if ( 2 * ( err - x ) + 1 > 0 ) {
      x--;
      err += 1 - 2 * x;
    }
  }
}

void Oled_FillCircle ( uint8_t *buf, int16_t cx, int16_t cy, int16_t r, bool on ) {
  int16_t x = r, y = 0, err = 0;
  Oled_DrawHLine ( buf, cx - r, cy, 2 * r + 1, on );

  while ( x >= y ) {
    y++;
    err += 1 + 2 * y;
    if ( 2 * ( err - x ) + 1 > 0 ) {
      x--;
      err += 1 - 2 * x;
    }
    Oled_DrawHLine ( buf, cx - x, cy + y, 2 * x + 1, on );
    Oled_DrawHLine ( buf, cx - x, cy - y, 2 * x + 1, on );
    Oled_DrawHLine ( buf, cx - y, cy + x, 2 * y + 1, on );
    Oled_DrawHLine ( buf, cx - y, cy - x, 2 * y + 1, on );
  }
}

/* ============================================================================
 * HUD & SPECIALIZED GRAPHICS
 * ============================================================================ */

/**
 * @brief Map RC value (1000–2000) to a signed pixel offset.
 */
static int8_t mapStick ( uint16_t v, uint8_t radius ) {
  v = ( uint16_t ) constrain ( v, 1000, 2000 );
  return ( int8_t ) ( ( ( int16_t ) v - 1500 ) * radius / 500 );
}

/**
 * @brief Draw a single joystick (internal).
 */
static void drawJoystick ( uint8_t *buf, uint8_t cx, uint8_t cy, uint16_t xValue, uint16_t yValue, bool invertY ) {
  Oled_DrawCircle ( buf, cx, cy, JOY_RADIUS, true );
  Oled_DrawHLine ( buf, cx - 4, cy, 8, true );
  Oled_DrawVLine ( buf, cx, cy - 4, 8, true );

  int8_t dx = mapStick ( xValue, JOY_RADIUS - 4 );
  int8_t dy = mapStick ( yValue, JOY_RADIUS - 4 );
  if ( invertY ) dy = ( int8_t ) ( -dy );

  Oled_FillCircle ( buf, ( uint8_t ) ( cx + dx ), ( uint8_t ) ( cy + dy ), STICK_RADIUS, true );
}

void Oled_DrawRCJoysticks ( uint8_t *buf, uint16_t throttle, uint16_t yaw, uint16_t roll, uint16_t pitch ) {
  drawJoystick ( buf, LEFT_JOY_X, JOY_Y, yaw, throttle, true );
  drawJoystick ( buf, RIGHT_JOY_X, JOY_Y, roll, pitch, true );
}

void Oled_DrawPitchIndicator ( uint8_t *buf, int16_t pitch ) {
  uint8_t centerY     = ( uint8_t ) ( SCREEN_HEIGHT / 2 );
  uint8_t pitchOffset = ( uint8_t ) ( ( pitch * ( SCREEN_HEIGHT / 2 ) ) / 90 );
  Oled_DrawLine ( buf, 0, ( uint8_t ) ( centerY + pitchOffset ), SCREEN_WIDTH - 1, ( uint8_t ) ( centerY + pitchOffset ), true );
}

void Oled_DrawRollIndicator ( uint8_t *buf, int16_t roll ) {
  uint8_t centerX    = ( uint8_t ) ( SCREEN_WIDTH / 2 );
  uint8_t rollOffset = ( uint8_t ) ( ( roll * ( SCREEN_WIDTH / 2 ) ) / 90 );
  Oled_DrawLine ( buf, ( uint8_t ) ( centerX + rollOffset ), 0, ( uint8_t ) ( centerX + rollOffset ), SCREEN_HEIGHT - 1, true );
}

void Oled_DrawArrow ( uint8_t *buf, int16_t cx, int16_t cy, int16_t size, oled_arrow_dir_t dir, bool on ) {
  if ( ! buf || size <= 0 ) return;
  int16_t half = size / 2;
  int16_t head = size / 3;

  switch ( dir ) {
    case OLED_ARROW_UP:
      Oled_DrawLine ( buf, cx, cy + half, cx, cy - half, on );
      Oled_DrawLine ( buf, cx, cy - half, cx - head, cy - half + head, on );
      Oled_DrawLine ( buf, cx, cy - half, cx + head, cy - half + head, on );
      break;
    case OLED_ARROW_DOWN:
      Oled_DrawLine ( buf, cx, cy - half, cx, cy + half, on );
      Oled_DrawLine ( buf, cx, cy + half, cx - head, cy + half - head, on );
      Oled_DrawLine ( buf, cx, cy + half, cx + head, cy + half - head, on );
      break;
    case OLED_ARROW_LEFT:
      Oled_DrawLine ( buf, cx + half, cy, cx - half, cy, on );
      Oled_DrawLine ( buf, cx - half, cy, cx - half + head, cy - head, on );
      Oled_DrawLine ( buf, cx - half, cy, cx - half + head, cy + head, on );
      break;
    case OLED_ARROW_RIGHT:
      Oled_DrawLine ( buf, cx - half, cy, cx + half, cy, on );
      Oled_DrawLine ( buf, cx + half, cy, cx + half - head, cy - head, on );
      Oled_DrawLine ( buf, cx + half, cy, cx + half - head, cy + head, on );
      break;
    default:
      break;
  }
}

/* ============================================================================
 * EYE GRAPHICS PRESETS
 * ============================================================================ */

static int8_t clampPupil ( int8_t v, uint8_t max ) {
  if ( v > ( int8_t ) max ) return ( int8_t ) max;
  if ( v < -( int8_t ) max ) return ( int8_t ) ( -( int8_t ) max );
  return v;
}

static void Oled_DrawEyeWithPupil ( uint8_t *buf, int16_t cx, int16_t cy, int16_t radius, int8_t pupilOffsetX, int8_t pupilOffsetY ) {
  Oled_FillCircle ( buf, cx, cy, radius, true );
  int16_t pupilRadius = ( int16_t ) ( radius / 3 );
  int16_t maxOffset   = ( int16_t ) ( radius - pupilRadius - 1 );
  pupilOffsetX        = clampPupil ( pupilOffsetX, maxOffset );
  pupilOffsetY        = clampPupil ( pupilOffsetY, maxOffset );
  Oled_FillCircle ( buf, ( uint8_t ) ( cx + pupilOffsetX ), ( uint8_t ) ( cy + pupilOffsetY ), pupilRadius, false );
}

static void Oled_DrawEyeOutlineWithPupil ( uint8_t *buf, int16_t cx, int16_t cy, int16_t radius, int8_t pupilOffsetX, int8_t pupilOffsetY ) {
  Oled_DrawCircle ( buf, cx, cy, radius, true );
  int16_t pupilRadius = ( int16_t ) ( radius / 3 );
  int16_t maxOffset   = ( int16_t ) ( radius - pupilRadius - 1 );
  pupilOffsetX        = clampPupil ( pupilOffsetX, maxOffset );
  pupilOffsetY        = clampPupil ( pupilOffsetY, maxOffset );
  Oled_FillCircle ( buf, ( uint8_t ) ( cx + pupilOffsetX ), ( uint8_t ) ( cy + pupilOffsetY ), pupilRadius, true );
}

void Oled_DrawEye ( uint8_t *buf, int16_t cx, int16_t cy, int16_t radius, int8_t pupilOffsetX, int8_t pupilOffsetY, bool filled ) {
  if ( filled )
    Oled_DrawEyeWithPupil ( buf, cx, cy, radius, pupilOffsetX, pupilOffsetY );
  else
    Oled_DrawEyeOutlineWithPupil ( buf, cx, cy, radius, pupilOffsetX, pupilOffsetY );
}

void Oled_DrawXEye ( uint8_t *buf, int16_t cx, int16_t cy, int16_t size ) {
  int16_t half = size / 2;
  Oled_DrawLine ( buf, cx - half, cy - half, cx + half, cy + half, true );
  Oled_DrawLine ( buf, cx - half, cy + half, cx + half, cy - half, true );
}

static void Oled_DrawBoxyEyeFilled ( uint8_t *buf, int16_t cx, int16_t cy, int16_t w, int16_t h, int16_t r, int8_t pupilOffsetX, int8_t pupilOffsetY ) {
  int16_t ex = ( int16_t ) ( cx - w / 2 );
  int16_t ey = ( int16_t ) ( cy - h / 2 );
  Oled_FillRoundedRect ( buf, ex, ey, w, h, r, true );

  int16_t pw      = ( int16_t ) ( w / 3 );
  int16_t ph      = ( int16_t ) ( h / 3 );
  int16_t pr      = ( int16_t ) ( ( r > 1 ) ? r / 2 : 1 );
  int16_t maxOffX = ( int16_t ) ( ( w - pw ) / 2 - 1 );
  int16_t maxOffY = ( int16_t ) ( ( h - ph ) / 2 - 1 );
  pupilOffsetX    = clampPupil ( pupilOffsetX, ( uint8_t ) maxOffX );
  pupilOffsetY    = clampPupil ( pupilOffsetY, ( uint8_t ) maxOffY );

  int16_t px = ( int16_t ) ( cx + pupilOffsetX - pw / 2 );
  int16_t py = ( int16_t ) ( cy + pupilOffsetY - ph / 2 );
  Oled_FillRoundedRect ( buf, px, py, pw, ph, pr, false );
}

static void Oled_DrawBoxyEyeOutline ( uint8_t *buf, int16_t cx, int16_t cy, int16_t w, int16_t h, int16_t r, int8_t pupilOffsetX, int8_t pupilOffsetY ) {
  int16_t ex = ( int16_t ) ( cx - w / 2 );
  int16_t ey = ( int16_t ) ( cy - h / 2 );
  Oled_DrawRoundedRect ( buf, ex, ey, w, h, r, true );

  int16_t pw      = ( int16_t ) ( w / 3 );
  int16_t ph      = ( int16_t ) ( h / 3 );
  int16_t pr      = ( int16_t ) ( ( r > 1 ) ? r / 2 : 1 );
  int16_t maxOffX = ( int16_t ) ( ( w - pw ) / 2 - 1 );
  int16_t maxOffY = ( int16_t ) ( ( h - ph ) / 2 - 1 );
  pupilOffsetX    = clampPupil ( pupilOffsetX, ( uint8_t ) maxOffX );
  pupilOffsetY    = clampPupil ( pupilOffsetY, ( uint8_t ) maxOffY );

  int16_t px = ( int16_t ) ( cx + pupilOffsetX - pw / 2 );
  int16_t py = ( int16_t ) ( cy + pupilOffsetY - ph / 2 );
  Oled_FillRoundedRect ( buf, px, py, pw, ph, pr, true );
}

void Oled_DrawBoxyEye ( uint8_t *buf, int16_t cx, int16_t cy, int16_t w, int16_t h, int16_t r, int8_t pupilOffsetX, int8_t pupilOffsetY, bool filled ) {
  if ( filled )
    Oled_DrawBoxyEyeFilled ( buf, cx, cy, w, h, r, pupilOffsetX, pupilOffsetY );
  else
    Oled_DrawBoxyEyeOutline ( buf, cx, cy, w, h, r, pupilOffsetX, pupilOffsetY );
}

void Oled_DrawBoxyXEye ( uint8_t *buf, int16_t cx, int16_t cy, int16_t w, int16_t h, int16_t r ) {
  int16_t ex = cx - w / 2;
  int16_t ey = cy - h / 2;
  Oled_DrawRoundedRect ( buf, ex, ey, w, h, r, true );
  int16_t margin = 4;
  Oled_DrawLine ( buf, ex + margin, ey + margin, ex + w - margin, ey + h - margin, true );
  Oled_DrawLine ( buf, ex + w - margin, ey + margin, ex + margin, ey + h - margin, true );
}
