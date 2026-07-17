/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: techsavvyomi                                                       #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\RGB-LED.cpp                                        #
 #  Created Date: Fri, 4th Apr 2026                                            #
 #  Brief: WS2812B RGB LED API — direct RGB-to-DMA, no HSV conversion          #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Fri, 4th Apr 2026                                           #
 #  Modified By: techsavvyomi                                                  #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-04-04	techsavvyomi	Created RGB LED API                            #
 #  2026-04-04	techsavvyomi	Reworked: direct RGB-to-DMA, bypass HSV        #
*******************************************************************************/
#include "platform.h"

#include "API/RGB-LED.h"

extern "C" {
#include "common/color.h"
#include "drivers/light_ws2811strip.h"
#include "drivers/system.h"
#include "io/ledstrip.h"
}

// Inputs for the RGB_SYSTEM flight-status renderer (rgbSystemTick).
#include "common/maths.h"           // leastSignificantBit()
#include "config/runtime_config.h"  // flightIndicatorFlag, FlightStatus_e
#include "rx/rx.h"                  // rc_connected

/*=============================================================================
 *  Internal RGB buffer — we own this, bypass HSV entirely
 *===========================================================================*/

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

static rgb_t    rgbBuffer[RGB_MAX_LEDS];
static uint8_t  rgbLedCount   = 0;
static uint8_t  rgbBrightness = 100;
static bool     rgbHwReady    = false;
static volatile bool rgbReleasePending = false;   // blank queued by RGB_Release()

// Animation state
static RGB_Animation_e  animType      = RGB_ANIM_NONE;
static RGB_Direction_e  animDirection  = RGB_DIR_FORWARD;
static uint16_t         animSpeedMs   = 100;
static uint8_t          animR         = 0;
static uint8_t          animG         = 0;
static uint8_t          animB         = 0;
static uint8_t          animFrame     = 0;
static uint16_t         animStep      = 0;
static uint32_t         animLastMs    = 0;
static bool             animPhase     = true;

/*=============================================================================
 *  Helpers
 *===========================================================================*/

static uint8_t mapIndex ( uint8_t frame )
{
    if ( animDirection == RGB_DIR_REVERSE ) {
        return ( uint8_t ) ( ( rgbLedCount - 1 ) - ( frame % rgbLedCount ) );
    }
    return frame % rgbLedCount;
}

static void rgbWriteDma ( void )
{
    // Non-blocking: if a DMA transfer is still in flight, skip this update
    // instead of waiting. The DMA engine owns ledStripDMABuffer until the
    // transfer completes — writing into it now would corrupt the in-flight
    // frame. We simply drop this frame; the next RGB_Show() pushes the
    // latest colors once DMA is free. This guarantees the flight loop can
    // never stall here, even if the transfer-complete IRQ never fires.
    if ( ws2811LedDataTransferInProgress ) {
        return;
    }

    // Write RGB buffer directly into DMA buffer in GRB order
    uint16_t offset = 0;

    for ( uint8_t i = 0; i < WS2811_LED_STRIP_LENGTH; i++ ) {

        uint8_t g, r, b;

        if ( i < rgbLedCount ) {
            r = rgbBuffer[i].r;
            g = rgbBuffer[i].g;
            b = rgbBuffer[i].b;
        } else {
            r = 0;
            g = 0;
            b = 0;
        }

        // GRB bit order, MSB first — 24 bits per LED
        uint32_t grb = ( ( uint32_t ) g << 16 ) | ( ( uint32_t ) r << 8 ) | b;

        for ( int8_t bit = 23; bit >= 0; bit-- ) {
            ledStripDMABuffer[offset++] = ( grb & ( 1 << bit ) )
                                          ? BIT_COMPARE_1
                                          : BIT_COMPARE_0;
        }
    }

    ws2811LedDataTransferInProgress = 1;
    ws2811LedStripDMAEnable();

    rgbReleasePending = false;   // a frame just went out; nothing left to flush
}

/**
 * Non-blocking retry for the "all-off" frame queued by RGB_Release().
 * The system LED loop (updateLedStrip) calls this every cycle once user
 * control is released; it pushes the blank as soon as the DMA channel is
 * free, then does nothing. Never spins — safe to call from the flight loop.
 */
void rgbReleaseFlushTick ( void )
{
    if ( rgbReleasePending ) {
        rgbWriteDma();   // no-op while DMA is busy; sends + clears once free
    }
}

// Standard HSV-to-RGB (s=255 means vivid, s=0 means white)
static void hsvToRgb ( uint16_t h, uint8_t s, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b )
{
    if ( s == 0 ) {
        *r = v; *g = v; *b = v;
        return;
    }

    uint8_t region = ( uint8_t ) ( h / 60 );
    uint16_t remainder = ( uint16_t ) ( ( h - ( region * 60 ) ) * 255 / 60 );

    uint8_t p = ( uint8_t ) ( ( uint16_t ) v * ( 255 - s ) / 255 );
    uint8_t q = ( uint8_t ) ( ( uint16_t ) v * ( 255 - ( ( uint16_t ) s * remainder / 255 ) ) / 255 );
    uint8_t t = ( uint8_t ) ( ( uint16_t ) v * ( 255 - ( ( uint16_t ) s * ( 255 - remainder ) / 255 ) ) / 255 );

    switch ( region ) {
        case 0:  *r = v; *g = t; *b = p; break;
        case 1:  *r = q; *g = v; *b = p; break;
        case 2:  *r = p; *g = v; *b = t; break;
        case 3:  *r = p; *g = q; *b = v; break;
        case 4:  *r = t; *g = p; *b = v; break;
        default: *r = v; *g = p; *b = q; break;
    }
}

/*=============================================================================
 *  Sine table for wave animation (quarter-wave, 64 entries)
 *===========================================================================*/

static const uint8_t sineTable[64] = {
    0,   6,  12,  19,  25,  31,  37,  44,  50,  56,  62,  68,  74,  80,  86,  92,
   98, 103, 109, 115, 120, 126, 131, 136, 142, 147, 152, 157, 162, 167, 171, 176,
  181, 185, 189, 193, 197, 201, 205, 209, 212, 216, 219, 222, 225, 228, 231, 234,
  236, 238, 241, 243, 244, 246, 248, 249, 251, 252, 253, 254, 254, 255, 255, 255
};

static uint8_t sineLookup ( uint8_t angle )
{
    uint8_t quadrant = angle >> 6;
    uint8_t idx      = angle & 0x3F;

    switch ( quadrant ) {
        case 0: return sineTable[idx];
        case 1: return sineTable[63 - idx];
        case 2: return sineTable[idx];
        case 3: return sineTable[63 - idx];
        default: return 0;
    }
}

/*=============================================================================
 *  Initialization & Control
 *===========================================================================*/

void RGB_Init ( peripheral_rgb_pin_e pin, uint8_t led_count )
{
    if ( led_count > RGB_MAX_LEDS ) {
        led_count = RGB_MAX_LEDS;
    }

    rgbLedCount   = led_count;
    rgbBrightness = 100;
    animType      = RGB_ANIM_NONE;
    rgbUserControl = false;    // default: firmware owns the strip (RGB_SYSTEM)

    // Hardware init only once. The data pin is locked to the slot passed on the
    // first RGB_Init() of this power cycle (selectable, not simultaneous).
    if ( !rgbHwReady ) {
        ws2811LedStripSetPin ( ( uint8_t ) pin );
        ws2811LedStripInit();
        rgbHwReady = true;
    }

    // Clear buffer
    for ( uint8_t i = 0; i < RGB_MAX_LEDS; i++ ) {
        rgbBuffer[i].r = 0;
        rgbBuffer[i].g = 0;
        rgbBuffer[i].b = 0;
    }
}

void RGB_Release ( void )
{
    animType      = RGB_ANIM_NONE;
    rgbBrightness = 100;

    for ( uint8_t i = 0; i < RGB_MAX_LEDS; i++ ) {
        rgbBuffer[i].r = 0;
        rgbBuffer[i].g = 0;
        rgbBuffer[i].b = 0;
    }

    // Queue the blank, then try to push it now. If a DMA transfer is in
    // flight, rgbWriteDma() skips (non-blocking) and leaves rgbReleasePending
    // set — rgbReleaseFlushTick() from the system LED loop lands it next cycle.
    rgbReleasePending = true;
    rgbWriteDma();

    rgbUserControl = false;   // hand control back to the system last
}

/*=============================================================================
 *  System / User control
 *===========================================================================*/

void RGB_Control ( rgb_control_e mode )
{
    if ( mode == RGB_SYSTEM ) {
        animType       = RGB_ANIM_NONE;   // stop any user animation
        rgbUserControl = false;           // firmware flight-status owns the strip
    } else {
        rgbUserControl = true;            // user code owns the strip
    }
}

/*
 * Flight-status indicator on the WS2812 strip — a faithful port of
 * ledStripFlightStatus() (io/ledstrip.c), driven through the RGB API instead
 * of the LED_STRIP feature (which is compiled out on this board). Called every
 * control loop from mw.cpp; self-timed and non-blocking. Renders only when the
 * strip is initialised AND in RGB_SYSTEM mode.
 */
void rgbSystemTick ( void )
{
    if ( rgbUserControl || !rgbHwReady ) {
        return;                           // user owns it, or strip not ready
    }

    static int32_t activeTime    = 500;
    static int     delay_time     = 100;
    static uint8_t counter        = 0;
    static uint8_t toggle_switch  = 1;

    int32_t ledTime = ( int32_t ) millis();
    if ( ( int32_t ) ( ledTime - activeTime ) < delay_time ) {
        return;
    }
    counter++;

    switch ( leastSignificantBit ( flightIndicatorFlag ) ) {
        case Mag_Calibration:
            delay_time = 100;
            if ( toggle_switch ) { RGB_SetColorAll ( 255, 255, 0 ); toggle_switch = 0; }   // yellow
            else                 { RGB_SetColorAll ( 0, 0, 0 );     toggle_switch = 1; }
            break;

        case Accel_Gyro_Calibration:
            delay_time = 100;
            if ( toggle_switch ) { RGB_SetColorAll ( 255, 0, 255 ); toggle_switch = 0; }   // magenta
            else                 { RGB_SetColorAll ( 0, 0, 0 );     toggle_switch = 1; }
            break;

        case Ok_to_arm:
            if ( rc_connected ) {
                delay_time = 100;
                RGB_SetColorAll ( 0, 255, 0 );                                             // green
            } else {
                delay_time = 120;
                switch ( counter % 4 ) {
                    case 0: RGB_SetColorAll ( 0, 255, 0 );     break;                       // green
                    case 1: RGB_SetColorAll ( 0, 255, 255 );   break;                       // cyan
                    case 2: RGB_SetColorAll ( 255, 255, 255 ); break;                       // white
                    case 3: RGB_SetColorAll ( 0, 0, 0 );       break;
                }
            }
            break;

        case Not_ok_to_arm:
            if ( rc_connected ) {
                delay_time = 100;
                if ( toggle_switch ) { RGB_SetColorAll ( 255, 0, 0 ); toggle_switch = 0; }  // red
                else                 { RGB_SetColorAll ( 0, 0, 0 );   toggle_switch = 1; }
            } else {
                delay_time = 120;
                switch ( counter % 4 ) {
                    case 0: RGB_SetColorAll ( 0, 255, 0 );     break;                       // green
                    case 1: RGB_SetColorAll ( 0, 255, 255 );   break;                       // cyan
                    case 2: RGB_SetColorAll ( 255, 255, 255 ); break;                       // white
                    case 3: RGB_SetColorAll ( 0, 0, 0 );       break;
                }
            }
            break;

        case Armed:
            if ( rc_connected ) {
                delay_time = 100;
                RGB_SetColorAll ( 0, 0, 255 );                                             // blue
            }
            break;

        case LowBattery_inFlight:
            delay_time = 100;
            if ( toggle_switch ) { RGB_SetColorAll ( 255, 0, 0 ); toggle_switch = 0; }      // red
            else                 { RGB_SetColorAll ( 0, 0, 0 );   toggle_switch = 1; }
            break;

        case Low_battery:
            delay_time = 100;
            RGB_SetColorAll ( 255, 0, 0 );                                                 // red
            break;

        case Signal_loss:
            delay_time = 100;
            if ( toggle_switch ) { RGB_SetColorAll ( 0, 0, 255 ); toggle_switch = 0; }      // blue
            else                 { RGB_SetColorAll ( 0, 0, 0 );   toggle_switch = 1; }
            break;

        case Crash:
            delay_time = 100;
            switch ( counter % 2 ) {
                case 0: RGB_SetColorAll ( 0, 255, 0 ); break;                              // green
                case 1: RGB_SetColorAll ( 255, 0, 0 ); break;                              // red
            }
            break;

        default:
            break;
    }

    RGB_Show();
    activeTime = ledTime + delay_time;
}

/*=============================================================================
 *  Color Control
 *===========================================================================*/

void RGB_SetColor ( uint8_t index, uint8_t r, uint8_t g, uint8_t b )
{
    if ( index >= rgbLedCount ) {
        return;
    }

    if ( rgbBrightness < 100 ) {
        r = ( uint8_t ) ( ( uint16_t ) r * rgbBrightness / 100 );
        g = ( uint8_t ) ( ( uint16_t ) g * rgbBrightness / 100 );
        b = ( uint8_t ) ( ( uint16_t ) b * rgbBrightness / 100 );
    }

    rgbBuffer[index].r = r;
    rgbBuffer[index].g = g;
    rgbBuffer[index].b = b;
}

void RGB_SetColorAll ( uint8_t r, uint8_t g, uint8_t b )
{
    for ( uint8_t i = 0; i < rgbLedCount; i++ ) {
        RGB_SetColor ( i, r, g, b );
    }
}

void RGB_SetColorHSV ( uint8_t index, uint16_t h, uint8_t s, uint8_t v )
{
    if ( index >= rgbLedCount ) {
        return;
    }

    if ( rgbBrightness < 100 ) {
        v = ( uint8_t ) ( ( uint16_t ) v * rgbBrightness / 100 );
    }

    if ( h > 359 ) h = 359;

    uint8_t r, g, b;
    hsvToRgb ( h, s, v, &r, &g, &b );

    // Write directly — brightness already applied via v
    rgbBuffer[index].r = r;
    rgbBuffer[index].g = g;
    rgbBuffer[index].b = b;
}

void RGB_FillColor ( uint8_t start, uint8_t end, uint8_t r, uint8_t g, uint8_t b )
{
    if ( start >= rgbLedCount ) return;
    if ( end >= rgbLedCount ) end = rgbLedCount - 1;

    for ( uint8_t i = start; i <= end; i++ ) {
        RGB_SetColor ( i, r, g, b );
    }
}

/*=============================================================================
 *  Brightness & Utility
 *===========================================================================*/

void RGB_SetBrightness ( uint8_t percent )
{
    if ( percent > 100 ) percent = 100;
    rgbBrightness = percent;
}

uint8_t RGB_GetLedCount ( void )
{
    return rgbLedCount;
}

void RGB_Clear ( void )
{
    for ( uint8_t i = 0; i < RGB_MAX_LEDS; i++ ) {
        rgbBuffer[i].r = 0;
        rgbBuffer[i].g = 0;
        rgbBuffer[i].b = 0;
    }
}

void RGB_Show ( void )
{
    rgbWriteDma();
}

/*=============================================================================
 *  Non-Blocking Animation Engine
 *===========================================================================*/

void RGB_StartAnimation ( RGB_Animation_e animation, uint16_t speed_ms,
                          RGB_Direction_e direction,
                          uint8_t r, uint8_t g, uint8_t b )
{
    animType      = animation;
    animSpeedMs   = speed_ms;
    animDirection = direction;
    animR         = r;
    animG         = g;
    animB         = b;
    animFrame     = 0;
    animStep      = 0;
    animPhase     = true;
    animLastMs    = millis();

    RGB_Clear();
    RGB_Show();
}

void RGB_StopAnimation ( void )
{
    animType = RGB_ANIM_NONE;
    RGB_Clear();
    RGB_Show();
}

bool RGB_IsAnimating ( void )
{
    return ( animType != RGB_ANIM_NONE );
}

void RGB_UpdateAnimation ( void )
{
    if ( animType == RGB_ANIM_NONE ) return;

    uint32_t now = millis();
    if ( ( now - animLastMs ) < animSpeedMs ) return;
    animLastMs = now;

    switch ( animType ) {

        /* ================================================================
         *  MOVEMENT
         * ============================================================= */

        case RGB_ANIM_CHASE: {
            RGB_Clear();
            RGB_SetColor ( mapIndex ( animFrame ), animR, animG, animB );
            RGB_Show();
            animFrame++;
            if ( animFrame >= rgbLedCount ) animFrame = 0;
            break;
        }

        case RGB_ANIM_METEOR: {
            RGB_Clear();
            uint8_t head = mapIndex ( animFrame );
            RGB_SetColor ( head, animR, animG, animB );
            for ( uint8_t t = 1; t <= 4 && t <= rgbLedCount; t++ ) {
                int16_t tf = ( int16_t ) animFrame - ( int16_t ) t;
                if ( tf < 0 ) tf += ( int16_t ) rgbLedCount;
                uint8_t fade = ( uint8_t ) ( 255 - ( t * 60 ) );
                uint8_t tr = ( uint8_t ) ( ( uint16_t ) animR * fade / 255 );
                uint8_t tg = ( uint8_t ) ( ( uint16_t ) animG * fade / 255 );
                uint8_t tb = ( uint8_t ) ( ( uint16_t ) animB * fade / 255 );
                RGB_SetColor ( mapIndex ( ( uint8_t ) ( tf & 0xFF ) ), tr, tg, tb );
            }
            RGB_Show();
            animFrame++;
            if ( animFrame >= rgbLedCount ) animFrame = 0;
            break;
        }

        case RGB_ANIM_DUAL_CHASE: {
            RGB_Clear();
            RGB_SetColor ( mapIndex ( animFrame ), animR, animG, animB );
            RGB_SetColor ( mapIndex ( animFrame + rgbLedCount / 2 ), animR, animG, animB );
            RGB_Show();
            animFrame++;
            if ( animFrame >= rgbLedCount ) animFrame = 0;
            break;
        }

        case RGB_ANIM_BOUNCE: {
            RGB_Clear();
            uint8_t pos = animPhase ? animFrame : ( uint8_t ) ( rgbLedCount - 1 - animFrame );
            RGB_SetColor ( pos, animR, animG, animB );
            RGB_Show();
            animFrame++;
            if ( animFrame >= rgbLedCount ) {
                animFrame = 0;
                animPhase = !animPhase;
            }
            break;
        }

        /* ================================================================
         *  FILL / DRAIN
         * ============================================================= */

        case RGB_ANIM_COLOR_WIPE: {
            RGB_SetColor ( mapIndex ( animFrame ), animR, animG, animB );
            RGB_Show();
            animFrame++;
            if ( animFrame >= rgbLedCount ) {
                animFrame = 0;
                RGB_Clear();
            }
            break;
        }

        case RGB_ANIM_FILL_DRAIN: {
            if ( animPhase ) {
                RGB_SetColor ( mapIndex ( animFrame ), animR, animG, animB );
            } else {
                RGB_SetColor ( mapIndex ( animFrame ), 0, 0, 0 );
            }
            RGB_Show();
            animFrame++;
            if ( animFrame >= rgbLedCount ) {
                animFrame = 0;
                animPhase = !animPhase;
            }
            break;
        }

        case RGB_ANIM_DROP: {
            uint8_t stackEnd = ( uint8_t ) ( rgbLedCount - 1 - ( uint8_t ) animStep );
            RGB_Clear();
            for ( uint8_t i = rgbLedCount - ( uint8_t ) animStep; i < rgbLedCount; i++ ) {
                RGB_SetColor ( mapIndex ( i ), animR, animG, animB );
            }
            RGB_SetColor ( mapIndex ( animFrame ), animR, animG, animB );
            RGB_Show();
            if ( animFrame >= stackEnd ) {
                animStep++;
                animFrame = 0;
                if ( animStep >= rgbLedCount ) {
                    animStep = 0;
                    RGB_Clear();
                    RGB_Show();
                }
            } else {
                animFrame++;
            }
            break;
        }

        case RGB_ANIM_HOURGLASS: {
            uint8_t half = rgbLedCount / 2;
            RGB_Clear();
            uint8_t topRemaining = half - ( uint8_t ) animStep;
            for ( uint8_t i = 0; i < topRemaining; i++ ) {
                RGB_SetColor ( mapIndex ( i ), animR, animG, animB );
            }
            for ( uint8_t i = 0; i < ( uint8_t ) animStep; i++ ) {
                RGB_SetColor ( mapIndex ( ( uint8_t ) ( rgbLedCount - 1 - i ) ), animR, animG, animB );
            }
            if ( animFrame == 0 ) {
                RGB_SetColor ( mapIndex ( half ), animR, animG, animB );
            }
            RGB_Show();
            animFrame = !animFrame;
            if ( animFrame == 0 ) {
                animStep++;
                if ( animStep > half ) animStep = 0;
            }
            break;
        }

        /* ================================================================
         *  STACK
         * ============================================================= */

        /*--- Sweep Stack: LED sweeps 0→N, stacks at end, sweep shrinks --*/
        /*    Pass 1: LED moves 0→7, parks at 7                          */
        /*    Pass 2: LED moves 0→6, parks at 6                          */
        /*    ...until all stacked, then clears and restarts             */
        case RGB_ANIM_SWEEP_STACK: {
            // animStep = how many LEDs stacked at the end
            // animFrame = current sweep position
            uint8_t sweepEnd = ( uint8_t ) ( rgbLedCount - 1 - ( uint8_t ) animStep );

            RGB_Clear();

            // Draw stacked LEDs at the end
            for ( uint8_t i = ( uint8_t ) ( rgbLedCount - ( uint8_t ) animStep ); i < rgbLedCount; i++ ) {
                RGB_SetColor ( mapIndex ( i ), animR, animG, animB );
            }

            // Draw the sweeping LED
            RGB_SetColor ( mapIndex ( animFrame ), animR, animG, animB );
            RGB_Show();

            if ( animFrame >= sweepEnd ) {
                // Landed — stack it
                animStep++;
                animFrame = 0;
                if ( animStep >= rgbLedCount ) {
                    animStep  = 0;
                    animFrame = 0;
                }
            } else {
                animFrame++;
            }
            break;
        }

        /*--- Sweep Unstack: reverse — unstacks, sweep grows -------------*/
        case RGB_ANIM_SWEEP_UNSTACK: {
            // animStep = how many have been removed
            // animFrame = current sweep position (going backwards)
            uint8_t stackStart = ( uint8_t ) animStep;

            RGB_Clear();

            // Draw remaining stacked LEDs
            for ( uint8_t i = stackStart; i < rgbLedCount; i++ ) {
                RGB_SetColor ( mapIndex ( i ), animR, animG, animB );
            }

            // Draw the sweeping LED moving back toward 0
            if ( animStep < rgbLedCount ) {
                uint8_t sweepPos = ( uint8_t ) ( stackStart - animFrame );
                RGB_SetColor ( mapIndex ( sweepPos ), 0, 0, 0 );  // erase as it sweeps back
                RGB_SetColor ( mapIndex ( animFrame ), animR, animG, animB );
            }
            RGB_Show();

            // Check if sweep reached the start
            if ( animFrame == 0 && animStep > 0 ) {
                animStep++;
                animFrame = ( uint8_t ) animStep;
                if ( animStep >= rgbLedCount ) {
                    animStep  = 0;
                    animFrame = 0;
                    // Refill for next cycle
                    RGB_SetColorAll ( animR, animG, animB );
                    RGB_Show();
                }
            } else if ( animFrame > 0 ) {
                animFrame--;
            } else {
                // First frame — fill all and start removing
                RGB_SetColorAll ( animR, animG, animB );
                RGB_Show();
                animStep  = 1;
                animFrame = 1;
            }
            break;
        }

        /*--- Converge: two LEDs from both ends meet at center -----------*/
        case RGB_ANIM_CONVERGE: {
            uint8_t half = rgbLedCount / 2;

            if ( animPhase ) {
                // Converging: LEDs move inward
                RGB_Clear();
                // Already placed LEDs (from previous converge steps)
                for ( uint8_t i = 0; i < ( uint8_t ) animStep; i++ ) {
                    RGB_SetColor ( mapIndex ( i ), animR, animG, animB );
                    RGB_SetColor ( mapIndex ( ( uint8_t ) ( rgbLedCount - 1 - i ) ), animR, animG, animB );
                }
                // Current moving pair
                uint8_t left  = ( uint8_t ) ( ( uint8_t ) animStep + animFrame );
                uint8_t right = ( uint8_t ) ( rgbLedCount - 1 - ( uint8_t ) animStep - animFrame );
                RGB_SetColor ( mapIndex ( left ), animR, animG, animB );
                if ( right != left ) {
                    RGB_SetColor ( mapIndex ( right ), animR, animG, animB );
                }
                RGB_Show();

                if ( left >= right || left >= half ) {
                    animStep++;
                    animFrame = 0;
                    if ( animStep >= half ) {
                        animPhase = false;
                        animStep  = 0;
                        animFrame = 0;
                    }
                } else {
                    animFrame++;
                }
            } else {
                // Pause with all lit, then restart
                RGB_SetColorAll ( animR, animG, animB );
                RGB_Show();
                animStep++;
                if ( animStep >= 3 ) {
                    animPhase = true;
                    animStep  = 0;
                    animFrame = 0;
                    RGB_Clear();
                    RGB_Show();
                }
            }
            break;
        }

        /*--- Expand: LEDs grow outward from center ----------------------*/
        case RGB_ANIM_EXPAND: {
            uint8_t half = rgbLedCount / 2;

            if ( animPhase ) {
                // Expanding outward
                RGB_Clear();
                uint8_t reach = ( uint8_t ) animStep + 1;
                for ( uint8_t i = 0; i < reach && i <= half; i++ ) {
                    uint8_t left  = ( uint8_t ) ( half - 1 - i );
                    uint8_t right = ( uint8_t ) ( half + i );
                    RGB_SetColor ( mapIndex ( left ), animR, animG, animB );
                    if ( right < rgbLedCount ) {
                        RGB_SetColor ( mapIndex ( right ), animR, animG, animB );
                    }
                }
                RGB_Show();
                animStep++;
                if ( animStep >= half ) {
                    animPhase = false;
                    animStep  = 0;
                }
            } else {
                // Contracting back to center
                RGB_Clear();
                uint8_t reach = ( uint8_t ) ( half - ( uint8_t ) animStep );
                for ( uint8_t i = 0; i < reach; i++ ) {
                    uint8_t left  = ( uint8_t ) ( half - 1 - i );
                    uint8_t right = ( uint8_t ) ( half + i );
                    RGB_SetColor ( mapIndex ( left ), animR, animG, animB );
                    if ( right < rgbLedCount ) {
                        RGB_SetColor ( mapIndex ( right ), animR, animG, animB );
                    }
                }
                RGB_Show();
                animStep++;
                if ( animStep >= half ) {
                    animPhase = true;
                    animStep  = 0;
                }
            }
            break;
        }

        /* ================================================================
         *  GLOW / FLASH
         * ============================================================= */

        case RGB_ANIM_BREATHE: {
            uint8_t val = ( uint8_t ) animStep;
            uint8_t r = ( uint8_t ) ( ( uint16_t ) animR * val / 255 );
            uint8_t g = ( uint8_t ) ( ( uint16_t ) animG * val / 255 );
            uint8_t b = ( uint8_t ) ( ( uint16_t ) animB * val / 255 );
            RGB_SetColorAll ( r, g, b );
            RGB_Show();
            if ( animPhase ) {
                animStep += 5;
                if ( animStep >= 255 ) { animStep = 255; animPhase = false; }
            } else {
                if ( animStep < 5 ) { animStep = 0; animPhase = true; }
                else animStep -= 5;
            }
            break;
        }

        case RGB_ANIM_BLINK: {
            if ( animFrame == 0 ) {
                RGB_SetColorAll ( animR, animG, animB );
            } else {
                RGB_Clear();
            }
            RGB_Show();
            animFrame = !animFrame;
            break;
        }

        case RGB_ANIM_ALTERNATE_BLINK: {
            RGB_Clear();
            for ( uint8_t i = 0; i < rgbLedCount; i++ ) {
                if ( ( i % 2 ) == animFrame ) {
                    RGB_SetColor ( i, animR, animG, animB );
                }
            }
            RGB_Show();
            animFrame = !animFrame;
            break;
        }

        case RGB_ANIM_SPARKLE: {
            uint8_t bR = ( uint8_t ) ( ( uint16_t ) animR * 40 / 255 );
            uint8_t bG = ( uint8_t ) ( ( uint16_t ) animG * 40 / 255 );
            uint8_t bB = ( uint8_t ) ( ( uint16_t ) animB * 40 / 255 );
            RGB_SetColorAll ( bR, bG, bB );
            uint8_t idx = ( uint8_t ) ( ( now * 7 + 13 ) % rgbLedCount );
            RGB_SetColor ( idx, animR, animG, animB );
            RGB_Show();
            break;
        }

        case RGB_ANIM_FIRE: {
            for ( uint8_t i = 0; i < rgbLedCount; i++ ) {
                uint8_t flicker = ( uint8_t ) ( ( ( now * ( i + 1 ) * 17 + i * 31 ) >> 2 ) & 0xFF );
                uint8_t heat = 150 + ( flicker % 106 );
                uint8_t r = heat;
                uint8_t g = ( uint8_t ) ( heat * 40 / 255 );
                RGB_SetColor ( i, r, g, 0 );
            }
            RGB_Show();
            break;
        }

        /* ================================================================
         *  COLOR PATTERNS
         * ============================================================= */

        case RGB_ANIM_RAINBOW: {
            for ( uint8_t i = 0; i < rgbLedCount; i++ ) {
                uint16_t hue = ( uint16_t ) ( ( animStep + ( uint16_t ) i * 360 / rgbLedCount ) % 360 );
                RGB_SetColorHSV ( i, hue, 255, 255 );
            }
            RGB_Show();
            animStep += 10;
            if ( animStep >= 360 ) animStep = 0;
            break;
        }

        case RGB_ANIM_THEATER_CHASE: {
            RGB_Clear();
            for ( uint8_t i = animFrame; i < rgbLedCount; i += 3 ) {
                RGB_SetColor ( mapIndex ( i ), animR, animG, animB );
            }
            RGB_Show();
            animFrame++;
            if ( animFrame >= 3 ) animFrame = 0;
            break;
        }

        case RGB_ANIM_WAVE: {
            for ( uint8_t i = 0; i < rgbLedCount; i++ ) {
                uint8_t angle = ( uint8_t ) ( animStep + i * 256 / rgbLedCount );
                uint8_t val   = sineLookup ( angle );
                uint8_t r = ( uint8_t ) ( ( uint16_t ) animR * val / 255 );
                uint8_t g = ( uint8_t ) ( ( uint16_t ) animG * val / 255 );
                uint8_t b = ( uint8_t ) ( ( uint16_t ) animB * val / 255 );
                RGB_SetColor ( i, r, g, b );
            }
            RGB_Show();
            animStep += 8;
            if ( animStep >= 256 ) animStep = 0;
            break;
        }

        case RGB_ANIM_DUAL_SCAN: {
            RGB_Clear();
            uint8_t half = rgbLedCount / 2;
            uint8_t pos = animPhase ? animFrame : ( uint8_t ) ( half - 1 - animFrame );
            RGB_SetColor ( mapIndex ( pos ), animR, animG, animB );
            RGB_SetColor ( mapIndex ( ( uint8_t ) ( rgbLedCount - 1 - pos ) ), animR, animG, animB );
            RGB_Show();
            animFrame++;
            if ( animFrame >= half ) {
                animFrame = 0;
                animPhase = !animPhase;
            }
            break;
        }

        default:
            break;
    }
}
