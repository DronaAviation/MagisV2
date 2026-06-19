/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: techsavvyomi                                                       #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\RGB-LED.h                                              #
 #  Created Date: Fri, 4th Apr 2026                                            #
 #  Brief: Public API for WS2812B RGB LED strip control                        #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Fri, 4th Apr 2026                                           #
 #  Modified By: techsavvyomi                                                  #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  2026-04-04	techsavvyomi	Created RGB LED public API                     #
 #  2026-04-04	techsavvyomi	Reworked: direct RGB-to-DMA, bypass HSV        #
*******************************************************************************/

#ifndef RGB_LED_H
#define RGB_LED_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of WS2812B LEDs supported by the hardware (8 LEDs). */
#define RGB_MAX_LEDS 8

/*=============================================================================
 *  Animation Types
 *
 *  Pass one of these to RGB_StartAnimation() to play a non-blocking LED
 *  effect. Call RGB_UpdateAnimation() in your plutoLoop() to advance frames.
 *
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │  Movement    : CHASE, METEOR, DUAL_CHASE, BOUNCE                      │
 *  │  Fill/Drain  : COLOR_WIPE, FILL_DRAIN, DROP, HOURGLASS               │
 *  │  Stack       : SWEEP_STACK, SWEEP_UNSTACK, CONVERGE, EXPAND          │
 *  │  Glow/Flash  : BREATHE, BLINK, ALTERNATE_BLINK, SPARKLE, FIRE        │
 *  │  Color       : RAINBOW, THEATER_CHASE, WAVE, DUAL_SCAN               │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *===========================================================================*/

typedef enum {
  RGB_ANIM_NONE = 0,         /**< No animation (idle)                         */

  /* ── Movement ── */
  RGB_ANIM_CHASE,            /**< One LED orbits the ring                     */
  RGB_ANIM_METEOR,           /**< LED with a fading comet tail                */
  RGB_ANIM_DUAL_CHASE,       /**< Two LEDs chase from opposite sides          */
  RGB_ANIM_BOUNCE,           /**< LED bounces back and forth                  */

  /* ── Fill / Drain ── */
  RGB_ANIM_COLOR_WIPE,       /**< Fill LEDs one-by-one, then clear            */
  RGB_ANIM_FILL_DRAIN,       /**< Fill all LEDs up, then drain them out       */
  RGB_ANIM_DROP,             /**< LED falls from top and stacks at bottom     */
  RGB_ANIM_HOURGLASS,        /**< Sand-clock: top empties, bottom fills       */

  /* ── Stack ── */
  RGB_ANIM_SWEEP_STACK,      /**< LED sweeps 0->7, 0->6, 0->5... stacks up   */
  RGB_ANIM_SWEEP_UNSTACK,    /**< Reverse: unstacks and sweep grows back      */
  RGB_ANIM_CONVERGE,         /**< Two LEDs from both ends meet at center      */
  RGB_ANIM_EXPAND,           /**< LEDs expand outward from center, then back  */

  /* ── Glow / Flash ── */
  RGB_ANIM_BREATHE,          /**< Smooth fade in / fade out pulse             */
  RGB_ANIM_BLINK,            /**< Flash all LEDs on / off                     */
  RGB_ANIM_ALTERNATE_BLINK,  /**< Even / odd LEDs blink alternately           */
  RGB_ANIM_SPARKLE,          /**< Random LED sparkles over a dim base         */
  RGB_ANIM_FIRE,             /**< Warm flickering fire effect                 */

  /* ── Color ── */
  RGB_ANIM_RAINBOW,          /**< Rotating rainbow across all LEDs            */
  RGB_ANIM_THEATER_CHASE,    /**< Marquee-style chasing dots                  */
  RGB_ANIM_WAVE,             /**< Sine-wave brightness ripple                 */
  RGB_ANIM_DUAL_SCAN,        /**< Two LEDs scan from ends, meet and return    */

} RGB_Animation_e;

/**
 * @brief  Animation direction — controls which way patterns move.
 */
typedef enum {
  RGB_DIR_FORWARD = 0,       /**< LED 0 -> LED N  (clockwise)                */
  RGB_DIR_REVERSE,           /**< LED N -> LED 0  (counter-clockwise)         */
} RGB_Direction_e;

/*=============================================================================
 *  Initialization & Control
 *===========================================================================*/

/**
 * @brief  Initialize the WS2812B LED strip and take control from the system.
 *
 *         Configures DMA, timer, and GPIO for the LED data pin (PA15).
 *         After calling this, the built-in flight-status LEDs are disabled
 *         and you have full control via the RGB_* functions.
 *
 *         Safe to call multiple times — hardware is only configured once.
 *         Call this inside onLoopStart() so it re-initializes on every
 *         Developer Mode activation.
 *
 * @param  led_count  Number of LEDs on your strip (1 to RGB_MAX_LEDS).
 *                    Values above RGB_MAX_LEDS are clamped to 8.
 *
 * @code
 *   void onLoopStart ( void ) {
 *     RGB_Init ( 8 );             // 8-LED ring
 *     RGB_SetBrightness ( 80 );   // 80% brightness
 *     RGB_SetColorAll ( 255, 0, 0 );
 *     RGB_Show();
 *   }
 * @endcode
 */
void RGB_Init ( uint8_t led_count );

/**
 * @brief  Release the LED strip back to the system.
 *
 *         Stops any running animation, turns off all LEDs, and restores
 *         the default flight-status LED behaviour (onboard LED patterns).
 *         Call this inside onLoopFinish() when you exit Developer Mode.
 *
 * @code
 *   void onLoopFinish ( void ) {
 *     RGB_Release();    // LEDs off, system resumes control
 *   }
 * @endcode
 */
void RGB_Release ( void );

/*=============================================================================
 *  Color Control
 *
 *  All color functions write to an internal buffer. Nothing changes on the
 *  physical LED strip until you call RGB_Show().
 *===========================================================================*/

/**
 * @brief  Set the color of a single LED (buffered).
 *
 *         Writes the color into the buffer at the given index. The LED
 *         will not change until RGB_Show() is called.
 *
 * @param  index  LED position on the strip.
 *                0 = first LED, 7 = last LED (on an 8-LED strip).
 *                Values >= led_count are silently ignored.
 * @param  r      Red   intensity (0 = off, 255 = full red).
 * @param  g      Green intensity (0 = off, 255 = full green).
 * @param  b      Blue  intensity (0 = off, 255 = full blue).
 *
 * @note   Common color values:
 *         | Color   | R   | G   | B   |
 *         |---------|-----|-----|-----|
 *         | Red     | 255 |   0 |   0 |
 *         | Green   |   0 | 255 |   0 |
 *         | Blue    |   0 |   0 | 255 |
 *         | White   | 255 | 255 | 255 |
 *         | Yellow  | 255 | 255 |   0 |
 *         | Cyan    |   0 | 255 | 255 |
 *         | Magenta | 255 |   0 | 255 |
 *         | Orange  | 255 | 165 |   0 |
 *         | Pink    | 255 | 105 | 180 |
 *         | Purple  | 128 |   0 | 128 |
 *
 * @code
 *   RGB_SetColor ( 0, 255, 0, 0 );   // LED 0 = red
 *   RGB_SetColor ( 1, 0, 255, 0 );   // LED 1 = green
 *   RGB_SetColor ( 2, 0, 0, 255 );   // LED 2 = blue
 *   RGB_Show();                       // push to strip
 * @endcode
 */
void RGB_SetColor ( uint8_t index, uint8_t r, uint8_t g, uint8_t b );

/**
 * @brief  Set every LED on the strip to the same color (buffered).
 *
 * @param  r  Red   (0 - 255).
 * @param  g  Green (0 - 255).
 * @param  b  Blue  (0 - 255).
 *
 * @code
 *   RGB_SetColorAll ( 255, 255, 255 );  // all white
 *   RGB_Show();
 * @endcode
 */
void RGB_SetColorAll ( uint8_t r, uint8_t g, uint8_t b );

/**
 * @brief  Set a single LED color using HSV (Hue-Saturation-Value).
 *
 *         HSV is useful for smooth rainbow transitions — sweep the hue
 *         from 0 to 359 while keeping saturation and value at 255.
 *
 * @param  index  LED position (0 = first LED).
 * @param  h      Hue on the color wheel (0 - 359 degrees).
 *                  0 = Red, 60 = Yellow, 120 = Green,
 *                  180 = Cyan, 240 = Blue, 300 = Magenta.
 * @param  s      Saturation (0 = white/pastel, 255 = vivid pure color).
 * @param  v      Value / brightness (0 = off/black, 255 = full brightness).
 *
 * @code
 *   // Rainbow across 8 LEDs:
 *   for ( uint8_t i = 0; i < RGB_GetLedCount(); i++ ) {
 *     RGB_SetColorHSV ( i, i * 45, 255, 255 );   // 0, 45, 90, 135...
 *   }
 *   RGB_Show();
 * @endcode
 */
void RGB_SetColorHSV ( uint8_t index, uint16_t h, uint8_t s, uint8_t v );

/**
 * @brief  Fill a range of LEDs with the same color (buffered).
 *
 * @param  start  First LED index (inclusive).
 * @param  end    Last  LED index (inclusive). Clamped to led_count - 1.
 * @param  r      Red   (0 - 255).
 * @param  g      Green (0 - 255).
 * @param  b      Blue  (0 - 255).
 *
 * @code
 *   // First half red, second half blue (8-LED strip):
 *   RGB_FillColor ( 0, 3, 255, 0, 0 );   // LEDs 0-3 = red
 *   RGB_FillColor ( 4, 7, 0, 0, 255 );   // LEDs 4-7 = blue
 *   RGB_Show();
 * @endcode
 */
void RGB_FillColor ( uint8_t start, uint8_t end, uint8_t r, uint8_t g, uint8_t b );

/*=============================================================================
 *  Brightness & Utility
 *===========================================================================*/

/**
 * @brief  Set global brightness for the entire strip.
 *
 *         All subsequent RGB_SetColor / RGB_SetColorAll / animation calls
 *         are scaled by this value. Set brightness BEFORE setting colors.
 *
 * @param  percent  Brightness level:
 *                    0   = completely off (all LEDs dark).
 *                    50  = half brightness.
 *                    100 = full brightness (default).
 *                  Values above 100 are clamped to 100.
 *
 * @code
 *   RGB_SetBrightness ( 30 );              // 30% brightness
 *   RGB_SetColorAll ( 255, 255, 255 );     // dim white
 *   RGB_Show();
 * @endcode
 */
void RGB_SetBrightness ( uint8_t percent );

/**
 * @brief  Get the number of active LEDs configured by RGB_Init().
 *
 * @return Number of active LEDs (0 if RGB_Init has not been called).
 *
 * @code
 *   for ( uint8_t i = 0; i < RGB_GetLedCount(); i++ ) {
 *     RGB_SetColor ( i, 0, 255, 0 );
 *   }
 * @endcode
 */
uint8_t RGB_GetLedCount ( void );

/**
 * @brief  Turn off all LEDs (buffered — sets every LED to black).
 *
 *         The strip will not go dark until you call RGB_Show().
 *
 * @code
 *   RGB_Clear();
 *   RGB_Show();   // LEDs go dark now
 * @endcode
 */
void RGB_Clear ( void );

/**
 * @brief  Push all buffered colors to the physical LED strip via DMA.
 *
 *         Triggers a DMA transfer that sends RGB data to the WS2812B LEDs.
 *         Call this after any RGB_SetColor / RGB_Clear / RGB_FillColor calls
 *         to make the changes visible on the strip.
 *
 * @note   You do NOT need to call this when using animations —
 *         RGB_UpdateAnimation() handles it automatically.
 *
 * @code
 *   RGB_SetColor ( 0, 255, 0, 0 );   // buffer red on LED 0
 *   RGB_SetColor ( 1, 0, 255, 0 );   // buffer green on LED 1
 *   RGB_Show();                       // both LEDs update now
 * @endcode
 */
void RGB_Show ( void );

/*=============================================================================
 *  Non-Blocking Animations
 *
 *  Usage pattern:
 *    1. Call RGB_StartAnimation() once to begin an effect.
 *    2. Call RGB_UpdateAnimation() every iteration in plutoLoop().
 *       It checks timing internally — safe to call as fast as you want.
 *    3. Animation runs until you call RGB_StopAnimation() or start a new one.
 *
 *  Example:
 *    void onLoopStart ( void ) {
 *      RGB_Init ( 8 );
 *      RGB_StartAnimation ( RGB_ANIM_RAINBOW, 100, RGB_DIR_FORWARD, 0, 0, 0 );
 *    }
 *    void plutoLoop ( void ) {
 *      RGB_UpdateAnimation();   // must call every loop
 *    }
 *    void onLoopFinish ( void ) {
 *      RGB_Release();
 *    }
 *===========================================================================*/

/**
 * @brief  Start a non-blocking LED animation.
 *
 *         Begins the chosen animation pattern. The animation advances
 *         automatically each time RGB_UpdateAnimation() is called.
 *         Starting a new animation stops the previous one.
 *
 * @param  animation  Which animation to play (see RGB_Animation_e).
 *
 *                    Movement:
 *                      RGB_ANIM_CHASE           — one LED orbits the ring
 *                      RGB_ANIM_METEOR          — comet with fading tail
 *                      RGB_ANIM_DUAL_CHASE      — two LEDs from opposite sides
 *                      RGB_ANIM_BOUNCE          — LED bounces back and forth
 *
 *                    Fill / Drain:
 *                      RGB_ANIM_COLOR_WIPE      — fill one-by-one
 *                      RGB_ANIM_FILL_DRAIN      — fill up, then drain out
 *                      RGB_ANIM_DROP            — LED falls and stacks
 *                      RGB_ANIM_HOURGLASS       — sand clock effect
 *
 *                    Stack:
 *                      RGB_ANIM_SWEEP_STACK     — sweep 0->7, 0->6, 0->5...
 *                      RGB_ANIM_SWEEP_UNSTACK   — reverse sweep unstack
 *                      RGB_ANIM_CONVERGE        — two LEDs meet at center
 *                      RGB_ANIM_EXPAND          — expand from center outward
 *
 *                    Glow / Flash:
 *                      RGB_ANIM_BREATHE         — fade in / out pulse
 *                      RGB_ANIM_BLINK           — flash on / off
 *                      RGB_ANIM_ALTERNATE_BLINK — even/odd blink
 *                      RGB_ANIM_SPARKLE         — random sparkle
 *                      RGB_ANIM_FIRE            — warm fire flicker
 *
 *                    Color:
 *                      RGB_ANIM_RAINBOW         — rotating rainbow (r,g,b ignored)
 *                      RGB_ANIM_THEATER_CHASE   — marquee chasing dots
 *                      RGB_ANIM_WAVE            — sine brightness ripple
 *                      RGB_ANIM_DUAL_SCAN       — scan from both ends
 *
 * @param  speed_ms   Time per animation frame in milliseconds.
 *                    Lower = faster animation. Typical range: 50 – 500 ms.
 *                    Recommended: 100-200 for movement, 30-50 for breathe,
 *                    300-500 for blink.
 *
 * @param  direction  RGB_DIR_FORWARD — LED 0 to LED N (clockwise).
 *                    RGB_DIR_REVERSE — LED N to LED 0 (counter-clockwise).
 *
 * @param  r          Red   component for the animation color (0 - 255).
 * @param  g          Green component for the animation color (0 - 255).
 * @param  b          Blue  component for the animation color (0 - 255).
 *                    Note: RGB_ANIM_RAINBOW ignores r, g, b (pass 0,0,0).
 *                    Note: RGB_ANIM_FIRE uses r,g,b as base warm tone.
 *
 * @code
 *   // Blue chase, clockwise, 200ms per step:
 *   RGB_StartAnimation ( RGB_ANIM_CHASE, 200, RGB_DIR_FORWARD, 0, 0, 255 );
 *
 *   // Red breathing pulse:
 *   RGB_StartAnimation ( RGB_ANIM_BREATHE, 40, RGB_DIR_FORWARD, 255, 0, 0 );
 *
 *   // Rainbow (color ignored):
 *   RGB_StartAnimation ( RGB_ANIM_RAINBOW, 100, RGB_DIR_FORWARD, 0, 0, 0 );
 *
 *   // Green sweep-and-stack:
 *   RGB_StartAnimation ( RGB_ANIM_SWEEP_STACK, 100, RGB_DIR_FORWARD, 0, 255, 0 );
 * @endcode
 */
void RGB_StartAnimation ( RGB_Animation_e animation, uint16_t speed_ms,
                          RGB_Direction_e direction,
                          uint8_t r, uint8_t g, uint8_t b );

/**
 * @brief  Advance the current animation by one tick (if timing is due).
 *
 *         Call this inside plutoLoop() on every iteration. It checks the
 *         elapsed time internally — calling it more often than speed_ms
 *         is harmless, the frame simply won't advance yet.
 *
 * @note   Does nothing if no animation is active.
 * @note   Calls RGB_Show() automatically — do not call it yourself
 *         when using animations.
 *
 * @code
 *   void plutoLoop ( void ) {
 *     RGB_UpdateAnimation();   // always call this
 *   }
 * @endcode
 */
void RGB_UpdateAnimation ( void );

/**
 * @brief  Stop the current animation and turn off all LEDs immediately.
 *
 * @code
 *   if ( someCondition ) {
 *     RGB_StopAnimation();   // LEDs go dark
 *   }
 * @endcode
 */
void RGB_StopAnimation ( void );

/**
 * @brief  Check whether an animation is currently running.
 *
 * @return true  — an animation is active and advancing.
 * @return false — no animation running (idle or stopped).
 *
 * @code
 *   if ( !RGB_IsAnimating() ) {
 *     RGB_StartAnimation ( RGB_ANIM_RAINBOW, 100, RGB_DIR_FORWARD, 0, 0, 0 );
 *   }
 * @endcode
 */
bool RGB_IsAnimating ( void );

#ifdef __cplusplus
}
#endif

#endif
