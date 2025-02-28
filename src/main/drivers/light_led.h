/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\light_led.h                                        #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sat, 22nd Feb 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Helpful macros
#ifdef LED_B
  #define LED_B_TOGGLE digitalToggle ( LED2_GPIO, LED2_PIN )
  #ifndef LED_B_INVERTED
    #define LED_B_OFF digitalHi ( LED2_GPIO, LED2_PIN )
    #define LED_B_ON  digitalLo ( LED2_GPIO, LED2_PIN )
  #else
    #define LED_B_OFF digitalLo ( LED2_GPIO, LED2_PIN )
    #define LED_B_ON  digitalHi ( LED2_GPIO, LED2_PIN )
  #endif    // inverted
#else
  #define LED_B_TOGGLE \
    do {               \
    } while ( 0 )
  #define LED_B_OFF \
    do {            \
    } while ( 0 )
  #define LED_B_ON \
    do {           \
    } while ( 0 )
#endif

#ifdef LED_R
  #define LED_R_TOGGLE digitalToggle ( LED0_GPIO, LED0_PIN )
  #ifndef LED_R_INVERTED
    #define LED_R_OFF digitalHi ( LED0_GPIO, LED0_PIN )
    #define LED_R_ON  digitalLo ( LED0_GPIO, LED0_PIN )
  #else
    #define LED_R_OFF digitalLo ( LED0_GPIO, LED0_PIN )
    #define LED_R_ON  digitalHi ( LED0_GPIO, LED0_PIN )
  #endif    // inverted
#else
  #define LED_R_TOGGLE \
    do {               \
    } while ( 0 )
  #define LED_R_OFF \
    do {            \
    } while ( 0 )
  #define LED_R_ON \
    do {           \
    } while ( 0 )
#endif

#ifdef LED_G
  #define LED_G_TOGGLE digitalToggle ( LED1_GPIO, LED1_PIN )
  #ifndef LED_G_INVERTED
    #define LED_G_OFF digitalHi ( LED1_GPIO, LED1_PIN )
    #define LED_G_ON  digitalLo ( LED1_GPIO, LED1_PIN )
  #else
    #define LED_G_OFF digitalLo ( LED1_GPIO, LED1_PIN )
    #define LED_G_ON  digitalHi ( LED1_GPIO, LED1_PIN )
  #endif    // inverted
#else
  #define LED_G_TOGGLE \
    do {               \
    } while ( 0 )
  #define LED_G_OFF \
    do {            \
    } while ( 0 )
  #define LED_G_ON \
    do {           \
    } while ( 0 )
#endif

#ifdef LED3
  #define LED3_TOGGLE digitalToggle ( LED3_GPIO, LED3_PIN )
  #ifndef LED3_INVERTED
    #define LED3_OFF digitalLo ( LED3_GPIO, LED3_PIN )
    #define LED3_ON  digitalHi ( LED3_GPIO, LED3_PIN )
  #else
    #define LED3_OFF digitalLo ( LED3_GPIO, LED3_PIN )
    #define LED3_ON  digitalHi ( LED3_GPIO, LED3_PIN )
  #endif    // inverted
#else
  #define LED3_TOGGLE \
    do {              \
    } while ( 0 )
  #define LED3_OFF \
    do {           \
    } while ( 0 )
  #define LED3_ON \
    do {          \
    } while ( 0 )
#endif

#ifdef LED4
  #define LED4_TOGGLE digitalToggle ( LED4_GPIO, LED4_PIN )
  #ifndef LED4_INVERTED
    #define LED4_OFF digitalLo ( LED4_GPIO, LED4_PIN )
    #define LED4_ON  digitalHi ( LED4_GPIO, LED4_PIN )
  #else
    #define LED4_OFF digitalLo ( LED4_GPIO, LED4_PIN )
    #define LED4_ON  digitalHi ( LED4_GPIO, LED4_PIN )
  #endif    // inverted
#else
  #define LED4_TOGGLE \
    do {              \
    } while ( 0 )
  #define LED4_OFF \
    do {           \
    } while ( 0 )
  #define LED4_ON \
    do {          \
    } while ( 0 )
#endif

#ifdef TEST_ENABLE

  #define LEDx_GPIO GPIOA    // DD
  #define LEDx_PIN  Pin_4    // PA4 (LED)
  #define LEDx

  #define LEDy_GPIO GPIOA
  #define LEDy_PIN  Pin_5    // PA5 (LED)
  #define LEDy

  #define LEDz_GPIO GPIOA
  #define LEDz_PIN  Pin_6    // PA6 (LED)
  #define LEDz

  #ifdef LEDx
    #define LEDx_OFF    digitalLo ( LEDx_GPIO, LEDx_PIN )
    #define LEDx_ON     digitalHi ( LEDx_GPIO, LEDx_PIN )
    #define LEDx_TOGGLE digitalToggle ( LEDx_GPIO, LEDx_PIN )
  #endif

  #ifdef LEDy
    #define LEDy_OFF    digitalLo ( LEDy_GPIO, LEDy_PIN )
    #define LEDy_ON     digitalHi ( LEDy_GPIO, LEDy_PIN )
    #define LEDy_TOGGLE digitalToggle ( LEDy_GPIO, LEDy_PIN )
  #endif

  #ifdef LEDz
    #define LEDz_OFF    digitalLo ( LEDz_GPIO, LEDz_PIN )
    #define LEDz_ON     digitalHi ( LEDz_GPIO, LEDz_PIN )
    #define LEDz_TOGGLE digitalToggle ( LEDz_GPIO, LEDz_PIN )
  #endif

#endif

void ledInit ( void );

/* Drona Aviation */

#ifdef LED_ENABLE

  #define LEDb       1
  #define LEDr       2
  #define LEDg       0
  #define LED_3      3
  #define LED_4      4

  #define LED_OFF    0
  #define LED_ON     1
  #define LED_TOGGLE 2

void ledOperator ( uint32_t led_no, uint32_t led_status );

#endif

#ifdef __cplusplus
}
#endif
