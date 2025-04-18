/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/utils.h"

#include "system.h"
#include "gpio.h"

#include "light_led.h"

void ledInit ( void ) {
#if defined( LED_B ) || defined( LED_R ) || defined( LED_G ) || defined( LED3 ) || defined( TEST_ENABLE )
  uint32_t i;

  struct {
    GPIO_TypeDef *gpio;
    gpio_config_t cfg;
  } gpio_setup [] = {
  #ifdef LED_B
    { .gpio = LED0_GPIO,
      .cfg  = { LED0_PIN, Mode_Out_PP, Speed_2MHz } },
  #endif
  #ifdef LED_R
    { .gpio = LED1_GPIO,
      .cfg  = { LED1_PIN, Mode_Out_PP, Speed_2MHz } },
  #endif
  #ifdef LED_G
    { .gpio = LED2_GPIO,
      .cfg  = { LED2_PIN, Mode_Out_PP, Speed_2MHz } },
  #endif
  #ifdef LED3
    { .gpio = LED3_GPIO,
      .cfg  = { LED3_PIN, Mode_Out_PP, Speed_2MHz } },
  #endif
  #ifdef LED4
    { .gpio = LED4_GPIO,
      .cfg  = { LED4_PIN, Mode_Out_PP, Speed_2MHz } },
  #endif
  #ifdef TEST_ENABLE
    #ifdef LEDx
    { .gpio = LEDx_GPIO,
      .cfg  = { LEDx_PIN, Mode_Out_PP, Speed_2MHz } },
    #endif
    #ifdef LEDy
    { .gpio = LEDy_GPIO,
      .cfg  = { LEDy_PIN, Mode_Out_PP, Speed_2MHz } },
    #endif
    #ifdef LEDz
    { .gpio = LEDz_GPIO,
      .cfg  = { LEDz_PIN, Mode_Out_PP, Speed_2MHz } },
    #endif
  #endif
  };

  uint8_t gpio_count = ARRAYLEN ( gpio_setup );

  #ifdef LED_B
  RCC_APB2PeriphClockCmd ( LED0_PERIPHERAL, ENABLE );
  #endif
  #ifdef LED_R
  RCC_APB2PeriphClockCmd ( LED1_PERIPHERAL, ENABLE );
  #endif
  #ifdef LED_G
  RCC_APB2PeriphClockCmd ( LED2_PERIPHERAL, ENABLE );
  #endif
  #ifdef LED3
  RCC_APB2PeriphClockCmd ( LED3_PERIPHERAL, ENABLE );
  #endif
  #ifdef LED4
  RCC_APB2PeriphClockCmd ( LED4_PERIPHERAL, ENABLE );
  #endif
  #ifdef TEST_ENABLE
  RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOA, ENABLE );
  LEDx_OFF;
  LEDy_OFF;
  LEDz_OFF;
  #endif

  LED_B_OFF;
  LED_R_OFF;
  LED_G_OFF;
  LED3_OFF;
  LED4_OFF;

  for ( i = 0; i < gpio_count; i++ ) {
    gpioInit ( gpio_setup [ i ].gpio, &gpio_setup [ i ].cfg );
  }

#endif
}

/* Drona Aviation */

#ifdef LED_ENABLE

void ledOperator ( uint32_t led_no, uint32_t led_status ) {

  switch ( led_no ) {

    case LEDb:

      if ( led_status == LED_OFF )
        LED_B_OFF;

      else if ( led_status == LED_ON )
        LED_B_ON;
      else
        LED_B_TOGGLE;

      break;

    case LEDr:

      if ( led_status == LED_OFF )
        LED_R_OFF;

      else if ( led_status == LED_ON )
        LED_R_ON;
      else
        LED_R_TOGGLE;

      break;

    case LEDg:

      if ( led_status == LED_OFF )
        LED_G_OFF;

      else if ( led_status == LED_ON )
        LED_G_ON;
      else
        LED_G_TOGGLE;

      break;

    case LED_3:

      if ( led_status == LED_OFF )
        LED3_OFF;

      else if ( led_status == LED_ON )
        LED3_ON;
      else
        LED3_TOGGLE;

      break;

    case LED_4:

      if ( led_status == LED_OFF )
        LED4_OFF;

      else if ( led_status == LED_ON )
        LED4_ON;
      else
        LED4_TOGGLE;

      break;

    default:

      break;
  }
}

#endif
