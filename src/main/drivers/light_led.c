/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\light_led.c                                        #
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

#include "light_led.h"

initLeds ( void ) {
  struct {
    GPIO_TypeDef *gpio;
    gpio_config_t cfg;
  } gpio_setup [] = {
#ifdef LED_R
    { .gpio = LED0_GPIO,
      .cfg  = { LED0_PIN, Mode_Out_PP, Speed_2MHz } },
#endif
#ifdef LED_G

    { .gpio = LED1_GPIO,
      .cfg  = { LED1_PIN, Mode_Out_PP, Speed_2MHz } },
#endif
#ifdef LED_B

    { .gpio = LED2_GPIO,
      .cfg  = { LED2_PIN, Mode_Out_PP, Speed_2MHz } },
#endif
  }

  uint8_t gpio_count
            = sizeof ( gpio_setup ) / sizeof ( gpio_setup [ 0 ] );
}
