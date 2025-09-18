/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\Status-LED.cpp                                     #
 #  Created Date: Fri, 16th May 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 18th Sep 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include "platform.h"

#include "API/Status-LED.h"

#include "drivers/light_led.h"

bool LedStatusState = true; // Indicates that the LED is currently ON

/**
 * @brief Sets the state of the LEDs based on the provided status and state.
 *
 * This function handles two types of operations:
 * 1. When the LED status is STATUS, it can turn off all LEDs or set the overall LED
 *    status to ON depending on the given state.
 * 2. For other LED statuses (RED, GREEN, BLUE), it directly controls the specified LED
 *    with the provided state (ON/OFF).
 *
 * @param _led_status The status of the LED to control (can be STATUS, RED, GREEN, or BLUE).
 * @param _state The desired state for the LED (ON or OFF).
 */
void Set_LED(led_status_e _led_status, led_state_e _state) {
    // Check if the provided LED status is STATUS
    if (_led_status == STATUS) {
        // If the state is OFF, turn off all LEDs and update the LED status state
        if (_state == OFF) {
            LedStatusState = false; // Set overall LED status to OFF
            ledOperator(RED, OFF);   // Turn off the RED LED
            ledOperator(GREEN, OFF); // Turn off the GREEN LED
            ledOperator(BLUE, OFF);  // Turn off the BLUE LED
        } 
        // If the state is ON, set the LED status state to ON
        else if (_state == ON) {
            LedStatusState = true; // Set overall LED status to ON
        }
        return; // Exit the function after handling the STATUS case
    }

    // For other LED statuses (GREEN, BLUE, RED), directly call ledOperator with the provided state
    ledOperator(_led_status, _state); // Control the specified LED with the given state
}

