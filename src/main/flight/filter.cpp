/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\flight\filter.cpp                                          #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sun, 20th Apr 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"

#include "flight/filter.h"

extern uint16_t cycleTime;

biquad_t accBiquad [ XYZ_AXIS_COUNT ];     // X, Y, Z
biquad_t gyroBiquad [ XYZ_AXIS_COUNT ];    // Optional, if you want to filter gyro too

// PT1 Low Pass filter
float filterApplyPt1 ( float input, filterStatePt1_t *state, uint8_t f_cut ) {
  float dT = ( float ) cycleTime * 0.000001f;
  float RC = 1.0f / ( 2.0f * ( float ) M_PI * f_cut );

  *state = *state + dT / ( RC + dT ) * ( input - *state );

  return *state;
}

//!
//! NEW : Bi-Quad Filter integration
//!

void biquadInitLPF ( biquad_t *filter, float cutoffFreq, float sampleRate ) {
  float omega     = 2.0f * M_PIf * cutoffFreq / sampleRate;
  float sin_omega = sinf ( omega );
  float cos_omega = cosf ( omega );
  float alpha     = sin_omega / ( 2.0f * sqrtf ( 2.0f ) );    // Q = √2 / 2

  float b0 = ( 1.0f - cos_omega ) / 2.0f;
  float b1 = 1.0f - cos_omega;
  float b2 = ( 1.0f - cos_omega ) / 2.0f;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cos_omega;
  float a2 = 1.0f - alpha;

  filter->b0 = b0 / a0;
  filter->b1 = b1 / a0;
  filter->b2 = b2 / a0;
  filter->a1 = a1 / a0;
  filter->a2 = a2 / a0;

  filter->x1 = filter->x2 = 0.0f;
  filter->y1 = filter->y2 = 0.0f;
}

float biquadApply ( biquad_t *filter, float x ) {
  float result = filter->b0 * x + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

  filter->x2 = filter->x1;
  filter->x1 = x;
  filter->y2 = filter->y1;
  filter->y1 = result;

  return result;
}

static bool filtersInitialized = false;
static uint32_t previousT      = 0;

// Initialization (to be called once)
void initIMUFilters (  ) {
  uint32_t currentT = millis ( );
  uint32_t deltaT   = currentT - previousT;
  previousT         = currentT;
  float cutoffAcc   = 30.0f;
  float cutoffGyro  = 60.0f;

  if ( ! filtersInitialized && deltaT > 0 ) {
    for ( int axis = 0; axis < 3; axis++ ) {
      biquadInitLPF ( &accBiquad [ axis ], cutoffAcc, deltaT );
      biquadInitLPF ( &gyroBiquad [ axis ], cutoffGyro, deltaT );
    }
    filtersInitialized = true;
  }
}