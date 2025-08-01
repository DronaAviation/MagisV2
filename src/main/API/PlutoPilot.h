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

#ifndef _PlutoPilot_H_
#define _PlutoPilot_H_

#include "API/RxConfig.h"
#include "API/Peripheral.h"
#include "API/Peripherals.h"
#include "API/Status-LED.h"
#include "API/Motor.h"

void plutoRxConfig ( void );

void plutoInit ( void );

void onLoopStart ( void );

void plutoLoop ( void );

void onLoopFinish ( void );

#endif
