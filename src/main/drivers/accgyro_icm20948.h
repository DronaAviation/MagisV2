/*
 * This file is part of Cleanflight and Magis.
 *
 * Cleanflight and Magis are free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight and Magis are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once


#ifdef __cplusplus
extern "C" {
#endif

#define ICM20948_GYRO_OUT        0x33
#define ICM20948_ACCEL_OUT         0x2D

// Bank select
#define REG_BANK_SEL 0x7F
#define BANK_0       0x00
#define BANK_2       0x20

// Bank 0
#define REG_WHO_AM_I      0x00
#define REG_USER_CTRL     0x03
#define REG_PWR_MGMT_1    0x06
#define REG_PWR_MGMT_2    0x07

#define WHO_AM_I_ICM20948 0xEA

#define BIT_H_RESET       0x80
#define BIT_SLEEP         0x40
#define CLKSEL_PLL        0x01    // auto-select PLL if ready (common practice)

// Bank 2
#define REG_GYRO_SMPLRT_DIV    0x00
#define REG_GYRO_CONFIG_1      0x01

#define REG_ACCEL_SMPLRT_DIV_1 0x10
#define REG_ACCEL_SMPLRT_DIV_2 0x11
#define REG_ACCEL_CONFIG       0x14

bool icm20948AccDetect(acc_t *acc);
bool icm20948GyroDetect(gyro_t *gyro);

void icm20948AccInit(void);
void icm20948GyroInit(uint16_t lpf);




#ifdef __cplusplus
}
#endif
