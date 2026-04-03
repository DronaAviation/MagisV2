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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "exti.h"
#include "gpio.h"
#include "bus_i2c.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_icm20948.h"
#include "config/runtime_config.h"

extern uint16_t acc_1G;
extern uint8_t mpuLowPassFilter;

static uint8_t icmCurrentBank = 0xFF;

static inline void icmSelectBank ( uint8_t bank ) {
  mpuConfiguration.write ( REG_BANK_SEL, bank );
}

bool icm20948AccDetect ( acc_t *acc ) {
  if ( mpuDetectionResult.sensor != MPU_ICM_20948 ) {
    return false;
  }

  acc->init = icm20948AccInit;
  acc->read = mpuAccRead;

  return true;
}

bool icm20948GyroDetect ( gyro_t *gyro ) {
  if ( mpuDetectionResult.sensor != MPU_ICM_20948 ) {
    return false;
  }

  gyro->init = icm20948GyroInit;
  gyro->read = mpuGyroRead;

  // 16.4 dps/lsb scalefactor
  // gyro->scale = 1.0f / 16.4f;
  gyro->scale = 1.0f / 131.0f;

  return true;
}

void icm20948AccInit ( void ) {
  mpuIntExtiInit ( );

  acc_1G = 512 * 8;
}

void icm20948GyroInit ( uint16_t lpf ) {
  // 1) INT pin / EXTI (your existing code)
  mpuIntExtiInit ( );

  // 2) Go to Bank 0 and reset
  icmSelectBank ( BANK_0 );
  mpuConfiguration.write ( REG_PWR_MGMT_1, BIT_H_RESET );
  delay ( 150 );

  // 3) Wake + choose clock (also clears sleep if it was set)
  //    Many guides recommend writing 0x01 to get it out of sleep. :contentReference[oaicite:1]{index=1}
  mpuConfiguration.write ( REG_PWR_MGMT_1, CLKSEL_PLL );
  delay ( 50 );

  uint8_t who = 0;

  mpuConfiguration.read ( REG_WHO_AM_I, 1, &who );
  if ( who != WHO_AM_I_ICM20948 ) {
    FC_Reboot_Led ( );
  }

  // 4) Ensure primary I2C stays enabled:
  //    DO NOT set I2C_IF_DIS for I2C-only.
  //    Also keep I2C_MST disabled unless you specifically need internal mag master mode.
  mpuConfiguration.write ( REG_USER_CTRL, 0x00 );
  delay ( 1 );

  // 5) Make sure accel+gyro are enabled (PWR_MGMT_2: 0 = enabled)
  mpuConfiguration.write ( REG_PWR_MGMT_2, 0x00 );
  delay ( 1 );

  // 7) Now configure gyro/accel in Bank 2 (your original part, with small cleanup)
  icmSelectBank ( BANK_2 );
  delay ( 1 );

  // --- Gyro ---
  // Example: DLPF ON + FS=2000dps + some DLPF cfg
  // Your existing value: 0x1F
  mpuConfiguration.write ( REG_GYRO_CONFIG_1, 0x1F );
  delay ( 1 );

  // Sample rate divider = 0 => max base ODR (depends on mode)
  mpuConfiguration.write ( REG_GYRO_SMPLRT_DIV, 0x00 );
  delay ( 1 );

  // --- Accel ---
  // Your existing value: 0x35
  mpuConfiguration.write ( REG_ACCEL_CONFIG, 0x35 );
  delay ( 1 );

  mpuConfiguration.write ( REG_ACCEL_SMPLRT_DIV_1, 0x00 );
  delay ( 1 );
  mpuConfiguration.write ( REG_ACCEL_SMPLRT_DIV_2, 0x00 );
  delay ( 1 );

  // 8) Return to Bank 0 (good hygiene; many later reads assume Bank 0)
  icmSelectBank ( BANK_0 );
  delay ( 1 );
}
