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
#include <string.h>

#include "platform.h"

#include "build_config.h"

#include "common/axis.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/exti.h"

#include "drivers/sensor.h"

#include "drivers/accgyro.h"
#include "drivers/accgyro_mpu.h"
#include "drivers/accgyro_icm20948.h"

#include "drivers/bus_spi.h"

#include "drivers/barometer.h"
#include "drivers/barometer_icp10111.h"

#include "drivers/compass.h"
#include "drivers/compass_ak09916.h"


#include "config/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/initialisation.h"



extern float magneticDeclination;

extern gyro_t gyro;
extern baro_t baro;
extern acc_t acc;

uint8_t detectedSensors[MAX_SENSORS_TO_DETECT] = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE };

const extiConfig_t *selectMPUIntExtiConfig(void)
{
    return NULL;
}

#ifdef USE_FAKE_GYRO
static void fakeGyroInit(uint16_t lpf)
{
    UNUSED(lpf);
}

static bool fakeGyroRead(int16_t *gyroADC)
{
    memset(gyroADC, 0, sizeof(int16_t[XYZ_AXIS_COUNT]));
    return true;
}

static bool fakeGyroReadTemp(int16_t *tempData)
{
    UNUSED(tempData);
    return true;
}

bool fakeGyroDetect(gyro_t *gyro)
{
    gyro->init = fakeGyroInit;
    gyro->read = fakeGyroRead;
    gyro->temperature = fakeGyroReadTemp;
    return true;
}
#endif

#ifdef USE_FAKE_ACC
static void fakeAccInit(void) {}
static bool fakeAccRead(int16_t *accData) {
    memset(accData, 0, sizeof(int16_t[XYZ_AXIS_COUNT]));
    return true;
}

bool fakeAccDetect(acc_t *acc)
{
    acc->init = fakeAccInit;
    acc->read = fakeAccRead;
    acc->revisionCode = 0;
    return true;
}
#endif

bool detectGyro(void)
{
    gyroSensor_e gyroHardware = GYRO_DEFAULT;

    gyroAlign = ALIGN_DEFAULT;

    switch (gyroHardware) {
        case GYRO_DEFAULT:
            ; // fallthrough
        case GYRO_ICM20948:

#ifdef USE_GYRO_ICM20948
//#ifdef USE_GYRO_SPI_MPU6500            //DD
//            if (mpu6500GyroDetect(&gyro) || mpu6500SpiGyroDetect(&gyro))
//#else
            if (icm20948GyroDetect(&gyro))
//#endif
            {
                gyroHardware = GYRO_ICM20948;
#ifdef GYRO_ICM20948_ALIGN
                gyroAlign = GYRO_ICM20948_ALIGN;
#endif

                break;
            }
#endif
            ; // fallthrough

        case GYRO_FAKE:
#ifdef USE_FAKE_GYRO
            if (fakeGyroDetect(&gyro)) {
                gyroHardware = GYRO_FAKE;
                break;
            }
#endif
            ; // fallthrough
        case GYRO_NONE:
            gyroHardware = GYRO_NONE;
    }

    if (gyroHardware == GYRO_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_GYRO] = gyroHardware;
    sensorsSet(SENSOR_GYRO);

    return true;
}

static void detectAcc(accelerationSensor_e accHardwareToUse)
{
    accelerationSensor_e accHardware;

#ifdef USE_ACC_ADXL345
    drv_adxl345_config_t acc_params;
#endif

    retry: accAlign = ALIGN_DEFAULT;

    switch (accHardwareToUse) {
        case ACC_DEFAULT:
            ; // fallthrough
        case ACC_ICM20948:
#ifdef USE_ACC_ICM20948
//changes made by DD
//#ifdef USE_ACC_SPI_MPU6500
//            if (mpu6500AccDetect(&acc) || mpu6500SpiAccDetect(&acc))
//#else
            if (icm20948AccDetect(&acc))
//#endif
            {
#ifdef ACC_ICM20948_ALIGN
                accAlign = ACC_ICM20948_ALIGN;
#endif
                accHardware = ACC_ICM20948;
                break;
            }
#endif
            ; // fallthrough


        case ACC_FAKE:
#ifdef USE_FAKE_ACC
            if (fakeAccDetect(&acc)) {
                accHardware = ACC_FAKE;
                break;
            }
#endif
            ; // fallthrough
        case ACC_NONE: // disable ACC
            accHardware = ACC_NONE;
            break;

    }

    // Found anything? Check if error or ACC is really missing.
    if (accHardware == ACC_NONE && accHardwareToUse != ACC_DEFAULT && accHardwareToUse != ACC_NONE) {
        // Nothing was found and we have a forced sensor that isn't present.
        accHardwareToUse = ACC_DEFAULT;
        goto retry;
    }

    if (accHardware == ACC_NONE) {
        return;
    }

    detectedSensors[SENSOR_INDEX_ACC] = accHardware;
    sensorsSet(SENSOR_ACC);
}

static bool detectBaro(baroSensor_e baroHardwareToUse)
{
#ifndef BARO
    UNUSED(baroHardwareToUse);
#else
    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function

    baroSensor_e baroHardware = baroHardwareToUse;



    switch (baroHardware) {
        case BARO_DEFAULT:
            ; // fallthough

        case BARO_ICP10111:

#ifdef USE_BARO_ICP10111
            if (icp10111Detect(&baro)) {
                baroHardware = BARO_ICP10111;
                break;
            }
#endif
            break;
        case BARO_NONE:
            baroHardware = BARO_NONE;
            break;
    }

    if (baroHardware == BARO_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_BARO] = baroHardware;
    sensorsSet(SENSOR_BARO);
    return true;
#endif
}
#ifdef MAG
static void detectMag(magSensor_e magHardwareToUse)
{
    magSensor_e magHardware;



    retry:

    magAlign = ALIGN_DEFAULT;

    switch (magHardwareToUse) {
        case MAG_DEFAULT:
            ; // fallthrough

        case MAG_AK09916:
#ifdef USE_MAG_AK09916
            if (ak09916Detect(&mag)) {
#ifdef MAG_AK09916_ALIGN
                magAlign = MAG_AK09916_ALIGN;
#endif
                magHardware = MAG_AK09916;
                break;
            }
#endif
            break;
        case MAG_NONE:
            magHardware = MAG_NONE;
            break;
    }

    if (magHardware == MAG_NONE && magHardwareToUse != MAG_DEFAULT && magHardwareToUse != MAG_NONE) {
        // Nothing was found and we have a forced sensor that isn't present.
        magHardwareToUse = MAG_DEFAULT;
        goto retry;
    }

    if (magHardware == MAG_NONE) {
        return;
    }

    detectedSensors[SENSOR_INDEX_MAG] = magHardware;
    sensorsSet(SENSOR_MAG);
}
#endif
void reconfigureAlignment(sensorAlignmentConfig_t *sensorAlignmentConfig)
{
    if (sensorAlignmentConfig->gyro_align != ALIGN_DEFAULT) {
        gyroAlign = sensorAlignmentConfig->gyro_align;
    }
    if (sensorAlignmentConfig->acc_align != ALIGN_DEFAULT) {
        accAlign = sensorAlignmentConfig->acc_align;
    }
    if (sensorAlignmentConfig->mag_align != ALIGN_DEFAULT) {
        magAlign = sensorAlignmentConfig->mag_align;
    }
}

bool sensorsAutodetectmpu(sensorAlignmentConfig_t *sensorAlignmentConfig, uint16_t gyroLpf, uint8_t accHardwareToUse, uint8_t magHardwareToUse)
{

    memset(&acc, 0, sizeof(acc));
    memset(&gyro, 0, sizeof(gyro));

#if defined(USE_GYRO_MPU6050) || defined(USE_GYRO_ICM20948) || defined(USE_GYRO_MPU3050) || defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6000) || defined(USE_ACC_MPU6050)

    const extiConfig_t *extiConfig = selectMPUIntExtiConfig();

    mpuDetectionResult_t *mpuDetectionResult = detectMpu(extiConfig);
    UNUSED(mpuDetectionResult);
#endif

    if (!detectGyro()) {
        return false;
    }
    detectAcc((accelerationSensor_e) accHardwareToUse);

    // Now time to init things, acc first
    if (sensors(SENSOR_ACC))
        acc.init();
    // this is safe because either mpu6050 or mpu3050 or lg3d20 sets it, and in case of fail, we never get here.
    gyro.init(gyroLpf);

    detectMag((magSensor_e) magHardwareToUse);

    reconfigureAlignment(sensorAlignmentConfig);

    /*	 block moved to compass.cpp

     // FIXME extract to a method to reduce dependencies, maybe move to sensors_compass.c
     if (sensors(SENSOR_MAG)) {
     // calculate magnetic declination
     deg = magDeclinationFromConfig / 100;
     min = magDeclinationFromConfig % 100;

     magneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
     } else {
     magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
     }

     */
    return true;
}
bool sensorsAutodetectbaro(uint8_t baroHardwareToUse)
{
    return detectBaro((baroSensor_e) baroHardwareToUse);
}

