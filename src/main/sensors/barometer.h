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

typedef enum {
    BARO_DEFAULT = 0,
    BARO_NONE = 1,
    BARO_BMP085 = 2,
    BARO_MS5611 = 3,
    BARO_BMP280 = 4,
    BARO_ICP10111=5
} baroSensor_e;

#define BARO_SAMPLE_COUNT_MAX   48
#define BARO_MAX BARO_MS5611

typedef struct barometerConfig_s {
        uint8_t baro_sample_count;              // size of baro filter array
        float baro_noise_lpf;                   // additional LPF to reduce baro noise
        float baro_cf_vel; // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
        float baro_cf_alt;                      // apply CF to use ACC for height estimation
} barometerConfig_t;

//int32_t baroTemperature;             // Use temperature for telemetry
//int32_t baroPressure;

extern float baroGroundPressure;
extern float baroGroundTemperature;
extern float BaroAlt;

#ifdef BARO
void useBarometerConfig(barometerConfig_t *barometerConfigToUse);
bool isBaroCalibrationComplete(void);
void baroSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void baroUpdate(uint32_t currentTime);
void baroInit(void);
bool isBaroReady(void);
float baroCalculateAltitude(void);
float getBaroPressure(void);
float getBaroTemperature(void);
uint32_t getBaroLastUpdate(void);
void performBaroCalibrationCycle(void);
void baroCalibrate(void);
// Add this declaration with other barometer function declarations
void baroResetGroundLevel(void);


#endif

#ifdef __cplusplus
}
#endif
