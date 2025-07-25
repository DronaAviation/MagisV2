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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/flash_m25p16.h"
#include "drivers/flash.h"
#include "drivers/ranging_vl53l0x.h"
#include "drivers/opticflow_paw3903.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/filter.h"
#include "flight/acrobats.h"
#include "flight/posEstimate.h"
#include "flight/posControl.h"
#include "flight/opticflow.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "mw.h"

#include "Utils.h"
#include "API-Utils.h"

uint8_t resetCounter       = 0;
uint8_t intLogCounter      = 0;
uint8_t floatLogCounter    = 0;
int16_t appHeading         = 0;
int16_t AUX3_VALUE         = 1500;
int16_t userHeading        = 0;
uint32_t userLoopFrequency = 1000000;
uint32_t autoRcTimerLoop   = 0;
int32_t user_GPS_coord [ 2 ];
int32_t MOTOR_ARRAY [ 4 ] = { 0 };
int32_t app_GPS_coord [ 2 ];
int32_t RC_ARRAY [ 4 ] = { 0 };

bool runUserCode            = false;
bool OledEnable             = false;
bool useAutoRC              = false;
bool developerMode          = false;
bool External_RC_FLAG [ 4 ] = { true, true, true, true };
bool userRCflag [ 4 ]       = { false, false, false, false };
bool callibrateAccelero     = true;
bool hasTakeOff             = true;
bool AutoAccCalibration     = false;
bool callOnPilotStart       = true;
bool callonPilotFinish      = false;
bool fsLowBattery           = true;
bool fsInFlightLowBattery   = true;
bool fsCrash                = true;
bool isUserHeadingSet       = false;
bool isUserGPSCoordSet      = false;
bool startShieldRanging     = false;
bool reverseReferenceFrame  = false;
bool motorMixer             = true;
bool isLocalisationOn       = false;
bool DONT_USE_STATUS_LED    = false;

bool isPwmInit [ 11 ]          = { false };
bool isUserFlightModeSet [ 6 ] = { false };
bool isXLaserInit [ 5 ]        = { false };

int32_t userDesiredAngle [ 3 ]  = { 0 };
int32_t userDesiredRate [ 3 ]   = { 0 };
int32_t userMotorPwm [ 4 ]      = { 0 };
int32_t userSetVelocity         = 0;
int16_t userHeadFreeHoldHeading = 0;
bool isUserDesiredAngle [ 3 ]   = { false };
bool isUserDesiredRate [ 3 ]    = { false };
bool isUserMotorPwm [ 4 ]       = { false };
bool isUserSetVelocity          = false;
bool isUserHeadFreeHoldSet      = false;



void resetUserRCflag ( void ) {
  if ( ( int32_t ) ( micros ( ) - autoRcTimerLoop ) >= 0 ) {

    for ( int i = 0; i < 4; i++ )
      userRCflag [ i ] = 0;

    autoRcTimerLoop = micros ( ) + 100000;
  }
}

void resetUser ( void ) {
  rcData [ AUX2 ] = 1200;
}



// Extracting GPIO port from unibus pin number
int getGPIOport ( unibus_e pin ) {
  int x = 99;

  switch ( pin ) {
    case Pin2:
    case Pin3:
    case Pin8:
    case Pin10:
    case Pin13:
    case Pin18:
    case Pin19:
      x = 1;    // GPIOA
      break;

    case Pin4:
    case Pin5:
    case Pin6:
    case Pin7:
    case Pin9:
    case Pin12:
    case Pin14:
    case Pin15:
    case Pin16:
    case Pin17:
      x = 2;    // GPIOB
      break;

    case Pin1:
    case Pin11:
    case Pin20:
      break;
  }

  return x;
}

GPIO_Pin getGPIOpin ( unibus_e pin ) {
  GPIO_Pin x;

  switch ( pin ) {
    case Pin12:
      x = Pin_0;
      break;

    case Pin19:
      x = Pin_2;
      break;

    case Pin9:
    case Pin18:
      x = Pin_3;
      break;

    case Pin13:
      x = Pin_4;
      break;

    case Pin8:
      x = Pin_5;
      break;

    case Pin7:
    case Pin10:
      x = Pin_8;
      break;

    case Pin6:
      x = Pin_9;
      break;

    case Pin4:
      x = Pin_10;
      break;

    case Pin5:
      x = Pin_11;
      break;

    case Pin14:
      x = Pin_12;
      break;

    case Pin2:
    case Pin15:
      x = Pin_13;
      break;

    case Pin3:
    case Pin16:
      x = Pin_14;
      break;

    case Pin17:
      x = Pin_15;
      break;

    case Pin1:
    case Pin11:
    case Pin20:
      break;
  }

  return x;
}

uint32_t getGPIOclock ( unibus_e pin ) {
  uint32_t x = 99;
  switch ( pin ) {
    case Pin2:
    case Pin3:
    case Pin8:
    case Pin10:
    case Pin13:
    case Pin18:
    case Pin19:
      x = RCC_AHBPeriph_GPIOA;
      break;

    case Pin4:
    case Pin5:
    case Pin6:
    case Pin7:
    case Pin9:
    case Pin12:
    case Pin14:
    case Pin15:
    case Pin16:
    case Pin17:
      x = RCC_AHBPeriph_GPIOB;
      break;

    case Pin1:
    case Pin11:
    case Pin20:
      break;
  }
  return x;
}

uint8_t getGPIOpinSource ( unibus_e pin ) {
  uint8_t x = 99;
  switch ( pin ) {
    case Pin12:
      x = GPIO_PinSource0;
      break;

    case Pin19:
      x = GPIO_PinSource2;
      break;

    case Pin9:
    case Pin18:
      x = GPIO_PinSource3;
      break;

    case Pin13:
      x = GPIO_PinSource4;
      break;

    case Pin8:
      x = GPIO_PinSource5;
      break;

    case Pin7:
    case Pin10:
      x = GPIO_PinSource8;
      break;

    case Pin6:
      x = GPIO_PinSource9;
      break;

    case Pin4:
      x = GPIO_PinSource10;
      break;

    case Pin5:
      x = GPIO_PinSource11;
      break;

    case Pin14:
      x = GPIO_PinSource12;
      break;

    case Pin2:
    case Pin15:
      x = GPIO_PinSource13;
      break;

    case Pin3:
    case Pin16:
      x = GPIO_PinSource14;
      break;

    case Pin17:
      x = GPIO_PinSource15;
      break;

    case Pin1:
    case Pin11:
    case Pin20:
      break;
  }

  return x;
}

uint16_t getTimerCh ( unibus_e pin ) {
  uint16_t temp = 99;

  switch ( pin ) {
    case Pin8:
    case Pin10:
      temp = TIM_Channel_1;
      break;

    case Pin3:
    case Pin9:
    case Pin13:
      temp = TIM_Channel_2;
      break;

    case Pin2:
    case Pin4:
    case Pin12:
    case Pin19:
      temp = TIM_Channel_3;
      break;

    case Pin5:
    case Pin18:
      temp = TIM_Channel_4;
      break;

    case Pin1:
    case Pin6:
    case Pin7:
    case Pin11:
    case Pin14:
    case Pin15:
    case Pin16:
    case Pin17:
    case Pin20:
      break;
  }

  return temp;
}

uint8_t getADCCh ( unibus_e pin ) {
  uint8_t temp = 99;

  switch ( pin ) {
    case Pin13:
      temp = ADC_Channel_1;
      break;

    case Pin8:
      temp = ADC_Channel_2;
      break;

    case Pin14:
    case Pin19:
      temp = ADC_Channel_3;
      break;

    case Pin16:
    case Pin18:
      temp = ADC_Channel_4;
      break;

    case Pin15:
    case Pin17:
      temp = ADC_Channel_5;
      break;

    case Pin12:
      temp = ADC_Channel_12;
      break;

    case Pin1:
    case Pin2:
    case Pin3:
    case Pin4:
    case Pin5:
    case Pin6:
    case Pin7:
    case Pin9:
    case Pin10:
    case Pin11:
    case Pin20:
      // shouldn't be allowed by IDE
      break;
  }

  return temp;
}

// void _Adc1init ( void ) {

//   if ( _isAdcEnable [ ADC1_IN4 ] || _isAdcEnable [ ADC1_IN3 ] ) {

//     ADC_InitTypeDef ADC_InitStructure;
//     DMA_InitTypeDef DMA_InitStructure;
//     GPIO_InitTypeDef GPIO_InitStructure;

//     uint8_t adcChannelCount = 0;

//     GPIO_StructInit ( &GPIO_InitStructure );
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

//     if ( _isAdcEnable [ ADC1_IN4 ] ) {

//       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//       GPIO_Init ( GPIOA, &GPIO_InitStructure );

//       _adcDmaIndex [ ADC1_IN4 ] = adcChannelCount;
//       adcChannelCount++;
//     }

//     if ( _isAdcEnable [ ADC1_IN3 ] ) {

//       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//       GPIO_Init ( GPIOA, &GPIO_InitStructure );

//       _adcDmaIndex [ ADC1_IN3 ] = adcChannelCount;
//       adcChannelCount++;
//     }

//     RCC_ADCCLKConfig ( RCC_ADC34PLLCLK_Div256 );    // 72 MHz divided by 256 = 281.25 kHz
//     RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_ADC12, ENABLE );

//     DMA_DeInit ( DMA1_Channel1 );

//     DMA_StructInit ( &DMA_InitStructure );
//     DMA_InitStructure.DMA_PeripheralBaseAddr = ( uint32_t ) &ADC1->DR;
//     DMA_InitStructure.DMA_MemoryBaseAddr     = ( uint32_t ) _adc1Values;
//     DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
//     DMA_InitStructure.DMA_BufferSize         = adcChannelCount;
//     DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
//     DMA_InitStructure.DMA_MemoryInc          = adcChannelCount > 1 ?
//                                                          DMA_MemoryInc_Enable :
//                                                          DMA_MemoryInc_Disable;
//     DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//     DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
//     DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
//     DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
//     DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;

//     DMA_Init ( DMA1_Channel1, &DMA_InitStructure );

//     DMA_Cmd ( DMA1_Channel1, ENABLE );

//     ADC_CommonInitTypeDef ADC_CommonInitStructure;

//     ADC_CommonStructInit ( &ADC_CommonInitStructure );
//     ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
//     ADC_CommonInitStructure.ADC_Clock            = ADC_Clock_SynClkModeDiv4;
//     ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_1;
//     ADC_CommonInitStructure.ADC_DMAMode          = ADC_DMAMode_Circular;
//     ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
//     ADC_CommonInit ( ADC1, &ADC_CommonInitStructure );

//     ADC_StructInit ( &ADC_InitStructure );

//     ADC_InitStructure.ADC_ContinuousConvMode    = ADC_ContinuousConvMode_Enable;
//     ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
//     ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
//     ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
//     ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
//     ADC_InitStructure.ADC_OverrunMode           = ADC_OverrunMode_Disable;
//     ADC_InitStructure.ADC_AutoInjMode           = ADC_AutoInjec_Disable;
//     ADC_InitStructure.ADC_NbrOfRegChannel       = adcChannelCount;

//     ADC_Init ( ADC1, &ADC_InitStructure );

//     uint8_t rank = 1;

//     if ( _isAdcEnable [ ADC1_IN3 ] )
//       ADC_RegularChannelConfig ( ADC1, ADC_Channel_3, rank++, ADC_SampleTime_601Cycles5 );

//     if ( _isAdcEnable [ ADC1_IN4 ] )
//       ADC_RegularChannelConfig ( ADC1, ADC_Channel_4, rank++, ADC_SampleTime_601Cycles5 );

//     ADC_Cmd ( ADC1, ENABLE );

//     while ( ! ADC_GetFlagStatus ( ADC1, ADC_FLAG_RDY ) );

//     ADC_DMAConfig ( ADC1, ADC_DMAMode_Circular );

//     ADC_DMACmd ( ADC1, ENABLE );

//     ADC_StartConversion ( ADC1 );

//     //  LED.set(BLUE,ON);
//   }
// }

// void _Adc2init ( void ) {

//   if ( _isAdcEnable [ ADC2_IN12 ] || _isAdcEnable [ ADC2_IN1 ] || _isAdcEnable [ ADC2_IN2 ] || _isAdcEnable [ ADC2_IN4 ] ) {

//     ADC_InitTypeDef ADC_InitStructure;
//     DMA_InitTypeDef DMA_InitStructure;
//     GPIO_InitTypeDef GPIO_InitStructure;

//     uint8_t adcChannelCount = 0;

//     GPIO_StructInit ( &GPIO_InitStructure );
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

//     if ( _isAdcEnable [ ADC2_IN12 ] ) {

//       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//       GPIO_Init ( GPIOB, &GPIO_InitStructure );

//       _adcDmaIndex [ ADC2_IN1 ] = adcChannelCount;
//       adcChannelCount++;
//     }

//     if ( _isAdcEnable [ ADC2_IN1 ] ) {

//       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//       GPIO_Init ( GPIOA, &GPIO_InitStructure );

//       _adcDmaIndex [ ADC2_IN1 ] = adcChannelCount;
//       adcChannelCount++;
//     }

//     if ( _isAdcEnable [ ADC2_IN2 ] ) {

//       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//       GPIO_Init ( GPIOA, &GPIO_InitStructure );

//       _adcDmaIndex [ ADC2_IN2 ] = adcChannelCount;
//       adcChannelCount++;
//     }

//     if ( _isAdcEnable [ ADC2_IN4 ] ) {

//       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//       GPIO_Init ( GPIOA, &GPIO_InitStructure );

//       _adcDmaIndex [ ADC2_IN2 ] = adcChannelCount;
//       adcChannelCount++;
//     }

//     RCC_ADCCLKConfig ( RCC_ADC34PLLCLK_Div256 );    // 72 MHz divided by 256 = 281.25 kHz
//     RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_DMA2 | RCC_AHBPeriph_ADC12, ENABLE );

//     DMA_DeInit ( DMA2_Channel1 );

//     DMA_StructInit ( &DMA_InitStructure );
//     DMA_InitStructure.DMA_PeripheralBaseAddr = ( uint32_t ) &ADC2->DR;
//     DMA_InitStructure.DMA_MemoryBaseAddr     = ( uint32_t ) _adc2Values;
//     DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
//     DMA_InitStructure.DMA_BufferSize         = adcChannelCount;
//     DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
//     DMA_InitStructure.DMA_MemoryInc          = adcChannelCount > 1 ?
//                                                          DMA_MemoryInc_Enable :
//                                                          DMA_MemoryInc_Disable;
//     DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//     DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
//     DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
//     DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
//     DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;

//     DMA_Init ( DMA2_Channel1, &DMA_InitStructure );

//     DMA_Cmd ( DMA2_Channel1, ENABLE );

//     ADC_CommonInitTypeDef ADC_CommonInitStructure;

//     ADC_CommonStructInit ( &ADC_CommonInitStructure );
//     ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
//     ADC_CommonInitStructure.ADC_Clock            = ADC_Clock_SynClkModeDiv4;
//     ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_1;
//     ADC_CommonInitStructure.ADC_DMAMode          = ADC_DMAMode_Circular;
//     ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
//     ADC_CommonInit ( ADC2, &ADC_CommonInitStructure );

//     ADC_StructInit ( &ADC_InitStructure );

//     ADC_InitStructure.ADC_ContinuousConvMode    = ADC_ContinuousConvMode_Enable;
//     ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
//     ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
//     ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
//     ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
//     ADC_InitStructure.ADC_OverrunMode           = ADC_OverrunMode_Disable;
//     ADC_InitStructure.ADC_AutoInjMode           = ADC_AutoInjec_Disable;
//     ADC_InitStructure.ADC_NbrOfRegChannel       = adcChannelCount;

//     ADC_Init ( ADC2, &ADC_InitStructure );

//     uint8_t rank = 1;

//     if ( _isAdcEnable [ ADC2_IN12 ] )
//       ADC_RegularChannelConfig ( ADC2, ADC_Channel_12, rank++, ADC_SampleTime_601Cycles5 );

//     if ( _isAdcEnable [ ADC2_IN1 ] )
//       ADC_RegularChannelConfig ( ADC2, ADC_Channel_1, rank++, ADC_SampleTime_601Cycles5 );

//     if ( _isAdcEnable [ ADC2_IN2 ] )
//       ADC_RegularChannelConfig ( ADC2, ADC_Channel_2, rank++, ADC_SampleTime_601Cycles5 );

//     if ( _isAdcEnable [ ADC2_IN4 ] )
//       ADC_RegularChannelConfig ( ADC2, ADC_Channel_4, rank++, ADC_SampleTime_601Cycles5 );

//     ADC_Cmd ( ADC2, ENABLE );

//     while ( ! ADC_GetFlagStatus ( ADC2, ADC_FLAG_RDY ) );

//     ADC_DMAConfig ( ADC2, ADC_DMAMode_Circular );

//     ADC_DMACmd ( ADC2, ENABLE );

//     ADC_StartConversion ( ADC2 );
//     LED.set ( BLUE, ON );
//   }
// }

// void _Adc3init ( void ) {

//   if ( _isAdcEnable [ ADC3_IN5 ] || _isAdcEnable [ ADC3_IN2 ] ) {

//     ADC_InitTypeDef ADC_InitStructure;
//     DMA_InitTypeDef DMA_InitStructure;
//     GPIO_InitTypeDef GPIO_InitStructure;

//     uint8_t adcChannelCount = 0;

//     GPIO_StructInit ( &GPIO_InitStructure );
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

//     if ( _isAdcEnable [ ADC3_IN5 ] ) {

//       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
//       GPIO_Init ( GPIOB, &GPIO_InitStructure );

//       _adcDmaIndex [ ADC3_IN5 ] = adcChannelCount;
//       adcChannelCount++;
//     }
//     if ( _isAdcEnable [ ADC3_IN2 ] ) {

//       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//       GPIO_Init ( GPIOB, &GPIO_InitStructure );

//       _adcDmaIndex [ ADC3_IN2 ] = adcChannelCount;
//       adcChannelCount++;
//     }

//     RCC_ADCCLKConfig ( RCC_ADC34PLLCLK_Div256 );    // 72 MHz divided by 256 = 281.25 kHz
//     RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_DMA2 | RCC_AHBPeriph_ADC34, ENABLE );

//     DMA_DeInit ( DMA2_Channel5 );

//     DMA_StructInit ( &DMA_InitStructure );
//     DMA_InitStructure.DMA_PeripheralBaseAddr = ( uint32_t ) &ADC3->DR;
//     DMA_InitStructure.DMA_MemoryBaseAddr     = ( uint32_t ) _adc3Values;
//     DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
//     DMA_InitStructure.DMA_BufferSize         = adcChannelCount;
//     DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
//     DMA_InitStructure.DMA_MemoryInc          = adcChannelCount > 1 ?
//                                                          DMA_MemoryInc_Enable :
//                                                          DMA_MemoryInc_Disable;
//     DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//     DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
//     DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
//     DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
//     DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;

//     DMA_Init ( DMA2_Channel5, &DMA_InitStructure );

//     DMA_Cmd ( DMA2_Channel5, ENABLE );

//     ADC_CommonInitTypeDef ADC_CommonInitStructure;

//     ADC_CommonStructInit ( &ADC_CommonInitStructure );
//     ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
//     ADC_CommonInitStructure.ADC_Clock            = ADC_Clock_SynClkModeDiv4;
//     ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_1;
//     ADC_CommonInitStructure.ADC_DMAMode          = ADC_DMAMode_Circular;
//     ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
//     ADC_CommonInit ( ADC3, &ADC_CommonInitStructure );

//     ADC_StructInit ( &ADC_InitStructure );

//     ADC_InitStructure.ADC_ContinuousConvMode    = ADC_ContinuousConvMode_Enable;
//     ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
//     ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
//     ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
//     ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
//     ADC_InitStructure.ADC_OverrunMode           = ADC_OverrunMode_Disable;
//     ADC_InitStructure.ADC_AutoInjMode           = ADC_AutoInjec_Disable;
//     ADC_InitStructure.ADC_NbrOfRegChannel       = adcChannelCount;

//     ADC_Init ( ADC3, &ADC_InitStructure );

//     uint8_t rank = 1;

//     if ( _isAdcEnable [ ADC3_IN5 ] )
//       ADC_RegularChannelConfig ( ADC3, ADC_Channel_5, rank++, ADC_SampleTime_601Cycles5 );

//     if ( _isAdcEnable [ ADC3_IN2 ] )
//       ADC_RegularChannelConfig ( ADC3, ADC_Channel_2, rank++, ADC_SampleTime_601Cycles5 );

//     ADC_Cmd ( ADC3, ENABLE );

//     while ( ! ADC_GetFlagStatus ( ADC3, ADC_FLAG_RDY ) );

//     ADC_DMAConfig ( ADC3, ADC_DMAMode_Circular );

//     ADC_DMACmd ( ADC3, ENABLE );

//     ADC_StartConversion ( ADC3 );
//   }
// }

// void _Adc4init ( void ) {

//   if ( _isAdcEnable [ ADC4_IN5 ] || _isAdcEnable [ ADC4_IN4 ] || _isAdcEnable [ ADC4_IN3 ] ) {

//     ADC_InitTypeDef ADC_InitStructure;
//     DMA_InitTypeDef DMA_InitStructure;
//     GPIO_InitTypeDef GPIO_InitStructure;

//     uint8_t adcChannelCount = 0;

//     GPIO_StructInit ( &GPIO_InitStructure );
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

//     if ( _isAdcEnable [ ADC4_IN5 ] ) {

//       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
//       GPIO_Init ( GPIOB, &GPIO_InitStructure );

//       _adcDmaIndex [ ADC4_IN5 ] = adcChannelCount;
//       adcChannelCount++;
//     }

//     if ( _isAdcEnable [ ADC4_IN4 ] ) {

//       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
//       GPIO_Init ( GPIOB, &GPIO_InitStructure );

//       _adcDmaIndex [ ADC4_IN4 ] = adcChannelCount;
//       adcChannelCount++;
//     }

//     if ( _isAdcEnable [ ADC4_IN3 ] ) {

//       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//       GPIO_Init ( GPIOB, &GPIO_InitStructure );

//       _adcDmaIndex [ ADC4_IN3 ] = adcChannelCount;
//       adcChannelCount++;
//     }

//     RCC_ADCCLKConfig ( RCC_ADC34PLLCLK_Div256 );    // 72 MHz divided by 256 = 281.25 kHz
//     RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_DMA2 | RCC_AHBPeriph_ADC34, ENABLE );

//     DMA_DeInit ( DMA2_Channel2 );

//     DMA_StructInit ( &DMA_InitStructure );
//     DMA_InitStructure.DMA_PeripheralBaseAddr = ( uint32_t ) &ADC4->DR;
//     DMA_InitStructure.DMA_MemoryBaseAddr     = ( uint32_t ) _adc4Values;
//     DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
//     DMA_InitStructure.DMA_BufferSize         = adcChannelCount;
//     DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
//     DMA_InitStructure.DMA_MemoryInc          = adcChannelCount > 1 ?
//                                                          DMA_MemoryInc_Enable :
//                                                          DMA_MemoryInc_Disable;
//     ;
//     DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//     DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
//     DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
//     DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
//     DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;

//     DMA_Init ( DMA2_Channel2, &DMA_InitStructure );

//     DMA_Cmd ( DMA2_Channel2, ENABLE );

//     ADC_CommonInitTypeDef ADC_CommonInitStructure;

//     ADC_CommonStructInit ( &ADC_CommonInitStructure );
//     ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
//     ADC_CommonInitStructure.ADC_Clock            = ADC_Clock_SynClkModeDiv4;
//     ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_1;
//     ADC_CommonInitStructure.ADC_DMAMode          = ADC_DMAMode_Circular;
//     ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
//     ADC_CommonInit ( ADC4, &ADC_CommonInitStructure );

//     ADC_StructInit ( &ADC_InitStructure );

//     ADC_InitStructure.ADC_ContinuousConvMode    = ADC_ContinuousConvMode_Enable;
//     ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
//     ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
//     ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
//     ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
//     ADC_InitStructure.ADC_OverrunMode           = ADC_OverrunMode_Disable;
//     ADC_InitStructure.ADC_AutoInjMode           = ADC_AutoInjec_Disable;
//     ADC_InitStructure.ADC_NbrOfRegChannel       = adcChannelCount;

//     ADC_Init ( ADC4, &ADC_InitStructure );

//     uint8_t rank = 1;

//     if ( _isAdcEnable [ ADC4_IN5 ] )
//       ADC_RegularChannelConfig ( ADC4, ADC_Channel_5, rank++, ADC_SampleTime_601Cycles5 );

//     if ( _isAdcEnable [ ADC4_IN4 ] )
//       ADC_RegularChannelConfig ( ADC4, ADC_Channel_4, rank++, ADC_SampleTime_601Cycles5 );

//     if ( _isAdcEnable [ ADC4_IN3 ] )
//       ADC_RegularChannelConfig ( ADC4, ADC_Channel_3, rank++, ADC_SampleTime_601Cycles5 );

//     ADC_Cmd ( ADC4, ENABLE );

//     while ( ! ADC_GetFlagStatus ( ADC4, ADC_FLAG_RDY ) );

//     ADC_DMAConfig ( ADC4, ADC_DMAMode_Circular );

//     ADC_DMACmd ( ADC4, ENABLE );

//     ADC_StartConversion ( ADC4 );
//   }
// }

// void _AdcInit ( void ) {

//   _Adc1init ( );
//   _Adc2init ( );
//   _Adc3init ( );
//   _Adc4init ( );
// }

// void Peripheral_M::init ( peripheral_adc_pin _adc_pin ) {
//   // Define a mapping from peripheral_adc_pin values to isADCEnable indices
//   static const int adcMapping [] = {
//     [ADC_1]  = ADC2_IN12,
//     [ADC_2]  = ADC4_IN5,
//     [ADC_3]  = ADC4_IN4,
//     [ADC_4]  = ADC3_IN5,
//     [ADC_5]  = ADC4_IN3,
//     [ADC_6]  = ADC2_IN1,
//     [ADC_7]  = ADC2_IN2,
//     [ADC_8]  = ADC1_IN4,
//     [ADC_9]  = ADC1_IN3,
//     [ADC_10] = ADC2_IN4,
//     [ADC_11] = ADC3_IN2
//   };

//   // Check if _adc_pin is within valid range
//   // if ( _adc_pin >= ADC_1 && _adc_pin <= ADC_11 ) {
//   _isAdcEnable [ adcMapping [ _adc_pin ] ] = true;
//   // }

//   // Log, assert, or halt
//   // static_assert ( ! ADC_7, "Error: ADC_7 cannot be enabled due to conflict with ADC1." );
// }

// uint16_t Peripheral_M::read ( peripheral_adc_pin _adc_pin ) {
//   switch ( _adc_pin ) {
//     case ADC_1:
//       return _adc2Values [ _adcDmaIndex [ ADC2_IN12 ] ];
//     case ADC_2:
//       return _adc4Values [ _adcDmaIndex [ ADC4_IN5 ] ];
//     case ADC_3:
//       return _adc4Values [ _adcDmaIndex [ ADC4_IN4 ] ];
//     case ADC_4:
//       return _adc3Values [ _adcDmaIndex [ ADC3_IN5 ] ];
//     case ADC_5:
//       return _adc4Values [ _adcDmaIndex [ ADC4_IN3 ] ];
//     case ADC_6:
//       return _adc2Values [ _adcDmaIndex [ ADC2_IN1 ] ];
//     case ADC_7:
//       return _adc2Values [ _adcDmaIndex [ ADC2_IN2 ] ];
//     case ADC_8:
//       return _adc1Values [ _adcDmaIndex [ ADC1_IN4 ] ];
//     case ADC_9:
//       return _adc1Values [ _adcDmaIndex [ ADC1_IN3 ] ];
//     case ADC_10:
//       return _adc2Values [ _adcDmaIndex [ ADC2_IN4 ] ];
//     case ADC_11:
//       return _adc3Values [ _adcDmaIndex [ ADC3_IN2 ] ];
//     default:
//       return 0;
//   }
// }