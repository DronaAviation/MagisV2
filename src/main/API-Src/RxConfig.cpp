/*******************************************************************************
 #  Copyright (c) 2025 DRONA AVIATION                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2-MechAsh-Dev                                               #
 #  File: \RxConfig.cpp                                                        #
 #  Created Date: Tue, 26th Jan 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 29th Apr 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 *******************************************************************************/
#include "platform.h"

#include "API/RxConfig.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/serial.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/serial.h"
#include "io/rc_controls.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

// #include "PlutoPilot.h"

// Global variables to store device mode configuration
uint8_t DevModeAUX       = 0;    // Auxiliary channel for device mode
uint16_t DevModeMinRange = 0;    // Minimum range for device mode
uint16_t DevModeMaxRange = 0;    // Maximum range for device mode

bool AuxChangeEnable = false;     // Indicates if auxiliary channel changes are allowed
bool ESP_WiFi_Status = true;      // current selested ESP Wi-Fi connection status
rx_mode_e RxMode     = Rx_ESP;    // Current selected receiver mode

/**
 * Configures auxiliary channels for a specific flight mode.
 *
 * @param flightMode The flight mode to configure (e.g., Mode_ARM, Mode_ANGLE).
 * @param rxChannel The receiver channel associated with the mode.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Receiver_Aux_Config ( flight_mode flightMode, rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  switch ( flightMode ) {
    case Mode_ARM:
      currentProfile->modeActivationConditions [ 0 ] = { ( boxId_e ) BOXARM, rxChannel - NON_AUX_CHANNEL_COUNT, CHANNEL_VALUE_TO_STEP ( minRange ), CHANNEL_VALUE_TO_STEP ( maxRange ) };
      break;
    case Mode_ANGLE:
      currentProfile->modeActivationConditions [ 1 ] = { ( boxId_e ) BOXANGLE, rxChannel - NON_AUX_CHANNEL_COUNT, CHANNEL_VALUE_TO_STEP ( minRange ), CHANNEL_VALUE_TO_STEP ( maxRange ) };
      break;
    case Mode_BARO:
      currentProfile->modeActivationConditions [ 2 ] = { ( boxId_e ) BOXBARO, rxChannel - NON_AUX_CHANNEL_COUNT, CHANNEL_VALUE_TO_STEP ( minRange ), CHANNEL_VALUE_TO_STEP ( maxRange ) };
      break;
    case Mode_MAG:
      currentProfile->modeActivationConditions [ 3 ] = { ( boxId_e ) BOXMAG, rxChannel - NON_AUX_CHANNEL_COUNT, CHANNEL_VALUE_TO_STEP ( minRange ), CHANNEL_VALUE_TO_STEP ( maxRange ) };
      break;
    case Mode_HEADFREE:
      currentProfile->modeActivationConditions [ 4 ] = { ( boxId_e ) BOXHEADFREE, rxChannel - NON_AUX_CHANNEL_COUNT, CHANNEL_VALUE_TO_STEP ( minRange ), CHANNEL_VALUE_TO_STEP ( maxRange ) };
      break;
    default:
      break;
  }
}

/**
 * @brief Configures the receiver mode and sets up auxiliary channels accordingly.
 *
 * Sets the feature flags, auxiliary channel configuration, and developer mode settings
 * depending on the selected receiver mode.
 *
 * @param[in] rxMode The receiver mode to configure (e.g., Rx_ESP, Rx_PPM, Rx_CAM).
 */
void Receiver_Mode ( rx_mode_e rxMode ) {
  // Clear all conflicting features before setting new ones
  featureClear ( FEATURE_RX_SERIAL );
  featureClear ( FEATURE_RX_PARALLEL_PWM );
  featureClear ( FEATURE_RX_PPM );
  featureClear ( FEATURE_RX_MSP );

  switch ( rxMode ) {
    case Rx_ESP:
    case Rx_CAM: {
      AuxChangeEnable = false;
      featureSet ( FEATURE_RX_MSP );

      Receiver_Aux_Config ( Mode_ARM, Rx_AUX4, 1300, 2100 );
      Receiver_Aux_Config ( Mode_ANGLE, Rx_AUX4, 900, 2100 );
      Receiver_Aux_Config ( Mode_BARO, Rx_AUX3, 1300, 2100 );
      Receiver_Aux_Config ( Mode_MAG, Rx_AUX1, 900, 1300 );
      Receiver_Aux_Config ( Mode_HEADFREE, Rx_AUX1, 1300, 1700 );

      DevModeAUX      = Rx_AUX2;
      DevModeMinRange = 1450;
      DevModeMaxRange = 1550;

      ESP_WiFi_Status = ( rxMode == Rx_ESP ); /* ? true : false; */ /* ( rxMode == Rx_ESP ) ;*/    // True only if Rx_ESP
      break;
    }

    case Rx_PPM: {
      AuxChangeEnable = true;
      featureSet ( FEATURE_RX_PPM );

      Receiver_Aux_Config ( Mode_ARM, Rx_AUX2, 1300, 2100 );
      Receiver_Aux_Config ( Mode_ANGLE, Rx_AUX2, 900, 2100 );
      Receiver_Aux_Config ( Mode_BARO, Rx_AUX3, 1300, 2100 );
      Receiver_Aux_Config ( Mode_MAG, Rx_AUX1, 900, 1300 );
      Receiver_Aux_Config ( Mode_HEADFREE, Rx_AUX1, 1300, 1700 );

      Receiver_Config_Mode_Dev ( Rx_AUX4, 1500, 2100 );

      ESP_WiFi_Status = true;
      break;
    }

    default:
      // Unknown receiver mode; no action taken
      break;
  }
}
/**
 * Configures ARM mode if auxiliary change is enabled.
 *
 * @param rxChannel The receiver channel to configure.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Receiver_Config_Arm ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  if ( AuxChangeEnable ) {
    Receiver_Aux_Config ( Mode_ARM, rxChannel, minRange, maxRange );
  }
}

/**
 * Configures ANGLE mode if auxiliary change is enabled.
 *
 * @param rxChannel The receiver channel to configure.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Receiver_Config_Mode_Angle ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  if ( AuxChangeEnable ) {
    Receiver_Aux_Config ( Mode_ANGLE, rxChannel, minRange, maxRange );
  }
}

/**
 * Configures BARO mode if auxiliary change is enabled.
 *
 * @param rxChannel The receiver channel to configure.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Receiver_Config_Mode_Baro ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  if ( AuxChangeEnable ) {
    Receiver_Aux_Config ( Mode_BARO, rxChannel, minRange, maxRange );
  }
}

/**
 * Configures MAG mode if auxiliary change is enabled.
 *
 * @param rxChannel The receiver channel to configure.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Receiver_Config_Mode_Mag ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  if ( AuxChangeEnable ) {
    Receiver_Aux_Config ( Mode_MAG, rxChannel, minRange, maxRange );
  }
}

/**
 * Configures HEADFREE mode if auxiliary change is enabled.
 *
 * @param rxChannel The receiver channel to configure.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Receiver_Config_Mode_HeadFree ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  if ( AuxChangeEnable ) {
    Receiver_Aux_Config ( Mode_HEADFREE, rxChannel, minRange, maxRange );
  }
}

/**
 * @brief Configures the device mode with specified parameters.
 *
 * This function sets the auxiliary channel, minimum range, and maximum range
 * for the device mode configuration. The configuration is applied only if
 * AuxChangeEnable is true.
 *
 * @param rxChannel The auxiliary channel to be used for device mode.
 * @param minRange The minimum range value for the device mode.
 * @param maxRange The maximum range value for the device mode.
 */
void Receiver_Config_Mode_Dev ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  // Check if change in auxiliary settings is enabled
  if ( AuxChangeEnable ) {
    DevModeAUX      = rxChannel;    // Set the auxiliary channel
    DevModeMinRange = minRange;     // Set the minimum range
    DevModeMaxRange = maxRange;     // Set the maximum range
  }
}

// Rc_Rx_Config_P Receiver;

/**
 * @brief Turns on the ESP WiFi by setting GPIOB Pin_3 to low.
 */
void ESP_WiFi_ON ( void ) {
  digitalLo ( GPIOB, Pin_3 );    // Set GPIOB Pin_3 to low to turn on WiFi
}

/**
 * @brief Turns off the ESP WiFi by setting GPIOB Pin_3 to high.
 */
void ESP_WiFi_OFF ( void ) {
  digitalHi ( GPIOB, Pin_3 );    // Set GPIOB Pin_3 to high to turn off WiFi
}

/**
 * @brief Initializes GPIOB Pin_3 for ESP IO14 with specific configurations.
 *
 * This function configures the GPIO pin PB3 as an output with push-pull mode
 * and a speed of 2MHz. It also enables the clock for GPIOB and sets the initial
 * state of the WiFi based on the current status.
 */
void STM_PB3_ESP_IO14_Init ( void ) {
  GPIO_TypeDef *gpio;
  gpio_config_t cfg;
  gpio      = GPIOB;                                        // Select GPIO port B
  cfg.pin   = Pin_3;                                        // Configure pin 3
  cfg.mode  = Mode_Out_PP;                                  // Set mode to output push-pull
  cfg.speed = Speed_2MHz;                                   // Set speed to 2MHz
  RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );    // Enable clock for GPIOB
  gpioInit ( gpio, &cfg );                                  // Initialize GPIO with the specified configuration

  if ( ! ESP_WiFi_Status ) {    // Check if WiFi is off
    ESP_WiFi_OFF ( );           // Turn off WiFi
  } else {
    ESP_WiFi_ON ( );    // Turn on WiFi
  }
}
