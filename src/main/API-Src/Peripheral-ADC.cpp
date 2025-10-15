/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\Peripheral-ADC.cpp                                 #
 #  Created Date: Thu, 8th May 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 2nd Sep 2025                                           #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include "platform.h"
#include "build_config.h"

#include "drivers/adc.h"
#include "drivers/adc_impl.h"
#include "drivers/system.h"

#include "API/Peripherals.h"

// Define a structure named Peripheral_M to handle ADC peripherals.
// Peripheral_M Peripheral;

// Boolean array to keep track of which ADC channels are enabled.
// Initially, all channels are set to false (disabled).
bool _isAdcEnable [ ADC_CHANNEL_COUNT ] = { false };

// Array to store the DMA index for each ADC channel.
uint8_t _adcDmaIndex [ ADC_CHANNEL_COUNT ];

// Volatile arrays to store ADC values for each ADC channel group.
// These arrays hold the most recent conversion results for each ADC.
volatile uint16_t _adc1Values [ ADC1_CHANNEL_COUNT ];
volatile uint16_t _adc2Values [ ADC2_CHANNEL_COUNT ];
volatile uint16_t _adc3Values [ ADC3_CHANNEL_COUNT ];
volatile uint16_t _adc4Values [ ADC4_CHANNEL_COUNT ];

// Structure to map ADC pins to their respective value arrays and indices.
struct ADC_PinMapping {
  volatile uint16_t *valueArray;    // Pointer to the correct value array for the ADC.
  uint8_t adcIndex;                 // Index of the ADC channel.
};

// Static constant array that maps each ADC pin to its corresponding
// value array and index. This defines how ADC pins are associated with ADC arrays.
// static const ADC_PinMapping adcPinMap [] = {
static const ADC_PinMapping adcPinMap [] = {
  [ADC_1] = { _adc2Values, ADC2_IN12 },
  [ADC_2] = { _adc4Values, ADC4_IN5 },
  [ADC_3] = { _adc4Values, ADC4_IN4 },
  [ADC_4] = { _adc3Values, ADC3_IN5 },
  [ADC_5] = { _adc4Values, ADC4_IN3 },
  [ADC_6] = { _adc2Values, ADC2_IN1 },
  [ADC_7] = { _adc2Values, ADC2_IN2 },
  [ADC_8] = { _adc1Values, ADC1_IN4 },
  [ADC_9] = { _adc1Values, ADC1_IN3 },
};

void Peripheral_Init ( peripheral_adc_pin _adc_pin ) {
  if ( _adc_pin >= ADC_1 && _adc_pin <= ADC_9 ) {
    _isAdcEnable [ adcPinMap [ _adc_pin ].adcIndex ] = true;
  }
}

uint16_t Peripheral_Read ( peripheral_adc_pin _adc_pin ) {
  if ( _adc_pin >= ADC_1 && _adc_pin <= ADC_9 ) {
    const auto &map = adcPinMap [ _adc_pin ];
    return map.valueArray [ _adcDmaIndex [ map.adcIndex ] ];
  }
  return 0;
}

// Structure definition for ADC configuration
// Structure definition for ADC configuration (UPDATED)
typedef struct {
  ADC_TypeDef *adc;                              // Pointer to the ADC peripheral
  DMA_Channel_TypeDef *dmaChannel;               // Pointer to the DMA channel associated with ADC
  uint32_t dmaPeripheral;                        // DMA peripheral identifier
  uint32_t dmaClock;                             // Clock for the DMA peripheral
  uint32_t adcClock;                             // Clock for the ADC common block used by this ADC
  volatile uint16_t *valueBuffer;                // Buffer to store ADC conversion values (DMA target)
  const uint8_t *enabledChannels;                // Array indicating which channels are enabled (per logical index)
  const uint16_t *gpioPins;                      // Array of GPIO pins used for ADC (same order as enabledChannels)
  GPIO_TypeDef **gpioPorts;                      // Array of GPIO ports (same order as enabledChannels)
  const uint8_t *adcChannels;                    // Array of ADC channel numbers (ADC_Channel_x) in same order
  const Peripheral_ADC_Channel *channelEnums;    // NEW: global enum for each entry above
  uint8_t channelCount;                          // Number of entries in the arrays above
} ADC_Config;

/**
 * @brief Initializes the ADC and its associated peripherals based on the provided configuration.
 *
 * This function configures the GPIO pins, DMA channel, and ADC settings according to
 * the specified `ADC_Config` structure. It sets up the ADC for continuous conversion mode
 * with circular DMA operation, enabling the reading of multiple channels in a cycle.
 *
 * @param config Pointer to an `ADC_Config` structure containing the desired configuration
 *               for the ADC and its peripherals.
 *
 * @note The function will return immediately if no channels are configured (i.e.,
 *       `config->channelCount` is zero).
 */
void _AdcInitGeneric ( const ADC_Config *config ) {
  if ( config->channelCount == 0 ) return;    // Return if no channels are configured

  // Enable GPIO clocks (safe to call repeatedly)
  RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE );

  // Define initialization structures for various peripherals
  ADC_InitTypeDef adcInit;
  DMA_InitTypeDef dmaInit;
  GPIO_InitTypeDef gpioInit;
  ADC_CommonInitTypeDef adcCommon;

  // Initialize GPIO structure and set mode and pull-up/pull-down configuration
  GPIO_StructInit ( &gpioInit );
  gpioInit.GPIO_Mode = GPIO_Mode_AN;
  gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;

  // Configure GPIO for each enabled channel and map global enum -> DMA slot index
  uint8_t count = 0;
  for ( uint8_t i = 0; i < config->channelCount; i++ ) {
    if ( config->enabledChannels [ i ] ) {
      gpioInit.GPIO_Pin = config->gpioPins [ i ];
      GPIO_Init ( config->gpioPorts [ i ], &gpioInit );

      // Map the global channel enum to the DMA position for this ADC
      _adcDmaIndex [ config->channelEnums [ i ] ] = count++;
    }
  }

  // Configure clocks for ADC and DMA
  RCC_ADCCLKConfig ( config->adcClock );
  RCC_AHBPeriphClockCmd ( config->dmaClock | config->dmaPeripheral, ENABLE );

  // Deinitialize the DMA channel before configuration
  DMA_DeInit ( config->dmaChannel );

  // Initialize DMA structure
  DMA_StructInit ( &dmaInit );
  dmaInit.DMA_PeripheralBaseAddr = ( uint32_t ) &config->adc->DR;       // ADC data register
  dmaInit.DMA_MemoryBaseAddr     = ( uint32_t ) config->valueBuffer;    // Target buffer
  dmaInit.DMA_DIR                = DMA_DIR_PeripheralSRC;               // Peripheral -> Memory
  dmaInit.DMA_BufferSize         = count;                               // Enabled channels
  dmaInit.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  dmaInit.DMA_MemoryInc          = ( count > 1 ) ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
  dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  dmaInit.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  dmaInit.DMA_Mode               = DMA_Mode_Circular;    // Circular DMA
  dmaInit.DMA_Priority           = DMA_Priority_High;
  dmaInit.DMA_M2M                = DMA_M2M_Disable;

  // Initialize and enable the DMA channel
  DMA_Init ( config->dmaChannel, &dmaInit );
  DMA_Cmd ( config->dmaChannel, ENABLE );

  // Initialize ADC common settings
  ADC_CommonStructInit ( &adcCommon );
  adcCommon.ADC_Mode             = ADC_Mode_Independent;
  adcCommon.ADC_Clock            = ADC_Clock_SynClkModeDiv4;
  adcCommon.ADC_DMAAccessMode    = ADC_DMAAccessMode_1;
  adcCommon.ADC_DMAMode          = ADC_DMAMode_Circular;
  adcCommon.ADC_TwoSamplingDelay = 0;
  ADC_CommonInit ( config->adc, &adcCommon );

  // Initialize ADC settings
  ADC_StructInit ( &adcInit );
  adcInit.ADC_ContinuousConvMode    = ADC_ContinuousConvMode_Enable;
  adcInit.ADC_Resolution            = ADC_Resolution_12b;
  adcInit.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
  adcInit.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  adcInit.ADC_DataAlign             = ADC_DataAlign_Right;
  adcInit.ADC_OverrunMode           = ADC_OverrunMode_Disable;
  adcInit.ADC_AutoInjMode           = ADC_AutoInjec_Disable;
  adcInit.ADC_NbrOfRegChannel       = count;

  // Initialize the ADC with the specified settings
  ADC_Init ( config->adc, &adcInit );

  // Configure each enabled ADC channel with sample time
  uint8_t rank = 1;
  for ( uint8_t i = 0; i < config->channelCount; i++ ) {
    if ( config->enabledChannels [ i ] ) {
      ADC_RegularChannelConfig ( config->adc, config->adcChannels [ i ], rank++, ADC_SampleTime_601Cycles5 );
    }
  }

  // Enable the ADC and wait until it's ready
  ADC_Cmd ( config->adc, ENABLE );
  while ( ! ADC_GetFlagStatus ( config->adc, ADC_FLAG_RDY ) );

  // Configure ADC for DMA operation and start conversion
  ADC_DMAConfig ( config->adc, ADC_DMAMode_Circular );
  ADC_DMACmd ( config->adc, ENABLE );
  ADC_StartConversion ( config->adc );
}

// Specific Initializers
void _Adc1init ( void ) {
  // ADC1 channels used: IN4 (PA3), IN3 (PA2)
  const uint8_t count      = 2;
  const uint8_t enabled [] = {
    _isAdcEnable [ ADC1_IN4 ],
    _isAdcEnable [ ADC1_IN3 ]
  };
  const uint16_t pins [] = {
    GPIO_Pin_3,    // PA3 -> ADC1_IN4
    GPIO_Pin_2     // PA2 -> ADC1_IN3
  };
  GPIO_TypeDef *ports [] = {
    GPIOA,
    GPIOA
  };
  const uint8_t channels [] = {
    ADC_Channel_4,    // ADC1_IN4
    ADC_Channel_3     // ADC1_IN3
  };
  const Peripheral_ADC_Channel channelEnums [] = {
    ADC1_IN4,
    ADC1_IN3
  };

  ADC_Config cfg = {
    ADC1,
    DMA1_Channel1,
    RCC_AHBPeriph_DMA1,
    RCC_AHBPeriph_ADC12,
    RCC_ADC12PLLCLK_Div256,    // ADC12 clock for ADC1/2
    _adc1Values,
    enabled,
    pins,
    ports,
    channels,
    channelEnums,
    count
  };
  _AdcInitGeneric ( &cfg );
}

void _Adc2init ( void ) {
  // ADC2 channels used: IN12 (PB2), IN1 (PA4), IN2 (PA5)
  const uint8_t count      = 3;
  const uint8_t enabled [] = {
    _isAdcEnable [ ADC2_IN12 ],
    _isAdcEnable [ ADC2_IN1 ],
    _isAdcEnable [ ADC2_IN2 ]
  };
  const uint16_t pins [] = {
    GPIO_Pin_2,    // PB2 -> ADC2_IN12
    GPIO_Pin_4,    // PA4 -> ADC2_IN1  (CORRECTED)
    GPIO_Pin_5     // PA5 -> ADC2_IN2  (CORRECTED)
  };
  GPIO_TypeDef *ports [] = {
    GPIOB,
    GPIOA,
    GPIOA
  };
  const uint8_t channels [] = {
    ADC_Channel_12,    // ADC2_IN12
    ADC_Channel_1,     // ADC2_IN1
    ADC_Channel_2      // ADC2_IN2
  };
  const Peripheral_ADC_Channel channelEnums [] = {
    ADC2_IN12,
    ADC2_IN1,
    ADC2_IN2
  };

  ADC_Config cfg = {
    ADC2,
    DMA2_Channel1,
    RCC_AHBPeriph_DMA2,
    RCC_AHBPeriph_ADC12,
    RCC_ADC12PLLCLK_Div256,    // ADC12 clock for ADC1/2
    _adc2Values,
    enabled,
    pins,
    ports,
    channels,
    channelEnums,
    count
  };
  _AdcInitGeneric ( &cfg );
}

void _Adc3init ( void ) {
  // ADC3 channels used: IN5 (PB13)
  const uint8_t count      = 1;
  const uint8_t enabled [] = {
    _isAdcEnable [ ADC3_IN5 ]
  };
  const uint16_t pins [] = {
    GPIO_Pin_13    // PB13 -> ADC3_IN5
  };
  GPIO_TypeDef *ports [] = {
    GPIOB
  };
  const uint8_t channels [] = {
    ADC_Channel_5    // ADC3_IN5
  };
  const Peripheral_ADC_Channel channelEnums [] = {
    ADC3_IN5
  };

  ADC_Config cfg = {
    ADC3,
    DMA2_Channel5,
    RCC_AHBPeriph_DMA2,
    RCC_AHBPeriph_ADC34,
    RCC_ADC34PLLCLK_Div256,    // ADC34 clock for ADC3/4
    _adc3Values,
    enabled,
    pins,
    ports,
    channels,
    channelEnums,
    count
  };
  _AdcInitGeneric ( &cfg );
}

void _Adc4init ( void ) {
  // ADC4 channels used: IN5 (PB15), IN4 (PB14), IN3 (PB12)
  const uint8_t count      = 3;
  const uint8_t enabled [] = {
    _isAdcEnable [ ADC4_IN5 ],
    _isAdcEnable [ ADC4_IN4 ],
    _isAdcEnable [ ADC4_IN3 ]
  };
  const uint16_t pins [] = {
    GPIO_Pin_15,    // PB15 -> ADC4_IN5
    GPIO_Pin_14,    // PB14 -> ADC4_IN4
    GPIO_Pin_12     // PB12 -> ADC4_IN3
  };
  GPIO_TypeDef *ports [] = {
    GPIOB,
    GPIOB,
    GPIOB
  };
  const uint8_t channels [] = {
    ADC_Channel_5,    // ADC4_IN5
    ADC_Channel_4,    // ADC4_IN4
    ADC_Channel_3     // ADC4_IN3
  };
  const Peripheral_ADC_Channel channelEnums [] = {
    ADC4_IN5,
    ADC4_IN4,
    ADC4_IN3
  };

  ADC_Config cfg = {
    ADC4,
    DMA2_Channel2,
    RCC_AHBPeriph_DMA2,
    RCC_AHBPeriph_ADC34,
    RCC_ADC34PLLCLK_Div256,    // ADC34 clock for ADC3/4
    _adc4Values,
    enabled,
    pins,
    ports,
    channels,
    channelEnums,
    count
  };
  _AdcInitGeneric ( &cfg );
}

void APIAdcInit ( void ) {
  // Make sure GPIO clocks are on before pin inits inside _AdcInitGeneric
  RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE );

  // Initialize all ADCs sequentially
  _Adc1init ( );
  _Adc2init ( );
  _Adc3init ( );
  _Adc4init ( );
}