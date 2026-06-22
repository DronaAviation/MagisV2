/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                               #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\dma_registry.h                                     #
 #  Brief: Central ownership registry for the STM32F303 DMA channels.         #
 #         Lets one peripheral reserve a DMA channel so another cannot reuse  #
 #         it. DMA1 has 7 channels, DMA2 has 5 channels (12 total).           #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#ifndef DMA_REGISTRY_H
#define DMA_REGISTRY_H

#include <stdbool.h>

#include "platform.h"    // DMA_Channel_TypeDef, DMA1_Channel1 .. DMA2_Channel5

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Identifies which subsystem currently owns a DMA channel.
 */
typedef enum {
  DMA_OWNER_FREE = 0,    // Channel is unclaimed and available
  DMA_OWNER_ADC,         // Reserved by the Peripheral ADC layer
  DMA_OWNER_SERIAL_TX,   // Reserved by a UART TX DMA
  DMA_OWNER_SERIAL_RX,   // Reserved by a UART RX DMA
  DMA_OWNER_LED_STRIP,   // Reserved by the WS2811 LED strip
  DMA_OWNER_USER,        // Reserved by user code (PlutoPilot / custom driver)
} dmaOwner_e;

/**
 * @brief Reserve a DMA channel for the given owner.
 *
 * @param channel CMSIS channel pointer (e.g. DMA2_Channel4).
 * @param owner   The subsystem claiming the channel.
 * @return true  if the channel was free, or already owned by the same owner.
 * @return false if the channel is owned by a different subsystem (claim refused),
 *               or the channel pointer / owner is invalid.
 */
bool dmaClaim ( DMA_Channel_TypeDef *channel, dmaOwner_e owner );

/**
 * @brief Release a DMA channel back to the free pool. No-op for unknown channels.
 */
void dmaRelease ( DMA_Channel_TypeDef *channel );

/**
 * @brief True if the channel is known and currently unclaimed.
 */
bool dmaIsFree ( DMA_Channel_TypeDef *channel );

/**
 * @brief Returns the current owner of a channel (DMA_OWNER_FREE if free/unknown).
 */
dmaOwner_e dmaGetOwner ( DMA_Channel_TypeDef *channel );

#ifdef __cplusplus
}
#endif

#endif    // DMA_REGISTRY_H
