/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                               #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\dma_registry.c                                     #
 #  Brief: Implementation of the DMA channel ownership registry.              #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#include "platform.h"

#include "drivers/dma_registry.h"

// 12 channels total: DMA1 Ch1..7 (slots 0..6), DMA2 Ch1..5 (slots 7..11).
#define DMA_REGISTRY_SLOTS 12

// Zero-initialised in .bss -> every slot starts as DMA_OWNER_FREE (0).
static dmaOwner_e dmaOwners[DMA_REGISTRY_SLOTS];

// Map a CMSIS channel pointer to a registry slot, or -1 if unknown.
static int dmaSlot ( const DMA_Channel_TypeDef *channel ) {
  if ( channel == DMA1_Channel1 ) return 0;
  if ( channel == DMA1_Channel2 ) return 1;
  if ( channel == DMA1_Channel3 ) return 2;
  if ( channel == DMA1_Channel4 ) return 3;
  if ( channel == DMA1_Channel5 ) return 4;
  if ( channel == DMA1_Channel6 ) return 5;
  if ( channel == DMA1_Channel7 ) return 6;
  if ( channel == DMA2_Channel1 ) return 7;
  if ( channel == DMA2_Channel2 ) return 8;
  if ( channel == DMA2_Channel3 ) return 9;
  if ( channel == DMA2_Channel4 ) return 10;
  if ( channel == DMA2_Channel5 ) return 11;
  return -1;
}

bool dmaClaim ( DMA_Channel_TypeDef *channel, dmaOwner_e owner ) {
  int slot = dmaSlot ( channel );
  if ( slot < 0 || owner == DMA_OWNER_FREE )
    return false;

  // Refuse if a different subsystem already owns it.
  if ( dmaOwners[slot] != DMA_OWNER_FREE && dmaOwners[slot] != owner )
    return false;

  dmaOwners[slot] = owner;
  return true;
}

void dmaRelease ( DMA_Channel_TypeDef *channel ) {
  int slot = dmaSlot ( channel );
  if ( slot >= 0 )
    dmaOwners[slot] = DMA_OWNER_FREE;
}

bool dmaIsFree ( DMA_Channel_TypeDef *channel ) {
  int slot = dmaSlot ( channel );
  return ( slot >= 0 ) && ( dmaOwners[slot] == DMA_OWNER_FREE );
}

dmaOwner_e dmaGetOwner ( DMA_Channel_TypeDef *channel ) {
  int slot = dmaSlot ( channel );
  return ( slot >= 0 ) ? dmaOwners[slot] : DMA_OWNER_FREE;
}
