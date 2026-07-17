/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\drivers\light_ws2811strip_stm32f30x.c                      #
 #  Created Date: Mon, 23rd Mar 2026                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 24th Mar 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "gpio.h"
#include "nvic.h"

#include "common/color.h"
#include "drivers/light_ws2811strip.h"
#include "drivers/dma_registry.h"

/*=============================================================================
 *  Selectable WS2812 data-pin table (indexed like RGB_n in API/RGB-LED.h)
 *
 *  Each slot is a vetted { GPIO, timer channel, DMA channel } combination that
 *  can generate the 800 kHz WS2812 waveform (timer CCx DMA request -> CCRx).
 *  Only ONE slot is active at a time; ws2811LedStripSetPin() picks it before
 *  ws2811LedStripHardwareInit(). See docs/fw-development-reference/{DMA,TIMER,PIN}_MAP.
 *===========================================================================*/
typedef struct {
    GPIO_TypeDef        *gpio;
    uint16_t             pin;
    uint8_t              pinSource;
    uint8_t              af;
    uint32_t             gpioAhbClk;
    TIM_TypeDef         *tim;
    uint32_t             timApbClk;
    bool                 timOnApb2;    // true: timer on APB2, false: APB1
    bool                 needsMoe;     // advanced-timer main output enable (BDTR MOE)
    uint8_t              channel;      // timer channel 1..4
    DMA_Channel_TypeDef *dma;
    uint32_t             dmaAhbClk;
    uint8_t              dmaIRQn;      // IRQn_Type value
    uint32_t             dmaTcFlag;    // DMA transfer-complete flag
} ws2811Hw_t;

#define WS2811_PIN_SLOT_COUNT 8
#define WS2811_DEFAULT_SLOT   7        // RGB_8 = PA15 (system flight-status pin)

static const ws2811Hw_t ws2811HwTable[WS2811_PIN_SLOT_COUNT] = {
    // RGB_1: PA8  TIM1_CH1  DMA1_Ch2  (shared with PPM RC-IN / 5th motor)
    { GPIOA, GPIO_Pin_8,  GPIO_PinSource8,  GPIO_AF_6,  RCC_AHBPeriph_GPIOA, TIM1,  RCC_APB2Periph_TIM1,  true,  true,  1, DMA1_Channel2, RCC_AHBPeriph_DMA1, DMA1_Channel2_IRQn, DMA1_FLAG_TC2 },
    // RGB_2: PB6  TIM4_CH1  DMA1_Ch1  (shares DMA with ADC1)
    { GPIOB, GPIO_Pin_6,  GPIO_PinSource6,  GPIO_AF_2,  RCC_AHBPeriph_GPIOB, TIM4,  RCC_APB1Periph_TIM4,  false, false, 1, DMA1_Channel1, RCC_AHBPeriph_DMA1, DMA1_Channel1_IRQn, DMA1_FLAG_TC1 },
    // RGB_3: PB14 TIM15_CH1 DMA1_Ch5  (! also SPI2 MISO = flash/blackbox)
    { GPIOB, GPIO_Pin_14, GPIO_PinSource14, GPIO_AF_1,  RCC_AHBPeriph_GPIOB, TIM15, RCC_APB2Periph_TIM15, true,  true,  1, DMA1_Channel5, RCC_AHBPeriph_DMA1, DMA1_Channel5_IRQn, DMA1_FLAG_TC5 },
    // RGB_4: PA13 TIM4_CH3  DMA1_Ch5  (! SWDIO debug pin)
    { GPIOA, GPIO_Pin_13, GPIO_PinSource13, GPIO_AF_10, RCC_AHBPeriph_GPIOA, TIM4,  RCC_APB1Periph_TIM4,  false, false, 3, DMA1_Channel5, RCC_AHBPeriph_DMA1, DMA1_Channel5_IRQn, DMA1_FLAG_TC5 },
    // RGB_5: PA14 TIM8_CH2  DMA2_Ch5  (! SWCLK debug pin)
    { GPIOA, GPIO_Pin_14, GPIO_PinSource14, GPIO_AF_5,  RCC_AHBPeriph_GPIOA, TIM8,  RCC_APB2Periph_TIM8,  true,  true,  2, DMA2_Channel5, RCC_AHBPeriph_DMA2, DMA2_Channel5_IRQn, DMA2_FLAG_TC5 },
    // RGB_6: PB4  TIM3_CH1  DMA1_Ch6  (free pin, recommended alternate)
    { GPIOB, GPIO_Pin_4,  GPIO_PinSource4,  GPIO_AF_2,  RCC_AHBPeriph_GPIOB, TIM3,  RCC_APB1Periph_TIM3,  false, false, 1, DMA1_Channel6, RCC_AHBPeriph_DMA1, DMA1_Channel6_IRQn, DMA1_FLAG_TC6 },
    // RGB_7: PB5  TIM17_CH1 DMA1_Ch1  (free pin, shares DMA with ADC1)
    { GPIOB, GPIO_Pin_5,  GPIO_PinSource5,  GPIO_AF_10, RCC_AHBPeriph_GPIOB, TIM17, RCC_APB2Periph_TIM17, true,  true,  1, DMA1_Channel1, RCC_AHBPeriph_DMA1, DMA1_Channel1_IRQn, DMA1_FLAG_TC1 },
    // RGB_8: PA15 TIM8_CH1  DMA2_Ch3  (default — system flight-status pin)
    { GPIOA, GPIO_Pin_15, GPIO_PinSource15, GPIO_AF_2,  RCC_AHBPeriph_GPIOA, TIM8,  RCC_APB2Periph_TIM8,  true,  true,  1, DMA2_Channel3, RCC_AHBPeriph_DMA2, DMA2_Channel3_IRQn, DMA2_FLAG_TC3 },
};

static uint8_t           ws2811Slot   = WS2811_DEFAULT_SLOT;
static const ws2811Hw_t *ws2811Active = &ws2811HwTable[WS2811_DEFAULT_SLOT];

void ws2811LedStripSetPin(uint8_t slot)
{
    if (slot < WS2811_PIN_SLOT_COUNT) {
        ws2811Slot = slot;
    }
}

// Compare-register (CCRx) address for the active slot's timer channel.
static volatile uint32_t *ws2811CcrAddr(const ws2811Hw_t *cfg)
{
    switch (cfg->channel) {
        case 1:  return &cfg->tim->CCR1;
        case 2:  return &cfg->tim->CCR2;
        case 3:  return &cfg->tim->CCR3;
        default: return &cfg->tim->CCR4;
    }
}

void ws2811LedStripHardwareInit(void)
{
    const ws2811Hw_t *cfg = &ws2811HwTable[ws2811Slot];
    ws2811Active = cfg;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    uint16_t prescalerValue;

    /* GPIO: alternate-function push-pull on the selected data pin */
    RCC_AHBPeriphClockCmd(cfg->gpioAhbClk, ENABLE);
    GPIO_PinAFConfig(cfg->gpio, cfg->pinSource, cfg->af);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = cfg->pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(cfg->gpio, &GPIO_InitStructure);

    /* Timer clock (APB1 or APB2 depending on the timer) */
    if (cfg->timOnApb2)
        RCC_APB2PeriphClockCmd(cfg->timApbClk, ENABLE);
    else
        RCC_APB1PeriphClockCmd(cfg->timApbClk, ENABLE);

    /* Time base: 800 kHz bit clock */
    prescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 29; // 800kHz
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(cfg->tim, &TIM_TimeBaseStructure);

    /* PWM1 mode on the selected channel */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    switch (cfg->channel) {
        case 1: TIM_OC1Init(cfg->tim, &TIM_OCInitStructure); TIM_OC1PreloadConfig(cfg->tim, TIM_OCPreload_Enable); break;
        case 2: TIM_OC2Init(cfg->tim, &TIM_OCInitStructure); TIM_OC2PreloadConfig(cfg->tim, TIM_OCPreload_Enable); break;
        case 3: TIM_OC3Init(cfg->tim, &TIM_OCInitStructure); TIM_OC3PreloadConfig(cfg->tim, TIM_OCPreload_Enable); break;
        case 4: TIM_OC4Init(cfg->tim, &TIM_OCInitStructure); TIM_OC4PreloadConfig(cfg->tim, TIM_OCPreload_Enable); break;
        default: break;
    }

    /* Advanced timers (TIM1/TIM8/TIM15/TIM17) need the main output enabled */
    if (cfg->needsMoe)
        TIM_CtrlPWMOutputs(cfg->tim, ENABLE);

    /* DMA: bit pattern in memory -> timer CCRx, paced by the CCx request */
    RCC_AHBPeriphClockCmd(cfg->dmaAhbClk, ENABLE);

    /* Reserve this DMA channel so nothing else can reuse it */
    dmaClaim(cfg->dma, DMA_OWNER_LED_STRIP);

    DMA_DeInit(cfg->dma);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ws2811CcrAddr(cfg);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = WS2811_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(cfg->dma, &DMA_InitStructure);

    switch (cfg->channel) {
        case 1: TIM_DMACmd(cfg->tim, TIM_DMA_CC1, ENABLE); break;
        case 2: TIM_DMACmd(cfg->tim, TIM_DMA_CC2, ENABLE); break;
        case 3: TIM_DMACmd(cfg->tim, TIM_DMA_CC3, ENABLE); break;
        case 4: TIM_DMACmd(cfg->tim, TIM_DMA_CC4, ENABLE); break;
        default: break;
    }

    DMA_ITConfig(cfg->dma, DMA_IT_TC, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = cfg->dmaIRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_WS2811_DMA);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_WS2811_DMA);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    const hsvColor_t green = {120, 255, 255};
    setStripColor(&green);
    ws2811UpdateStrip();
}

/*
 * Shared transfer-complete handler. Only one slot is ever active, so each DMA
 * vector we own routes here with its own channel + transfer-complete flag.
 * (USART2/3 TX-DMA handlers for Ch2/Ch7 are compiled out in serial_uart, so
 * there is no duplicate-symbol clash on this target.)
 */
static void ws2811DmaIsr(DMA_Channel_TypeDef *channel, uint32_t tcFlag)
{
    if (DMA_GetFlagStatus(tcFlag)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(channel, DISABLE);
        DMA_ClearFlag(tcFlag);
    }
}

void DMA1_Channel1_IRQHandler(void) { ws2811DmaIsr(DMA1_Channel1, DMA1_FLAG_TC1); } // RGB_2 (PB6), RGB_7 (PB5)
void DMA1_Channel2_IRQHandler(void) { ws2811DmaIsr(DMA1_Channel2, DMA1_FLAG_TC2); } // RGB_1 (PA8)
void DMA1_Channel5_IRQHandler(void) { ws2811DmaIsr(DMA1_Channel5, DMA1_FLAG_TC5); } // RGB_3 (PB14), RGB_4 (PA13)
void DMA1_Channel6_IRQHandler(void) { ws2811DmaIsr(DMA1_Channel6, DMA1_FLAG_TC6); } // RGB_6 (PB4)
void DMA2_Channel3_IRQHandler(void) { ws2811DmaIsr(DMA2_Channel3, DMA2_FLAG_TC3); } // RGB_8 (PA15, default)
void DMA2_Channel5_IRQHandler(void) { ws2811DmaIsr(DMA2_Channel5, DMA2_FLAG_TC5); } // RGB_5 (PA14)

void ws2811LedStripDMAEnable(void)
{
    const ws2811Hw_t *cfg = ws2811Active;
    DMA_SetCurrDataCounter(cfg->dma, WS2811_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(cfg->tim, 0);
    TIM_Cmd(cfg->tim, ENABLE);
    DMA_Cmd(cfg->dma, ENABLE);
}


