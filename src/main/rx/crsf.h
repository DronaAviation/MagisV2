/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Author: Omkar Dandekar (techsavvyomi)                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\rx\crsf.h                                                  #
 #  Brief: CRSF protocol header — RX channels + battery telemetry TX API.      #
 *******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "rx/rx.h"

/**
 * @brief Process a received CRSF frame.
 *
 * Called from the RX task. Validates CRC, unpacks RC channels (0x16),
 * parses link statistics (0x14), and sends battery telemetry (0x08)
 * in the inter-frame gap after each RC frame.
 *
 * @return SERIAL_RX_FRAME_COMPLETE if an RC channel frame was processed,
 *         SERIAL_RX_FRAME_PENDING otherwise.
 */
uint8_t crsfFrameStatus(void);

/**
 * @brief Initialise the CRSF driver on the configured serial port.
 *
 * Opens the UART at 420,000 baud in RXTX mode (half-duplex) and
 * registers the ISR callback for incoming CRSF frames.
 *
 * @param rxConfig       Pointer to the receiver configuration.
 * @param rxRuntimeConfig Pointer to the runtime RX config (channel count set here).
 * @param callback       Output pointer that receives the raw RC read function.
 * @return true if the serial port was opened successfully, false otherwise.
 */
bool crsfInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);

/**
 * @brief Update battery data for CRSF telemetry.
 *
 * Stores the latest battery values. The actual UART transmission happens
 * inside crsfFrameStatus() right after an RC frame is received, ensuring
 * safe timing on the half-duplex bus.
 *
 * @param voltage    Battery voltage in deci-volts (0.1V units, e.g. 42 = 4.2V).
 * @param current    Current draw in deci-amps (0.1A units, e.g. 15 = 1.5A).
 * @param capacity   Capacity consumed since power-on in mAh.
 * @param remaining  Battery remaining percentage (0–100).
 */
void crsfSetBatteryTelemetry(uint16_t voltage, uint16_t current, uint32_t capacity, uint8_t remaining);

#ifdef __cplusplus
}
#endif
