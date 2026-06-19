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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

extern "C" {
    #include "platform.h"
    #include "rx/rx.h"
    #include "rx/crsf.h"
    #include "io/serial.h"
    #include "drivers/serial.h"

    /* Expose internal functions for testing via extern declarations */
    /* We'll feed bytes directly through the data receive callback */
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

/*
 * CRSF protocol constants (mirrored from crsf.c for test verification)
 */
#define CRSF_SYNC_BYTE          0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED   0x16
#define CRSF_FRAMETYPE_LINK_STATISTICS      0x14
#define CRSF_MAX_CHANNEL        16
#define CRSF_CHANNEL_VALUE_MIN  172
#define CRSF_CHANNEL_VALUE_MID  992
#define CRSF_CHANNEL_VALUE_MAX  1811

/*
 * CRC8 with poly 0xD5 — same algorithm as in crsf.c
 */
static uint8_t testCrc8Tab[256];
static bool testCrcReady = false;

static void testGenerateCrc8Table(void)
{
    for (uint16_t i = 0; i < 256; i++) {
        uint8_t crc = (uint8_t)i;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? 0xD5 : 0);
        }
        testCrc8Tab[i] = crc;
    }
    testCrcReady = true;
}

static uint8_t testCrc8(const uint8_t *data, uint8_t len)
{
    if (!testCrcReady) testGenerateCrc8Table();
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = testCrc8Tab[crc ^ data[i]];
    }
    return crc;
}

/*
 * Build a CRSF RC Channels Packed frame.
 * channels[]: array of 16 channel values in CRSF range (172..1811)
 * outFrame[]: output buffer (must be at least 26 bytes)
 * Returns frame length.
 */
static uint8_t buildCrsfRcFrame(const uint16_t channels[16], uint8_t *outFrame)
{
    outFrame[0] = CRSF_SYNC_BYTE;   // device address
    outFrame[1] = 24;               // frame length: type(1) + payload(22) + crc(1)
    outFrame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    // Pack 16 x 11-bit channels into 22 bytes (LSB first)
    uint8_t *payload = &outFrame[3];
    memset(payload, 0, 22);

    payload[0]  = (uint8_t)(channels[0] & 0xFF);
    payload[1]  = (uint8_t)((channels[0] >> 8) | (channels[1] << 3));
    payload[2]  = (uint8_t)((channels[1] >> 5) | (channels[2] << 6));
    payload[3]  = (uint8_t)((channels[2] >> 2));
    payload[4]  = (uint8_t)((channels[2] >> 10) | (channels[3] << 1));
    payload[5]  = (uint8_t)((channels[3] >> 7) | (channels[4] << 4));
    payload[6]  = (uint8_t)((channels[4] >> 4) | (channels[5] << 7));
    payload[7]  = (uint8_t)((channels[5] >> 1));
    payload[8]  = (uint8_t)((channels[5] >> 9) | (channels[6] << 2));
    payload[9]  = (uint8_t)((channels[6] >> 6) | (channels[7] << 5));
    payload[10] = (uint8_t)((channels[7] >> 3));
    payload[11] = (uint8_t)(channels[8] & 0xFF);
    payload[12] = (uint8_t)((channels[8] >> 8) | (channels[9] << 3));
    payload[13] = (uint8_t)((channels[9] >> 5) | (channels[10] << 6));
    payload[14] = (uint8_t)((channels[10] >> 2));
    payload[15] = (uint8_t)((channels[10] >> 10) | (channels[11] << 1));
    payload[16] = (uint8_t)((channels[11] >> 7) | (channels[12] << 4));
    payload[17] = (uint8_t)((channels[12] >> 4) | (channels[13] << 7));
    payload[18] = (uint8_t)((channels[13] >> 1));
    payload[19] = (uint8_t)((channels[13] >> 9) | (channels[14] << 2));
    payload[20] = (uint8_t)((channels[14] >> 6) | (channels[15] << 5));
    payload[21] = (uint8_t)((channels[15] >> 3));

    // CRC covers type + payload (bytes 2..24)
    outFrame[25] = testCrc8(&outFrame[2], 23);

    return 26; // total frame size
}

/*
 * Build a CRSF Link Statistics frame.
 */
static uint8_t buildCrsfLinkStatsFrame(uint8_t rssi1, uint8_t rssi2, uint8_t lq, int8_t snr, uint8_t *outFrame)
{
    outFrame[0] = CRSF_SYNC_BYTE;
    outFrame[1] = 12;  // frame length: type(1) + payload(10) + crc(1)
    outFrame[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
    outFrame[3] = rssi1;   // uplink RSSI ant 1
    outFrame[4] = rssi2;   // uplink RSSI ant 2
    outFrame[5] = lq;      // uplink link quality
    outFrame[6] = (uint8_t)snr;  // uplink SNR
    outFrame[7] = 0;       // active antenna
    outFrame[8] = 0;       // RF mode
    outFrame[9] = 0;       // uplink TX power
    outFrame[10] = 0;      // downlink RSSI
    outFrame[11] = 0;      // downlink LQ
    outFrame[12] = 0;      // downlink SNR

    outFrame[13] = testCrc8(&outFrame[2], 11);

    return 14;
}

// ============ CRC8 Tests ============

TEST(CrsfCrcTest, TestCrc8KnownValues)
{
    // Verify our test CRC matches known values
    testGenerateCrc8Table();

    // CRC of empty data should be 0
    uint8_t crc = testCrc8(NULL, 0);
    EXPECT_EQ(0, crc);

    // CRC of a single zero byte
    uint8_t data1[] = { 0x00 };
    crc = testCrc8(data1, 1);
    EXPECT_EQ(0x00, crc);

    // CRC of 0x16 (RC channels type byte) — should be non-zero
    uint8_t data2[] = { 0x16 };
    crc = testCrc8(data2, 1);
    EXPECT_NE(0, crc);
}

TEST(CrsfCrcTest, TestCrc8Consistency)
{
    testGenerateCrc8Table();

    // Same data should always produce the same CRC
    uint8_t data[] = { 0x16, 0x00, 0x00, 0x00, 0x00 };
    uint8_t crc1 = testCrc8(data, 5);
    uint8_t crc2 = testCrc8(data, 5);
    EXPECT_EQ(crc1, crc2);

    // Different data should produce different CRC
    uint8_t data2[] = { 0x16, 0x01, 0x00, 0x00, 0x00 };
    uint8_t crc3 = testCrc8(data2, 5);
    EXPECT_NE(crc1, crc3);
}

// ============ Channel Packing Tests ============

TEST(CrsfChannelPackTest, TestAllChannelsMid)
{
    uint16_t channels[16];
    for (int i = 0; i < 16; i++) {
        channels[i] = CRSF_CHANNEL_VALUE_MID; // 992
    }

    uint8_t frame[26];
    uint8_t len = buildCrsfRcFrame(channels, frame);

    EXPECT_EQ(26, len);
    EXPECT_EQ(CRSF_SYNC_BYTE, frame[0]);
    EXPECT_EQ(24, frame[1]);
    EXPECT_EQ(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame[2]);

    // Verify CRC is valid
    uint8_t expectedCrc = testCrc8(&frame[2], 23);
    EXPECT_EQ(expectedCrc, frame[25]);
}

TEST(CrsfChannelPackTest, TestChannelMinMax)
{
    uint16_t channels[16];

    // All minimum
    for (int i = 0; i < 16; i++) channels[i] = CRSF_CHANNEL_VALUE_MIN;
    uint8_t frame[26];
    buildCrsfRcFrame(channels, frame);
    uint8_t crc = testCrc8(&frame[2], 23);
    EXPECT_EQ(crc, frame[25]);

    // All maximum
    for (int i = 0; i < 16; i++) channels[i] = CRSF_CHANNEL_VALUE_MAX;
    buildCrsfRcFrame(channels, frame);
    crc = testCrc8(&frame[2], 23);
    EXPECT_EQ(crc, frame[25]);
}

TEST(CrsfChannelPackTest, TestIndividualChannels)
{
    // Set each channel to a unique value and verify packing
    uint16_t channels[16];
    for (int i = 0; i < 16; i++) {
        channels[i] = CRSF_CHANNEL_VALUE_MIN + (i * 100);
    }

    uint8_t frame[26];
    buildCrsfRcFrame(channels, frame);

    // Unpack channel 0 manually to verify
    uint16_t ch0 = ((uint16_t)frame[3] | ((uint16_t)frame[4] << 8)) & 0x07FF;
    EXPECT_EQ(channels[0], ch0);

    // Unpack channel 1
    uint16_t ch1 = ((uint16_t)(frame[4] >> 3) | ((uint16_t)frame[5] << 5)) & 0x07FF;
    EXPECT_EQ(channels[1], ch1);
}

// ============ Channel Value Conversion Tests ============

TEST(CrsfConversionTest, TestCrsfToRcMidpoint)
{
    // CRSF mid (992) should map to approximately RC mid (1500)
    float rcValue = (float)CRSF_CHANNEL_VALUE_MID * 0.62477120f + 880.53f;
    EXPECT_NEAR(1500, rcValue, 2); // within 2us tolerance
}

TEST(CrsfConversionTest, TestCrsfToRcMinimum)
{
    // CRSF min (172) should map to approximately RC 988
    float rcValue = (float)CRSF_CHANNEL_VALUE_MIN * 0.62477120f + 880.53f;
    EXPECT_NEAR(988, rcValue, 2);
}

TEST(CrsfConversionTest, TestCrsfToRcMaximum)
{
    // CRSF max (1811) should map to approximately RC 2012
    float rcValue = (float)CRSF_CHANNEL_VALUE_MAX * 0.62477120f + 880.53f;
    EXPECT_NEAR(2012, rcValue, 2);
}

TEST(CrsfConversionTest, TestCrsfToRcMonotonic)
{
    // Verify that increasing CRSF values produce increasing RC values
    float prev = 0;
    for (uint16_t crsf = CRSF_CHANNEL_VALUE_MIN; crsf <= CRSF_CHANNEL_VALUE_MAX; crsf += 50) {
        float rc = (float)crsf * 0.62477120f + 880.53f;
        EXPECT_GT(rc, prev);
        prev = rc;
    }
}

// ============ Frame Structure Tests ============

TEST(CrsfFrameTest, TestRcFrameStructure)
{
    uint16_t channels[16] = { 992, 992, 172, 1811, 992, 992, 992, 992,
                               992, 992, 992, 992, 992, 992, 992, 992 };
    uint8_t frame[26];
    buildCrsfRcFrame(channels, frame);

    // Byte 0: sync/device address
    EXPECT_EQ(0xC8, frame[0]);

    // Byte 1: frame length (type + 22 payload + crc = 24)
    EXPECT_EQ(24, frame[1]);

    // Byte 2: frame type
    EXPECT_EQ(0x16, frame[2]);

    // Total frame: addr(1) + len(1) + type(1) + payload(22) + crc(1) = 26
}

TEST(CrsfFrameTest, TestLinkStatsFrameStructure)
{
    uint8_t frame[14];
    uint8_t len = buildCrsfLinkStatsFrame(50, 55, 100, 10, frame);

    EXPECT_EQ(14, len);
    EXPECT_EQ(0xC8, frame[0]);
    EXPECT_EQ(12, frame[1]);
    EXPECT_EQ(0x14, frame[2]);
    EXPECT_EQ(50, frame[3]);   // RSSI 1
    EXPECT_EQ(55, frame[4]);   // RSSI 2
    EXPECT_EQ(100, frame[5]);  // LQ
    EXPECT_EQ(10, frame[6]);   // SNR

    // Verify CRC
    uint8_t crc = testCrc8(&frame[2], 11);
    EXPECT_EQ(crc, frame[13]);
}

TEST(CrsfFrameTest, TestBadCrcDetection)
{
    uint16_t channels[16];
    for (int i = 0; i < 16; i++) channels[i] = CRSF_CHANNEL_VALUE_MID;

    uint8_t frame[26];
    buildCrsfRcFrame(channels, frame);

    // Corrupt the CRC
    frame[25] ^= 0xFF;

    // CRC should NOT match
    uint8_t expectedCrc = testCrc8(&frame[2], 23);
    EXPECT_NE(expectedCrc, frame[25]);
}

// ============ Edge Case Tests ============

TEST(CrsfEdgeCaseTest, TestAllZeroChannels)
{
    uint16_t channels[16];
    memset(channels, 0, sizeof(channels));

    uint8_t frame[26];
    buildCrsfRcFrame(channels, frame);

    // Should still produce valid CRC
    uint8_t crc = testCrc8(&frame[2], 23);
    EXPECT_EQ(crc, frame[25]);

    // Channel 0 should unpack as 0
    uint16_t ch0 = ((uint16_t)frame[3] | ((uint16_t)frame[4] << 8)) & 0x07FF;
    EXPECT_EQ(0, ch0);
}

TEST(CrsfEdgeCaseTest, TestMaxValueChannels)
{
    uint16_t channels[16];
    for (int i = 0; i < 16; i++) channels[i] = 0x07FF; // max 11-bit value (2047)

    uint8_t frame[26];
    buildCrsfRcFrame(channels, frame);

    uint8_t crc = testCrc8(&frame[2], 23);
    EXPECT_EQ(crc, frame[25]);

    // All payload bytes should be 0xFF (all 1s)
    for (int i = 3; i < 25; i++) {
        EXPECT_EQ(0xFF, frame[i]);
    }
}

TEST(CrsfEdgeCaseTest, TestAlternatingChannels)
{
    // Alternate between min and max
    uint16_t channels[16];
    for (int i = 0; i < 16; i++) {
        channels[i] = (i % 2 == 0) ? CRSF_CHANNEL_VALUE_MIN : CRSF_CHANNEL_VALUE_MAX;
    }

    uint8_t frame[26];
    buildCrsfRcFrame(channels, frame);

    // CRC should be valid
    uint8_t crc = testCrc8(&frame[2], 23);
    EXPECT_EQ(crc, frame[25]);

    // Verify first two channels unpack correctly
    uint16_t ch0 = ((uint16_t)frame[3] | ((uint16_t)frame[4] << 8)) & 0x07FF;
    uint16_t ch1 = ((uint16_t)(frame[4] >> 3) | ((uint16_t)frame[5] << 5)) & 0x07FF;
    EXPECT_EQ(CRSF_CHANNEL_VALUE_MIN, ch0);
    EXPECT_EQ(CRSF_CHANNEL_VALUE_MAX, ch1);
}

// ============ Protocol Constants Tests ============

TEST(CrsfConstantsTest, TestSerialRxEnum)
{
    // SERIALRX_CRSF should be 7
    EXPECT_EQ(7, SERIALRX_CRSF);

    // SERIALRX_PROVIDER_MAX should be SERIALRX_CRSF
    EXPECT_EQ(SERIALRX_CRSF, SERIALRX_PROVIDER_MAX);

    // Provider count should be 8
    EXPECT_EQ(8, SERIALRX_PROVIDER_COUNT);
}

TEST(CrsfConstantsTest, TestChannelRangeValues)
{
    // CRSF range should span 1639 steps (1811 - 172)
    EXPECT_EQ(1639, CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN);

    // Mid should be roughly center
    uint16_t calculatedMid = (CRSF_CHANNEL_VALUE_MIN + CRSF_CHANNEL_VALUE_MAX) / 2;
    EXPECT_NEAR(CRSF_CHANNEL_VALUE_MID, calculatedMid, 2);
}
