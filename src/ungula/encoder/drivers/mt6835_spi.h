// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "ungula/encoder/i_encoder.h"
#include "ungula/hal/spi/spi_master.h"

/// @brief MagnTek MT6835 21-bit absolute angle sensor — SPI transport.
///
/// 2'097'152 positions per revolution. Reads use a 6-byte burst:
///   - byte 0: command (0xA0 = read normal angle).
///   - bytes 1..2: 14-bit register address (big-endian, here 0x003).
///   - byte 3: angle high   (bits 20..13).
///   - byte 4: angle middle (bits 12.. 5).
///   - byte 5: bits 7..3 = angle low (bits  4.. 0); bits 2..0 = status.
///   - byte 6: CRC-8 over the prior bytes (validated by caller).
///
/// Burst length above is one byte more than typical SPI angle reads —
/// MT6835 includes an in-band CRC. This driver currently consumes the
/// raw 21-bit angle and ignores the CRC byte; CRC validation is a TODO
/// behind the same SpiMaster transaction shape.
///
/// Magnet-status bits (LFB / OBT / OW) are exposed through the chip's
/// status nibble in byte 5; mapped onto `MagnetStatus` here.

namespace ungula::encoder::drivers
{

constexpr uint32_t MT6835_RESOLUTION = 2'097'152U; // 2^21

class Mt6835Spi final : public IEncoder {
    public:
        Mt6835Spi(const char *name, ungula::hal::spi::SpiMaster &spi);

        bool hasMagnetSensing() const override
        {
                return true;
        }

        bool begin() override;
        bool isFunctional() override;
        bool isConnected() override;

        float readPosition() override;
        float position() const override;
        bool resetPosition(uint16_t initial_position) override;
        Status readStatus() override;

        MagnetStatus magnetStatus() override;
        bool isMagnetFound() override;
        bool isMagnetTooStrong() override;
        bool isMagnetTooWeak() override;

    private:
        // Returns angle in [0, 2^21). Sets last_error_ on failure;
        // statusNibble receives the in-band status bits when present.
        uint32_t readRawAngle(uint8_t *statusNibble = nullptr);

        void calibrateZero(uint32_t initial_position);

        ungula::hal::spi::SpiMaster &spi_;

        uint32_t zero_raw_position_ = 0;
        uint32_t last_raw_position_ = 0;
        int64_t cumulative_position_ = 0;
        bool hasFirstSample_ = false;
        uint8_t lastStatus_ = 0;
};

} // namespace ungula::encoder::drivers
