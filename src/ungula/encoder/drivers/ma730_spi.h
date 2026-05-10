// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "ungula/encoder/i_encoder.h"
#include "ungula/hal/spi/spi_master.h"

/// @brief Monolithic Power MA730 14-bit angle sensor — SPI transport.
///
/// 16384 positions per revolution. The angle read is a single 16-bit
/// transaction: the chip latches the angle on CS-low and clocks it out
/// MSB-first on the next 14 SCLK cycles (left-aligned in the 16-bit
/// frame), so the host shifts the response right by 2 to get the angle.
///
/// The MA730 ships no magnet diagnostics over the angle path (a separate
/// MGT/MGL bit pair is exposed via the field-strength register), so this
/// driver leaves the `IEncoder` magnet defaults in place.

namespace ungula::encoder::drivers
{

    constexpr uint16_t MA730_RESOLUTION = 16384;

    class Ma730Spi final : public IEncoder {
    public:
        Ma730Spi(const char *name, ungula::hal::spi::SpiMaster &spi);

        bool begin() override;
        bool isFunctional() override;
        bool isConnected() override;

        float readPosition() override;
        float position() const override;
        bool resetPosition(uint16_t initial_position) override;
        Status readStatus() override;

    private:
        uint16_t readRawAngle();
        void calibrateZero(uint16_t initial_position);

        ungula::hal::spi::SpiMaster &spi_;

        uint16_t zero_raw_position_ = 0;
        uint16_t last_raw_position_ = 0;
        int cumulative_position_ = 0;
        bool hasFirstSample_ = false;
    };

} // namespace ungula::encoder::drivers
