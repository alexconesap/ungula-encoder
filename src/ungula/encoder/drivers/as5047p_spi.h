// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "ungula/encoder/i_encoder.h"
#include "ungula/hal/spi/spi_master.h"

/// @brief AS5047P 14-bit magnetic rotary encoder — SPI transport.
///
/// 16384 positions per revolution. Reads the ANGLECOM register (0x3FFE)
/// over SPI mode 1, MSB-first, 16-bit transactions. The response carries
/// even-parity in bit 15 and an error flag in bit 14; bits 13:0 are the
/// angle.
///
/// Magnet status is reported through the chip's DIAAGC register (0x3FFC).
/// Watchdog is not a feature of this chip.

namespace ungula::encoder::drivers {

    constexpr uint16_t AS5047P_RESOLUTION = 16384;

    class As5047pSpi final : public IEncoder {
        public:
            /// @param name  Caller-chosen tag.
            /// @param spi   Already-begun SPI master (one device = one CS).
            As5047pSpi(const char* name, ungula::hal::spi::SpiMaster& spi);

            bool hasMagnetSensing() const override {
                return true;
            }

            // ---- Driver contract ----
            bool begin() override;
            bool isFunctional() override;
            bool isConnected() override;

            float readPosition() override;
            float position() const override;
            bool resetPosition(uint16_t initial_position) override;
            Status readStatus() override;

            // ---- Magnet ----
            MagnetStatus magnetStatus() override;
            bool isMagnetFound() override;
            bool isMagnetTooStrong() override;
            bool isMagnetTooWeak() override;

        private:
            // Read a 14-bit register. Sets `last_error_` on failure.
            uint16_t readRegister(uint16_t address);

            void calibrateZero(uint16_t initial_position);

            ungula::hal::spi::SpiMaster& spi_;

            uint16_t zero_raw_position_ = 0;
            uint16_t last_raw_position_ = 0;
            int cumulative_position_ = 0;
            bool hasFirstSample_ = false;
    };

}  // namespace ungula::encoder::drivers
