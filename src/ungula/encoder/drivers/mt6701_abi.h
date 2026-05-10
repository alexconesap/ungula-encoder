// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "ungula/encoder/i_encoder.h"
#include "ungula/hal/quadrature/i_decoder.h"

/// @brief MT6701 magnetic rotary encoder — ABI / quadrature transport.
///
/// In ABI mode the chip emits a quadrature A/B pulse pair (configurable
/// resolution: 1024 PPR by default; the chip can also be configured for
/// 256 / 512 / 2048 / 4096 PPR). With 4× decoding that yields up to
/// 16384 counts per revolution.
///
/// The driver is a thin adapter on top of `IDecoder` — the host wires
/// the quadrature decoder up once and hands it to the encoder. Position
/// reads are zero-cost: they only read the decoder's count.
///
/// Magnet status / watchdog are I2C-side features and are not available
/// in pure ABI mode. Use `As5600I2cPwm`-style composition (a future
/// `Mt6701I2cAbi`) if both diagnostics and ABI fast reads are needed.

namespace ungula::encoder::drivers
{

    /// Default counts per revolution after 4× quadrature decode.
    /// 1024 PPR × 4 edges = 4096.
    constexpr uint16_t MT6701_DEFAULT_RESOLUTION = 4096;

    class Mt6701Abi final : public IEncoder {
    public:
        /// @param name        Caller-chosen tag.
        /// @param decoder     Already-begun quadrature decoder.
        /// @param resolution  Counts per revolution after decoding.
        ///                    Default: 4096 (1024 PPR × 4× decode).
        Mt6701Abi(const char *name, ungula::hal::quadrature::IDecoder &decoder,
                  uint16_t resolution = MT6701_DEFAULT_RESOLUTION);

        // ---- Driver contract ----
        bool begin() override;
        bool isFunctional() override;
        bool isConnected() override;

        float readPosition() override;
        float position() const override;
        bool resetPosition(uint16_t initial_position) override;
        Status readStatus() override;

    private:
        ungula::hal::quadrature::IDecoder &decoder_;
        int32_t lastReadCount_ = 0;
        int signFactor_ = 1;
    };

} // namespace ungula::encoder::drivers
