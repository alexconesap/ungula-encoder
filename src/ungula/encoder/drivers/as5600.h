// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "ungula/encoder/i_encoder.h"
#include "ungula/hal/i2c/i2c_master.h"

/// @brief AS5600 12-bit magnetic rotary encoder driver.
///
/// 4096 positions per revolution, I2C interface (fixed address 0x36).
/// Optional DIR pin selects the direction of angle increment:
///   - DIR=GND (0): clockwise increases the angle.
///   - DIR=VDD (1): counter-clockwise increases the angle.
///   - Datasheet requires DIR to be tied either way; do not leave floating.
///
/// Supports two deployment modes:
///   - Behind a `IMultiplexer` (e.g. several encoders on the same bus
///     sharing a TCA9548A).
///   - Direct-connect (multiplexer is `nullptr` at construction).

namespace ungula::encoder::drivers {

    constexpr uint8_t AS5600_DEFAULT_ADDRESS = 0x36;
    constexpr uint16_t AS5600_RESOLUTION = 4096;

    class AS5600 final : public IEncoder {
        public:
            /// @param name        Caller-chosen tag, e.g. "vertical".
            /// @param bus         I2C bus the encoder lives on. Borrowed.
            /// @param multiplexer Optional. `nullptr` means direct-connect.
            AS5600(const char* name, ungula::hal::i2c::I2cMaster& bus,
                   ungula::hal::multiplexer::IMultiplexer* multiplexer = nullptr)
                    : IEncoder("AS5600", name, multiplexer), bus_(bus) {}

            bool begin(uint8_t multiplexerChannel, uint8_t directionPin) override;

            bool isFunctional() override;
            bool isConnected() override;

            float readPosition() override;

            float angleFromPosition(int position,
                                    float calibration_steps_to_degrees) const override;
            float angleFromCurrentPosition(float calibration_steps_to_degrees) const override;
            int getEncoderResolution() const override;
            bool resetPosition(uint16_t initial_position) override;

            Status readStatus() override;

            bool setDirection(Direction direction) override;
            Direction getDirection() override;

            bool isMagnetFound() override;
            bool isMagnetTooStrong() override;
            bool isMagnetTooWeak() override;
            MagnetStatus magnetStatus() override;

            bool setWatchDog(bool enabled) override;
            bool isWatchDogEnabled() override;

        private:
            ungula::hal::i2c::I2cMaster& bus_;

            Direction direction_ = Direction::ClockWise;
            uint8_t directionPin_ = ENCODER_NO_DIRECTION_PIN;

            uint16_t zero_raw_position_ = 0;
            uint16_t last_raw_position_ = 0;
            int cumulative_position_ = 0;

            // Low-level register helpers.
            uint16_t rawAngle();
            uint8_t readStatusRegister();
            uint16_t readRegister16b(uint8_t reg, uint8_t maxRetries = 3);
            uint8_t readRegister8b(uint8_t reg, uint8_t maxRetries = 3);
            bool writeRegister16b(uint8_t reg, uint16_t value, uint8_t maxRetries = 3);

            bool configureDirection(Direction direction);
            void calibrateZero(uint16_t initial_position = 0);
            bool isCurrentDirectionReadingNegative() const;

            void logPosition(uint16_t current_raw_position, int diff_raw);
    };

}  // namespace ungula::encoder::drivers
