// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "ungula/encoder/i_encoder.h"
#include "ungula/hal/i2c/i2c_master.h"
#include "ungula/hal/multiplexer/i_multiplexer.h"

/// @brief AS5600 12-bit magnetic rotary encoder — I2C transport.
///
/// 4096 positions per revolution, fixed I2C address 0x36. This driver
/// reads the angle by polling the chip over I2C; for the lower-latency
/// PWM-input transport, see (forthcoming) `As5600Pwm` and
/// `As5600I2cPwm`.
///
/// Optional DIR pin selects the direction of angle increment in
/// hardware:
///   - DIR=GND (0): clockwise increases the angle.
///   - DIR=VDD (1): counter-clockwise increases the angle.
///   - Datasheet requires DIR to be tied either way; do not float it.
///
/// All wiring (bus, optional multiplexer + channel, optional DIR pin)
/// is captured at construction. `begin()` takes no arguments. Channel
/// routing is handled internally — host code calls `readAngle()` and
/// the multiplexer is selected automatically.

namespace ungula::encoder::drivers {

    constexpr uint8_t AS5600_DEFAULT_ADDRESS = 0x36;
    constexpr uint16_t AS5600_RESOLUTION = 4096;

    class As5600I2c : public IEncoder {
        public:
            /// @param name              Caller-chosen tag, e.g. "vertical".
            /// @param bus               I2C bus the encoder lives on. Borrowed.
            /// @param multiplexer       Optional. `nullptr` = direct connect.
            /// @param multiplexerChannel Channel the encoder is wired to
            ///                          on the multiplexer. Ignored when
            ///                          no multiplexer was passed.
            /// @param directionPin      MCU pin wired to the chip's DIR
            ///                          input, or `ENCODER_NO_DIRECTION_PIN`.
            As5600I2c(const char* name, ungula::hal::i2c::I2cMaster& bus,
                      ungula::hal::multiplexer::IMultiplexer* multiplexer = nullptr,
                      uint8_t multiplexerChannel = 0,
                      uint8_t directionPin = ENCODER_NO_DIRECTION_PIN)
                : IEncoder("AS5600", name, AS5600_RESOLUTION),
                  bus_(bus),
                  multiplexer_(multiplexer),
                  multiplexerChannel_(multiplexerChannel),
                  directionPin_(directionPin) {}

            // ---- Capabilities (override the safe defaults) ----------------

            bool hasMagnetSensing() const override {
                return true;
            }
            bool hasWatchDog() const override {
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

            bool isMagnetFound() override;
            bool isMagnetTooStrong() override;
            bool isMagnetTooWeak() override;
            MagnetStatus magnetStatus() override;

            bool setWatchDog(bool enabled) override;
            bool isWatchDogEnabled() override;

            // ---- Inspectors (driver-private state, useful for tests + logs)
            uint8_t address() const {
                return address_;
            }
            uint8_t multiplexerChannel() const {
                return multiplexerChannel_;
            }
            bool hasMultiplexer() const {
                return multiplexer_ != nullptr;
            }

        protected:
            bool applyDirection(Direction direction) override;
            size_t formatLogPrefix(char* buf, size_t bufSize) const override;

        private:
            // Auto-select the multiplexer channel before any I2C
            // transaction. No-op when no multiplexer was passed at
            // construction. Sets `MultiplexerError` on failure.
            bool selectMux();

            ungula::hal::i2c::I2cMaster& bus_;
            ungula::hal::multiplexer::IMultiplexer* multiplexer_;
            uint8_t multiplexerChannel_;
            uint8_t directionPin_;
            uint8_t address_ = AS5600_DEFAULT_ADDRESS;

            uint16_t zero_raw_position_ = 0;
            uint16_t last_raw_position_ = 0;
            int cumulative_position_ = 0;

            // Low-level register helpers.
            uint16_t rawAngle();
            uint8_t readStatusRegister();
            uint16_t readRegister16b(uint8_t reg, uint8_t maxRetries = 3);
            uint8_t readRegister8b(uint8_t reg, uint8_t maxRetries = 3);
            bool writeRegister16b(uint8_t reg, uint16_t value, uint8_t maxRetries = 3);

            void calibrateZero(uint16_t initial_position = 0);
            bool isCurrentDirectionReadingNegative() const;

            void logPosition(uint16_t current_raw_position, int diff_raw);
    };

}  // namespace ungula::encoder::drivers
