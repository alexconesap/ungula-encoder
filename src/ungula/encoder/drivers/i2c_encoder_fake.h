// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <math.h>
#include <stdint.h>

#include "ungula/encoder/i_encoder.h"
#include "ungula/hal/multiplexer/i_multiplexer.h"

/// @brief Test fake for `IEncoder` drivers that sit behind an I2C
/// multiplexer.
///
/// Mirrors the wiring contract of `As5600I2c` (and any future I2C-backed
/// driver): a borrowed `IMultiplexer*` plus a channel are stored on
/// the driver, and every read calls `selectChannel(channel)` first so
/// the multiplexer is correctly routed before the simulated transaction.
///
/// Tests can use `MultiplexerFake` from `lib_hal` to observe the
/// channel-select traffic and inject failures.

namespace ungula::encoder::drivers {

    constexpr int I2C_FAKE_DEFAULT_RESOLUTION = 4096;

    class I2cEncoderFake final : public IEncoder {
        public:
            I2cEncoderFake(const char* name = "fake-i2c",
                           ungula::hal::multiplexer::IMultiplexer* multiplexer = nullptr,
                           uint8_t multiplexerChannel = 0,
                           int resolution = I2C_FAKE_DEFAULT_RESOLUTION)
                    : IEncoder("FAKE_I2C", name, resolution),
                      multiplexer_(multiplexer),
                      multiplexerChannel_(multiplexerChannel) {}

            bool hasMagnetSensing() const override {
                return true;
            }

            // ---- Driver contract ----

            bool begin() override {
                isInitialized_ = true;
                if (multiplexer_ != nullptr && !multiplexer_->selectChannel(multiplexerChannel_)) {
                    setInitializationStatus(Error::MultiplexerError);
                    return false;
                }
                applyDirection(direction_);
                return true;
            }

            bool isFunctional() override {
                return true;
            }
            bool isConnected() override {
                return true;
            }

            float readPosition() override {
                if (!selectMux()) {
                    return NAN;
                }
                return scriptedPosition_;
            }

            float position() const override {
                return scriptedPosition_;
            }

            bool resetPosition(uint16_t initial_position) override {
                if (!selectMux()) {
                    return false;
                }
                scriptedPosition_ = static_cast<float>(initial_position);
                return true;
            }

            Status readStatus() override {
                return Status::Ok;
            }

            // ---- Test knobs / inspectors ----

            void setScriptedPosition(float p) {
                scriptedPosition_ = p;
            }
            uint8_t multiplexerChannel() const {
                return multiplexerChannel_;
            }
            bool hasMultiplexer() const {
                return multiplexer_ != nullptr;
            }
            uint32_t applyDirectionCallCount() const {
                return applyDirectionCallCount_;
            }
            Direction lastAppliedDirection() const {
                return lastAppliedDirection_;
            }

        protected:
            bool applyDirection(Direction direction) override {
                ++applyDirectionCallCount_;
                lastAppliedDirection_ = direction;
                return true;
            }

        private:
            bool selectMux() {
                if (!isInitialized_) {
                    setStatus(Error::NotInitialized);
                    return false;
                }
                if (multiplexer_ == nullptr) {
                    clearLastError();
                    return true;
                }
                if (!multiplexer_->selectChannel(multiplexerChannel_)) {
                    setStatus(Error::MultiplexerError);
                    return false;
                }
                clearLastError();
                return true;
            }

            ungula::hal::multiplexer::IMultiplexer* multiplexer_;
            uint8_t multiplexerChannel_;
            float scriptedPosition_ = 0.0f;
            uint32_t applyDirectionCallCount_ = 0;
            Direction lastAppliedDirection_ = Direction::ClockWise;
    };

}  // namespace ungula::encoder::drivers
