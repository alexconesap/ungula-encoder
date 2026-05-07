// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <math.h>
#include <stdint.h>

#include "ungula/encoder/i_encoder.h"

/// @brief Header-only test fake for `IEncoder`.
///
/// Drop-in for any test that needs to inject a fake encoder. Records call
/// counts, exposes scriptable return values, and lets the test push raw
/// positions directly. Lives under `drivers/` so every test can pick it
/// up with one include.
///
/// Tests rely on this fake to detect interface drift: every pure-virtual
/// `IEncoder` method must be implemented here, otherwise the file
/// fails to compile and the contract test fires immediately.

namespace ungula::encoder::drivers {

    class EncoderFake final : public IEncoder {
        public:
            EncoderFake(const char* name = "fake",
                        ungula::hal::multiplexer::IMultiplexer* multiplexer = nullptr)
                    : IEncoder("FAKE", name, multiplexer) {}

            // ---- Driver contract ----

            bool begin(uint8_t multiplexerChannel, uint8_t directionPin) override {
                ++beginCallCount_;
                multiplexerChannel_ = multiplexerChannel;
                lastDirectionPin_ = directionPin;
                isInitialized_ = true;
                if (!beginResult_) {
                    setInitializationStatus(Error::BeginFailed);
                }
                return beginResult_;
            }

            bool isFunctional() override {
                ++isFunctionalCallCount_;
                return isFunctionalResult_;
            }

            bool isConnected() override {
                ++isConnectedCallCount_;
                return isConnectedResult_;
            }

            float readPosition() override {
                ++readPositionCallCount_;
                if (!selectMultiplexerChannel()) {
                    return NAN;
                }
                return scriptedPosition_;
            }

            float angleFromPosition(int position,
                                    float calibration_steps_to_degrees) const override {
                return static_cast<float>(position) / calibration_steps_to_degrees;
            }

            float angleFromCurrentPosition(float calibration_steps_to_degrees) const override {
                return scriptedPosition_ / calibration_steps_to_degrees;
            }

            bool resetPosition(uint16_t initial_position) override {
                ++resetCallCount_;
                scriptedPosition_ = static_cast<float>(initial_position);
                return true;
            }

            int getEncoderResolution() const override {
                return resolution_;
            }

            Status readStatus() override {
                ++readStatusCallCount_;
                return scriptedStatus_;
            }

            bool setDirection(Direction direction) override {
                direction_ = direction;
                return true;
            }
            Direction getDirection() override {
                return direction_;
            }

            bool isMagnetFound() override {
                return magnetStatus_ != MagnetStatus::NotFound;
            }
            bool isMagnetTooStrong() override {
                return magnetStatus_ == MagnetStatus::TooHigh;
            }
            bool isMagnetTooWeak() override {
                return magnetStatus_ == MagnetStatus::TooLow;
            }
            MagnetStatus magnetStatus() override {
                return magnetStatus_;
            }

            bool setWatchDog(bool enabled) override {
                watchDogEnabled_ = enabled;
                return true;
            }
            bool isWatchDogEnabled() override {
                return watchDogEnabled_;
            }

            // ---- Test knobs ----

            void setBeginResult(bool ok) {
                beginResult_ = ok;
            }
            void setIsConnected(bool ok) {
                isConnectedResult_ = ok;
            }
            void setIsFunctional(bool ok) {
                isFunctionalResult_ = ok;
            }
            void setScriptedPosition(float position) {
                scriptedPosition_ = position;
            }
            void setScriptedStatus(Status status) {
                scriptedStatus_ = status;
            }
            void setMagnetStatus(MagnetStatus s) {
                magnetStatus_ = s;
            }
            void setResolution(int r) {
                resolution_ = r;
            }

            // ---- Inspectors ----

            uint32_t beginCallCount() const {
                return beginCallCount_;
            }
            uint32_t readPositionCallCount() const {
                return readPositionCallCount_;
            }
            uint32_t readStatusCallCount() const {
                return readStatusCallCount_;
            }
            uint32_t resetCallCount() const {
                return resetCallCount_;
            }
            uint32_t isConnectedCallCount() const {
                return isConnectedCallCount_;
            }
            uint32_t isFunctionalCallCount() const {
                return isFunctionalCallCount_;
            }
            uint8_t lastDirectionPin() const {
                return lastDirectionPin_;
            }

        private:
            bool beginResult_ = true;
            bool isConnectedResult_ = true;
            bool isFunctionalResult_ = true;
            float scriptedPosition_ = 0.0f;
            Status scriptedStatus_ = Status::Ok;
            MagnetStatus magnetStatus_ = MagnetStatus::Ok;
            int resolution_ = 4096;
            Direction direction_ = Direction::ClockWise;
            bool watchDogEnabled_ = false;

            uint8_t lastDirectionPin_ = ENCODER_NO_DIRECTION_PIN;
            uint32_t beginCallCount_ = 0;
            uint32_t readPositionCallCount_ = 0;
            uint32_t readStatusCallCount_ = 0;
            uint32_t resetCallCount_ = 0;
            uint32_t isConnectedCallCount_ = 0;
            uint32_t isFunctionalCallCount_ = 0;
    };

}  // namespace ungula::encoder::drivers
