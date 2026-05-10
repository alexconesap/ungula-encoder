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
/// `IEncoder` method must be implemented here, otherwise the file fails
/// to compile and the contract test fires immediately.
///
/// This fake is **transport-agnostic** by design — it knows nothing
/// about I2C, multiplexers, SPI, PWM. For tests that need to observe
/// the multiplexer channel-select pattern, see `i2c_encoder_fake.h`.

namespace ungula::encoder::drivers
{

    constexpr int FAKE_DEFAULT_RESOLUTION = 4096;

    class EncoderFake final : public IEncoder {
    public:
        EncoderFake(const char *name = "fake", int resolution = FAKE_DEFAULT_RESOLUTION)
                : IEncoder("FAKE", name, resolution)
        {
        }

        // ---- Capability flags (default: not supported) -----------------
        //
        // The fake exposes both magnet and watchdog so the contract
        // test can exercise every method without ambiguity. A test
        // that wants to observe the "no magnet sensing" path uses a
        // different fake (or a derived class that keeps the defaults).

        bool hasMagnetSensing() const override
        {
            return true;
        }
        bool hasWatchDog() const override
        {
            return true;
        }

        // ---- Driver contract ----

        bool begin() override
        {
            ++beginCallCount_;
            isInitialized_ = true;
            if (!beginResult_) {
                setInitializationStatus(Error::BeginFailed);
                return false;
            }
            // Apply whatever direction was set before begin() — same
            // contract every real driver follows.
            applyDirection(direction_);
            return true;
        }

        bool isFunctional() override
        {
            ++isFunctionalCallCount_;
            return isFunctionalResult_;
        }

        bool isConnected() override
        {
            ++isConnectedCallCount_;
            return isConnectedResult_;
        }

        float readPosition() override
        {
            ++readPositionCallCount_;
            if (!isInitialized_) {
                setStatus(Error::NotInitialized);
                return NAN;
            }
            return scriptedPosition_;
        }

        float position() const override
        {
            return scriptedPosition_;
        }

        bool resetPosition(uint16_t initial_position) override
        {
            ++resetCallCount_;
            scriptedPosition_ = static_cast<float>(initial_position);
            return true;
        }

        Status readStatus() override
        {
            ++readStatusCallCount_;
            return scriptedStatus_;
        }

        bool isMagnetFound() override
        {
            return magnetStatus_ != MagnetStatus::NotFound;
        }
        bool isMagnetTooStrong() override
        {
            return magnetStatus_ == MagnetStatus::TooHigh;
        }
        bool isMagnetTooWeak() override
        {
            return magnetStatus_ == MagnetStatus::TooLow;
        }
        MagnetStatus magnetStatus() override
        {
            return magnetStatus_;
        }

        bool setWatchDog(bool enabled) override
        {
            watchDogEnabled_ = enabled;
            return true;
        }
        bool isWatchDogEnabled() override
        {
            return watchDogEnabled_;
        }

        // Making the accessors public so tests can script error
        // conditions without subclassing.
        void public_setStatus(Error error)
        {
            setStatus(error);
        }
        void public_setInitializationStatus(Error error)
        {
            setInitializationStatus(error);
        }

        // ---- Test knobs ----

        void setBeginResult(bool ok)
        {
            beginResult_ = ok;
        }
        void setIsConnected(bool ok)
        {
            isConnectedResult_ = ok;
        }
        void setIsFunctional(bool ok)
        {
            isFunctionalResult_ = ok;
        }
        void setScriptedPosition(float position)
        {
            scriptedPosition_ = position;
        }
        void setScriptedStatus(Status status)
        {
            scriptedStatus_ = status;
        }
        void setMagnetStatus(MagnetStatus s)
        {
            magnetStatus_ = s;
        }

        // ---- Inspectors ----

        uint32_t beginCallCount() const
        {
            return beginCallCount_;
        }
        uint32_t readPositionCallCount() const
        {
            return readPositionCallCount_;
        }
        uint32_t readStatusCallCount() const
        {
            return readStatusCallCount_;
        }
        uint32_t resetCallCount() const
        {
            return resetCallCount_;
        }
        uint32_t isConnectedCallCount() const
        {
            return isConnectedCallCount_;
        }
        uint32_t isFunctionalCallCount() const
        {
            return isFunctionalCallCount_;
        }

        // Direction-pin spy: every applyDirection() call records the
        // value passed. Tests use this to verify pre-`begin()`
        // direction settings reach hardware once `begin()` runs.
        uint32_t applyDirectionCallCount() const
        {
            return applyDirectionCallCount_;
        }
        Direction lastAppliedDirection() const
        {
            return lastAppliedDirection_;
        }

    protected:
        bool applyDirection(Direction direction) override
        {
            ++applyDirectionCallCount_;
            lastAppliedDirection_ = direction;
            return true;
        }

    private:
        bool beginResult_ = true;
        bool isConnectedResult_ = true;
        bool isFunctionalResult_ = true;
        float scriptedPosition_ = 0.0f;
        Status scriptedStatus_ = Status::Ok;
        MagnetStatus magnetStatus_ = MagnetStatus::Ok;
        bool watchDogEnabled_ = false;

        uint32_t beginCallCount_ = 0;
        uint32_t readPositionCallCount_ = 0;
        uint32_t readStatusCallCount_ = 0;
        uint32_t resetCallCount_ = 0;
        uint32_t isConnectedCallCount_ = 0;
        uint32_t isFunctionalCallCount_ = 0;

        uint32_t applyDirectionCallCount_ = 0;
        Direction lastAppliedDirection_ = Direction::ClockWise;
    };

} // namespace ungula::encoder::drivers
