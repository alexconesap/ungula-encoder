// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "as5600_pwm.h"

#include <math.h>
#include <stdio.h>

#include "ungula/hal/gpio/gpio.h"

namespace ungula::encoder::drivers
{

namespace
{
        // Same effective resolution as the I2C transport. Imported here
        // (rather than from as5600_i2c.h) so the PWM driver does not
        // pull the I2C transport surface into headers it does not need.
        constexpr uint16_t kAs5600Resolution = 4096;
} // namespace

As5600Pwm::As5600Pwm(const char *name, ungula::hal::pwm_input::IPwmInput &pwm, uint8_t directionPin)
        : IEncoder("AS5600", name, kAs5600Resolution)
        , pwm_(pwm)
        , directionPin_(directionPin)
{
}

bool As5600Pwm::begin()
{
        isInitialized_ = true;
        if (directionPin_ != ENCODER_NO_DIRECTION_PIN) {
                ungula::hal::gpio::configOutput(directionPin_);
        }
        applyDirection(direction_);

        // Snapshot the current sample (if any) as zero so the first
        // `readPosition()` returns 0. If no sample is available yet
        // calibration is deferred until the first successful read.
        if (pwm_.hasSample()) {
                const uint16_t raw = rawFromSample(pwm_.lastHighTimeUs(), pwm_.lastPeriodUs());
                if (raw != 0xFFFFU) {
                        calibrateZero(raw);
                        hasFirstSample_ = true;
                }
        }
        clearLastError();
        return true;
}

bool As5600Pwm::applyDirection(Direction direction)
{
        if (directionPin_ == ENCODER_NO_DIRECTION_PIN) {
                return true;
        }
        ungula::hal::gpio::write(directionPin_, direction == Direction::ClockWise);
        return true;
}

bool As5600Pwm::isConnected()
{
        // "Connected" through PWM == "we have a recent sample".
        if (!pwm_.hasSample()) {
                return false;
        }
        return pwm_.sampleAgeUs() <= staleThresholdUs_;
}

bool As5600Pwm::isFunctional()
{
        return readStatus() == Status::Ok;
}

Status As5600Pwm::readStatus()
{
        if (!isInitialized_) {
                setStatus(Error::NotInitialized);
                return Status::Error;
        }
        if (!isConnected()) {
                setStatus(Error::NotConnected);
                return Status::Error;
        }
        clearLastError();
        return Status::Ok;
}

uint16_t As5600Pwm::rawFromSample(uint32_t highUs, uint32_t periodUs)
{
        if (periodUs == 0U || highUs > periodUs) {
                return 0xFFFFU;
        }
        // raw = round(high/period * frame_counts) - preamble
        // Done in 64-bit fixed-point to avoid float on the read path.
        const uint64_t scaled =
            (static_cast<uint64_t>(highUs) * static_cast<uint64_t>(AS5600_PWM_FRAME_COUNTS) +
             (periodUs / 2U)) /
            static_cast<uint64_t>(periodUs);
        if (scaled <= AS5600_PWM_PREAMBLE_COUNTS) {
                return 0;
        }
        const uint64_t raw = scaled - AS5600_PWM_PREAMBLE_COUNTS;
        if (raw >= kAs5600Resolution) {
                return static_cast<uint16_t>(kAs5600Resolution - 1U);
        }
        return static_cast<uint16_t>(raw);
}

void As5600Pwm::calibrateZero(uint16_t initial_position)
{
        zero_raw_position_ = initial_position;
        last_raw_position_ = initial_position;
        cumulative_position_ = 0;
}

bool As5600Pwm::isCurrentDirectionReadingNegative() const
{
        return direction_ == Direction::ClockWise;
}

bool As5600Pwm::resetPosition(uint16_t initial_position)
{
        if (!isInitialized_) {
                setStatus(Error::NotInitialized);
                return false;
        }
        if (initial_position == 0U) {
                // "Capture current angle as the new zero".
                if (!pwm_.hasSample()) {
                        setStatus(Error::NotConnected);
                        return false;
                }
                const uint16_t raw = rawFromSample(pwm_.lastHighTimeUs(), pwm_.lastPeriodUs());
                if (raw == 0xFFFFU) {
                        setStatus(Error::NotConnected);
                        return false;
                }
                calibrateZero(raw);
                hasFirstSample_ = true;
        } else {
                calibrateZero(initial_position);
        }
        return true;
}

float As5600Pwm::readPosition()
{
        clearLastError();
        if (!isInitialized_) {
                setStatus(Error::NotInitialized);
                return NAN;
        }
        if (!pwm_.hasSample() || pwm_.sampleAgeUs() > staleThresholdUs_) {
                setStatus(Error::NotConnected);
                return NAN;
        }

        // ISR-driven path: the cumulative count is maintained by
        // `updatePositionFromIsr()` from the backend's per-period
        // callback. `readPosition()` becomes a cheap snapshot read with
        // no decoding — staleness is still validated above so callers
        // get the same `NotConnected` diagnostic when the wire goes
        // quiet.
        if (isrUpdatesEnabled_) {
                return static_cast<float>(cumulative_position_);
        }

        const uint16_t current = rawFromSample(pwm_.lastHighTimeUs(), pwm_.lastPeriodUs());
        if (current == 0xFFFFU) {
                setStatus(Error::NotConnected);
                return NAN;
        }

        if (!hasFirstSample_) {
                calibrateZero(current);
                hasFirstSample_ = true;
                return 0.0f;
        }

        if (current == last_raw_position_) {
                return static_cast<float>(cumulative_position_);
        }
        int diff = static_cast<int>(current) - static_cast<int>(last_raw_position_);
        constexpr int half = static_cast<int>(kAs5600Resolution) / 2;
        if (diff > half) {
                diff -= static_cast<int>(kAs5600Resolution);
        } else if (diff < -half) {
                diff += static_cast<int>(kAs5600Resolution);
        }
        const int rotationFactor = isCurrentDirectionReadingNegative() ? -1 : 1;
        cumulative_position_ = cumulative_position_ + (rotationFactor * diff);
        last_raw_position_ = current;
        return static_cast<float>(cumulative_position_);
}

float As5600Pwm::position() const
{
        return static_cast<float>(cumulative_position_);
}

size_t As5600Pwm::formatLogPrefix(char *buf, size_t bufSize) const
{
        if (buf == nullptr || bufSize == 0) {
                return 0;
        }
        const int n = snprintf(buf, bufSize, "[%s %s pin=%u]", getModel(), getName(), pwm_.pin());
        return (n < 0) ? 0 : static_cast<size_t>(n);
}

// ------------------------------------------------------------------
//  ISR-driven update path
// ------------------------------------------------------------------

UNGULA_ISR_ATTR void As5600Pwm::onPwmSampleIsr(void *ctx)
{
        if (ctx == nullptr) {
                return;
        }
        static_cast<As5600Pwm *>(ctx)->updatePositionFromIsr();
}

UNGULA_ISR_ATTR void As5600Pwm::updatePositionFromIsr()
{
        // Same decode + wrap-around logic as `readPosition()`, but no
        // floats, no `setStatus()` (would race with the host), no
        // logging. Just cumulative-counter maintenance.
        const uint16_t current = rawFromSample(pwm_.lastHighTimeUs(), pwm_.lastPeriodUs());
        if (current == 0xFFFFU) {
                return; // bad sample — wait for the next frame
        }
        if (!hasFirstSample_) {
                zero_raw_position_ = current;
                last_raw_position_ = current;
                cumulative_position_ = 0;
                hasFirstSample_ = true;
                return;
        }
        if (current == last_raw_position_) {
                return;
        }
        int diff = static_cast<int>(current) - static_cast<int>(last_raw_position_);
        constexpr int half = static_cast<int>(kAs5600Resolution) / 2;
        if (diff > half) {
                diff -= static_cast<int>(kAs5600Resolution);
        } else if (diff < -half) {
                diff += static_cast<int>(kAs5600Resolution);
        }
        const int rotationFactor = isCurrentDirectionReadingNegative() ? -1 : 1;
        cumulative_position_ = cumulative_position_ + (rotationFactor * diff);
        last_raw_position_ = current;
}

void As5600Pwm::enableIsrUpdates()
{
        pwm_.setSampleCallback(&As5600Pwm::onPwmSampleIsr, this);
        isrUpdatesEnabled_ = true;
}

void As5600Pwm::disableIsrUpdates()
{
        pwm_.setSampleCallback(nullptr, nullptr);
        isrUpdatesEnabled_ = false;
}

} // namespace ungula::encoder::drivers
