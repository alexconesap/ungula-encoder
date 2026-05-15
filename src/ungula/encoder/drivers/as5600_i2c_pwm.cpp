// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "as5600_i2c_pwm.h"

#include <math.h>

namespace ungula::encoder::drivers
{

namespace
{
        constexpr int kResolution = static_cast<int>(AS5600_RESOLUTION);
        // PWM frame constants come from `as5600_pwm.h` via the umbrella
        // include — `AS5600_PWM_FRAME_COUNTS`, `AS5600_PWM_PREAMBLE_COUNTS`.

        uint16_t rawFromSample(uint32_t highUs, uint32_t periodUs)
        {
                if (periodUs == 0U || highUs > periodUs) {
                        return 0xFFFFU;
                }
                const uint64_t scaled = (static_cast<uint64_t>(highUs) *
                                             static_cast<uint64_t>(AS5600_PWM_FRAME_COUNTS) +
                                         (periodUs / 2U)) /
                                        static_cast<uint64_t>(periodUs);
                if (scaled <= AS5600_PWM_PREAMBLE_COUNTS) {
                        return 0;
                }
                const uint64_t raw = scaled - AS5600_PWM_PREAMBLE_COUNTS;
                if (raw >= static_cast<uint64_t>(kResolution)) {
                        return static_cast<uint16_t>(kResolution - 1);
                }
                return static_cast<uint16_t>(raw);
        }
} // namespace

As5600I2cPwm::As5600I2cPwm(const char *name, ungula::hal::i2c::I2cMaster &bus,
                           ungula::hal::pwm_input::IPwmInput &pwm,
                           ungula::hal::multiplexer::IMultiplexer *multiplexer,
                           uint8_t multiplexerChannel, uint8_t directionPin)
        : As5600I2c(name, bus, multiplexer, multiplexerChannel, directionPin)
        , pwm_(pwm)
{
}

bool As5600I2cPwm::isConnected()
{
        // Bus must ACK and the PWM input must carry a fresh frame.
        if (!As5600I2c::isConnected()) {
                return false;
        }
        if (!pwm_.hasSample() || pwm_.sampleAgeUs() > staleThresholdUs_) {
                return false;
        }
        return true;
}

uint16_t As5600I2cPwm::rawFromCurrentSample() const
{
        return rawFromSample(pwm_.lastHighTimeUs(), pwm_.lastPeriodUs());
}

float As5600I2cPwm::readPosition()
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
        const uint16_t current = rawFromCurrentSample();
        if (current == 0xFFFFU) {
                setStatus(Error::NotConnected);
                return NAN;
        }

        if (!hasFirstPwmSample_) {
                pwmZeroRaw_ = current;
                pwmLastRaw_ = current;
                pwmCumulative_ = 0;
                hasFirstPwmSample_ = true;
                return 0.0f;
        }

        if (current == pwmLastRaw_) {
                return static_cast<float>(pwmCumulative_);
        }
        int diff = static_cast<int>(current) - static_cast<int>(pwmLastRaw_);
        constexpr int half = kResolution / 2;
        if (diff > half) {
                diff -= kResolution;
        } else if (diff < -half) {
                diff += kResolution;
        }
        // Same direction-sign convention as As5600I2c — clockwise reads
        // count down, so flip the sign there to keep a monotonic angle.
        const int rotationFactor = (direction_ == Direction::ClockWise) ? -1 : 1;
        pwmCumulative_ += rotationFactor * diff;
        pwmLastRaw_ = current;
        return static_cast<float>(pwmCumulative_);
}

float As5600I2cPwm::position() const
{
        return static_cast<float>(pwmCumulative_);
}

bool As5600I2cPwm::resetPosition(uint16_t initial_position)
{
        if (!isInitialized_) {
                setStatus(Error::NotInitialized);
                return false;
        }
        if (initial_position == 0U) {
                if (!pwm_.hasSample()) {
                        setStatus(Error::NotConnected);
                        return false;
                }
                const uint16_t raw = rawFromCurrentSample();
                if (raw == 0xFFFFU) {
                        setStatus(Error::NotConnected);
                        return false;
                }
                pwmZeroRaw_ = raw;
                pwmLastRaw_ = raw;
        } else {
                pwmZeroRaw_ = initial_position;
                pwmLastRaw_ = initial_position;
        }
        pwmCumulative_ = 0;
        hasFirstPwmSample_ = true;
        return true;
}

} // namespace ungula::encoder::drivers
