// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "ma730_spi.h"

#include <math.h>

namespace ungula::encoder::drivers
{

    Ma730Spi::Ma730Spi(const char *name, ungula::hal::spi::SpiMaster &spi)
            : IEncoder("MA730", name, MA730_RESOLUTION)
            , spi_(spi)
    {
    }

    bool Ma730Spi::begin()
    {
        isInitialized_ = true;
        applyDirection(direction_);
        const uint16_t raw = readRawAngle();
        if (getLastError() == Error::None) {
            calibrateZero(raw);
            hasFirstSample_ = true;
        }
        clearLastError();
        return true;
    }

    uint16_t Ma730Spi::readRawAngle()
    {
        // The MA730 returns the 14-bit angle left-aligned in a 16-bit
        // frame. Send anything (NOPs) — the chip ignores MOSI in this
        // mode.
        const uint8_t tx[2] = { 0, 0 };
        uint8_t rx[2] = { 0, 0 };
        if (!spi_.transfer(tx, rx, sizeof(tx))) {
            last_error_ = Error::I2CReadError;
            return 0;
        }
        clearLastError();
        const uint16_t resp = static_cast<uint16_t>((rx[0] << 8) | rx[1]);
        return static_cast<uint16_t>(resp >> 2);
    }

    bool Ma730Spi::isConnected()
    {
        (void)readRawAngle();
        return getLastError() == Error::None;
    }

    bool Ma730Spi::isFunctional()
    {
        return readStatus() == Status::Ok;
    }

    Status Ma730Spi::readStatus()
    {
        if (!isInitialized_) {
            setStatus(Error::NotInitialized);
            return Status::Error;
        }
        if (!isConnected()) {
            setStatus(Error::NotConnected);
            return Status::Error;
        }
        return Status::Ok;
    }

    void Ma730Spi::calibrateZero(uint16_t initial_position)
    {
        zero_raw_position_ = initial_position;
        last_raw_position_ = initial_position;
        cumulative_position_ = 0;
    }

    bool Ma730Spi::resetPosition(uint16_t initial_position)
    {
        if (!isInitialized_) {
            setStatus(Error::NotInitialized);
            return false;
        }
        if (initial_position == 0U) {
            const uint16_t raw = readRawAngle();
            if (getLastError() != Error::None) {
                return false;
            }
            calibrateZero(raw);
        } else {
            calibrateZero(initial_position);
        }
        hasFirstSample_ = true;
        return true;
    }

    float Ma730Spi::readPosition()
    {
        clearLastError();
        if (!isInitialized_) {
            setStatus(Error::NotInitialized);
            return NAN;
        }
        const uint16_t current = readRawAngle();
        if (getLastError() != Error::None) {
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
        constexpr int half = static_cast<int>(MA730_RESOLUTION) / 2;
        if (diff > half) {
            diff -= static_cast<int>(MA730_RESOLUTION);
        } else if (diff < -half) {
            diff += static_cast<int>(MA730_RESOLUTION);
        }
        const int rotationFactor = (direction_ == Direction::ClockWise) ? -1 : 1;
        cumulative_position_ += rotationFactor * diff;
        last_raw_position_ = current;
        return static_cast<float>(cumulative_position_);
    }

    float Ma730Spi::position() const
    {
        return static_cast<float>(cumulative_position_);
    }

} // namespace ungula::encoder::drivers
