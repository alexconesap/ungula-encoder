// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "mt6835_spi.h"

#include <math.h>

namespace ungula::encoder::drivers
{

namespace
{

        constexpr uint8_t kCmdReadAngle = 0xA0;
        constexpr uint16_t kAngleReg = 0x003;

        // Status nibble bit map (byte 5, bits 2..0):
        //   bit 2: LFB — magnet too low / lost
        //   bit 1: OBT — magnet over-saturated
        //   bit 0: OW  — over-rotation warning (out of scope for status)
        constexpr uint8_t kStatusLfb = 0x04;
        constexpr uint8_t kStatusObt = 0x02;

} // namespace

Mt6835Spi::Mt6835Spi(const char *name, ungula::hal::spi::SpiMaster &spi)
        : IEncoder("MT6835", name, static_cast<int>(MT6835_RESOLUTION))
        , spi_(spi)
{
}

bool Mt6835Spi::begin()
{
        isInitialized_ = true;
        applyDirection(direction_);
        const uint32_t raw = readRawAngle(&lastStatus_);
        if (getLastError() == Error::None) {
                calibrateZero(raw);
                hasFirstSample_ = true;
        }
        clearLastError();
        return true;
}

uint32_t Mt6835Spi::readRawAngle(uint8_t *statusNibble)
{
        // Single CS assertion: the chip expects the command + address
        // first, then clocks the response into the same transaction.
        const uint8_t tx[7] = {
                kCmdReadAngle,
                static_cast<uint8_t>((kAngleReg >> 8) & 0xFF),
                static_cast<uint8_t>(kAngleReg & 0xFF),
                0,
                0,
                0,
                0, // dummy bytes for the response
        };
        uint8_t rx[7] = { 0 };
        if (!spi_.transfer(tx, rx, sizeof(tx))) {
                last_error_ = Error::I2CReadError;
                return 0;
        }
        clearLastError();
        // Bytes 3..5 carry the angle + status; byte 6 is CRC.
        const uint32_t high = static_cast<uint32_t>(rx[3]);
        const uint32_t mid = static_cast<uint32_t>(rx[4]);
        const uint32_t low = static_cast<uint32_t>(rx[5] >> 3) & 0x1FU;
        const uint32_t angle = (high << 13) | (mid << 5) | low;
        if (statusNibble != nullptr) {
                *statusNibble = static_cast<uint8_t>(rx[5] & 0x07U);
        }
        return angle & (MT6835_RESOLUTION - 1U);
}

bool Mt6835Spi::isConnected()
{
        (void)readRawAngle();
        return getLastError() == Error::None;
}

bool Mt6835Spi::isFunctional()
{
        return readStatus() == Status::Ok;
}

Status Mt6835Spi::readStatus()
{
        if (!isInitialized_) {
                setStatus(Error::NotInitialized);
                return Status::Error;
        }
        if (!isConnected()) {
                setStatus(Error::NotConnected);
                return Status::Error;
        }
        if (magnetStatus() != MagnetStatus::Ok) {
                return Status::Error;
        }
        return Status::Ok;
}

void Mt6835Spi::calibrateZero(uint32_t initial_position)
{
        zero_raw_position_ = initial_position;
        last_raw_position_ = initial_position;
        cumulative_position_ = 0;
}

bool Mt6835Spi::resetPosition(uint16_t initial_position)
{
        if (!isInitialized_) {
                setStatus(Error::NotInitialized);
                return false;
        }
        if (initial_position == 0U) {
                const uint32_t raw = readRawAngle(&lastStatus_);
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

float Mt6835Spi::readPosition()
{
        clearLastError();
        if (!isInitialized_) {
                setStatus(Error::NotInitialized);
                return NAN;
        }
        const uint32_t current = readRawAngle(&lastStatus_);
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
        int64_t diff = static_cast<int64_t>(current) - static_cast<int64_t>(last_raw_position_);
        constexpr int64_t half = static_cast<int64_t>(MT6835_RESOLUTION) / 2;
        if (diff > half) {
                diff -= static_cast<int64_t>(MT6835_RESOLUTION);
        } else if (diff < -half) {
                diff += static_cast<int64_t>(MT6835_RESOLUTION);
        }
        const int rotationFactor = (direction_ == Direction::ClockWise) ? -1 : 1;
        cumulative_position_ += rotationFactor * diff;
        last_raw_position_ = current;
        return static_cast<float>(cumulative_position_);
}

float Mt6835Spi::position() const
{
        return static_cast<float>(cumulative_position_);
}

MagnetStatus Mt6835Spi::magnetStatus()
{
        // Use the cached status nibble from the last successful read.
        // Tells the host what the chip thought of the magnet at that
        // moment without forcing another transaction.
        if ((lastStatus_ & kStatusLfb) != 0U) {
                return MagnetStatus::TooLow;
        }
        if ((lastStatus_ & kStatusObt) != 0U) {
                return MagnetStatus::TooHigh;
        }
        return MagnetStatus::Ok;
}
bool Mt6835Spi::isMagnetFound()
{
        return magnetStatus() != MagnetStatus::NotFound;
}
bool Mt6835Spi::isMagnetTooStrong()
{
        return magnetStatus() == MagnetStatus::TooHigh;
}
bool Mt6835Spi::isMagnetTooWeak()
{
        return magnetStatus() == MagnetStatus::TooLow;
}

} // namespace ungula::encoder::drivers
