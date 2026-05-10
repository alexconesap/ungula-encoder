// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "as5600_i2c.h"

#include <math.h>
#include <stdio.h>

#include "ungula/core/time/time_control.h"
#include "ungula/hal/gpio/gpio_access.h"

namespace ungula::encoder::drivers
{

    namespace
    {

        constexpr uint16_t MASK_12_BITS = 0x0FFF;

        // Register map (subset).
        constexpr uint8_t REG_CONFIGURATION = 0x07;
        constexpr uint8_t REG_STATUS = 0x0B;
        constexpr uint8_t REG_RAW_ANGLE = 0x0C;

        // STATUS register bits.
        constexpr uint8_t STATUS_BIT_MAGNET_HIGH = 0x08; // bit 3 — too strong
        constexpr uint8_t STATUS_BIT_MAGNET_LOW = 0x10; // bit 4 — too weak
        constexpr uint8_t STATUS_BIT_MAGNET_FOUND = 0x20; // bit 5

        // CONFIGURATION register: WD bit toggles the 1.6 s watchdog.
        constexpr uint16_t CFG_WATCHDOG_MASK = 0x2000; // bit 13
        constexpr uint8_t CFG_WATCHDOG_BIT = 13;

        // I2C pacing: between consecutive 16-bit reads the chip needs a
        // brief gap so it can latch the next register. 50 µs after the
        // address phase is the sweet spot on 400 kHz.
        constexpr int64_t READ_PREP_US = 50;
        constexpr int64_t READ_INTERBYTE_US = 25;

    } // namespace

    bool As5600I2c::selectMux()
    {
        if (!isInitialized_) {
            // Programmer error — caller forgot begin(). Surface as a
            // hard error rather than silently moving on.
            setStatus(Error::NotInitialized);
            logErrorf("not initialised, call begin() first");
            return false;
        }
        if (multiplexer_ == nullptr) {
            // Direct-connect deployment — bus already points at us.
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

    bool As5600I2c::begin()
    {
        // Mark initialised before the probe so subsequent failures
        // surface as the actual error (BeginFailed / MultiplexerError)
        // rather than the generic NotInitialized.
        isInitialized_ = true;

        if (!selectMux()) {
            setInitializationStatus(Error::MultiplexerError);
            return false;
        }
        if (!isConnected()) {
            setInitializationStatus(Error::BeginFailed);
            logErrorf("not connected on begin()");
            return false;
        }

        if (directionPin_ != ENCODER_NO_DIRECTION_PIN) {
            ungula::hal::gpio::configOutput(directionPin_);
        }
        // Apply whatever direction the host set before / after construction
        // (default ClockWise) so the DIR pin reflects it from boot.
        applyDirection(direction_);

        calibrateZero();

        clearLastError();
        return true;
    }

    bool As5600I2c::applyDirection(Direction direction)
    {
        if (directionPin_ == ENCODER_NO_DIRECTION_PIN) {
            return true; // no DIR pin wired — logical-only
        }
        ungula::hal::gpio::write(directionPin_, direction == Direction::ClockWise);
        return true;
    }

    bool As5600I2c::isConnected()
    {
        return bus_.write(address_, nullptr, 0);
    }

    bool As5600I2c::isFunctional()
    {
        return readStatus() == Status::Ok;
    }

    Status As5600I2c::readStatus()
    {
        if (status_ == Status::InitializationError) {
            return Status::InitializationError;
        }
        if (!selectMux()) {
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

    uint8_t As5600I2c::readStatusRegister()
    {
        return readRegister8b(REG_STATUS);
    }

    bool As5600I2c::writeRegister16b(uint8_t reg, uint16_t value, uint8_t maxRetries)
    {
        clearLastError();
        const uint8_t buf[3] = {
            reg,
            static_cast<uint8_t>((value >> 8) & 0xFF),
            static_cast<uint8_t>(value & 0xFF),
        };
        for (uint8_t attempt = 0; attempt < maxRetries; ++attempt) {
            if (bus_.write(address_, buf, sizeof(buf))) {
                return true;
            }
        }
        last_error_ = Error::I2CWriteError;
        return false;
    }

    uint16_t As5600I2c::readRegister16b(uint8_t reg, uint8_t maxRetries)
    {
        clearLastError();
        uint8_t out[2] = { 0, 0 };
        for (uint8_t attempt = 0; attempt < maxRetries; ++attempt) {
            ungula::core::time::delayUs(READ_PREP_US);
            if (bus_.writeRead(address_, &reg, 1, out, sizeof(out))) {
                ungula::core::time::delayUs(READ_INTERBYTE_US);
                return static_cast<uint16_t>((static_cast<uint16_t>(out[0]) << 8) | out[1]);
            }
        }
        logErrorf("I2C read16 reg=0x%02X failed", reg);
        last_error_ = Error::I2CReadError;
        return 0;
    }

    uint8_t As5600I2c::readRegister8b(uint8_t reg, uint8_t maxRetries)
    {
        clearLastError();
        uint8_t out = 0;
        for (uint8_t attempt = 0; attempt < maxRetries; ++attempt) {
            if (bus_.writeRead(address_, &reg, 1, &out, 1)) {
                return out;
            }
        }
        logErrorf("I2C read8 reg=0x%02X failed", reg);
        last_error_ = Error::I2CReadError;
        return 0;
    }

    bool As5600I2c::isMagnetFound()
    {
        return magnetStatus() != MagnetStatus::NotFound;
    }
    bool As5600I2c::isMagnetTooStrong()
    {
        return magnetStatus() == MagnetStatus::TooHigh;
    }
    bool As5600I2c::isMagnetTooWeak()
    {
        return magnetStatus() == MagnetStatus::TooLow;
    }

    MagnetStatus As5600I2c::magnetStatus()
    {
        if (!selectMux()) {
            return MagnetStatus::EncoderError;
        }
        const uint8_t status = readStatusRegister();
        if (getLastError() != Error::None) {
            return MagnetStatus::EncoderError;
        }
        if ((status & STATUS_BIT_MAGNET_FOUND) == 0) {
            return MagnetStatus::NotFound;
        }
        if ((status & STATUS_BIT_MAGNET_HIGH) != 0) {
            return MagnetStatus::TooHigh;
        }
        if ((status & STATUS_BIT_MAGNET_LOW) != 0) {
            return MagnetStatus::TooLow;
        }
        return MagnetStatus::Ok;
    }

    bool As5600I2c::setWatchDog(bool enabled)
    {
        if (!selectMux()) {
            return false;
        }
        uint16_t cfg = readRegister16b(REG_CONFIGURATION);
        if (getLastError() != Error::None) {
            return false;
        }
        cfg = static_cast<uint16_t>(cfg & ~CFG_WATCHDOG_MASK);
        if (enabled) {
            cfg = static_cast<uint16_t>(cfg | CFG_WATCHDOG_MASK);
        }
        return writeRegister16b(REG_CONFIGURATION, cfg);
    }

    bool As5600I2c::isWatchDogEnabled()
    {
        if (!selectMux()) {
            return false;
        }
        const uint16_t cfg = readRegister16b(REG_CONFIGURATION);
        if (getLastError() != Error::None) {
            return false;
        }
        return ((cfg >> CFG_WATCHDOG_BIT) & 0x01U) != 0;
    }

    bool As5600I2c::isCurrentDirectionReadingNegative() const
    {
        // Hardware quirk on the original Rachel rig: clockwise rotation
        // returns decrementing raw values when DIR is grounded. Keeping
        // the rule explicit here so future drivers can override.
        return direction_ == Direction::ClockWise;
    }

    bool As5600I2c::resetPosition(uint16_t initial_position)
    {
        if (!selectMux()) {
            return false;
        }
        calibrateZero(initial_position);
        return true;
    }

    uint16_t As5600I2c::rawAngle()
    {
        return static_cast<uint16_t>(readRegister16b(REG_RAW_ANGLE) & MASK_12_BITS);
    }

    /// @brief Capture the current raw angle as the new zero (or use a
    /// persisted value when restoring after reboot).
    void As5600I2c::calibrateZero(uint16_t initial_position)
    {
        zero_raw_position_ = (initial_position == 0) ? rawAngle() : initial_position;
        last_raw_position_ = zero_raw_position_;
        cumulative_position_ = static_cast<int>(initial_position);
    }

    float As5600I2c::readPosition()
    {
        clearLastError();
        if (!selectMux()) {
            return NAN;
        }
        const uint16_t current = rawAngle();
        if (getLastError() != Error::None) {
            return NAN;
        }
        if (current == last_raw_position_) {
            logPosition(current, 0);
            return static_cast<float>(cumulative_position_);
        }

        int diff = static_cast<int>(current) - static_cast<int>(last_raw_position_);

        // Wrap-around handling: a 4095→0 step is +1, 0→4095 is -1.
        constexpr int half = static_cast<int>(AS5600_RESOLUTION) / 2;
        if (diff > half) {
            diff -= static_cast<int>(AS5600_RESOLUTION);
        } else if (diff < -half) {
            diff += static_cast<int>(AS5600_RESOLUTION);
        }

        const int rotationFactor = isCurrentDirectionReadingNegative() ? -1 : 1;
        cumulative_position_ += rotationFactor * diff;

        logPosition(current, diff);
        last_raw_position_ = current;
        return static_cast<float>(cumulative_position_);
    }

    float As5600I2c::position() const
    {
        // Cached cumulative position — no I/O. Updated on every successful
        // `readPosition()`. The base class's `angle()` divides this by the
        // stored calibration to produce degrees.
        return static_cast<float>(cumulative_position_);
    }

    size_t As5600I2c::formatLogPrefix(char *buf, size_t bufSize) const
    {
        if (buf == nullptr || bufSize == 0) {
            return 0;
        }
        // Driver override that includes the I2C address and the
        // currently-selected mux channel — handy when several encoders
        // share a bus and the log line needs to identify which one.
        const int n =
            snprintf(buf, bufSize, "[%s %s @0x%02X:%u]", getModel(), getName(), address_, multiplexerChannel_);
        return (n < 0) ? 0 : static_cast<size_t>(n);
    }

    void As5600I2c::logPosition(uint16_t current_raw_position, int diff_raw)
    {
        const uint8_t channel = multiplexer_ != nullptr ? multiplexer_->getCurrentChannel() : 0xFF;
        logDebugf("ch=%u zero=%u cur=%u last=%u diff=%d cum=%d", channel, zero_raw_position_, current_raw_position,
                  last_raw_position_, diff_raw, cumulative_position_);
    }

} // namespace ungula::encoder::drivers
