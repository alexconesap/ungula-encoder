// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "as5600.h"

#include <math.h>

#include "ungula/core/time/time_control.h"
#include "ungula/hal/gpio/gpio_access.h"

namespace ungula::encoder::drivers {

    namespace {

        constexpr uint16_t MASK_12_BITS = 0x0FFF;

        // Register map (subset).
        constexpr uint8_t REG_CONFIGURATION = 0x07;
        constexpr uint8_t REG_STATUS = 0x0B;
        constexpr uint8_t REG_RAW_ANGLE = 0x0C;

        // STATUS register bits.
        constexpr uint8_t STATUS_BIT_MAGNET_HIGH = 0x08;   // bit 3 — too strong
        constexpr uint8_t STATUS_BIT_MAGNET_LOW = 0x10;    // bit 4 — too weak
        constexpr uint8_t STATUS_BIT_MAGNET_FOUND = 0x20;  // bit 5

        // CONFIGURATION register: WD bit toggles the 1.6 s watchdog.
        constexpr uint16_t CFG_WATCHDOG_MASK = 0x2000;  // bit 13
        constexpr uint8_t CFG_WATCHDOG_BIT = 13;

        // I2C pacing: between consecutive 16-bit reads the chip needs a
        // brief gap so it can latch the next register. 50 µs after the
        // address phase is the sweet spot on 400 kHz.
        constexpr int64_t READ_PREP_US = 50;
        constexpr int64_t READ_INTERBYTE_US = 25;

    }  // namespace

    bool AS5600::begin(uint8_t multiplexerChannel, uint8_t directionPin) {
        multiplexerChannel_ = multiplexerChannel;
        directionPin_ = directionPin;
        address_ = AS5600_DEFAULT_ADDRESS;
        // Even on failure we mark the driver "initialised" so that
        // subsequent calls fail with the real error rather than a
        // generic NotInitialized.
        isInitialized_ = true;

        if (!selectMultiplexerChannel()) {
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
        configureDirection(Direction::ClockWise);

        calibrateZero();

        clearLastError();
        return true;
    }

    bool AS5600::isConnected() {
        return bus_.write(address_, nullptr, 0);
    }

    bool AS5600::isFunctional() {
        return readStatus() == Status::Ok;
    }

    Status AS5600::readStatus() {
        if (status_ == Status::InitializationError) {
            return Status::InitializationError;
        }
        if (!selectMultiplexerChannel()) {
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

    bool AS5600::setDirection(Direction direction) {
        if (!selectMultiplexerChannel()) {
            return false;
        }
        return configureDirection(direction);
    }

    Direction AS5600::getDirection() {
        return direction_;
    }

    int AS5600::getEncoderResolution() const {
        return static_cast<int>(AS5600_RESOLUTION);
    }

    bool AS5600::configureDirection(Direction direction) {
        direction_ = direction;
        if (directionPin_ != ENCODER_NO_DIRECTION_PIN) {
            ungula::hal::gpio::write(directionPin_,
                                     direction_ == Direction::ClockWise);
        }
        return true;
    }

    uint8_t AS5600::readStatusRegister() {
        return readRegister8b(REG_STATUS);
    }

    bool AS5600::writeRegister16b(uint8_t reg, uint16_t value, uint8_t maxRetries) {
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

    uint16_t AS5600::readRegister16b(uint8_t reg, uint8_t maxRetries) {
        clearLastError();

        uint8_t out[2] = {0, 0};
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

    uint8_t AS5600::readRegister8b(uint8_t reg, uint8_t maxRetries) {
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

    bool AS5600::isMagnetFound() {
        return magnetStatus() != MagnetStatus::NotFound;
    }
    bool AS5600::isMagnetTooStrong() {
        return magnetStatus() == MagnetStatus::TooHigh;
    }
    bool AS5600::isMagnetTooWeak() {
        return magnetStatus() == MagnetStatus::TooLow;
    }

    MagnetStatus AS5600::magnetStatus() {
        if (!selectMultiplexerChannel()) {
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

    bool AS5600::setWatchDog(bool enabled) {
        if (!selectMultiplexerChannel()) {
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

    bool AS5600::isWatchDogEnabled() {
        if (!selectMultiplexerChannel()) {
            return false;
        }
        const uint16_t cfg = readRegister16b(REG_CONFIGURATION);
        if (getLastError() != Error::None) {
            return false;
        }
        return ((cfg >> CFG_WATCHDOG_BIT) & 0x01U) != 0;
    }

    bool AS5600::isCurrentDirectionReadingNegative() const {
        // Hardware quirk on the original Rachel rig: clockwise rotation
        // returns decrementing raw values when DIR is grounded. Keeping
        // the rule explicit here so future drivers can override.
        return direction_ == Direction::ClockWise;
    }

    bool AS5600::resetPosition(uint16_t initial_position) {
        if (!selectMultiplexerChannel()) {
            return false;
        }
        calibrateZero(initial_position);
        return true;
    }

    uint16_t AS5600::rawAngle() {
        return static_cast<uint16_t>(readRegister16b(REG_RAW_ANGLE) & MASK_12_BITS);
    }

    /// @brief Capture the current raw angle as the new zero (or use a
    /// persisted value when restoring after reboot).
    void AS5600::calibrateZero(uint16_t initial_position) {
        zero_raw_position_ = (initial_position == 0) ? rawAngle() : initial_position;
        last_raw_position_ = zero_raw_position_;
        cumulative_position_ = static_cast<int>(initial_position);
    }

    float AS5600::readPosition() {
        clearLastError();
        if (!selectMultiplexerChannel()) {
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

    float AS5600::angleFromPosition(int position,
                                    float calibration_steps_to_degrees) const {
        return static_cast<float>(position) / calibration_steps_to_degrees;
    }

    float AS5600::angleFromCurrentPosition(float calibration_steps_to_degrees) const {
        return angleFromPosition(cumulative_position_, calibration_steps_to_degrees);
    }

    void AS5600::logPosition(uint16_t current_raw_position, int diff_raw) {
        const uint8_t channel =
                multiplexer_ != nullptr ? multiplexer_->getCurrentChannel() : 0xFF;
        logDebugf("ch=%u zero=%u cur=%u last=%u diff=%d cum=%d",
                  channel, zero_raw_position_, current_raw_position,
                  last_raw_position_, diff_raw, cumulative_position_);
    }

}  // namespace ungula::encoder::drivers
