// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "as5047p_spi.h"

#include <math.h>

namespace ungula::encoder::drivers {

    namespace {

        constexpr uint16_t kReadFlag = 0x4000;    // bit 14: 1 = read
        constexpr uint16_t kAngleReg = 0x3FFE;    // ANGLECOM
        constexpr uint16_t kDiagAgcReg = 0x3FFC;  // DIAAGC
        constexpr uint16_t kAngleMask = 0x3FFF;   // 14 bits
        constexpr uint16_t kErrorFlag = 0x4000;   // bit 14 of response = error

        // DIAAGC bit map (lower 12 bits):
        //   bit 11: MAGL  — magnet too low
        //   bit 10: MAGH  — magnet too high
        //   bit  9: COF   — CORDIC overflow
        //   bit  8: LF    — offset compensation finished
        //   bits 7..0: AGC value
        constexpr uint16_t kDiagMagL = 1U << 11;
        constexpr uint16_t kDiagMagH = 1U << 10;
        constexpr uint16_t kDiagLfReady = 1U << 8;

        bool evenParity(uint16_t v) {
            v ^= v >> 8;
            v ^= v >> 4;
            v ^= v >> 2;
            v ^= v >> 1;
            return (v & 1U) == 0U;
        }

        uint16_t buildCommand(uint16_t address, bool isRead) {
            uint16_t frame = static_cast<uint16_t>(address & 0x3FFFU);
            if (isRead) {
                frame |= kReadFlag;
            }
            // Bit 15 is even-parity over the lower 15 bits.
            uint16_t parityBits = static_cast<uint16_t>(frame & 0x7FFFU);
            if (!evenParity(parityBits)) {
                frame |= 0x8000U;
            }
            return frame;
        }

    }  // namespace

    As5047pSpi::As5047pSpi(const char* name, ungula::hal::spi::SpiMaster& spi)
        : IEncoder("AS5047P", name, AS5047P_RESOLUTION), spi_(spi) {}

    bool As5047pSpi::begin() {
        isInitialized_ = true;
        applyDirection(direction_);
        // Probe by reading the angle once. Failure is non-fatal — the
        // host can retry once the wiring is up.
        const uint16_t raw = readRegister(kAngleReg);
        if (getLastError() == Error::None) {
            calibrateZero(raw);
            hasFirstSample_ = true;
        }
        clearLastError();
        return true;
    }

    uint16_t As5047pSpi::readRegister(uint16_t address) {
        // AS5047P: send the read frame on transaction 1, the data comes
        // back on transaction 2. Issue a NOP on the second transaction.
        const uint16_t cmd = buildCommand(address, /*isRead=*/true);
        const uint16_t nop = buildCommand(0x0000, /*isRead=*/true);
        const uint8_t txCmd[2] = {static_cast<uint8_t>(cmd >> 8), static_cast<uint8_t>(cmd & 0xFF)};
        const uint8_t txNop[2] = {static_cast<uint8_t>(nop >> 8), static_cast<uint8_t>(nop & 0xFF)};
        uint8_t rxDiscard[2] = {0, 0};
        uint8_t rx[2] = {0, 0};
        if (!spi_.transfer(txCmd, rxDiscard, sizeof(txCmd))) {
            last_error_ = Error::I2CReadError;
            return 0;
        }
        if (!spi_.transfer(txNop, rx, sizeof(rx))) {
            last_error_ = Error::I2CReadError;
            return 0;
        }
        const uint16_t resp = static_cast<uint16_t>((rx[0] << 8) | rx[1]);
        if ((resp & kErrorFlag) != 0U) {
            last_error_ = Error::I2CReadError;
            return 0;
        }
        clearLastError();
        return static_cast<uint16_t>(resp & kAngleMask);
    }

    bool As5047pSpi::isConnected() {
        // No "ping" register — a successful angle read is the proxy.
        const uint16_t raw = readRegister(kAngleReg);
        (void)raw;
        return getLastError() == Error::None;
    }

    bool As5047pSpi::isFunctional() {
        return readStatus() == Status::Ok;
    }

    Status As5047pSpi::readStatus() {
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

    void As5047pSpi::calibrateZero(uint16_t initial_position) {
        zero_raw_position_ = initial_position;
        last_raw_position_ = initial_position;
        cumulative_position_ = 0;
    }

    bool As5047pSpi::resetPosition(uint16_t initial_position) {
        if (!isInitialized_) {
            setStatus(Error::NotInitialized);
            return false;
        }
        if (initial_position == 0U) {
            const uint16_t raw = readRegister(kAngleReg);
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

    float As5047pSpi::readPosition() {
        clearLastError();
        if (!isInitialized_) {
            setStatus(Error::NotInitialized);
            return NAN;
        }
        const uint16_t current = readRegister(kAngleReg);
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
        constexpr int half = static_cast<int>(AS5047P_RESOLUTION) / 2;
        if (diff > half) {
            diff -= static_cast<int>(AS5047P_RESOLUTION);
        } else if (diff < -half) {
            diff += static_cast<int>(AS5047P_RESOLUTION);
        }
        const int rotationFactor = (direction_ == Direction::ClockWise) ? -1 : 1;
        cumulative_position_ += rotationFactor * diff;
        last_raw_position_ = current;
        return static_cast<float>(cumulative_position_);
    }

    float As5047pSpi::position() const {
        return static_cast<float>(cumulative_position_);
    }

    MagnetStatus As5047pSpi::magnetStatus() {
        const uint16_t diag = readRegister(kDiagAgcReg);
        if (getLastError() != Error::None) {
            return MagnetStatus::EncoderError;
        }
        if ((diag & kDiagMagH) != 0U) {
            return MagnetStatus::TooHigh;
        }
        if ((diag & kDiagMagL) != 0U) {
            return MagnetStatus::TooLow;
        }
        if ((diag & kDiagLfReady) == 0U) {
            // Offset compensation hasn't finished yet — caller treats as
            // "not yet usable" rather than a hard error.
            return MagnetStatus::NotFound;
        }
        return MagnetStatus::Ok;
    }

    bool As5047pSpi::isMagnetFound() {
        return magnetStatus() != MagnetStatus::NotFound;
    }
    bool As5047pSpi::isMagnetTooStrong() {
        return magnetStatus() == MagnetStatus::TooHigh;
    }
    bool As5047pSpi::isMagnetTooWeak() {
        return magnetStatus() == MagnetStatus::TooLow;
    }

}  // namespace ungula::encoder::drivers
