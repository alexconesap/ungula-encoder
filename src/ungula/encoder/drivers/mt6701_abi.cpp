// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "mt6701_abi.h"

namespace ungula::encoder::drivers
{

    Mt6701Abi::Mt6701Abi(const char *name, ungula::hal::quadrature::IDecoder &decoder, uint16_t resolution)
            : IEncoder("MT6701", name, resolution)
            , decoder_(decoder)
    {
    }

    bool Mt6701Abi::begin()
    {
        isInitialized_ = true;
        applyDirection(direction_);
        signFactor_ = (direction_ == Direction::ClockWise) ? -1 : 1;
        lastReadCount_ = decoder_.count();
        clearLastError();
        return true;
    }

    bool Mt6701Abi::isConnected()
    {
        // ABI is one-way: the chip emits pulses, we count them. There
        // is no per-transaction handshake, so "connected" just means
        // the decoder is up. Higher-level health comes from watching
        // the count change over time on the host side.
        return isInitialized_;
    }

    bool Mt6701Abi::isFunctional()
    {
        return readStatus() == Status::Ok;
    }

    Status Mt6701Abi::readStatus()
    {
        if (!isInitialized_) {
            setStatus(Error::NotInitialized);
            return Status::Error;
        }
        return Status::Ok;
    }

    float Mt6701Abi::readPosition()
    {
        clearLastError();
        if (!isInitialized_) {
            setStatus(Error::NotInitialized);
            return 0.0f; // 0 here, not NaN — count of 0 is valid.
        }
        lastReadCount_ = decoder_.count();
        return static_cast<float>(signFactor_ * lastReadCount_);
    }

    float Mt6701Abi::position() const
    {
        return static_cast<float>(signFactor_ * lastReadCount_);
    }

    bool Mt6701Abi::resetPosition(uint16_t initial_position)
    {
        if (!isInitialized_) {
            setStatus(Error::NotInitialized);
            return false;
        }
        decoder_.reset(static_cast<int32_t>(initial_position));
        lastReadCount_ = static_cast<int32_t>(initial_position);
        return true;
    }

} // namespace ungula::encoder::drivers
