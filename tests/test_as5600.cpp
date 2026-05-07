// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cmath>

#include <ungula/encoder/drivers/as5600.h>
#include <ungula/encoder/i_encoder.h>
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/hal/multiplexer/drivers/multiplexer_fake.h>

namespace {

    using ungula::encoder::IEncoder;
    using ungula::encoder::drivers::AS5600;
    using ungula::encoder::drivers::AS5600_DEFAULT_ADDRESS;
    using ungula::encoder::drivers::AS5600_RESOLUTION;
    using ungula::hal::i2c::I2cMaster;
    using ungula::hal::multiplexer::drivers::MultiplexerFake;

    // The host I2cMaster is a no-op stub: every transfer returns false
    // unless a real platform backend is linked. These tests therefore
    // verify behaviour at the boundary that the driver controls — what
    // happens when the wire is unreachable, that the driver reports the
    // correct error, and that the multiplexer-optional contract holds.
    // The full hardware path is exercised only on the device.

    TEST(As5600, IsAValidIEncoder) {
        I2cMaster bus(0);
        AS5600 enc("vertical", bus);
        IEncoder* api = static_cast<IEncoder*>(&enc);
        EXPECT_NE(api, nullptr);
    }

    TEST(As5600, ResolutionIs4096) {
        I2cMaster bus(0);
        AS5600 enc("vertical", bus);
        EXPECT_EQ(enc.getEncoderResolution(), static_cast<int>(AS5600_RESOLUTION));
    }

    TEST(As5600, AngleFromPositionMatchesCalibrationDivisor) {
        I2cMaster bus(0);
        AS5600 enc("vertical", bus);
        // 4093 raw counts → 360°: real horizontal-encoder calibration.
        EXPECT_NEAR(enc.angleFromPosition(4093, 4093.0f / 360.0f), 360.0f, 0.001f);
    }

    TEST(As5600, BeginWithoutMultiplexerOnUnreachableBusFlagsBeginFailed) {
        I2cMaster bus(0);
        AS5600 enc("vertical", bus);  // no multiplexer
        EXPECT_FALSE(enc.begin(0, ungula::encoder::ENCODER_NO_DIRECTION_PIN));
        EXPECT_EQ(enc.getLastError(), ungula::encoder::Error::BeginFailed);
    }

    TEST(As5600, BeginWithMultiplexerSelectsTheChannel) {
        I2cMaster bus(0);
        MultiplexerFake mux;
        mux.begin();
        AS5600 enc("vertical", bus, &mux);

        // The bus stub will fail isConnected(), so begin() reports
        // BeginFailed — but the multiplexer must still have been asked
        // to switch channels first. That covers the wiring contract.
        (void)enc.begin(/*channel=*/2, ungula::encoder::ENCODER_NO_DIRECTION_PIN);
        EXPECT_EQ(mux.lastChannel(), 2U);
        EXPECT_GE(mux.selectCallCount(), 1U);
    }

    TEST(As5600, ReadPositionWithoutBeginReturnsNan) {
        I2cMaster bus(0);
        AS5600 enc("vertical", bus);
        EXPECT_TRUE(std::isnan(enc.readPosition()));
        EXPECT_EQ(enc.getLastError(), ungula::encoder::Error::NotInitialized);
    }

    TEST(As5600, AddressIsTheChipDefault) {
        I2cMaster bus(0);
        AS5600 enc("vertical", bus);
        // Address is set during begin(), but the constant is part of
        // the public contract; lock it.
        EXPECT_EQ(static_cast<uint8_t>(AS5600_DEFAULT_ADDRESS), 0x36);
    }

}  // namespace
