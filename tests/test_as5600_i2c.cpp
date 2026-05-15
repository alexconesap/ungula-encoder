// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cmath>

#include <ungula/encoder/drivers/as5600_i2c.h>
#include <ungula/encoder/i_encoder.h>
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/hal/multiplexer/drivers/multiplexer_fake.h>

namespace
{

using ungula::encoder::IEncoder;
using ungula::encoder::drivers::As5600I2c;
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

TEST(As5600I2c, IsAValidIEncoder)
{
        I2cMaster bus(0);
        As5600I2c enc("vertical", bus);
        IEncoder *api = static_cast<IEncoder *>(&enc);
        EXPECT_NE(api, nullptr);
}

TEST(As5600I2c, ResolutionIs4096)
{
        I2cMaster bus(0);
        As5600I2c enc("vertical", bus);
        EXPECT_EQ(enc.getResolution(), static_cast<int>(AS5600_RESOLUTION));
}

TEST(As5600I2c, AngleFromPositionMatchesCalibrationDivisor)
{
        I2cMaster bus(0);
        As5600I2c enc("vertical", bus);
        // 4093 raw counts → 360°: real horizontal-encoder calibration.
        enc.setCalibration(4093.0f / 360.0f);
        EXPECT_NEAR(enc.angleFromPosition(4093), 360.0f, 0.001f);
}

TEST(As5600I2c, BeginWithoutMultiplexerOnUnreachableBusFlagsBeginFailed)
{
        I2cMaster bus(0);
        As5600I2c enc("vertical", bus); // no multiplexer
        EXPECT_FALSE(enc.begin());
        EXPECT_EQ(enc.getLastError(), ungula::encoder::Error::BeginFailed);
}

TEST(As5600I2c, BeginWithMultiplexerSelectsTheChannel)
{
        I2cMaster bus(0);
        MultiplexerFake mux;
        mux.begin();
        As5600I2c enc("vertical", bus, &mux, /*channel=*/2);

        // The bus stub will fail isConnected(), so begin() reports
        // BeginFailed — but the multiplexer must still have been asked
        // to switch channels first. That covers the wiring contract.
        (void)enc.begin();
        EXPECT_EQ(mux.lastChannel(), 2U);
        EXPECT_GE(mux.selectCallCount(), 1U);
        EXPECT_EQ(enc.multiplexerChannel(), 2U);
        EXPECT_TRUE(enc.hasMultiplexer());
}

TEST(As5600I2c, ReadPositionWithoutBeginReturnsNan)
{
        I2cMaster bus(0);
        As5600I2c enc("vertical", bus);
        EXPECT_TRUE(std::isnan(enc.readPosition()));
        EXPECT_EQ(enc.getLastError(), ungula::encoder::Error::NotInitialized);
}

TEST(As5600I2c, AddressIsTheChipDefault)
{
        EXPECT_EQ(static_cast<uint8_t>(AS5600_DEFAULT_ADDRESS), 0x36);
}

TEST(As5600I2c, DirectionSetBeforeBeginIsHonoured)
{
        // The whole point of the Phase A refactor: setDirection*()
        // works pre-`begin()` and the value reaches hardware once
        // `begin()` runs. Even though begin() fails (bus stub doesn't
        // ACK), the driver records the direction and the cached
        // accessor reflects it.
        I2cMaster bus(0);
        As5600I2c enc("vertical", bus);
        EXPECT_TRUE(enc.setDirectionCounterClockWise());
        EXPECT_EQ(enc.getDirection(), ungula::encoder::Direction::CounterClockWise);

        (void)enc.begin(); // fails on stub bus; direction state still preserved
        EXPECT_EQ(enc.getDirection(), ungula::encoder::Direction::CounterClockWise);
}

TEST(As5600I2c, CapabilityFlagsAdvertiseMagnetAndWatchDog)
{
        I2cMaster bus(0);
        As5600I2c enc("vertical", bus);
        // AS5600 has both magnet sensing and a watchdog — overrides on
        // the driver must report true.
        EXPECT_TRUE(enc.hasMagnetSensing());
        EXPECT_TRUE(enc.hasWatchDog());
}

} // namespace
