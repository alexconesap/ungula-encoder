// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cmath>

#include <ungula/encoder/drivers/as5047p_spi.h>
#include <ungula/encoder/drivers/ma730_spi.h>
#include <ungula/encoder/drivers/mt6835_spi.h>
#include <ungula/encoder/i_encoder.h>
#include <ungula/hal/spi/spi_master.h>

namespace
{

    using ungula::encoder::IEncoder;
    using ungula::encoder::drivers::As5047pSpi;
    using ungula::encoder::drivers::Ma730Spi;
    using ungula::encoder::drivers::Mt6835Spi;
    using ungula::hal::spi::SpiMaster;

    // The SPI host stub never completes a transfer, so on the host side
    // every read fails. Tests therefore verify boundary behaviour: type
    // identity, capability flags, error reporting, and that begin() does
    // not crash even without a working bus. The full hardware path is
    // exercised on real targets.

    // ---- AS5047P --------------------------------------------------------

    TEST(As5047pSpi, IsAValidIEncoder)
    {
        SpiMaster bus;
        As5047pSpi enc("vert", bus);
        IEncoder *api = static_cast<IEncoder *>(&enc);
        EXPECT_NE(api, nullptr);
        EXPECT_EQ(enc.getResolution(), 16384);
    }

    TEST(As5047pSpi, ReadFailsOnUnreachableBus)
    {
        SpiMaster bus;
        As5047pSpi enc("vert", bus);
        enc.begin();
        EXPECT_TRUE(std::isnan(enc.readPosition()));
    }

    TEST(As5047pSpi, AdvertisesMagnetSensingButNoWatchdog)
    {
        SpiMaster bus;
        As5047pSpi enc("vert", bus);
        EXPECT_TRUE(enc.hasMagnetSensing());
        EXPECT_FALSE(enc.hasWatchDog());
    }

    TEST(As5047pSpi, DirectionSetBeforeBeginPersists)
    {
        SpiMaster bus;
        As5047pSpi enc("vert", bus);
        EXPECT_TRUE(enc.setDirectionCounterClockWise());
        enc.begin();
        EXPECT_EQ(enc.getDirection(), ungula::encoder::Direction::CounterClockWise);
    }

    // ---- MA730 ----------------------------------------------------------

    TEST(Ma730Spi, IsAValidIEncoder)
    {
        SpiMaster bus;
        Ma730Spi enc("vert", bus);
        IEncoder *api = static_cast<IEncoder *>(&enc);
        EXPECT_NE(api, nullptr);
        EXPECT_EQ(enc.getResolution(), 16384);
    }

    TEST(Ma730Spi, NoMagnetSensingByDefault)
    {
        SpiMaster bus;
        Ma730Spi enc("vert", bus);
        EXPECT_FALSE(enc.hasMagnetSensing());
        EXPECT_FALSE(enc.hasWatchDog());
    }

    TEST(Ma730Spi, ReadFailsOnUnreachableBus)
    {
        SpiMaster bus;
        Ma730Spi enc("vert", bus);
        enc.begin();
        EXPECT_TRUE(std::isnan(enc.readPosition()));
    }

    // ---- MT6835 ---------------------------------------------------------

    TEST(Mt6835Spi, IsAValidIEncoder)
    {
        SpiMaster bus;
        Mt6835Spi enc("vert", bus);
        IEncoder *api = static_cast<IEncoder *>(&enc);
        EXPECT_NE(api, nullptr);
        EXPECT_EQ(enc.getResolution(), static_cast<int>(ungula::encoder::drivers::MT6835_RESOLUTION));
    }

    TEST(Mt6835Spi, AdvertisesMagnetSensing)
    {
        SpiMaster bus;
        Mt6835Spi enc("vert", bus);
        EXPECT_TRUE(enc.hasMagnetSensing());
    }

    TEST(Mt6835Spi, ReadFailsOnUnreachableBus)
    {
        SpiMaster bus;
        Mt6835Spi enc("vert", bus);
        enc.begin();
        EXPECT_TRUE(std::isnan(enc.readPosition()));
    }

    TEST(Mt6835Spi, ResolutionMatches21Bits)
    {
        EXPECT_EQ(static_cast<int>(ungula::encoder::drivers::MT6835_RESOLUTION), 1 << 21);
    }

} // namespace
