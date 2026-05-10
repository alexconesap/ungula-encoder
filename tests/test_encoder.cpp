// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#include <ungula/encoder/drivers/encoder_fake.h>
#include <ungula/encoder/drivers/i2c_encoder_fake.h>
#include <ungula/encoder/i_encoder.h>
#include <ungula/hal/multiplexer/drivers/multiplexer_fake.h>

namespace
{

    using ungula::encoder::Direction;
    using ungula::encoder::Error;
    using ungula::encoder::IEncoder;
    using ungula::encoder::MagnetStatus;
    using ungula::encoder::Status;
    using ungula::encoder::drivers::EncoderFake;
    using ungula::encoder::drivers::I2cEncoderFake;
    using ungula::hal::multiplexer::drivers::MultiplexerFake;

    // ----------------------------------------------------------------------
    // Interface-stability check
    //
    // Every pure virtual on `IEncoder` must be implemented by the fake.
    // Renaming or changing a signature without updating EncoderFake
    // makes this file fail to compile.
    // ----------------------------------------------------------------------

    TEST(IEncoderContract, FakeImplementsEveryPureVirtual)
    {
        EncoderFake fake;
        IEncoder *api = static_cast<IEncoder *>(&fake);
        EXPECT_TRUE(api->begin());
        EXPECT_TRUE(api->isFunctional());
        EXPECT_TRUE(api->isConnected());
        EXPECT_FALSE(std::isnan(api->readPosition()));
        EXPECT_FALSE(std::isnan(api->position()));
        api->setCalibration(1.0f);
        EXPECT_TRUE(api->hasCalibration());
        EXPECT_FLOAT_EQ(api->calibration(), 1.0f);
        EXPECT_FLOAT_EQ(api->angleFromPosition(360), 360.0f);
        EXPECT_FLOAT_EQ(api->angle(), 0.0f);
        EXPECT_FALSE(std::isnan(api->readAngle()));
        EXPECT_TRUE(api->resetPosition(0));
        EXPECT_EQ(api->getResolution(), 4096);
        EXPECT_EQ(api->readStatus(), Status::Ok);
        EXPECT_TRUE(api->setDirection(Direction::CounterClockWise));
        EXPECT_EQ(api->getDirection(), Direction::CounterClockWise);
        EXPECT_TRUE(api->setDirectionClockWise());
        EXPECT_EQ(api->getDirection(), Direction::ClockWise);
        EXPECT_TRUE(api->setDirectionCounterClockWise());
        EXPECT_EQ(api->getDirection(), Direction::CounterClockWise);
        EXPECT_TRUE(api->isMagnetFound());
        EXPECT_FALSE(api->isMagnetTooStrong());
        EXPECT_FALSE(api->isMagnetTooWeak());
        EXPECT_EQ(api->magnetStatus(), MagnetStatus::Ok);
        EXPECT_TRUE(api->setWatchDog(true));
        EXPECT_TRUE(api->isWatchDogEnabled());
    }

    // ----------------------------------------------------------------------
    // Capabilities — defaults vs override
    // ----------------------------------------------------------------------

    // A bare-bones fake with no overrides — used to verify the safe
    // defaults defined on `IEncoder` itself. ABI / optical drivers
    // inherit this shape.
    class MinimalFake final : public IEncoder {
    public:
        MinimalFake()
                : IEncoder("MIN", "min", 1024)
        {
        }
        bool begin() override
        {
            isInitialized_ = true;
            return true;
        }
        bool isFunctional() override
        {
            return true;
        }
        bool isConnected() override
        {
            return true;
        }
        float readPosition() override
        {
            return 0.0f;
        }
        float position() const override
        {
            return 0.0f;
        }
        bool resetPosition(uint16_t) override
        {
            return true;
        }
        Status readStatus() override
        {
            return Status::Ok;
        }
    };

    TEST(IEncoderCapabilities, DefaultsAreAllFalseAndSafe)
    {
        MinimalFake bare;
        EXPECT_FALSE(bare.hasMagnetSensing());
        EXPECT_FALSE(bare.hasWatchDog());

        // Magnet defaults: "everything is fine" — non-magnetic encoders
        // should never trigger downstream "magnet missing" alarms.
        EXPECT_TRUE(bare.isMagnetFound());
        EXPECT_FALSE(bare.isMagnetTooStrong());
        EXPECT_FALSE(bare.isMagnetTooWeak());
        EXPECT_EQ(bare.magnetStatus(), MagnetStatus::Ok);

        // Watchdog defaults: not supported — set returns false, get reports false.
        EXPECT_FALSE(bare.setWatchDog(true));
        EXPECT_FALSE(bare.isWatchDogEnabled());
    }

    TEST(IEncoderCapabilities, FakeOverridesAdvertiseSupport)
    {
        EncoderFake fake; // overrides hasMagnetSensing / hasWatchDog → true
        EXPECT_TRUE(fake.hasMagnetSensing());
        EXPECT_TRUE(fake.hasWatchDog());
    }

    // ----------------------------------------------------------------------
    // Direction works before begin()
    // ----------------------------------------------------------------------

    TEST(IEncoderDirection, SetBeforeBeginDoesNotPushToHardwareYet)
    {
        EncoderFake fake;
        // Pre-begin: setDirection updates the cached value but must NOT
        // call applyDirection (no hardware to talk to yet).
        EXPECT_TRUE(fake.setDirectionCounterClockWise());
        EXPECT_EQ(fake.getDirection(), Direction::CounterClockWise);
        EXPECT_EQ(fake.applyDirectionCallCount(), 0U);
    }

    TEST(IEncoderDirection, BeginAppliesPreBeginDirectionToHardware)
    {
        EncoderFake fake;
        fake.setDirectionCounterClockWise();
        EXPECT_TRUE(fake.begin());
        // begin() must call applyDirection() so the pre-begin setting
        // lands on the wire — this was the bug we set out to fix.
        EXPECT_EQ(fake.applyDirectionCallCount(), 1U);
        EXPECT_EQ(fake.lastAppliedDirection(), Direction::CounterClockWise);
    }

    TEST(IEncoderDirection, SetAfterBeginPushesImmediately)
    {
        EncoderFake fake;
        fake.begin();
        const uint32_t baseline = fake.applyDirectionCallCount();
        EXPECT_TRUE(fake.setDirectionCounterClockWise());
        EXPECT_EQ(fake.applyDirectionCallCount(), baseline + 1U);
        EXPECT_EQ(fake.lastAppliedDirection(), Direction::CounterClockWise);
    }

    TEST(IEncoderDirection, RoundTripDirection)
    {
        EncoderFake fake;
        EXPECT_TRUE(fake.setDirection(Direction::CounterClockWise));
        EXPECT_EQ(fake.getDirection(), Direction::CounterClockWise);
        EXPECT_TRUE(fake.setDirection(Direction::ClockWise));
        EXPECT_EQ(fake.getDirection(), Direction::ClockWise);
    }

    // ----------------------------------------------------------------------
    // I2C-backed driver multiplexer routing — through I2cEncoderFake
    // ----------------------------------------------------------------------

    TEST(I2cEncoderFakeMux, DirectConnectReportsNoMultiplexer)
    {
        I2cEncoderFake fake; // no multiplexer
        EXPECT_FALSE(fake.hasMultiplexer());
    }

    TEST(I2cEncoderFakeMux, ReadPositionSelectsTheConfiguredChannel)
    {
        MultiplexerFake mux;
        mux.begin();

        I2cEncoderFake fake("vertical", &mux, /*channel=*/3);
        EXPECT_TRUE(fake.begin());
        fake.setScriptedPosition(42.0f);

        EXPECT_FLOAT_EQ(fake.readPosition(), 42.0f);
        EXPECT_EQ(mux.lastChannel(), 3U);
        EXPECT_GE(mux.selectCallCount(), 1U);
    }

    TEST(I2cEncoderFakeMux, MultiplexerFailurePropagatesAsError)
    {
        MultiplexerFake mux;
        mux.begin();
        mux.setSelectAlwaysFails(true);

        I2cEncoderFake fake("vertical", &mux, /*channel=*/2);
        // begin() will fail because the mux can't switch channels — we
        // still want isInitialized_ to stick so subsequent calls report
        // MultiplexerError instead of NotInitialized.
        (void)fake.begin();

        const float r = fake.readPosition();
        EXPECT_TRUE(std::isnan(r));
        EXPECT_EQ(fake.getLastError(), Error::MultiplexerError);
    }

    TEST(I2cEncoderFakeMux, BackToBackReadsHitMuxOnceDueToCache)
    {
        MultiplexerFake mux;
        mux.begin();

        I2cEncoderFake fake("vertical", &mux, /*channel=*/1);
        EXPECT_TRUE(fake.begin());
        fake.setScriptedPosition(7.0f);

        const uint32_t baseline = mux.selectCallCount();
        for (int i = 0; i < 3; ++i) {
            (void)fake.readPosition();
        }
        // begin() did one selectChannel, the three reads should be
        // cache hits on the multiplexer side.
        EXPECT_EQ(mux.selectCallCount(), baseline);
    }

    TEST(I2cEncoderFakeMux, ReadFailsWhenNotInitialised)
    {
        I2cEncoderFake fake;
        const float r = fake.readPosition();
        EXPECT_TRUE(std::isnan(r));
        EXPECT_EQ(fake.getLastError(), Error::NotInitialized);
    }

    // ----------------------------------------------------------------------
    // Logging toggle
    // ----------------------------------------------------------------------

    TEST(IEncoderLogging, DefaultsOff)
    {
        EncoderFake fake;
        EXPECT_FALSE(fake.isLoggingEnabled());
    }

    TEST(IEncoderLogging, EnableDisableFlipsFlag)
    {
        EncoderFake fake;
        fake.enableLogging();
        EXPECT_TRUE(fake.isLoggingEnabled());
        fake.disableLogging();
        EXPECT_FALSE(fake.isLoggingEnabled());
    }

    // ----------------------------------------------------------------------
    // Status / error mapping
    // ----------------------------------------------------------------------

    TEST(IEncoderStatus, StatusToStrIncludesModelAndName)
    {
        EncoderFake fake("vertical");
        const char *s = fake.statusToStr();
        ASSERT_NE(s, nullptr);
        // Default IEncoder prefix is "[MODEL name] ..." — driver overrides
        // can add more (e.g. address + channel for I2C drivers).
        EXPECT_NE(strstr(s, "FAKE"), nullptr);
        EXPECT_NE(strstr(s, "vertical"), nullptr);
    }

    TEST(IEncoderStatus, GetLastErrorAsStrCoversEveryEnumValue)
    {
        EncoderFake fake;
        const Error allValues[] = {
            Error::None,         Error::NotInitialized,   Error::BeginFailed,
            Error::NotConnected, Error::MultiplexerError, Error::MagnetNotDetected,
            Error::MagnetError,  Error::MagnetErrorHigh,  Error::MagnetErrorLow,
            Error::I2CReadError, Error::I2CWriteError,
        };

        for (Error e : allValues) {
            fake.public_setStatus(e);
            const char *msg = fake.getLastErrorAsStr();
            ASSERT_NE(msg, nullptr);
            EXPECT_GT(strlen(msg), 0U) << "missing message for enum " << static_cast<int>(e);
        }
    }

    // ----------------------------------------------------------------------
    // Calibration + angle helpers (unchanged surface)
    // ----------------------------------------------------------------------

    TEST(IEncoderCalibration, DefaultsToUncalibrated)
    {
        EncoderFake fake;
        EXPECT_FALSE(fake.hasCalibration());
        EXPECT_FLOAT_EQ(fake.calibration(), 0.0f);
    }

    TEST(IEncoderCalibration, SetCalibrationFlipsHasCalibrationAndIsReadable)
    {
        EncoderFake fake;
        fake.setCalibration(11.377f);
        EXPECT_TRUE(fake.hasCalibration());
        EXPECT_FLOAT_EQ(fake.calibration(), 11.377f);
    }

    TEST(IEncoderCalibration, ZeroCalibrationKeepsItUncalibrated)
    {
        EncoderFake fake;
        fake.setCalibration(5.0f);
        EXPECT_TRUE(fake.hasCalibration());
        fake.setCalibration(0.0f);
        EXPECT_FALSE(fake.hasCalibration());
    }

    TEST(IEncoderAngle, AngleFromPositionReturnsNanWithoutCalibration)
    {
        EncoderFake fake;
        EXPECT_TRUE(std::isnan(fake.angleFromPosition(360)));
        EXPECT_TRUE(std::isnan(fake.angle()));
        EXPECT_TRUE(std::isnan(fake.readAngle()));
    }

    TEST(IEncoderAngle, AngleFromPositionDividesByStoredCalibration)
    {
        EncoderFake fake;
        fake.setCalibration(1.0f);
        EXPECT_FLOAT_EQ(fake.angleFromPosition(0), 0.0f);
        EXPECT_FLOAT_EQ(fake.angleFromPosition(11377), 11377.0f);

        fake.setCalibration(4093.0f / 360.0f);
        EXPECT_NEAR(fake.angleFromPosition(4093), 360.0f, 0.001f);
    }

    TEST(IEncoderAngle, AngleTracksCachedPositionWithoutIo)
    {
        EncoderFake fake;
        fake.setScriptedPosition(180.0f);
        fake.setCalibration(1.0f);
        const uint32_t before = fake.readPositionCallCount();
        EXPECT_FLOAT_EQ(fake.angle(), 180.0f);
        EXPECT_EQ(fake.readPositionCallCount(), before);
    }

    TEST(IEncoderAngle, ReadAngleDrivesReadPositionAndConvertsToDegrees)
    {
        EncoderFake fake;
        fake.begin();
        fake.setScriptedPosition(720.0f);
        fake.setCalibration(2.0f);
        const uint32_t before = fake.readPositionCallCount();
        EXPECT_FLOAT_EQ(fake.readAngle(), 360.0f);
        EXPECT_EQ(fake.readPositionCallCount(), before + 1U);
    }

    TEST(IEncoderAngle, ReadAngleReturnsNanWhenReadPositionFails)
    {
        EncoderFake fake;
        fake.setCalibration(1.0f);
        EXPECT_TRUE(std::isnan(fake.readAngle()));
    }

    // ----------------------------------------------------------------------
    // Reset
    // ----------------------------------------------------------------------

    TEST(IEncoderReset, ResetCountsAndUpdatesPosition)
    {
        EncoderFake fake;
        fake.begin();
        EXPECT_TRUE(fake.resetPosition(123));
        EXPECT_EQ(fake.resetCallCount(), 1U);
        EXPECT_FLOAT_EQ(fake.readPosition(), 123.0f);
    }

    // ----------------------------------------------------------------------
    // Magnet status
    // ----------------------------------------------------------------------

    TEST(IEncoderMagnet, MagnetStatusFlowsToBoolHelpers)
    {
        EncoderFake fake;
        fake.setMagnetStatus(MagnetStatus::Ok);
        EXPECT_TRUE(fake.isMagnetFound());
        EXPECT_FALSE(fake.isMagnetTooStrong());
        EXPECT_FALSE(fake.isMagnetTooWeak());

        fake.setMagnetStatus(MagnetStatus::TooHigh);
        EXPECT_TRUE(fake.isMagnetFound());
        EXPECT_TRUE(fake.isMagnetTooStrong());

        fake.setMagnetStatus(MagnetStatus::TooLow);
        EXPECT_TRUE(fake.isMagnetTooWeak());

        fake.setMagnetStatus(MagnetStatus::NotFound);
        EXPECT_FALSE(fake.isMagnetFound());
    }

} // namespace
