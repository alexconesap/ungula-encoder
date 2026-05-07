// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include <ungula/encoder/drivers/encoder_fake.h>
#include <ungula/encoder/i_encoder.h>
#include <ungula/hal/multiplexer/drivers/multiplexer_fake.h>

namespace {

    using ungula::encoder::Direction;
    using ungula::encoder::Error;
    using ungula::encoder::IEncoder;
    using ungula::encoder::MagnetStatus;
    using ungula::encoder::Status;
    using ungula::encoder::drivers::EncoderFake;
    using ungula::hal::multiplexer::drivers::MultiplexerFake;

    // ----------------------------------------------------------------------
    // Interface-stability check
    //
    // Goal stated by the project owner: "ensure that a minimal change to the
    // interface that is not properly implemented fails". Every pure virtual
    // on `IEncoder` is touched here through a `IEncoder*`. Renaming or
    // changing a signature without updating EncoderFake makes the file
    // fail to compile.
    // ----------------------------------------------------------------------

    TEST(IEncoderContract, FakeImplementsEveryPureVirtual) {
        EncoderFake fake;
        IEncoder* api = static_cast<IEncoder*>(&fake);
        EXPECT_TRUE(api->begin(0, ungula::encoder::ENCODER_NO_DIRECTION_PIN));
        EXPECT_TRUE(api->isFunctional());
        EXPECT_TRUE(api->isConnected());
        EXPECT_FALSE(std::isnan(api->readPosition()));
        EXPECT_FLOAT_EQ(api->angleFromPosition(360, 1.0f), 360.0f);
        EXPECT_FLOAT_EQ(api->angleFromCurrentPosition(1.0f), 0.0f);
        EXPECT_TRUE(api->resetPosition(0));
        EXPECT_EQ(api->getEncoderResolution(), 4096);
        EXPECT_EQ(api->readStatus(), Status::Ok);
        EXPECT_TRUE(api->setDirection(Direction::CounterClockWise));
        EXPECT_EQ(api->getDirection(), Direction::CounterClockWise);
        EXPECT_TRUE(api->isMagnetFound());
        EXPECT_FALSE(api->isMagnetTooStrong());
        EXPECT_FALSE(api->isMagnetTooWeak());
        EXPECT_EQ(api->magnetStatus(), MagnetStatus::Ok);
        EXPECT_TRUE(api->setWatchDog(true));
        EXPECT_TRUE(api->isWatchDogEnabled());
    }

    // ----------------------------------------------------------------------
    // Multiplexer is OPTIONAL — direct-connect deployment
    // ----------------------------------------------------------------------

    TEST(IEncoderOptionalMux, DirectConnectEncoderHasNoMultiplexer) {
        EncoderFake fake;  // multiplexer defaults to nullptr
        EXPECT_FALSE(fake.hasMultiplexer());
    }

    TEST(IEncoderOptionalMux, DirectConnectReadPositionDoesNotCallSelectChannel) {
        EncoderFake fake;
        fake.begin(0, 0xFF);
        fake.setScriptedPosition(123.0f);

        EXPECT_FLOAT_EQ(fake.readPosition(), 123.0f);
        // No multiplexer to count, but the call must not crash and the
        // position must come through.
        EXPECT_EQ(fake.readPositionCallCount(), 1U);
    }

    TEST(IEncoderOptionalMux, DirectConnectReadFailsWhenNotInitialised) {
        EncoderFake fake;
        // begin() never called → readPosition must return NaN via the
        // selectMultiplexerChannel guard, even with no multiplexer.
        const float r = fake.readPosition();
        EXPECT_TRUE(std::isnan(r));
        EXPECT_EQ(fake.getLastError(), Error::NotInitialized);
    }

    // ----------------------------------------------------------------------
    // Multiplexer is OPTIONAL — multiplexed deployment
    // ----------------------------------------------------------------------

    TEST(IEncoderWithMux, MultiplexedEncoderReportsMultiplexer) {
        MultiplexerFake mux;
        EncoderFake fake("vertical", &mux);
        EXPECT_TRUE(fake.hasMultiplexer());
    }

    TEST(IEncoderWithMux, ReadPositionSelectsTheConfiguredChannel) {
        MultiplexerFake mux;
        mux.begin();

        EncoderFake fake("vertical", &mux);
        fake.begin(/*channel=*/3, ungula::encoder::ENCODER_NO_DIRECTION_PIN);
        fake.setScriptedPosition(42.0f);

        EXPECT_FLOAT_EQ(fake.readPosition(), 42.0f);
        EXPECT_EQ(mux.lastChannel(), 3U);
        EXPECT_GE(mux.selectCallCount(), 1U);
    }

    TEST(IEncoderWithMux, MultiplexerFailurePropagatesAsError) {
        MultiplexerFake mux;
        mux.begin();
        mux.setSelectAlwaysFails(true);

        EncoderFake fake("vertical", &mux);
        fake.begin(2, ungula::encoder::ENCODER_NO_DIRECTION_PIN);

        const float r = fake.readPosition();
        EXPECT_TRUE(std::isnan(r));
        EXPECT_EQ(fake.getLastError(), Error::MultiplexerError);
    }

    TEST(IEncoderWithMux, BackToBackReadsHitMuxOnceDueToCache) {
        MultiplexerFake mux;
        mux.begin();

        EncoderFake fake("vertical", &mux);
        fake.begin(1, ungula::encoder::ENCODER_NO_DIRECTION_PIN);
        fake.setScriptedPosition(7.0f);

        // First read forces a selectChannel; the next two are cache hits.
        for (int i = 0; i < 3; ++i) {
            (void)fake.readPosition();
        }
        EXPECT_EQ(mux.selectCallCount(), 1U);
        EXPECT_EQ(fake.readPositionCallCount(), 3U);
    }

    // ----------------------------------------------------------------------
    // Logging toggle — same shape as IMultiplexer
    // ----------------------------------------------------------------------

    TEST(IEncoderLogging, DefaultsOff) {
        EncoderFake fake;
        EXPECT_FALSE(fake.isLoggingEnabled());
    }

    TEST(IEncoderLogging, EnableDisableFlipsFlag) {
        EncoderFake fake;
        fake.enableLogging();
        EXPECT_TRUE(fake.isLoggingEnabled());
        fake.disableLogging();
        EXPECT_FALSE(fake.isLoggingEnabled());
    }

    // ----------------------------------------------------------------------
    // Status / error mapping
    // ----------------------------------------------------------------------

    TEST(IEncoderStatus, StatusToStrIncludesModelAndName) {
        EncoderFake fake("vertical");
        const char* s = fake.statusToStr();
        ASSERT_NE(s, nullptr);
        // Format is "[FAKE vertical @ 0xNN:N] ..."
        EXPECT_NE(strstr(s, "FAKE"), nullptr);
        EXPECT_NE(strstr(s, "vertical"), nullptr);
    }

    TEST(IEncoderStatus, GetLastErrorAsStrCoversEveryEnumValue) {
        EncoderFake fake;
        const Error allValues[] = {
                Error::None,            Error::NotInitialized,
                Error::BeginFailed,     Error::NotConnected,
                Error::MultiplexerError, Error::MagnetNotDetected,
                Error::MagnetError,     Error::MagnetErrorHigh,
                Error::MagnetErrorLow,  Error::I2CReadError,
                Error::I2CWriteError,
        };
        for (Error e : allValues) {
            fake.setStatus(e);
            const char* msg = fake.getLastErrorAsStr();
            ASSERT_NE(msg, nullptr);
            EXPECT_GT(strlen(msg), 0U) << "missing message for enum " << static_cast<int>(e);
        }
    }

    // ----------------------------------------------------------------------
    // Angle helpers
    // ----------------------------------------------------------------------

    TEST(IEncoderAngle, AngleFromPositionDividesByCalibration) {
        EncoderFake fake;
        EXPECT_FLOAT_EQ(fake.angleFromPosition(0, 11.377f), 0.0f);
        EXPECT_FLOAT_EQ(fake.angleFromPosition(11377, 1.0f), 11377.0f);
        // 4093 raw counts → 360° on the original Rachel rig. This is the
        // calibration callers actually use, captured here to lock the
        // divisor convention into the contract.
        EXPECT_NEAR(fake.angleFromPosition(4093, 4093.0f / 360.0f), 360.0f, 0.001f);
    }

    TEST(IEncoderAngle, AngleFromCurrentPositionTracksScriptedPosition) {
        EncoderFake fake;
        fake.setScriptedPosition(180.0f);
        EXPECT_FLOAT_EQ(fake.angleFromCurrentPosition(1.0f), 180.0f);
    }

    // ----------------------------------------------------------------------
    // Reset and direction
    // ----------------------------------------------------------------------

    TEST(IEncoderReset, ResetCountsAndUpdatesPosition) {
        EncoderFake fake;
        fake.begin(0, ungula::encoder::ENCODER_NO_DIRECTION_PIN);
        EXPECT_TRUE(fake.resetPosition(123));
        EXPECT_EQ(fake.resetCallCount(), 1U);
        EXPECT_FLOAT_EQ(fake.readPosition(), 123.0f);
    }

    TEST(IEncoderDirection, RoundTripDirection) {
        EncoderFake fake;
        EXPECT_TRUE(fake.setDirection(Direction::CounterClockWise));
        EXPECT_EQ(fake.getDirection(), Direction::CounterClockWise);
        EXPECT_TRUE(fake.setDirection(Direction::ClockWise));
        EXPECT_EQ(fake.getDirection(), Direction::ClockWise);
    }

    // ----------------------------------------------------------------------
    // Magnet status
    // ----------------------------------------------------------------------

    TEST(IEncoderMagnet, MagnetStatusFlowsToBoolHelpers) {
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

}  // namespace
