// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <cmath>

#include <ungula/encoder/drivers/as5600_i2c_pwm.h>
#include <ungula/encoder/drivers/as5600_pwm.h>
#include <ungula/encoder/i_encoder.h>
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/hal/pwm_input/drivers/pwm_input_fake.h>

namespace
{

using ungula::encoder::IEncoder;
using ungula::encoder::drivers::As5600I2cPwm;
using ungula::encoder::drivers::As5600Pwm;
using ungula::hal::i2c::I2cMaster;
using ungula::hal::pwm_input::drivers::PwmInputFake;

// For an AS5600 frame of 4351 chip clocks with a 128-clock preamble,
// the helper resolves angle samples like this regardless of the
// physical clock rate. Pick `period_us` arbitrarily; the ratio is
// what matters. Use 8700 us for a ~115 Hz frame.
//
// For raw_angle = R, the high time fraction is (128 + R) / 4351, so
// `highUs = round(periodUs * (128 + R) / 4351)`.
constexpr uint32_t kPeriodUs = 8'700;
constexpr uint32_t highForRaw(uint16_t raw)
{
        return static_cast<uint32_t>(
            (static_cast<uint64_t>(kPeriodUs) * (128U + raw) + (4351U / 2U)) / 4351U);
}

// ============================================================
//  As5600Pwm
// ============================================================

TEST(As5600Pwm, IsAValidIEncoder)
{
        PwmInputFake pwm;
        pwm.begin(34);
        As5600Pwm enc("vertical", pwm);
        IEncoder *api = static_cast<IEncoder *>(&enc);
        EXPECT_NE(api, nullptr);
}

TEST(As5600Pwm, ResolutionIs4096)
{
        PwmInputFake pwm;
        As5600Pwm enc("vertical", pwm);
        EXPECT_EQ(enc.getResolution(), 4096);
}

TEST(As5600Pwm, BeginSucceedsEvenWithoutSample)
{
        PwmInputFake pwm;
        pwm.begin(34);
        As5600Pwm enc("vertical", pwm);
        EXPECT_TRUE(enc.begin());
}

TEST(As5600Pwm, ReadPositionWithoutSampleReportsNotConnected)
{
        PwmInputFake pwm;
        pwm.begin(34);
        As5600Pwm enc("vertical", pwm);
        enc.begin();
        EXPECT_TRUE(std::isnan(enc.readPosition()));
        EXPECT_EQ(enc.getLastError(), ungula::encoder::Error::NotConnected);
}

TEST(As5600Pwm, FirstSampleZeroesOutPosition)
{
        PwmInputFake pwm;
        pwm.begin(34);
        As5600Pwm enc("vertical", pwm);
        enc.begin();

        pwm.injectSample(highForRaw(/*raw=*/2'000), kPeriodUs);
        const float p = enc.readPosition();
        EXPECT_FALSE(std::isnan(p));
        EXPECT_NEAR(p, 0.0f, 0.001f);
}

TEST(As5600Pwm, AngleAccumulatesAcrossFrames)
{
        PwmInputFake pwm;
        pwm.begin(34);
        As5600Pwm enc("vertical", pwm);
        enc.setDirectionCounterClockWise(); // positive on incrementing raw
        enc.begin();

        pwm.injectSample(highForRaw(1'000), kPeriodUs);
        EXPECT_NEAR(enc.readPosition(), 0.0f, 1.0f);
        pwm.injectSample(highForRaw(1'500), kPeriodUs);
        EXPECT_NEAR(enc.readPosition(), 500.0f, 2.0f);
        pwm.injectSample(highForRaw(2'000), kPeriodUs);
        EXPECT_NEAR(enc.readPosition(), 1'000.0f, 2.0f);
}

TEST(As5600Pwm, WrapAroundIsHandledCorrectly)
{
        PwmInputFake pwm;
        pwm.begin(34);
        As5600Pwm enc("vertical", pwm);
        enc.setDirectionCounterClockWise();
        enc.begin();

        pwm.injectSample(highForRaw(4'080), kPeriodUs);
        enc.readPosition();
        // Step forward across the 4095→0 boundary.
        pwm.injectSample(highForRaw(20), kPeriodUs);
        const float p = enc.readPosition();
        // Diff should be +36 (4096 - 4080 + 20), not -4060.
        EXPECT_NEAR(p, 36.0f, 2.0f);
}

TEST(As5600Pwm, StaleSampleSurfacesAsNotConnected)
{
        PwmInputFake pwm;
        pwm.begin(34);
        As5600Pwm enc("vertical", pwm);
        enc.begin();
        pwm.injectSample(highForRaw(1'000), kPeriodUs);
        enc.readPosition();
        // Mark the sample as 100 ms old — past the 50 ms default.
        pwm.setSampleAgeUs(100'000);
        EXPECT_TRUE(std::isnan(enc.readPosition()));
        EXPECT_EQ(enc.getLastError(), ungula::encoder::Error::NotConnected);
}

TEST(As5600Pwm, CapabilitiesDefaultFalse)
{
        PwmInputFake pwm;
        As5600Pwm enc("vertical", pwm);
        EXPECT_FALSE(enc.hasMagnetSensing());
        EXPECT_FALSE(enc.hasWatchDog());
}

TEST(As5600Pwm, DirectionSetBeforeBeginIsHonoured)
{
        PwmInputFake pwm;
        pwm.begin(34);
        As5600Pwm enc("vertical", pwm);
        enc.setDirectionCounterClockWise();
        EXPECT_EQ(enc.getDirection(), ungula::encoder::Direction::CounterClockWise);
        enc.begin();
        EXPECT_EQ(enc.getDirection(), ungula::encoder::Direction::CounterClockWise);
}

TEST(As5600Pwm, IsrUpdatesAccumulateWithoutPolling)
{
        PwmInputFake pwm;
        pwm.begin(34);
        As5600Pwm enc("vertical", pwm);
        enc.setDirectionCounterClockWise(); // positive on incrementing raw
        enc.begin();
        enc.enableIsrUpdates();
        EXPECT_TRUE(enc.isIsrUpdatesEnabled());

        // Each `triggerSample()` mimics one PWM frame landing in the ISR.
        // The encoder's callback runs the wrap-around math, so
        // `position()` reflects the latest cumulative count without any
        // host-side polling between frames.
        pwm.triggerSample(highForRaw(1'000), kPeriodUs); // first sample → calibrates zero
        EXPECT_FLOAT_EQ(enc.position(), 0.0f);

        pwm.triggerSample(highForRaw(1'500), kPeriodUs);
        EXPECT_NEAR(enc.position(), 500.0f, 2.0f);

        pwm.triggerSample(highForRaw(2'000), kPeriodUs);
        EXPECT_NEAR(enc.position(), 1'000.0f, 2.0f);

        // `readPosition()` in ISR mode is just a snapshot read — no
        // decode, but the staleness gate still runs.
        EXPECT_NEAR(enc.readPosition(), 1'000.0f, 2.0f);
}

TEST(As5600Pwm, IsrPathHandlesWrapAround)
{
        PwmInputFake pwm;
        pwm.begin(34);
        As5600Pwm enc("vertical", pwm);
        enc.setDirectionCounterClockWise();
        enc.begin();
        enc.enableIsrUpdates();

        pwm.triggerSample(highForRaw(4'080), kPeriodUs);
        pwm.triggerSample(highForRaw(20), kPeriodUs);
        EXPECT_NEAR(enc.position(), 36.0f, 2.0f);
}

TEST(As5600Pwm, DisableIsrUpdatesFallsBackToPolling)
{
        PwmInputFake pwm;
        pwm.begin(34);
        As5600Pwm enc("vertical", pwm);
        enc.setDirectionCounterClockWise();
        enc.begin();
        enc.enableIsrUpdates();
        pwm.triggerSample(highForRaw(1'000), kPeriodUs); // calibrate zero
        pwm.triggerSample(highForRaw(1'500), kPeriodUs);
        EXPECT_NEAR(enc.position(), 500.0f, 2.0f);

        enc.disableIsrUpdates();
        EXPECT_FALSE(enc.isIsrUpdatesEnabled());
        // After disarming, the next `triggerSample()` does fire the
        // callback — but the encoder unsubscribed, so the counter
        // stops moving until the host polls.
        pwm.triggerSample(highForRaw(2'000), kPeriodUs);
        EXPECT_NEAR(enc.position(), 500.0f, 2.0f);
        // Polling resumes the count, picking up where the ISR left off.
        EXPECT_NEAR(enc.readPosition(), 1'000.0f, 2.0f);
}

// ============================================================
//  As5600I2cPwm
// ============================================================

TEST(As5600I2cPwm, IsAValidIEncoderAndAs5600I2c)
{
        I2cMaster bus(0);
        PwmInputFake pwm;
        pwm.begin(34);
        As5600I2cPwm enc("vertical", bus, pwm);
        IEncoder *api = static_cast<IEncoder *>(&enc);
        EXPECT_NE(api, nullptr);
        EXPECT_TRUE(enc.hasMagnetSensing()); // inherited override
        EXPECT_TRUE(enc.hasWatchDog());
}

TEST(As5600I2cPwm, ReadPositionUsesPwmAndIgnoresI2cBus)
{
        // Bus stub never ACKs; if readPosition was going through I2C it
        // would return NaN. Going through PWM, the sample drives it.
        I2cMaster bus(0);
        PwmInputFake pwm;
        pwm.begin(34);
        As5600I2cPwm enc("vertical", bus, pwm);
        enc.setDirectionCounterClockWise();
        // begin() will fail because the bus stub doesn't ACK isConnected().
        // That's fine — we still want to verify the PWM read path is
        // independent.
        (void)enc.begin();
        // The driver records initialised even when begin() fails the I2C
        // probe — it sets BeginFailed. Force the init flag back on so we
        // can isolate the PWM read path.
        // (In real use the host would either retry begin() once the bus
        // is up, or accept I2C-degraded mode by skipping it.)
        pwm.injectSample(highForRaw(1'000), kPeriodUs);
        // Even with a failed I2C probe, calling readPosition() goes through
        // PWM; but isInitialized_ is set inside the parent's begin(). The
        // base sets it before probing, and our override does not reset it.
        EXPECT_FALSE(std::isnan(enc.readPosition()));
}

TEST(As5600I2cPwm, ReadFailsWithoutSample)
{
        I2cMaster bus(0);
        PwmInputFake pwm;
        pwm.begin(34);
        As5600I2cPwm enc("vertical", bus, pwm);
        (void)enc.begin();
        EXPECT_TRUE(std::isnan(enc.readPosition()));
        EXPECT_EQ(enc.getLastError(), ungula::encoder::Error::NotConnected);
}

} // namespace
