// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <ungula/encoder/drivers/mt6701_abi.h>
#include <ungula/encoder/i_encoder.h>
#include <ungula/hal/quadrature/drivers/decoder_fake.h>

namespace {

    using ungula::encoder::IEncoder;
    using ungula::encoder::drivers::Mt6701Abi;
    using ungula::hal::quadrature::drivers::DecoderFake;

    TEST(Mt6701Abi, IsAValidIEncoder) {
        DecoderFake dec;
        dec.begin(34, 35);
        Mt6701Abi enc("rotor", dec);
        IEncoder* api = static_cast<IEncoder*>(&enc);
        EXPECT_NE(api, nullptr);
        EXPECT_EQ(enc.getResolution(), 4096);
    }

    TEST(Mt6701Abi, NoMagnetSensingByDefault) {
        DecoderFake dec;
        Mt6701Abi enc("rotor", dec);
        EXPECT_FALSE(enc.hasMagnetSensing());
        EXPECT_FALSE(enc.hasWatchDog());
    }

    TEST(Mt6701Abi, ReadPositionMirrorsDecoderCount) {
        DecoderFake dec;
        dec.begin(34, 35);
        Mt6701Abi enc("rotor", dec);
        enc.setDirectionCounterClockWise();  // sign = +1
        enc.begin();

        dec.tick(+10);
        EXPECT_FLOAT_EQ(enc.readPosition(), 10.0f);
        dec.tick(+25);
        EXPECT_FLOAT_EQ(enc.readPosition(), 35.0f);
        dec.tick(-100);
        EXPECT_FLOAT_EQ(enc.readPosition(), -65.0f);
    }

    TEST(Mt6701Abi, ClockWiseFlipsTheSign) {
        DecoderFake dec;
        dec.begin(34, 35);
        Mt6701Abi enc("rotor", dec);
        enc.setDirectionClockWise();
        enc.begin();
        dec.setCount(50);
        EXPECT_FLOAT_EQ(enc.readPosition(), -50.0f);
    }

    TEST(Mt6701Abi, ResetPositionRoutedToDecoder) {
        DecoderFake dec;
        dec.begin(34, 35);
        Mt6701Abi enc("rotor", dec);
        enc.begin();
        dec.tick(99);
        EXPECT_TRUE(enc.resetPosition(0));
        EXPECT_EQ(dec.count(), 0);
        EXPECT_EQ(dec.resetCallCount(), 1U);
    }

    TEST(Mt6701Abi, CustomResolutionPropagates) {
        DecoderFake dec;
        Mt6701Abi enc("rotor", dec, /*resolution=*/16384);
        EXPECT_EQ(enc.getResolution(), 16384);
    }

    TEST(Mt6701Abi, DirectionSetBeforeBeginIsHonoured) {
        DecoderFake dec;
        dec.begin(34, 35);
        Mt6701Abi enc("rotor", dec);
        enc.setDirectionClockWise();
        enc.begin();
        // CW means positive decoder counts surface as negative angle.
        dec.setCount(7);
        EXPECT_FLOAT_EQ(enc.readPosition(), -7.0f);
    }

}  // namespace
