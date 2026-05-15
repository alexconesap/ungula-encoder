// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "ungula/encoder/i_encoder.h"
#include "ungula/hal/pwm_input/i_pwm_input.h"

/// @brief AS5600 12-bit magnetic rotary encoder — PWM-output transport.
///
/// In PWM mode the chip outputs a single-pin PWM signal where the high
/// time within each frame encodes the angle. No I2C bus is needed —
/// great for projects that have only a free input pin to spare.
///
/// Frame format (datasheet — values in chip clock counts):
///   - Total frame period: 4351 counts.
///   - High preamble: 128 counts (always high).
///   - Data:           raw_angle counts (also high, 0..4095).
///   - Trailer:        rest of the frame (low).
///
/// Decoded angle: `raw_angle = round(high_us / period_us * 4351) - 128`,
/// clamped to [0, 4095].
///
/// Capabilities through PWM alone:
///   - No magnet status (datasheet exposes that over I2C only). Reported
///     as the safe default ("OK / present") via `IEncoder` defaults; if
///     you need real magnet readings, use `As5600I2cPwm`.
///   - No watchdog control (I2C only).
///
/// All wiring (PWM input source, optional DIR pin) is captured at
/// construction. The `IPwmInput` is borrowed — the host application
/// installs and begins it before constructing the encoder.

namespace ungula::encoder::drivers
{

constexpr uint16_t AS5600_PWM_FRAME_COUNTS = 4351; // datasheet
constexpr uint16_t AS5600_PWM_PREAMBLE_COUNTS = 128;

class As5600Pwm final : public IEncoder {
    public:
        /// @param name           Caller-chosen tag, e.g. "vertical".
        /// @param pwm            Already-begun PWM input. Borrowed.
        /// @param directionPin   MCU pin wired to the chip's DIR
        ///                       input, or `ENCODER_NO_DIRECTION_PIN`.
        As5600Pwm(const char *name, ungula::hal::pwm_input::IPwmInput &pwm,
                  uint8_t directionPin = ENCODER_NO_DIRECTION_PIN);

        // ---- Driver contract ----
        bool begin() override;
        bool isFunctional() override;
        bool isConnected() override;

        float readPosition() override;
        float position() const override;

        bool resetPosition(uint16_t initial_position) override;
        Status readStatus() override;

        uint8_t directionPin() const
        {
                return directionPin_;
        }

        /// @brief Stale-signal threshold. If the last edge is older
        ///        than this, `readPosition()` returns NaN and sets
        ///        `NotConnected`. Default 50 ms (~6 frames at the
        ///        slowest 115 Hz mode).
        void setStaleThresholdUs(uint32_t us)
        {
                staleThresholdUs_ = us;
        }
        uint32_t staleThresholdUs() const
        {
                return staleThresholdUs_;
        }

        // ---- ISR-driven update path ------------------------------------
        //
        // The default consumption pattern is "host calls
        // `readPosition()` periodically and the driver folds wrap-
        // around into the cumulative count". For consumers that
        // want zero-latency angle updates without a polling task,
        // `enableIsrUpdates()` registers an ISR-context callback on
        // the underlying `IPwmInput` that runs the wrap-around math
        // every period. After enabling, `position()` reflects the
        // latest cumulative count without any host-side action;
        // `readPosition()` returns the same value cheaply (no
        // decoding) so consumers can keep using the standard
        // `IEncoder` surface.
        //
        // The callback runs under the same constraints as a GPIO
        // ISR: short, no logging, no I2C / SPI traffic, IRAM-safe.
        // It only touches the cumulative-position state on this
        // instance, never the I2C side. Magnet status / watchdog
        // calls (on `As5600I2cPwm`) must still come from a task.

        /// @brief Wire up the per-period ISR callback. Idempotent —
        ///        rearming refreshes the registration with no gap.
        void enableIsrUpdates();

        /// @brief Drop the ISR callback and resume the polling
        ///        model on the next `readPosition()`.
        void disableIsrUpdates();

        /// @brief True when the ISR callback is currently armed.
        bool isIsrUpdatesEnabled() const
        {
                return isrUpdatesEnabled_;
        }

    protected:
        bool applyDirection(Direction direction) override;
        size_t formatLogPrefix(char *buf, size_t bufSize) const override;

    private:
        // Decode raw angle from the most recent (highUs, periodUs)
        // sample. Returns a value in [0, 4095] or 0xFFFF on bad input.
        static uint16_t rawFromSample(uint32_t highUs, uint32_t periodUs);

        void calibrateZero(uint16_t initial_position);
        bool isCurrentDirectionReadingNegative() const;

        // Trampoline used by `enableIsrUpdates()` to bridge the
        // backend's free-function callback to the instance's
        // wrap-around update. Marked IRAM-safe.
        static void onPwmSampleIsr(void *ctx);
        // Per-sample wrap-around update. Runs in ISR context when
        // `enableIsrUpdates()` is active. No logging, no float ops
        // on hosts that lack hardware FP, and only touches state
        // owned by this instance.
        void updatePositionFromIsr();

        ungula::hal::pwm_input::IPwmInput &pwm_;
        uint8_t directionPin_;

        uint32_t staleThresholdUs_ = 50'000;

        uint16_t zero_raw_position_ = 0;
        uint16_t last_raw_position_ = 0;
        // Volatile because the ISR-mode path writes these from
        // interrupt context while the host task may concurrently
        // read them through `position()` / `readPosition()`. Single
        // 32-bit aligned writes are atomic on Xtensa / RISC-V.
        volatile int cumulative_position_ = 0;
        volatile bool hasFirstSample_ = false;
        bool isrUpdatesEnabled_ = false;
};

} // namespace ungula::encoder::drivers
