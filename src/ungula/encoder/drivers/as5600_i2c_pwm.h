// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "ungula/encoder/drivers/as5600_i2c.h"
#include "ungula/encoder/drivers/as5600_pwm.h"
#include "ungula/encoder/i_encoder.h"
#include "ungula/hal/i2c/i2c_master.h"
#include "ungula/hal/multiplexer/i_multiplexer.h"
#include "ungula/hal/pwm_input/i_pwm_input.h"

/// @brief AS5600 with both I2C and PWM transports active.
///
/// Use this driver when the host both routes the chip's PWM output to a
/// capture pin AND has the I2C bus wired up. Position reads come from
/// the PWM input (lower latency, cheaper than a register read), while
/// magnet status / watchdog / chip discovery still use I2C.
///
/// Inherits from `As5600I2c` so the I2C-side features (`isConnected`,
/// `magnetStatus`, `setWatchDog`, etc.) come for free; only the read
/// path is overridden.

namespace ungula::encoder::drivers
{

    class As5600I2cPwm final : public As5600I2c {
    public:
        /// @param name              Caller-chosen tag.
        /// @param bus               I2C bus the encoder lives on.
        /// @param pwm               Already-begun PWM input.
        /// @param multiplexer       Optional. `nullptr` = direct connect.
        /// @param multiplexerChannel Channel on the mux. Ignored when
        ///                          no multiplexer was passed.
        /// @param directionPin      Optional DIR pin.
        As5600I2cPwm(const char *name, ungula::hal::i2c::I2cMaster &bus, ungula::hal::pwm_input::IPwmInput &pwm,
                     ungula::hal::multiplexer::IMultiplexer *multiplexer = nullptr, uint8_t multiplexerChannel = 0,
                     uint8_t directionPin = ENCODER_NO_DIRECTION_PIN);

        // ---- Read path uses PWM ----
        float readPosition() override;
        float position() const override;
        bool resetPosition(uint16_t initial_position) override;

        // ---- Connection check fuses both transports --------------------
        //
        // We need both the bus to ACK and a fresh PWM sample for this
        // driver to be considered connected.
        bool isConnected() override;

        void setStaleThresholdUs(uint32_t us)
        {
            staleThresholdUs_ = us;
        }
        uint32_t staleThresholdUs() const
        {
            return staleThresholdUs_;
        }

    private:
        uint16_t rawFromCurrentSample() const;

        ungula::hal::pwm_input::IPwmInput &pwm_;

        uint32_t staleThresholdUs_ = 50'000;

            uint16_t pwmZeroRaw_ = 0;
        uint16_t pwmLastRaw_ = 0;
        int pwmCumulative_ = 0;
        bool hasFirstPwmSample_ = false;
    };

} // namespace ungula::encoder::drivers
