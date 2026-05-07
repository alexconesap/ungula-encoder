// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "ungula/hal/multiplexer/i_multiplexer.h"

/// @brief Chip-neutral encoder interface.
///
/// Every concrete encoder driver inherits from `IEncoder` and implements
/// the pure virtuals. Code that consumes encoders depends only on this
/// header and stays portable across drivers.
///
/// ## Multiplexer is optional
///
/// Some boards run an encoder behind a TCA9548A-style I2C bus
/// multiplexer (channel-select before each transaction); other boards
/// wire the encoder straight onto a dedicated bus. Both cases are first
/// class:
///
///   - **With multiplexer**: pass a `IMultiplexer*` and a channel to
///     `begin(channel, ...)`. Every transaction calls `selectChannel()`
///     first.
///   - **Without multiplexer**: pass `nullptr` for the multiplexer.
///     `selectMultiplexerChannel()` is a no-op and the channel argument
///     is ignored.
///
/// One driver, two deployments. The split is documented at every public
/// entry point and verified by the test suite.
///
/// ## Logging
///
/// Off by default. `enableLogging()` routes diagnostics through EmblogX
/// with the module tag `encoder`. Per-instance toggle so multi-encoder
/// rigs can debug one channel without flooding the log with the others.

namespace ungula::encoder {

    /// @brief Sentinel for "no DIR pin wired" on encoders that expose one
    /// (e.g. AS5600). When `directionPin == ENCODER_NO_DIRECTION_PIN` the
    /// driver does not touch the pin and trusts the hardware default.
    constexpr uint8_t ENCODER_NO_DIRECTION_PIN = 255;

    enum class Direction : uint8_t {
        None = 0,
        ClockWise,
        CounterClockWise,
    };

    enum class MagnetStatus : uint8_t {
        Ok = 0,
        TooHigh,
        TooLow,
        NotFound,
        EncoderError,
    };

    enum class Status : uint8_t {
        Ok = 0,
        InitializationError = 1,
        Error = 2,
    };

    enum class Error : uint8_t {
        None = 0,
        NotInitialized = 1,
        BeginFailed = 2,
        NotConnected = 3,
        MultiplexerError = 4,
        MagnetNotDetected = 5,
        MagnetError = 6,
        MagnetErrorHigh = 7,
        MagnetErrorLow = 8,
        I2CReadError = 9,
        I2CWriteError = 10,
    };

    /// @brief Abstract base for all encoders.
    class IEncoder {
        public:
            /// @param model       Short label, e.g. "AS5600". Borrowed.
            /// @param name        Caller-chosen tag, e.g. "vertical". Borrowed.
            /// @param multiplexer Optional. `nullptr` means "wired direct".
            IEncoder(const char* model, const char* name,
                     ungula::hal::multiplexer::IMultiplexer* multiplexer)
                    : model_(model), name_(name), multiplexer_(multiplexer) {}

            virtual ~IEncoder() = default;

            IEncoder(const IEncoder&) = delete;
            IEncoder& operator=(const IEncoder&) = delete;

            // ---- Identity / status helpers ----

            const char* getName() const {
                return name_;
            }
            const char* getModel() const {
                return model_;
            }

            Error getLastError() const {
                return last_error_;
            }
            const char* getLastErrorAsStr() const;
            void clearLastError() {
                setStatus(Error::None);
            }
            void setStatus(Error error) {
                last_error_ = error;
                status_ = (error == Error::None) ? Status::Ok : Status::Error;
            }
            void setInitializationStatus(Error error) {
                last_error_ = error;
                status_ = (error == Error::None) ? Status::Ok : Status::InitializationError;
            }
            const char* statusToStr() const;

            bool hasMultiplexer() const {
                return multiplexer_ != nullptr;
            }

            // ---- Driver contract ----

            /// @brief Initialise the encoder.
            /// @param multiplexerChannel Channel on the multiplexer, ignored
            ///                           when `hasMultiplexer() == false`.
            /// @param directionPin       MCU pin wired to the encoder's DIR
            ///                           input, or `ENCODER_NO_DIRECTION_PIN`.
            /// @return true on success. On failure call `getLastError()`
            ///         for the reason.
            virtual bool begin(uint8_t multiplexerChannel, uint8_t directionPin) = 0;

            virtual bool isFunctional() = 0;
            virtual bool isConnected() = 0;

            /// @return Current cumulative position in encoder steps,
            ///         NaN on read error (caller checks via std::isnan).
            virtual float readPosition() = 0;

            /// @return Angle in degrees for `position` divided by
            ///         `calibration_steps_to_degrees`.
            virtual float angleFromPosition(int position,
                                            float calibration_steps_to_degrees) const = 0;

            /// @return Same conversion using the internal cumulative position.
            virtual float angleFromCurrentPosition(float calibration_steps_to_degrees) const = 0;

            /// @brief Reset the cumulative position. `initial_position == 0`
            ///        snapshots the current raw angle as the new zero.
            virtual bool resetPosition(uint16_t initial_position) = 0;

            /// @return Encoder full-scale resolution in steps (e.g. 4096).
            virtual int getEncoderResolution() const = 0;

            /// @brief Re-read the encoder, update internal status fields.
            virtual Status readStatus() = 0;

            virtual bool setDirection(Direction direction) = 0;
            virtual Direction getDirection() = 0;

            virtual bool isMagnetFound() = 0;
            virtual bool isMagnetTooStrong() = 0;
            virtual bool isMagnetTooWeak() = 0;
            virtual MagnetStatus magnetStatus() = 0;

            virtual bool setWatchDog(bool enabled) = 0;
            virtual bool isWatchDogEnabled() = 0;

            // ---- Optional logging (off by default) ----

            void enableLogging() {
                loggingEnabled_ = true;
            }
            void disableLogging() {
                loggingEnabled_ = false;
            }
            bool isLoggingEnabled() const {
                return loggingEnabled_;
            }

        protected:
            /// @brief Drivers call this before any I2C transaction. Returns
            /// true when the bus is ready to talk to this encoder (i.e.
            /// the multiplexer channel is selected, OR no multiplexer is
            /// in use). On failure sets `MultiplexerError` and returns false.
            bool selectMultiplexerChannel();

            /// @brief EmblogX module tag used by every log line emitted
            /// from this hierarchy.
            static constexpr const char* LOG_MODULE = "encoder";

            bool shouldLog() const {
                return loggingEnabled_;
            }

            /// @brief Per-instance log helpers. Each prepends the prefix
            /// produced by `formatLogPrefix()` so drivers never repeat
            /// the `[<model> <name> @0x<addr>:<channel>]` boilerplate.
            /// No-op when logging is disabled. Format-checked.
            void logInfof(const char* fmt, ...) const
                    __attribute__((format(printf, 2, 3)));
            void logWarnf(const char* fmt, ...) const
                    __attribute__((format(printf, 2, 3)));
            void logErrorf(const char* fmt, ...) const
                    __attribute__((format(printf, 2, 3)));
            void logDebugf(const char* fmt, ...) const
                    __attribute__((format(printf, 2, 3)));

            /// @brief Build the per-instance log prefix into `buf`.
            /// Default shape: `[<model> <name> @0x<addr>:<channel>]`.
            /// Drivers may override to add per-class fields.
            virtual size_t formatLogPrefix(char* buf, size_t bufSize) const;

            // ---- Construction parameters ----
            const char* model_;
            const char* name_;
            ungula::hal::multiplexer::IMultiplexer* multiplexer_;  // nullable

            // ---- Run-time state ----
            bool isInitialized_ = false;
            uint8_t address_ = 0x00;
            uint8_t multiplexerChannel_ = 0;

            Status status_ = Status::Ok;
            Error last_error_ = Error::None;

        private:
            bool loggingEnabled_ = false;
    };

}  // namespace ungula::encoder
