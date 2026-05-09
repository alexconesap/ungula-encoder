// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stddef.h>
#include <stdint.h>

/// @brief Chip-neutral encoder interface.
///
/// Every concrete encoder driver inherits from `IEncoder` and implements
/// the pure virtuals. Code that consumes encoders depends only on this
/// header and stays portable across drivers — including the transport
/// path (I2C, SPI, PWM input, ABI quadrature). Transport details live
/// in the concrete driver class, not here.
///
/// ## Direction is logical, not hardware
///
/// `setDirection()` always succeeds, before or after `begin()`. The
/// stored value is captured immediately; the hardware effect (DIR pin
/// transition on chips that have one) lands when the driver is up. A
/// pre-`begin()` `setDirectionCounterClockWise()` is honoured the
/// moment `begin()` finishes its own setup.
///
/// ## Capabilities — magnet, watchdog
///
/// Not every encoder is magnetic, and not every magnetic encoder
/// exposes a watchdog. The base ships safe defaults: `hasMagnetSensing()`
/// and `hasWatchDog()` return `false`, the related getters return
/// "everything is fine" values. Concrete drivers that actually expose
/// these features override `hasXxx()` to `true` and implement the
/// underlying methods properly. Callers gate on `hasMagnetSensing()`
/// before treating `magnetStatus()` as authoritative.
///
/// ## Logging
///
/// Off by default. `enableLogging()` routes diagnostics through EmblogX
/// with the module tag `encoder`. Per-instance toggle so multi-encoder
/// rigs can debug one channel without flooding the log with the others.

namespace ungula::encoder {

    /// @brief Sentinel for "no DIR pin wired" on encoders that expose one
    /// (e.g. AS5600). Drivers that don't have a DIR pin ignore this.
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
            /// @param model      Short label, e.g. "AS5600". Borrowed.
            /// @param name       Caller-chosen tag, e.g. "vertical". Borrowed.
            /// @param resolution Full-scale steps per revolution
            ///                   (e.g. 4096 for AS5600). Returned by
            ///                   `getResolution()`. Chip property — set
            ///                   once at construction, never changes.
            IEncoder(const char* model, const char* name, int resolution)
                : model_(model), name_(name), resolution_(resolution) {}

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
            const char* statusToStr() const;

            // ---- Calibration -----------------------------------------------
            //
            // Encoders are usually consumed in degrees, not raw counts. The
            // conversion factor — encoder steps required for one degree of
            // rotation — is a constant of the mechanical setup, not of the
            // call site. Set once via `setCalibration()`. Default 0 means
            // "uncalibrated"; every angle-returning method returns NaN
            // until a positive value is set.

            void setCalibration(float steps_per_degree) {
                steps_per_degree_ = steps_per_degree;
            }
            float calibration() const {
                return steps_per_degree_;
            }
            bool hasCalibration() const {
                return steps_per_degree_ > 0.0f;
            }

            // ---- Direction (works before begin()) --------------------------
            //
            // Direction is a pure logical flag. The hardware effect (DIR
            // pin write on chips that have one) only happens when the
            // driver is initialised; before `begin()` the value is
            // captured and applied during `begin()` itself. Drivers
            // override `applyDirection()` to push the logical value to
            // hardware.

            bool setDirection(Direction direction) {
                direction_ = direction;
                if (isInitialized_) {
                    return applyDirection(direction);
                }
                return true;
            }
            Direction getDirection() const {
                return direction_;
            }
            bool setDirectionClockWise() {
                return setDirection(Direction::ClockWise);
            }
            bool setDirectionCounterClockWise() {
                return setDirection(Direction::CounterClockWise);
            }

            // ---- Driver contract ----

            /// @brief Initialise the encoder. Drivers consume the
            /// transport / pin parameters they were constructed with;
            /// `begin()` itself takes nothing. After successful
            /// initialisation drivers must call `applyDirection(direction_)`
            /// so a pre-`begin()` direction setting takes effect.
            /// @return true on success. On failure call `getLastError()`.
            virtual bool begin() = 0;

            virtual bool isFunctional() = 0;
            virtual bool isConnected() = 0;

            // ---- Reading the chip (talks to hardware, side effects) -------

            /// @brief Re-read the chip and return the cumulative
            /// position in encoder steps.
            /// @return NaN on read error or before `begin()`.
            virtual float readPosition() = 0;

            /// @brief Re-read the chip and return the angle in degrees.
            /// Default: `readPosition() / calibration()`. NaN on read
            /// error or `!hasCalibration()`.
            virtual float readAngle();

            // ---- Cached getters (no I/O) ----------------------------------

            virtual float position() const = 0;
            float angle() const;
            float angleFromPosition(int position) const;

            /// @brief Reset the cumulative position. `initial_position == 0`
            ///        snapshots the current raw angle as the new zero.
            virtual bool resetPosition(uint16_t initial_position) = 0;

            /// @return Encoder full-scale resolution in steps. Set
            /// once at construction (chip property).
            int getResolution() const {
                return resolution_;
            }

            /// @brief Re-read the encoder and refresh internal status.
            virtual Status readStatus() = 0;

            // ---- Capabilities (default: not supported) --------------------
            //
            // Magnetic encoders override to true; ABI / optical /
            // PWM-only encoders inherit the safe defaults. Callers
            // gate on `hasMagnetSensing()` before trusting
            // `magnetStatus()` and friends.

            virtual bool hasMagnetSensing() const {
                return false;
            }
            virtual MagnetStatus magnetStatus() {
                return MagnetStatus::Ok;
            }
            virtual bool isMagnetFound() {
                return true;
            }
            virtual bool isMagnetTooStrong() {
                return false;
            }
            virtual bool isMagnetTooWeak() {
                return false;
            }

            virtual bool hasWatchDog() const {
                return false;
            }
            virtual bool setWatchDog(bool /*enabled*/) {
                return false;
            }
            virtual bool isWatchDogEnabled() {
                return false;
            }

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
            void setStatus(Error error) {
                last_error_ = error;
                status_ = (error == Error::None) ? Status::Ok : Status::Error;
            }
            void setInitializationStatus(Error error) {
                last_error_ = error;
                status_ = (error == Error::None) ? Status::Ok : Status::InitializationError;
            }

            /// @brief Push the logical direction to hardware. Drivers
            /// with a DIR pin override; the default is a no-op (returns
            /// true). Called from `setDirection()` post-`begin()` and
            /// from the driver's own `begin()` after the pin is up.
            virtual bool applyDirection(Direction /*direction*/) {
                return true;
            }

            /// @brief EmblogX module tag used by every log line emitted
            /// from this hierarchy.
            static constexpr const char* LOG_MODULE = "encoder";

            bool shouldLog() const {
                return loggingEnabled_;
            }

            /// @brief Per-instance log helpers. Each prepends the prefix
            /// produced by `formatLogPrefix()` so drivers never repeat
            /// the `[<model> <name>]` boilerplate. No-op when logging
            /// is disabled. Format-checked.
            void logInfof(const char* fmt, ...) const __attribute__((format(printf, 2, 3)));
            void logWarnf(const char* fmt, ...) const __attribute__((format(printf, 2, 3)));
            void logErrorf(const char* fmt, ...) const __attribute__((format(printf, 2, 3)));
            void logDebugf(const char* fmt, ...) const __attribute__((format(printf, 2, 3)));

            /// @brief Build the per-instance log prefix into `buf`.
            /// Default shape: `[<model> <name>]`. Drivers may override
            /// to add per-class fields (address, channel, etc.).
            virtual size_t formatLogPrefix(char* buf, size_t bufSize) const;

            // ---- Construction parameters ----
            const char* model_;
            const char* name_;
            const int resolution_;

            // ---- Run-time state ----
            bool isInitialized_ = false;

            // Steps required to rotate by one degree on this rig. 0
            // means "uncalibrated"; angle methods return NaN.
            float steps_per_degree_ = 0.0f;

            Direction direction_ = Direction::ClockWise;

            Status status_ = Status::Ok;
            Error last_error_ = Error::None;

        private:
            bool loggingEnabled_ = false;
    };

}  // namespace ungula::encoder
