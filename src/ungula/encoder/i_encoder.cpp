// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "i_encoder.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include <emblogx/logger.h>

namespace ungula::encoder
{

    // ---- Default angle helpers -----------------------------------------
    //
    // All three depend on the stored calibration. They return NaN when no
    // calibration has been set — there's no honest "0 degrees" answer
    // when steps-per-degree is unknown.

    float IEncoder::readAngle()
    {
        const float pos = readPosition();
        if (isnan(pos) || !hasCalibration()) {
            return NAN;
        }
        return pos / steps_per_degree_;
    }

    float IEncoder::angle() const
    {
        if (!hasCalibration()) {
            return NAN;
        }
        return position() / steps_per_degree_;
    }

    float IEncoder::angleFromPosition(int position) const
    {
        if (!hasCalibration()) {
            return NAN;
        }
        return static_cast<float>(position) / steps_per_degree_;
    }

    namespace
    {
        // Body buffer for the caller-supplied portion of the log line.
        // Sized so prefix+body fits comfortably under EmblogX's own
        // line buffer; truncation is silent (vsnprintf-style).
        constexpr size_t LOG_BODY_CAPACITY = 96;
        constexpr size_t LOG_PREFIX_CAPACITY = 64;
    } // namespace

    size_t IEncoder::formatLogPrefix(char *buf, size_t bufSize) const
    {
        if (buf == nullptr || bufSize == 0) {
            return 0;
        }
        // Default: just `[<model> <name>]`. Drivers with extra wiring
        // info (address, mux channel, etc.) override this to add it.
        const int n =
            snprintf(buf, bufSize, "[%s %s]", model_ != nullptr ? model_ : "?", name_ != nullptr ? name_ : "?");
        return (n < 0) ? 0 : static_cast<size_t>(n);
    }

#define UNGULA_ENC_DEFINE_LOG_HELPER(NAME, EMIT)    \
    void IEncoder::NAME(const char *fmt, ...) const \
    {                                               \
        if (!loggingEnabled_) {                     \
            return;                                 \
        }                                           \
        char prefix[LOG_PREFIX_CAPACITY];           \
        formatLogPrefix(prefix, sizeof(prefix));    \
        char body[LOG_BODY_CAPACITY];               \
        va_list ap;                                 \
        va_start(ap, fmt);                          \
        vsnprintf(body, sizeof(body), fmt, ap);     \
        va_end(ap);                                 \
        EMIT(LOG_MODULE, "%s %s", prefix, body);    \
    }

    UNGULA_ENC_DEFINE_LOG_HELPER(logInfof, log_info_m)
    UNGULA_ENC_DEFINE_LOG_HELPER(logWarnf, log_warn_m)
    UNGULA_ENC_DEFINE_LOG_HELPER(logErrorf, log_error_m)
    UNGULA_ENC_DEFINE_LOG_HELPER(logDebugf, log_debug_m)

#undef UNGULA_ENC_DEFINE_LOG_HELPER

    /// @brief Format a one-line description of the encoder's current state.
    ///
    /// Uses an internal static buffer — not thread-safe and not reentrant.
    /// Intended for occasional human-readable diagnostics, not for
    /// automated log lines (those should use logInfof/logErrorf/...).
    const char *IEncoder::statusToStr() const
    {
        // 128 bytes covers the longest message + the longest expected
        // prefix the driver overrides will emit.
        static char buffer[128];

        const char *base = nullptr;
        switch (status_) {
        case Status::Ok:
            base = "is working fine.";
            break;
        case Status::Error:
        case Status::InitializationError:
            base = getLastErrorAsStr();
            break;
        }

        char prefix[LOG_PREFIX_CAPACITY];
        formatLogPrefix(prefix, sizeof(prefix));
        snprintf(buffer, sizeof(buffer), "%s %s", prefix, base != nullptr ? base : "");
        return buffer;
    }

    const char *IEncoder::getLastErrorAsStr() const
    {
        switch (last_error_) {
        case Error::None:
            return "No errors reported";
        case Error::NotInitialized:
            return "is not initialised. Call begin() first.";
        case Error::BeginFailed:
            return "failed during initialisation (begin).";
        case Error::NotConnected:
            return "not connected / not found.";
        case Error::MultiplexerError:
            return "multiplexer channel-select failed.";
        case Error::MagnetError:
            return "reports a magnet error.";
        case Error::MagnetNotDetected:
            return "is not detecting the magnet.";
        case Error::MagnetErrorHigh:
            return "magnet signal too strong.";
        case Error::MagnetErrorLow:
            return "magnet signal too weak.";
        case Error::I2CReadError:
            return "I2C read error.";
        case Error::I2CWriteError:
            return "I2C write error.";
            // No default — let the compiler flag any new enum values
            // we forget to map here.
        }
        return "";
    }

} // namespace ungula::encoder
