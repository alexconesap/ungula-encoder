// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "i_encoder.h"

#include <stdarg.h>
#include <stdio.h>

#include <emblogx/logger.h>

namespace ungula::encoder {

    namespace {
        // Body buffer for the caller-supplied portion of the log line.
        // Sized so prefix+body fits comfortably under EmblogX's own
        // line buffer; truncation is silent (vsnprintf-style).
        constexpr size_t LOG_BODY_CAPACITY = 96;
        constexpr size_t LOG_PREFIX_CAPACITY = 64;
    }  // namespace

    size_t IEncoder::formatLogPrefix(char* buf, size_t bufSize) const {
        if (buf == nullptr || bufSize == 0) {
            return 0;
        }
        const int n = snprintf(buf, bufSize, "[%s %s @0x%02X:%u]",
                               model_ != nullptr ? model_ : "?",
                               name_ != nullptr ? name_ : "?", address_,
                               multiplexerChannel_);
        return (n < 0) ? 0 : static_cast<size_t>(n);
    }

#define UNGULA_ENC_DEFINE_LOG_HELPER(NAME, EMIT)                            \
    void IEncoder::NAME(const char* fmt, ...) const {                       \
        if (!loggingEnabled_) {                                             \
            return;                                                         \
        }                                                                   \
        char prefix[LOG_PREFIX_CAPACITY];                                   \
        formatLogPrefix(prefix, sizeof(prefix));                            \
        char body[LOG_BODY_CAPACITY];                                       \
        va_list ap;                                                         \
        va_start(ap, fmt);                                                  \
        vsnprintf(body, sizeof(body), fmt, ap);                             \
        va_end(ap);                                                         \
        EMIT(LOG_MODULE, "%s %s", prefix, body);                            \
    }

    UNGULA_ENC_DEFINE_LOG_HELPER(logInfof, log_info_m)
    UNGULA_ENC_DEFINE_LOG_HELPER(logWarnf, log_warn_m)
    UNGULA_ENC_DEFINE_LOG_HELPER(logErrorf, log_error_m)
    UNGULA_ENC_DEFINE_LOG_HELPER(logDebugf, log_debug_m)

#undef UNGULA_ENC_DEFINE_LOG_HELPER

    bool IEncoder::selectMultiplexerChannel() {
        if (!isInitialized_) {
            // Programmer error — caller forgot begin(). Surface as a
            // hard error rather than silently moving on.
            setStatus(Error::NotInitialized);
            logErrorf("not initialised, call begin() first");
            return false;
        }

        // No multiplexer wired — direct-connect deployment. Nothing to
        // do; the bus is already pointed at us.
        if (multiplexer_ == nullptr) {
            clearLastError();
            return true;
        }

        if (!multiplexer_->selectChannel(multiplexerChannel_)) {
            setStatus(Error::MultiplexerError);
            return false;
        }

        clearLastError();
        return true;
    }

    /// @brief Format a one-line description of the encoder's current state.
    ///
    /// Uses an internal static buffer — not thread-safe and not reentrant.
    /// Intended for occasional human-readable diagnostics, not for
    /// automated log lines (those should use logInfof/logErrorf/...).
    const char* IEncoder::statusToStr() const {
        // 128 bytes covers the longest message + name + model. Sized so
        // that we don't overflow on the longest realistic combination
        // ("[AS5600 vertical @ 0x36:1] reports a magnet problem...").
        static char buffer[128];

        const char* base = nullptr;
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
        snprintf(buffer, sizeof(buffer), "%s %s", prefix,
                 base != nullptr ? base : "");
        return buffer;
    }

    const char* IEncoder::getLastErrorAsStr() const {
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

}  // namespace ungula::encoder
