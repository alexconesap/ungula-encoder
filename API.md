# UngulaEncoder (`lib_encoder`)

LLM-oriented public-API reference for the encoder library. For the
human-facing overview see [`README.md`](README.md). For project-wide
rules see `code/CLAUDE.md`.

The library provides a chip-neutral encoder interface
(`ungula::encoder::IEncoder`) and concrete drivers under
`ungula::encoder::drivers`. The multiplexer is optional — every driver
must work with `multiplexer == nullptr` (direct-connect deployment) or
with a `IMultiplexer*` (multiplexed deployment).

---

## Usage

### Use case: direct-connect AS5600

```cpp
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/encoder/drivers/as5600.h>

ungula::hal::i2c::I2cMaster bus(0);
ungula::encoder::drivers::AS5600 enc("wheel", bus);   // no multiplexer

void setup() {
    bus.begin(21, 22, 400000);
    enc.begin(/*channel=*/0, /*directionPin=*/4);     // channel ignored
}

void loop() {
    const float pos = enc.readPosition();
    const float deg = enc.angleFromCurrentPosition(11.377f);
}
```

When to use this: an encoder has its own bus or is the only device on a
shared bus.

### Use case: two AS5600s behind a TCA9548A

```cpp
#include <ungula/hal/multiplexer/drivers/multiplexer_tca9548.h>
#include <ungula/encoder/drivers/as5600.h>

namespace enc = ungula::encoder;
namespace mux = ungula::hal::multiplexer;

ungula::hal::i2c::I2cMaster bus(0);
mux::drivers::MultiplexerTCA9548 mux70(0x70, bus);
enc::drivers::AS5600 horiz("horizontal", bus, &mux70);
enc::drivers::AS5600 vert ("vertical",   bus, &mux70);

void setup() {
    bus.begin(21, 22, 400000);
    mux70.begin();
    horiz.begin(0, enc::ENCODER_NO_DIRECTION_PIN);
    vert .begin(1, enc::ENCODER_NO_DIRECTION_PIN);
}
```

When to use this: two or more encoders share an I2C bus and clash on
address (AS5600 has a fixed `0x36`).

### Use case: per-instance debugging

```cpp
vert.enableLogging();   // EmblogX module = "encoder"
// (lines from horiz remain quiet)
```

When to use this: one encoder is misbehaving and you want its raw-angle
trace without flooding the log with the other devices.

### Use case: persisted zero across reboots

```cpp
const uint16_t saved = readFromNvs();
enc.begin(0, 4);
enc.resetPosition(saved);   // 0 → take current angle as zero
```

When to use this: long-lived rigs where the mechanical zero must
survive power cycles.

---

## Public types

| Type | Header | Purpose |
| ---- | ------ | ------- |
| `ungula::encoder::IEncoder` | `ungula/encoder/i_encoder.h` | Chip-neutral interface, multiplexer optional |
| `ungula::encoder::Direction` (enum) | same | `None`, `ClockWise`, `CounterClockWise` |
| `ungula::encoder::MagnetStatus` (enum) | same | `Ok`, `TooHigh`, `TooLow`, `NotFound`, `EncoderError` |
| `ungula::encoder::Status` (enum) | same | `Ok`, `InitializationError`, `Error` |
| `ungula::encoder::Error` (enum) | same | `None`, `NotInitialized`, `BeginFailed`, `NotConnected`, `MultiplexerError`, `MagnetNotDetected`, `MagnetError`, `MagnetErrorHigh`, `MagnetErrorLow`, `I2CReadError`, `I2CWriteError` |
| `ungula::encoder::drivers::AS5600` | `ungula/encoder/drivers/as5600.h` | 12-bit magnetic rotary encoder |
| `ungula::encoder::drivers::EncoderFake` | `ungula/encoder/drivers/encoder_fake.h` | Header-only test fake |

`ENCODER_NO_DIRECTION_PIN = 255` — sentinel for "DIR pin not wired".

---

## `ungula::encoder::IEncoder`

### Construction

- **`IEncoder(const char* model, const char* name, IMultiplexer* multiplexer)`** — borrows all three pointers. `multiplexer == nullptr` is a first-class direct-connect deployment.

### Identity

- **`getName() / getModel()`** — borrowed pointers passed at construction.
- **`hasMultiplexer()`** — `true` iff `multiplexer != nullptr`.

### Lifecycle

- **`virtual bool begin(uint8_t multiplexerChannel, uint8_t directionPin) = 0`**
  Drivers must:
  1. Set internal address / channel / direction-pin state.
  2. Mark `isInitialized_ = true` (even on failure, so subsequent calls report the real error rather than `NotInitialized`).
  3. Select the multiplexer channel via `selectMultiplexerChannel()` — works in both deployments.
  4. Configure the DIR pin via `ungula::hal::gpio` if `directionPin != ENCODER_NO_DIRECTION_PIN`.
  5. Capture the initial raw angle as zero.

### Status

- **`isFunctional()`** — current health, refreshed by the driver as part of the call.
- **`isConnected()`** — bus-level probe (zero-length write).
- **`readStatus()`** — full status update (mux + bus + magnet).
- **`getLastError() / getLastErrorAsStr()`** — last operation error.
- **`statusToStr()`** — formatted "[MODEL name @ 0xNN:N] message" string. Internal static buffer; not reentrant. Use `_m`-flavoured EmblogX calls for log lines instead.

### Position

- **`readPosition()`** — current cumulative position in encoder steps. Returns NaN on read error or before `begin()`.
- **`angleFromPosition(int position, float calibration_steps_to_degrees)`** — `position / calibration_steps_to_degrees`.
- **`angleFromCurrentPosition(float calibration_steps_to_degrees)`** — same on the internal cumulative position.
- **`resetPosition(uint16_t initial_position)`** — reset cumulative position; `0` snapshots the current angle.
- **`getEncoderResolution()`** — full-scale steps (e.g. 4096 for AS5600).

### Direction / magnet / watchdog

- **`setDirection(Direction)` / `getDirection()`**
- **`isMagnetFound()` / `isMagnetTooStrong()` / `isMagnetTooWeak()` / `magnetStatus()`**
- **`setWatchDog(bool)` / `isWatchDogEnabled()`**

### Logging (off by default)

- **`enableLogging() / disableLogging() / isLoggingEnabled()`**
  EmblogX module tag is `encoder`. Per-instance — there is no global toggle.

### Protected helpers

- **`selectMultiplexerChannel()`** — drivers call this before any I2C transaction. Returns true when the bus is ready (channel selected, or no mux). Sets `Error::MultiplexerError` on failure, `Error::NotInitialized` if `begin()` was never called.

---

## `ungula::encoder::drivers::AS5600`

12-bit magnetic encoder, fixed I2C address `0x36`, default 400 kHz.

```cpp
AS5600(const char* name, ungula::hal::i2c::I2cMaster& bus,
       ungula::hal::multiplexer::IMultiplexer* multiplexer = nullptr);
```

- Borrows the `I2cMaster`. Caller owns it and must call `bus.begin(...)` before `enc.begin(...)`.
- `multiplexer` defaults to `nullptr` — direct-connect.
- `getEncoderResolution()` returns 4096.
- DIR pin is optional; pass `ENCODER_NO_DIRECTION_PIN` to skip.
- Wrap-around (`4095 → 0`, `0 → 4095`) is handled internally; cumulative position is monotonic until `resetPosition()`.

### Constants

- `AS5600_DEFAULT_ADDRESS = 0x36`
- `AS5600_RESOLUTION = 4096`

### Hardware notes

- Watchdog: enabling `setWatchDog(true)` makes the chip self-reset after ~1.6 s without I2C traffic. Useful for long-running rigs that don't poll continuously.
- Direction quirk: on the original Rachel rig, clockwise rotation returned **decreasing** raw values when DIR was tied to GND. The driver inverts the diff sign internally when `direction_ == ClockWise`. New rigs that wire DIR differently must call `setDirection(CounterClockWise)` to flip the convention.

---

## `ungula::encoder::drivers::EncoderFake`

Header-only test fake. Drop-in for any code that takes `IEncoder*`.

### Test knobs

- `setBeginResult(bool)`, `setIsConnected(bool)`, `setIsFunctional(bool)` — script return values.
- `setScriptedPosition(float)` — value returned by `readPosition()`.
- `setScriptedStatus(Status)` — value returned by `readStatus()`.
- `setMagnetStatus(MagnetStatus)` — drives `isMagnetFound/TooStrong/TooWeak`.
- `setResolution(int)` — override `getEncoderResolution()`.

### Inspectors

- `beginCallCount()`, `readPositionCallCount()`, `readStatusCallCount()`,
  `resetCallCount()`, `isConnectedCallCount()`, `isFunctionalCallCount()`,
  `lastDirectionPin()`.

### Drift-detection role

`test_encoder.cpp::IEncoderContract.FakeImplementsEveryPureVirtual`
calls every pure-virtual through an `IEncoder*`. Renaming or re-typing
any method without updating the fake breaks the build — that's the
safety net the user asked for.

---

## Lifecycle and error handling

- **Initialisation order**: `bus.begin()` → `multiplexer.begin()` (if any) → `enc.begin()`. The encoder driver assumes the bus and multiplexer are already up.
- **No exceptions**. Failures surface as `false` returns, `NaN`, or non-`Ok` enum values.
- **`readPosition()` returns NaN** on multiplexer error, I2C read error, or before `begin()`. Caller should `std::isnan()`-check before using the result.
- **`getLastError()`** is the canonical source of truth for "what went wrong"; the boolean return only says yes/no.

## Threading / hardware

- The AS5600 driver is not thread-safe — use one instance per task or wrap calls in a mutex. The shared `I2cMaster` is also single-task by design.
- Calls go through `ungula::core::time::delayUs` for I2C pacing — host tests use `std::this_thread`, ESP32 builds use `esp_rom_delay_us`.
- DIR pin transitions are configured once during `begin()`; runtime calls to `setDirection()` only flip the GPIO when a DIR pin is wired.

## LLM usage rules

- Use `IEncoder*` in code that consumes encoders. Do not depend on the concrete driver type — the multiplexer-optional contract is at the interface level, not the driver level.
- Pass `nullptr` for the multiplexer when the encoder is direct-connect. Do **not** invent a "null multiplexer" wrapper class.
- Treat `readPosition() == NaN` as the only error signal; do not assume a sentinel value.
- Use `EncoderFake` (under `drivers/`) for host tests. Do not reach into private state.
- Use `ungula::core::time::delay*` for any pacing — never `delay()` / `delayMicroseconds()`.
- Use `ungula::hal::gpio` for the DIR pin — never Arduino `pinMode` / `digitalWrite`.
- Logging is per-instance: `enc.enableLogging()`. There is no project-global toggle. Module tag is fixed (`encoder`).
