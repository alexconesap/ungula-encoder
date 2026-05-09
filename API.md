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

### Use case: direct-connect AS5600 over I2C

```cpp
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/encoder/drivers/as5600_i2c.h>

ungula::hal::i2c::I2cMaster bus(0);
ungula::encoder::drivers::As5600I2c enc("wheel", bus,
                                        /*multiplexer=*/nullptr,
                                        /*channel=*/0,
                                        /*directionPin=*/4);

void setup() {
    bus.begin(21, 22, 400000);
    enc.begin();                       // no arguments — wiring set at construction
    enc.setCalibration(11.377f);       // 4096 / 360
}

void loop() {
    const float pos = enc.readPosition();   // raw counts
    const float deg = enc.readAngle();      // degrees, uses stored calibration
}
```

When to use this: an encoder has its own bus or is the only device on a
shared bus.

### Use case: two AS5600s behind a TCA9548A

```cpp
#include <ungula/hal/multiplexer/drivers/multiplexer_tca9548.h>
#include <ungula/encoder/drivers/as5600_i2c.h>

namespace enc = ungula::encoder;
namespace mux = ungula::hal::multiplexer;

ungula::hal::i2c::I2cMaster bus(0);
mux::drivers::MultiplexerTCA9548 mux70(0x70, bus);
enc::drivers::As5600I2c horiz("horizontal", bus, &mux70, /*channel=*/0);
enc::drivers::As5600I2c vert ("vertical",   bus, &mux70, /*channel=*/1);

void setup() {
    bus.begin(21, 22, 400000);
    mux70.begin();
    horiz.begin();
    vert.begin();
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
enc.begin();
enc.resetPosition(saved);   // 0 → take current angle as zero
```

When to use this: long-lived rigs where the mechanical zero must
survive power cycles.

---

## Public types

| Type | Header | Purpose |
| ---- | ------ | ------- |
| `ungula::encoder::IEncoder` | `ungula/encoder/i_encoder.h` | Chip-neutral interface, transport-agnostic |
| `ungula::encoder::Direction` (enum) | same | `None`, `ClockWise`, `CounterClockWise` |
| `ungula::encoder::MagnetStatus` (enum) | same | `Ok`, `TooHigh`, `TooLow`, `NotFound`, `EncoderError` |
| `ungula::encoder::Status` (enum) | same | `Ok`, `InitializationError`, `Error` |
| `ungula::encoder::Error` (enum) | same | `None`, `NotInitialized`, `BeginFailed`, `NotConnected`, `MultiplexerError`, `MagnetNotDetected`, `MagnetError`, `MagnetErrorHigh`, `MagnetErrorLow`, `I2CReadError`, `I2CWriteError` |
| `ungula::encoder::drivers::As5600I2c` | `ungula/encoder/drivers/as5600_i2c.h` | 12-bit magnetic rotary encoder over I2C |
| `ungula::encoder::drivers::EncoderFake` | `ungula/encoder/drivers/encoder_fake.h` | Header-only transport-agnostic test fake |
| `ungula::encoder::drivers::I2cEncoderFake` | `ungula/encoder/drivers/i2c_encoder_fake.h` | Test fake that exercises the I2C-multiplexer routing path |

`ENCODER_NO_DIRECTION_PIN = 255` — sentinel for "DIR pin not wired".

---

## `ungula::encoder::IEncoder`

### Construction

- **`IEncoder(const char* model, const char* name, int resolution)`** — pure logical constructor. Borrows `model` and `name`; `resolution` (steps per revolution, e.g. 4096 for AS5600) is stored on the base and returned by `getResolution()`. **No transport, multiplexer, or pin numbers** at this level — those belong to concrete drivers.

### Identity

- **`getName() / getModel()`** — borrowed pointers passed at construction.

### Lifecycle

- **`virtual bool begin() = 0`** — no arguments. Drivers consume the transport / pin parameters they were constructed with.
  Drivers must:
  1. Mark `isInitialized_ = true` (even on failure, so subsequent calls report the real error rather than `NotInitialized`).
  2. Initialise their transport (probe the chip, configure GPIO, etc.).
  3. Call `applyDirection(direction_)` so any pre-`begin()` direction setting takes effect.
  4. Capture the initial position / zero / state.

### Status

- **`isFunctional()`** — current health, refreshed by the driver as part of the call.
- **`isConnected()`** — bus-level probe (zero-length write).
- **`readStatus()`** — full status update (mux + bus + magnet).
- **`getLastError() / getLastErrorAsStr()`** — last operation error.
- **`statusToStr()`** — formatted "[MODEL name @ 0xNN:N] message" string. Internal static buffer; not reentrant. Use `_m`-flavoured EmblogX calls for log lines instead.

### Calibration (set once, then forget)

- **`setCalibration(float steps_per_degree)`** — store the rig's encoder-steps-per-degree. Default is `0.0f` (uncalibrated); pass `0.0f` to clear.
- **`calibration()`** — current value.
- **`hasCalibration()`** — `calibration() > 0.0f`.

Every angle-returning method below returns `NaN` while `!hasCalibration()`. The library refuses to invent a conversion factor.

### Position (talks to hardware, side effects)

- **`readPosition()`** — re-read the chip, update the cached cumulative position, return it. NaN on read error or before `begin()`.
- **`readAngle()`** — `readPosition()` then divide by the stored calibration. Default base implementation; drivers can override. NaN on read error or when `!hasCalibration()`.

### Position (cached, no I/O)

- **`position()`** — last cumulative position (steps). `0.0f` before the first successful `readPosition()`.
- **`angle()`** — `position() / calibration()`. NaN when `!hasCalibration()`.
- **`angleFromPosition(int position)`** — pure conversion using the stored calibration. NaN when `!hasCalibration()`.

### Misc

- **`resetPosition(uint16_t initial_position)`** — reset cumulative position; `0` snapshots the current raw angle.
- **`getResolution()`** — full-scale steps per revolution (e.g. 4096 for AS5600).

### Direction (works before begin())

- **`setDirection(Direction)` / `getDirection()`** — pure logical setter / getter on the base. `setDirection()` always succeeds: it caches the value and, if the driver is already initialised, calls the protected virtual `applyDirection()` so the DIR pin (when wired) updates immediately. Pre-`begin()` calls land on the wire when `begin()` finishes.
- **`setDirectionClockWise()` / `setDirectionCounterClockWise()`** — convenience wrappers.

### Capabilities (default: not supported)

`IEncoder` ships safe defaults so non-magnetic / no-watchdog drivers don't have to implement these. Concrete drivers that actually expose the feature override `hasXxx()` to `true` and the underlying methods.

- **`hasMagnetSensing()`** — default `false`. AS5600/AS5047P/etc. override to `true`.
- **`magnetStatus()` / `isMagnetFound()` / `isMagnetTooStrong()` / `isMagnetTooWeak()`** — default to "everything is fine". Callers should gate on `hasMagnetSensing()` before treating the result as authoritative.
- **`hasWatchDog()`** — default `false`. AS5600 overrides to `true`.
- **`setWatchDog(bool)` / `isWatchDogEnabled()`** — default no-op (returns `false`).

### Logging (off by default)

- **`enableLogging() / disableLogging() / isLoggingEnabled()`**
  EmblogX module tag is `encoder`. Per-instance — there is no global toggle.

### Protected helpers (for driver authors)

- **`applyDirection(Direction)`** — protected virtual. Drivers with a DIR pin override to push the logical value to hardware. Default is a no-op (returns `true`). Called from `setDirection()` post-`begin()` and from the driver's own `begin()` to honour pre-`begin()` settings.
- **`logInfof / logWarnf / logErrorf / logDebugf`** — printf-style helpers that prepend the per-instance prefix automatically.
- **`virtual size_t formatLogPrefix(char*, size_t)`** — overrideable prefix builder. Default: `[<model> <name>]`. The I2C driver overrides to add address + channel.

---

## `ungula::encoder::drivers::As5600I2c`

12-bit magnetic encoder, fixed I2C address `0x36`, default 400 kHz.
This is the **I2C-only transport**; future drivers will cover PWM input
(`As5600Pwm`) and combined I2C+PWM (`As5600I2cPwm`).

```cpp
As5600I2c(const char* name,
          ungula::hal::i2c::I2cMaster& bus,
          ungula::hal::multiplexer::IMultiplexer* multiplexer = nullptr,
          uint8_t multiplexerChannel = 0,
          uint8_t directionPin = ENCODER_NO_DIRECTION_PIN);
```

- All wiring (bus, multiplexer + channel, DIR pin) is captured at
  construction. `begin()` takes no arguments.
- Borrows the `I2cMaster`. Caller owns it and must call `bus.begin(...)`
  before `enc.begin()`.
- `multiplexer` defaults to `nullptr` (direct-connect). When a multiplexer
  is wired, the driver auto-selects the channel before every I2C
  transaction; host code stays unchanged.
- `getResolution()` returns 4096.
- DIR pin is optional; pass `ENCODER_NO_DIRECTION_PIN` to skip.
- Wrap-around (`4095 → 0`, `0 → 4095`) is handled internally; cumulative
  position is monotonic until `resetPosition()`.
- `hasMagnetSensing()` and `hasWatchDog()` both report `true`.
- `setDirection*()` calls work before `begin()` — value is captured and
  the DIR pin is written when the pin is configured during `begin()`.

### Constants

- `AS5600_DEFAULT_ADDRESS = 0x36`
- `AS5600_RESOLUTION = 4096`

### Hardware notes

- Watchdog: enabling `setWatchDog(true)` makes the chip self-reset after ~1.6 s without I2C traffic. Useful for long-running rigs that don't poll continuously.

#### Ungula's hardware notes

- Direction quirk: on the original Rachel rig, clockwise rotation returned **decreasing** raw values when DIR was tied to GND. The driver inverts the diff sign internally when `direction_ == ClockWise`. New rigs that wire DIR differently must call `setDirection(CounterClockWise)` to flip the convention.

---

## `ungula::encoder::drivers::EncoderFake`

Header-only test fake. Drop-in for any code that takes `IEncoder*`.

### Test knobs

- `setBeginResult(bool)`, `setIsConnected(bool)`, `setIsFunctional(bool)` — script return values.
- `setScriptedPosition(float)` — value returned by `readPosition()`.
- `setScriptedStatus(Status)` — value returned by `readStatus()`.
- `setMagnetStatus(MagnetStatus)` — drives `isMagnetFound/TooStrong/TooWeak`.
- `setResolution(int)` — override `getResolution()`.

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
