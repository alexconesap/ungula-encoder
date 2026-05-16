# UngulaEncoder

> **High-performance embedded C++ for ESP32 and friends** — magnetic / optical encoder drivers.

> **LLM usage note:** if this library is consumed from a coding AI workflow, explicitly point the agent to `API.md` first. `API.md` is the LLM-facing contract (public API + examples + constraints) and avoids wasting time/tokens scanning source files and this human-oriented README.

Chip-neutral encoder interface plus drivers. Designed around two real
deployments that look different on the board but identical in code:

1. **One bus, several encoders, one multiplexer** (the original "Rachel"
   rig — two AS5600s sharing a TCA9548A).
2. **Multiple buses, no multiplexer** (the new project — every encoder
   wired to its own I2C peripheral or to dedicated channels).

Both work through the same `IEncoder` API. The multiplexer is **optional**.

## Table of Contents

- [Features](#features)
- [Supported chips](#supported-chips)
- [Dependencies](#dependencies)
  - [Include reference](#include-reference)
- [Architecture](#architecture)
- [Quick start — direct connect](#quick-start-direct-connect)
- [Quick start — behind a multiplexer](#quick-start-behind-a-multiplexer)
- [Quick start — PWM-only (`As5600Pwm`)](#quick-start-pwm-only-as5600pwm)
  - [ISR-driven angle updates — no polling required](#isr-driven-angle-updates-no-polling-required)
- [Quick start — I2C + PWM combined (`As5600I2cPwm`)](#quick-start-i2c-pwm-combined-as5600i2cpwm)
- [Multiplexer is optional (per driver)](#multiplexer-is-optional-per-driver)
- [Changing direction](#changing-direction)
- [Logging](#logging)
- [Testing](#testing)
- [Acknowledgements](#acknowledgements)
- [License](#license)
- [Arduino CLI symlink note (rarely relevant)](#arduino-cli-symlink-note-rarely-relevant)

## Features

- Clean `IEncoder` interface — every method drives a single concrete behaviour.
- Multiplexer is **optional**: pass a pointer or `nullptr`, the driver does the right thing.
- Degree-first API. Set the calibration once via `setCalibration(steps_per_degree)`; `readAngle()` / `angle()` return degrees from then on, no per-call divisor needed. Raw `readPosition()` / `position()` are still there for callers that want counts.
- Wrap-around handled — the AS5600 returns 12-bit raw angles; the driver tracks 4095↔0 transitions and accumulates correctly.
- Per-instance EmblogX logging toggle. Off by default.
- Header-only `EncoderFake` for tests; locks the interface so a future signature change cannot silently break drivers.

## Supported chips

| Chip   | Bus | Resolution | Driver |
| ------ | --- | ---------- | ------ |
| AS5600 | I2C (`I2cMaster`) | 12 bit / 4096 steps | `ungula::encoder::drivers::As5600I2c` |
| AS5600 | PWM (`IPwmInput`) | 12 bit / 4096 steps | `ungula::encoder::drivers::As5600Pwm` |
| AS5600 | I2C + PWM (combined) | 12 bit / 4096 steps | `ungula::encoder::drivers::As5600I2cPwm` |
| AS5047P | SPI (`SpiMaster`) | 14 bit / 16384 steps | `ungula::encoder::drivers::As5047pSpi` |
| MA730 | SPI (`SpiMaster`) | 14 bit / 16384 steps | `ungula::encoder::drivers::Ma730Spi` |
| MT6835 | SPI (`SpiMaster`) | 21 bit / 2097152 steps | `ungula::encoder::drivers::Mt6835Spi` |
| MT6701 | ABI / quadrature (`IDecoder`) | up to 16384 counts (decoder-dependent) | `ungula::encoder::drivers::Mt6701Abi` |

The `IEncoder` interface is transport-agnostic so new drivers (additional ABI chips, RS-485 absolute encoders, optical encoders) slot in without breaking host code.

## Dependencies

- `UngulaCore` — `ungula::core::time::delay*` for I2C pacing.
- `UngulaHal` — `i2c::I2cMaster`, `gpio` for the optional DIR pin, `multiplexer::IMultiplexer` for the optional channel select.
- `EmblogX` — runtime diagnostics. Logging is off by default.

### Include reference

- Core encoder API only:

```cpp
#include <ungula/encoder.h>
```

- AS5600 (I2C transport) direct-connect example:

```cpp
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/encoder/drivers/as5600_i2c.h>
```

- AS5600 (I2C transport) + TCA9548A multiplexer example:

```cpp
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/hal/multiplexer/drivers/multiplexer_tca9548.h>
#include <ungula/encoder/drivers/as5600_i2c.h>
```

## Architecture

```text
ungula::encoder
├── IEncoder                       ← chip-neutral, transport-agnostic
│    ├── setCalibration() / readAngle() / position() / direction
│    └── enableLogging() / disableLogging()
└── drivers/
     ├── As5600I2c                 ← AS5600 over I2C (+ optional mux + DIR pin)
     ├── As5600Pwm                 ← AS5600 over PWM input only (no bus)
     ├── As5600I2cPwm              ← AS5600 over I2C + PWM (fast read path)
     ├── As5047pSpi                ← AMS AS5047P over SPI (14-bit, magnet diag)
     ├── Ma730Spi                  ← Monolithic Power MA730 over SPI (14-bit)
     ├── Mt6835Spi                 ← MagnTek MT6835 over SPI (21-bit, in-band CRC)
     ├── Mt6701Abi                 ← MT6701 over ABI quadrature (PCNT-backed)
     ├── EncoderFake               ← header-only fake (transport-agnostic)
     └── I2cEncoderFake            ← header-only fake exercising the mux path
```

`IEncoder` knows nothing about I2C, multiplexers, or pin numbers.
Transport details (bus reference, mux pointer, channel, DIR pin) live
in the concrete driver's constructor. Channel-routing is internal to
the I2C-backed drivers — host code calls `enc.readAngle()` and the
multiplexer is selected automatically.

Drivers do not depend on Arduino. All time and pin operations go through
`ungula::core::time` and `ungula::hal::gpio`.

## Quick start — direct connect

Encoder wired straight to the MCU's I2C peripheral. No multiplexer.

```cpp
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/encoder/drivers/as5600_i2c.h>

namespace enc = ungula::encoder;

ungula::hal::i2c::I2cMaster bus(0);
enc::drivers::As5600I2c wheel("wheel", bus,
                              /*multiplexer=*/nullptr,
                              /*channel=*/0,
                              /*directionPin=*/4);

void setup() {
    bus.begin(/*sda=*/21, /*scl=*/22, /*hz=*/400000);
    if (!wheel.begin()) {
        // wheel.statusToStr() / wheel.getLastErrorAsStr()
    }
    // Set the calibration once — the value is rig-specific (mechanical
    // gearing, encoder mounting). 11.377 = AS5600's 4096 counts / 360°.
    wheel.setCalibration(11.377f);
}

void loop() {
    const float pos = wheel.readPosition();   // raw counts
    const float deg = wheel.readAngle();      // degrees, uses stored calibration
    // …
}
```

Until `setCalibration()` lands a positive value, `readAngle()`,
`angle()`, and `angleFromPosition()` all return NaN — there's no honest
conversion without a known steps-per-degree.

The constructor took `multiplexer = nullptr`, so channel selection is a
zero-cost no-op on every transaction.

## Quick start — behind a multiplexer

Two AS5600s sharing one TCA9548A on a single bus. The pattern from Ungula's Rachel photogrammetry project.

```cpp
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/hal/multiplexer/drivers/multiplexer_tca9548.h>
#include <ungula/encoder/drivers/as5600_i2c.h>

namespace enc = ungula::encoder;
namespace mux = ungula::hal::multiplexer;

ungula::hal::i2c::I2cMaster i2c_bus(0);
mux::drivers::MultiplexerTCA9548 mux70(0x70, i2c_bus);

enc::drivers::As5600I2c horizontal("horizontal", i2c_bus, &mux70, /*channel=*/0);
enc::drivers::As5600I2c vertical  ("vertical",   i2c_bus, &mux70, /*channel=*/1);

void setup() {
    i2c_bus.begin(21, 22, 400000);
    mux70.begin();

    horizontal.begin();
    vertical.begin();
}

void loop() {
    const float h = horizontal.readPosition();
    const float v = vertical.readPosition();
    // …
}
```

The multiplexer caches the active channel. Reading `vertical` twice in a
row only writes the channel once — the second call is a cache hit and
goes straight to the AS5600.

## Quick start — PWM-only (`As5600Pwm`)

The AS5600 emits a single-pin PWM where the duty cycle encodes the angle.
With this transport, you do not need a free I2C bus — only one input pin
plus the chip's PWM output. Magnet diagnostics and the watchdog are
**not** available in PWM mode (those live behind the I2C register file);
the driver reports `hasMagnetSensing()` and `hasWatchDog()` as `false`,
so callers that gate on those flags do the right thing automatically.

```cpp
#include <ungula/encoder/drivers/as5600_pwm.h>
#include <ungula/hal/pwm_input/drivers/pwm_input.h>

namespace enc = ungula::encoder;
namespace pwm = ungula::hal::pwm_input;

pwm::drivers::PwmInput cap;
enc::drivers::As5600Pwm wheel("wheel", cap,
                              /*directionPin=*/ungula::encoder::ENCODER_NO_DIRECTION_PIN);

void setup() {
    cap.begin(/*pin=*/34);     // PWM output of the chip lands here
    wheel.begin();
    wheel.setCalibration(11.377f);  // 4096 counts / 360°
}

void loop() {
    const float deg = wheel.readAngle();   // NaN until first PWM frame arrives
    if (!std::isnan(deg)) {
        // use deg
    }
}
```

The driver flags a stalled signal as `Error::NotConnected` when the last
edge is older than the (configurable) stale threshold (default 50 ms,
~6 frames at the slowest 115 Hz mode). Tune it via
`wheel.setStaleThresholdUs(...)` — pick a window that comfortably
covers two frame periods on the host's PWMF setting.

### ISR-driven angle updates — no polling required

The PWM ISR captures `(highUs, periodUs)` for every frame whether the
host polls or not. What the host *does* still need to do, in the polling
model, is feed the cumulative-position counter — wrap-around tracking
needs to see two consecutive samples that are less than half a
revolution apart. Skip the read for a full turn and the next decode
will pick the wrong direction.

For consumers that need true per-frame latency, `enableIsrUpdates()`
moves the wrap-around math into the same ISR that captures the sample.
After it is enabled, `position()` and `readAngle()` return the latest
cumulative count without any polling task running:

```cpp
#include <ungula/encoder/drivers/as5600_pwm.h>
#include <ungula/hal/pwm_input/drivers/pwm_input.h>

namespace enc = ungula::encoder;
namespace pwm = ungula::hal::pwm_input;

pwm::drivers::PwmInput cap;
enc::drivers::As5600Pwm wheel("wheel", cap);

void setup() {
    cap.begin(/*pin=*/34);
    wheel.begin();
    wheel.setCalibration(11.377f);
    wheel.enableIsrUpdates();   // arm the per-period ISR callback
}

void loop() {
    // No periodic readPosition() needed — the ISR keeps the cumulative
    // count fresh. Read it whenever the application wants it.
    const float deg = wheel.readAngle();
    if (!std::isnan(deg)) {
        // use deg
    }
}
```

What the ISR does and does not do:

- **Does**: decode raw angle from the latest sample, fold the
  4095↔0 transition into the cumulative count, update the seed on the
  first sample. All in IRAM, no float ops on the position state, no
  logging, no I2C / SPI.
- **Does not**: call `setStatus()` (would race with the host),
  `magnetStatus()` / `setWatchDog()` (those are I2C-bound and live on
  `As5600I2cPwm`'s polling side), or anything that allocates.

`disableIsrUpdates()` removes the callback; the next `readPosition()`
falls back to the polling decode path and picks up where the ISR left
off. Mixing the two modes is supported but only makes sense during a
controlled handoff — pick one for steady-state operation.

## Quick start — I2C + PWM combined (`As5600I2cPwm`)

The AS5600 is happy to drive both transports at the same time: register
access (magnet status, watchdog, zero-snapshot) over I2C, and a
lower-latency angle stream on the PWM pin. `As5600I2cPwm` is exactly
that — it inherits `As5600I2c` for the diagnostic register path and
overrides the read path to use the PWM input. Capability flags reflect
both transports: `hasMagnetSensing()` and `hasWatchDog()` are `true`.

This is the right driver when:

- the read loop is hot and the I2C round-trip would dominate it, **and**
- you still want the magnet-quality / watchdog signals the chip exposes
  only over I2C.

```cpp
#include <ungula/encoder/drivers/as5600_i2c_pwm.h>
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/hal/pwm_input/drivers/pwm_input.h>

namespace enc = ungula::encoder;
namespace pwm = ungula::hal::pwm_input;

ungula::hal::i2c::I2cMaster bus(0);
pwm::drivers::PwmInput cap;

enc::drivers::As5600I2cPwm wheel("wheel", bus, cap,
                                 /*multiplexer=*/nullptr,
                                 /*channel=*/0,
                                 /*directionPin=*/4);

void setup() {
    bus.begin(/*sda=*/21, /*scl=*/22, /*hz=*/400000);
    cap.begin(/*pwmPin=*/34);

    if (!wheel.begin()) {
        // I2C side failed (chip missing / wiring fault). The PWM read
        // path can still work in degraded mode if the chip is alive
        // and the host accepts running without magnet diagnostics.
    }
    wheel.setCalibration(11.377f);
}

void loop() {
    // Hot path — angle comes from the PWM pin, no I2C traffic.
    const float deg = wheel.readAngle();

    // Slow housekeeping — magnet check is still I2C.
    if (wheel.magnetStatus() != enc::MagnetStatus::Ok) {
        // log, alarm, recover, etc.
    }
}
```

The combined driver also accepts an optional `IMultiplexer*` and
channel for I2C-side wiring, just like `As5600I2c` (the PWM input is
its own pin and is unaffected).

## Multiplexer is optional (per driver)

`IEncoder` itself has no concept of a multiplexer — that's a transport
concern that lives inside the I2C-backed drivers. For `As5600I2c`,
multiplexer support is one constructor argument:

| Wiring | Construction |
| --- | --- |
| Direct to bus | `As5600I2c enc("name", bus);` |
| Behind a `IMultiplexer` | `As5600I2c enc("name", bus, &mux, channel);` |

In both cases host code calls `enc.readAngle()` / `enc.readPosition()`
identically; channel selection is handled internally. Future SPI / PWM
/ ABI drivers won't have a multiplexer parameter at all — the
multiplexer is an I2C-bus concept.

## Changing direction

Three methods are provided for managing the direction of the readings logically:

`setDirection(ungula::encoder::Direction direction)`

and two convenient helpers:

`setDirectionClockWise()`
`setDirectionCounterClockWise()`

**Direction calls work before `begin()`.** The setting is captured
immediately and pushed to the DIR pin (when one is wired) the moment
the driver finishes initialising. The DIR pin number is fixed at
construction time — wiring is hardware, set once.

Changing the direction lets you, for example, always read positive
values if the encoder will only ever turn one way.

```cpp
    horizontal.setDirection(ungula::encoder::Direction::ClockWise);
    vertical  .setDirection(ungula::encoder::Direction::CounterClockWise);

    // or
    horizontal.setDirectionClockWise();
    vertical  .setDirectionCounterClockWise();
```

## Logging

Off by default. `enableLogging()` routes diagnostics through EmblogX with
the module tag `encoder`. Enable per-instance, not globally — debugging a
flaky encoder doesn't have to flood the log with the others.

```cpp
#include <ungula/encoder/drivers/as5600_i2c.h>

void enable_debug_for_one_encoder(ungula::encoder::drivers::As5600I2c& vertical) {
    vertical.enableLogging();   // now everything from this instance shows up
}
```

Errors flow through `log_error_m`, raw-angle traces through
`log_debug_m`. There is no global state — turning logging on for one
encoder leaves the rest quiet.

## Testing

Host-side unit tests live in `tests/`:

- `test_encoder.cpp` — full `IEncoder` contract through `EncoderFake`.
  Covers the logging toggle, capability defaults, direction-before-`begin`
  contract, status/error mapping, angle helpers, magnet status. Also
  uses `I2cEncoderFake` to verify the multiplexer routing path. **Drift
  detection**: every pure-virtual on `IEncoder` is touched through an
  `IEncoder*` reference, so any signature change without a matching
  fake update breaks the build immediately.
- `test_as5600_i2c.cpp` — driver smoke test against the desktop I2C
  stub. Verifies the driver compiles, links, behaves as `IEncoder`,
  honours pre-`begin()` direction settings, and reports the right
  errors when the bus is unreachable.

```bash
cd tests
cmake -S . -B build
cmake --build build
./build/test_ungula_encoder
./build/test_ungula_encoder_as5600
```

or just:

```bash
cd tests
chmod +x *sh
./1_build.sh
./2_run.sh
```

---

## Acknowledgements

Thanks to Claude and ChatGPT for helping on generating this documentation.

## License

MIT License — see [LICENSE](LICENSE) file.

---

## Arduino CLI symlink note (rarely relevant)

This library ships a flat forwarder header at `src/ungula_encoder.h` that
just `#include`s `ungula/encoder.h`. `library.properties` `includes=`
points at the forwarder. It only exists to work around an Arduino CLI
quirk when the library is consumed through a symlink. Host code keeps
including the real header (`#include <ungula/encoder.h>`).
