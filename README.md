# UngulaEncoder

> **High-performance embedded C++ for ESP32 and friends** — magnetic / optical encoder drivers.

Chip-neutral encoder interface plus drivers. Designed around two real
deployments that look different on the board but identical in code:

1. **One bus, several encoders, one multiplexer** (the original "Rachel"
   rig — two AS5600s sharing a TCA9548A).
2. **Multiple buses, no multiplexer** (the new project — every encoder
   wired to its own I2C peripheral or to dedicated channels).

Both work through the same `IEncoder` API. The multiplexer is **optional**.

## Table of contents

- [Features](#features)
- [Supported chips](#supported-chips)
- [Dependencies](#dependencies)
- [Architecture](#architecture)
- [Quick start — direct connect](#quick-start--direct-connect)
- [Quick start — behind a multiplexer](#quick-start--behind-a-multiplexer)
- [Multiplexer is optional](#multiplexer-is-optional)
- [Logging](#logging)
- [Testing](#testing)
- [License](#license)

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

More chips (AS5047P SPI, MA730 SPI, MT6835 SPI, MT6701 ABI) and more transports for AS5600 (PWM input, I2C+PWM combined) are queued for follow-up phases. The `IEncoder` interface is now transport-agnostic so they slot in without breaking host code.

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
