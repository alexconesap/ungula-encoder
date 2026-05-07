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
- Calibrated output (cumulative position in encoder steps, plus `angleFrom*` helpers in degrees).
- Wrap-around handled — the AS5600 returns 12-bit raw angles; the driver tracks 4095↔0 transitions and accumulates correctly.
- Per-instance EmblogX logging toggle. Off by default.
- Header-only `EncoderFake` for tests; locks the interface so a future signature change cannot silently break drivers.

## Supported chips

| Chip   | Bus | Resolution | Driver |
| ------ | --- | ---------- | ------ |
| AS5600 | I2C (`I2cMaster`) | 12 bit / 4096 steps | `ungula::encoder::drivers::AS5600` |

## Dependencies

- `UngulaCore` — `ungula::core::time::delay*` for I2C pacing.
- `UngulaHal` — `i2c::I2cMaster`, `gpio` for the optional DIR pin, `multiplexer::IMultiplexer` for the optional channel select.
- `EmblogX` — runtime diagnostics. Logging is off by default.

### Include reference

- Core encoder API only:

```cpp
#include <ungula/encoder.h>
```

- AS5600 direct-connect example:

```cpp
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/encoder/drivers/as5600.h>
```

- AS5600 + TCA9548A multiplexer example:

```cpp
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/hal/multiplexer/drivers/multiplexer_tca9548.h>
#include <ungula/encoder/drivers/as5600.h>
```

## Architecture

```text
ungula::encoder
├── IEncoder                       ← chip-neutral interface
│    ├── enableLogging() / disableLogging()
│    └── selectMultiplexerChannel() — no-op when no mux
└── drivers/
     ├── AS5600                    ← real driver (I2cMaster + optional GPIO)
     └── EncoderFake               ← header-only test fake
```

Drivers do not depend on Arduino. All time and pin operations go through
`ungula::core::time` and `ungula::hal::gpio`.

## Quick start — direct connect

Encoder wired straight to the MCU's I2C peripheral. No multiplexer.

```cpp
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/encoder/drivers/as5600.h>

namespace enc = ungula::encoder;

ungula::hal::i2c::I2cMaster bus(0);
enc::drivers::AS5600 wheel("wheel", bus);   // multiplexer = nullptr

void setup() {
    bus.begin(/*sda=*/21, /*scl=*/22, /*hz=*/400000);
    if (!wheel.begin(/*channel=*/0,        // ignored, no multiplexer
                     /*directionPin=*/4)) {
        // wheel.statusToStr() / wheel.getLastErrorAsStr()
    }
}

void loop() {
    const float pos = wheel.readPosition();
    const float deg = wheel.angleFromCurrentPosition(/*steps_per_deg=*/11.377f);
    // …
}
```

`wheel.hasMultiplexer()` is `false` here. `selectMultiplexerChannel()`
returns true without touching any I2C bus.

## Quick start — behind a multiplexer

Two AS5600s sharing one TCA9548A on a single bus. The pattern from the
Rachel project.

```cpp
#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/hal/multiplexer/drivers/multiplexer_tca9548.h>
#include <ungula/encoder/drivers/as5600.h>

namespace enc = ungula::encoder;
namespace mux = ungula::hal::multiplexer;

ungula::hal::i2c::I2cMaster bus(0);
mux::drivers::MultiplexerTCA9548 mux70(0x70, bus);

enc::drivers::AS5600 horizontal("horizontal", bus, &mux70);
enc::drivers::AS5600 vertical  ("vertical",   bus, &mux70);

void setup() {
    bus.begin(21, 22, 400000);
    mux70.begin();
    horizontal.begin(/*channel=*/0, ungula::encoder::ENCODER_NO_DIRECTION_PIN);
    vertical  .begin(/*channel=*/1, ungula::encoder::ENCODER_NO_DIRECTION_PIN);
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

## Multiplexer is optional

The same driver, the same code path, two deployments:

| Wiring | Construction | `selectMultiplexerChannel()` |
| --- | --- | --- |
| Direct to bus | `AS5600 enc("name", bus);` | no-op, returns true |
| Behind a `IMultiplexer` | `AS5600 enc("name", bus, &mux);` | calls `mux.selectChannel(channel)` |

The contract is checked by the test suite. Both modes share the same
guard against `begin()` not being called yet — `readPosition()` returns
NaN with `Error::NotInitialized` either way.

## Logging

Off by default. `enableLogging()` routes diagnostics through EmblogX with
the module tag `encoder`. Enable per-instance, not globally — debugging a
flaky encoder doesn't have to flood the log with the others.

```cpp
#include <ungula/encoder/drivers/as5600.h>

void enable_debug_for_one_encoder(ungula::encoder::drivers::AS5600& vertical) {
    vertical.enableLogging();   // now everything from this instance shows up
}
```

Errors flow through `log_error_m`, raw-angle traces through
`log_debug_m`. There is no global state — turning logging on for one
encoder leaves the rest quiet.

## Testing

Host-side unit tests live in `tests/`:

- `test_encoder.cpp` — full `IEncoder` contract through `EncoderFake`.
  Covers both deployment modes (with/without multiplexer), the logging
  toggle, status/error mapping, angle helpers, magnet status. **Drift
  detection**: every pure-virtual on `IEncoder` is touched through an
  `IEncoder*` reference, so any signature change without a matching
  fake update breaks the build immediately.
- `test_as5600.cpp` — driver smoke test against the desktop I2C stub.
  Verifies the driver compiles, links, behaves as `IEncoder`, and
  reports the right errors when the bus is unreachable.

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
