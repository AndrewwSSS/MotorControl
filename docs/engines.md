# Engines

A `Stepper` decides what the axis does; a `StepEngine` makes the pulses. The
split is there because there is more than one good way to make them, and the
endstops, homing, ramp and fleet should not care which:

| Engine | Pulses come from | Boards |
| --- | --- | --- |
| `PolledEngine` | `update()`, timed against `micros()` | any |
| `FasEngine` | FastAccelStepper, in hardware | ESP32, with `gin66/FastAccelStepper@1.4.0` in the sketch's `lib_deps` |
| `TimerEngine` (planned) | a hardware timer running our own `tick()` | ESP32 |

`StepDirStepper` is a `Stepper` with a `PolledEngine` built in, which is why the
[README](../README.md) examples need no engine. To choose one, name it in an `EngineStepper`; the
arguments after the steps per revolution go to the engine:

```cpp
#include "EngineStepper.h"
#include "PolledEngine.h"

axes.addMotor(new EngineStepper<PolledEngine>(/*stepsPerRev*/ 3200,
                                              /*step*/ 2, /*dir*/ 5));
```

Or build the two apart, when the engine has to live somewhere else; a plain
`Stepper` borrows its engine, so declare it at file scope like an endstop:

```cpp
#include "PolledEngine.h"
#include "Stepper.h"

PolledEngine xEngine(/*step*/ 2, /*dir*/ 5, PolledEngine::NO_ENABLE_PIN,
                     /*stepIntervalUs*/ 1000);

axes.addMotor(new Stepper(xEngine, /*stepsPerRev*/ 3200));
```

A polled engine is only as steady as the loop that calls it: anything that holds
`loop()` up holds the next pulse up too. An engine that steps on its own
(`stepsOnItsOwn()`) keeps going in between, and homing then checks on the axis
once a millisecond instead of spinning on `update()`.

`FasEngine` is such an engine. Its STEP pulse is half the step interval, so it
takes no pulse width, and its ramp is the profile's `accelerationFor()`. A move
that reverses a running one ramps down first and then goes back, where a polled
engine reverses at full speed. See [`examples/FasEngine`](../examples/FasEngine/FasEngine.ino).

## Driver timing

`PolledEngine` (and so `StepDirStepper`) drives any step+dir driver; what differs
between them is timing.

- **STEP pulse width** is the last constructor argument: `A4988_PULSE_US` (2 µs,
  the default, also fine for a DRV8825) or `TB6600_PULSE_US` (5 µs), or any
  number from the driver's datasheet.
- **DIR setup** - how long DIR settles before the next STEP edge - is
  `MC_DIR_SETUP_US`, 5 µs by default, and only spent when the direction
  changes. Set it per project with `build_flags = -D MC_DIR_SETUP_US=10`, or
  override `applyDirection()` in a `PolledEngine` subclass for anything other
  than a fixed wait.

An opto-isolated driver (TB6600, DM542) must be wired common-cathode - PUL-,
DIR-, ENA- to GND and the + inputs to the pins - so that HIGH means active, as
the class assumes.
