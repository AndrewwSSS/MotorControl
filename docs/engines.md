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
[usage](usage.md) examples need no engine. To choose one, name it in an `EngineStepper`; the
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

The engine, not the `Stepper`, owns the position, the target and the ramp:
an engine that steps on its own is the only part that knows where the axis is
between two `update()` calls. Positions are signed step counts from the base
position.

## FasEngine

`FasEngine` is an engine that steps on its own. FastAccelStepper makes its
pulses in hardware on the ESP32 (MCPWM, counted by PCNT), so the pace holds
however busy `loop()` is. See [`examples/FasEngine`](../examples/FasEngine/FasEngine.ino).

- **Pulse width:** the STEP pulse is half the step interval, so there is no
  pulse width to set. A TB6600's 5 µs is met by any interval of 10 µs or more.
- **DIR setup:** the library's direction-change delay, which rounds
  `MC_DIR_SETUP_US` up to its own minimum (200 µs on the ESP32). It is only
  spent on a reversal.
- **Ramp:** the library ramps from one constant acceleration, the profile's
  `accelerationFor()`.

Where FastAccelStepper and `StepEngine` behave differently, `FasEngine` bridges
the gap:

- **Reversals:** the library ignores a move that reverses a running one. Here
  the axis ramps down first, and `service()` sends the new move once it stops.
  A polled engine reverses at full speed.
- **Immediate stops:** `stopNow()` drops the library's queue at once, instead of
  letting about 20 ms of queued steps run out, which at speed would be a hundred
  steps onto a switch.
- **The fleet's pulse gate** does not exist in the library. A move commanded
  while it is shut waits; a move running when it shuts stops and resumes later.
- **One library instance** runs every axis. `FasEngine` starts it on first use,
  so an axis can be built from its pins alone, like any other engine.

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
