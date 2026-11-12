# Testing

The library's logic runs on the host, against fakes, so it is tested without a
board:

```sh
pio test -e native     # unit tests only
scripts/check.sh       # unit tests, plus every example built for each board
```

CI runs `scripts/check.sh` on every push and pull request.

## The fake Arduino core

[`test/fake/Arduino.h`](../test/fake/Arduino.h) is just enough of the Arduino
core for the headers to compile and run on the host. Only the `native`
environment puts `test/fake/` on the include path; board builds use their real
core. Nothing touches hardware, so a test controls everything a sketch would get
from outside:

| Area | Behaviour |
| --- | --- |
| Time | Stands still until the test moves it (`fake::advanceUs`), or moves a fixed amount on every `micros()` call (`fake::setAutoAdvance`), so busy-wait loops such as homing make progress. |
| Writes | Every `digitalWrite` is recorded with its time, so tests can count step pulses and check their spacing. |
| Reads | `digitalRead` returns what the test scripted, per pin or through a handler that can follow the axis (for example, a switch that closes at some step). |
| Serial | Collects everything printed, log output included. |

`fake::reset()` returns to power-on: time 0, no writes, every input reading HIGH
(a pulled-up pin), Serial empty. Call it from `setUp()`.

`F()` strings keep their own type, as on AVR, so overloads resolve exactly as
they do on a board.

**Known gap:** `unsigned long` is 64 bits on the host and 32 bits on the boards,
so the `micros()` rollover cannot be reproduced. Tests keep time well clear of it.

## FakeEngine

[`test/fake/FakeEngine.h`](../test/fake/FakeEngine.h) is a `StepEngine` with no
pins and no timing, for testing the logic above the engine (`Stepper`, homing,
the fleet) on its own. It records what it was told and moves in one of two
modes:

- **POLLED:** one step per `service()`, which returns true, like `PolledEngine`.
- **OWN:** `stepsPerService` steps per `service()`, which returns false, like an
  engine that steps on its own. The steps come in a burst, the worst case for
  how late the endstop guard sees a switch.

Each step moves fake time on by the step interval, so endstop debounce runs as
it would on a real axis. `shaft()` is where the motor really is: unlike
`position()`, `setPosition()` does not reset it, so homing tests can check where
step 0 ended up.

## Fake FastAccelStepper

[`test/fake/FastAccelStepper.h`](../test/fake/FastAccelStepper.h) stands in for
FastAccelStepper 1.4.0, with only the calls `FasEngine` makes. It copies the
library's behaviour where that shapes `FasEngine`:

- `moveTo()` while running is ignored when it would reverse the move, or stop it
  where it is.
- `stopMove()` ramps down over `stepsToStop()` steps, never past the target.
- `forceStopAndNewPosition()` stops at once.
- Intervals faster than the hardware can step are refused.

Nothing moves on its own: a test calls `run(n)` to let up to n steps go out.
