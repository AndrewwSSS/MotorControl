# MotorControl

Project-private PlatformIO library for the Ruka arm and the CNC-shield test rig.
Everything motor-related lives here; each project's `src/main.cpp` only holds
its sketch (Bluetooth protocol, playback, serial jog console).

## Layout

| File | Role |
| --- | --- |
| `src/AngularMotor.h` | Abstract base: target clamping, non-blocking move loop, state + `reachedTarget()` edge |
| `src/ServoMotor.h` | Backed by the Arduino `Servo` library (one hardware PWM pin per motor) |
| `src/PCA9685Servo.h` | Backed by a shared PCA9685 16-channel PWM driver over I2C |
| `src/Stepper.h` | One step-driven axis: what it should do - target, endstop guard, `MOVING`/`HOLD`, `reachedTarget()` - with the pulses left to its `StepEngine` |
| `src/StepEngine.h` | Interface: how an axis's pulses get made. Owns position, target and the ramp |
| `src/SoftEngine.h` | The engine code shared whenever our own code times the pulses: counting, ramp, endstop inhibit, `tick()` |
| `src/PolledEngine.h` | A `SoftEngine` on step+dir pins, timed against `micros()` from `update()`; any board |
| `src/StepProfile.h` | Interface for a speed profile: the interval before the next pulse |
| `src/LinearRamp.h` | A `StepProfile` that eases each move in and out |
| `src/StepDirStepper.h` | A `Stepper` with its own `PolledEngine`, for any step+dir driver - A4988/DRV8825 (a CNC Shield socket), TB6600, DM542 |
| `src/Endstop.h` | A limit switch on its own pin: NO/NC wiring, debounce, which end of travel it guards |
| `src/Homing.h` | `homeAxis()`: finds an axis's base position against one of its endstops |
| `src/MotorManager.h` | Owns a fixed-size fleet of actuators, addressed by index; the part that is the same for servos and steppers |
| `src/ServoManager.h` | `MotorManager<AngularMotor>` + commands in degrees |
| `src/StepperManager.h` | `MotorManager<Stepper>` + commands in steps, and the fleet's enable line |
| `src/Log.h` | Compile-time log levels and the output stream every header logs through |

Header-only: there are no `.cpp` files, so nothing is compiled unless a
translation unit includes it.

## The two roots

`AngularMotor` models an absolute, clamped, unsigned angle written straight to
the hardware - a servo. A stepper's natural unit is a signed, unbounded step count
relative to a base position it has to earn, so it has its own root, `Stepper`.

What the two share is their lifecycle: `begin()`, a non-blocking `update()`, a
`State` of `MOVING` or `HOLD`, the one-shot `reachedTarget()` edge and the debug
toggle. That is exactly what fleet management needs, and it is all
`MotorManager<T>` touches - hence one manager, parameterised on the root, with
the unit-specific commands added by each subclass.

## Usage - servos

```cpp
#include "ServoManager.h"
#include "PCA9685Servo.h"

Adafruit_PWMServoDriver pwm;   // SDA -> A4, SCL -> A5 on an Uno
ServoManager arm(6);

void setup() {
    pwm.begin();
    pwm.setPWMFreq(50);        // must run before any PCA9685Servo::begin()

    for (uint8_t ch = 0; ch < 6; ch++)
        arm.addMotor(new PCA9685Servo(pwm, ch, /*defaultAngle*/ 90, /*rotationDelay*/ 10));

    arm.begin();
}

void loop() {
    arm.update();              // call every iteration; movement is non-blocking
}
```

## Usage - steppers

```cpp
#include "StepperManager.h"
#include "StepDirStepper.h"

StepperManager axes(2, /*shared enable pin*/ 8);

void setup() {
    axes.addMotor(new StepDirStepper(/*step*/ 2, /*dir*/ 5, /*stepsPerRev*/ 3200,
                                     StepDirStepper::NO_ENABLE_PIN,
                                     /*stepIntervalUs*/ 1000));              // id 0
    axes.addMotor(new StepDirStepper(3, 6, 3200,
                                     StepDirStepper::NO_ENABLE_PIN, 1000,
                                     StepDirStepper::TB6600_PULSE_US));      // id 1, on a TB6600

    axes.begin();

    axes.moveSteps(0, 400);    // both start now and advance together:
    axes.moveToStep(1, -800);  // update() interleaves their pulses
}

void loop() {
    axes.update();
}
```

### Engines

A `Stepper` decides what the axis does; a `StepEngine` makes the pulses. The
split is there because there is more than one good way to make them, and the
endstops, homing, ramp and fleet should not care which:

| Engine | Pulses come from | Boards |
| --- | --- | --- |
| `PolledEngine` | `update()`, timed against `micros()` | any |
| `FasEngine` (planned) | FastAccelStepper, in hardware | ESP32 |
| `TimerEngine` (planned) | a hardware timer running our own `tick()` | ESP32 |

`StepDirStepper` is a `Stepper` with a `PolledEngine` built in, which is why the
examples above need no engine. To choose one, build the two apart; the engine is
borrowed, so declare it at file scope like an endstop:

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

### Driver timing

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

### Ramping

Without a profile an axis starts and stops at one flat interval. A stepper can
only be pulled in from rest below some step rate, set by the rotor and the load -
command it faster and the rotor stops following the field, and the counter goes
on counting steps the axis never took. So a flat interval caps the top speed at
whatever the axis can *start* at, well below what it can sustain.

A profile fixes that. Attach one and it decides the pace of each pulse:

```cpp
#include "LinearRamp.h"

LinearRamp ramp(/*startIntervalUs*/ 4000, /*rampSteps*/ 400);

axes.setProfiles(&ramp);          // whole fleet, or setProfile(id, &ramp)
axes.setStepInterval(0, 400);     // now the cruise speed, reached over the ramp
```

`setStepInterval()` keeps its meaning as the speed the axis is asked for; the
profile decides how it gets there and back. `getCurrentInterval(id)` reports the
pace right now, which is what shows a ramp working on a bench.

Profiles are stateless and `const` - the move's state lives in the engine - so
one instance serves a whole fleet and costs each axis nothing but a pointer.
`nullptr` (the default) is a flat interval, exactly as before.

Two things behave differently once a profile is attached:

- `stopHere()` stays **immediate**, because that is what an endstop hit needs.
  `stopRamped()` is the one that slows down over the profile's stopping distance,
  for a stop that is asked for rather than forced.
- Homing (see below) detaches the profile for the duration of the search. The trip point of a
  snap-action switch is only repeatable at a constant low speed, which is the
  whole reason the search has a slow second pass.

To write another profile - constant acceleration, an S-curve, a lookup table -
implement `StepProfile::intervalFor()`; see `LinearRamp.h`.

`onReached(cb)` fires once per axis as it settles, instead of polling
`reachedTarget()` per motor. A mixed fleet is fine because the manager only talks
through `Stepper`; reach a subtype's own methods through `operator[]`:

```cpp
static_cast<StepDirStepper&>(axes[0]).getStepPin();
```

### Endstops and homing

A limit switch is an `Endstop`: its pin, how it is wired, and which end of travel
it sits at. Attach it to any `Stepper` and the axis refuses to step further onto
it once it is pressed; moving away stays allowed. Each axis has a slot at each
end, so a switch at both ends is two `Endstop`s on two pins:

```cpp
#include "Endstop.h"

// COM -> GND, NC -> pin. No resistor: the pin uses INPUT_PULLUP.
Endstop xMin(A4, Endstop::NORMALLY_CLOSED, Endstop::MIN);
Endstop xMax(A5, Endstop::NORMALLY_CLOSED, Endstop::MAX);

auto* x = new StepDirStepper(2, 5, 3200, StepDirStepper::NO_ENABLE_PIN, 1000);
x->setEndstop(&xMin);              // goes into the slot for its side
x->setEndstop(&xMax);
axes.addMotor(x);
```

Endstops are borrowed like profiles - declare them at file scope. `begin()` sets
up their pins. Prefer `NORMALLY_CLOSED`: a cut wire then reads as a pressed
switch and stops the axis, where `NORMALLY_OPEN` would read "all clear".

Homing is a separate, explicit, blocking step after `begin()`. It drives the axis
onto the switch at the end you name, makes the trip point step 0 and parks at
the rest step:

```cpp
#include "Homing.h"

//                  towards       restStep maxSteps backoffSteps slowIntervalUs
HomingConfig xHoming = { Endstop::MIN, 100,     20000,   40,          4000 };

axes.begin();
HomingResult r = axes.home(0, xHoming);   // energises the drivers first
if (r != HOMED) Serial.println(homingResultName(r));
```

The search is two-pass: a fast seek finds the switch, the axis backs off until it
releases, and a slow re-approach takes the reading - a snap-action switch only
trips at a repeatable point at low speed. A failed search stops and says which
stage failed (`HOMING_NEVER_CLOSED`, `HOMING_NEVER_RELEASED`, `HOMING_NOT_FOUND`,
`HOMING_NO_ENDSTOP`). `isHomed(id)` reports the outcome until `setOrigin()`
redefines step 0. Without the manager, `homeAxis(stepper, cfg)` does the same
on a single axis whose driver is already energised.

### Releasing the coils when idle

A stepper draws its full phase current standing still, and that is where most of
a rig's idle heat comes from. Hand the manager the EN pin and a delay, and it
releases the drivers once the whole fleet has been at rest that long, taking them
back by itself on the next move:

```cpp
StepperManager axes(2, /*enablePin*/ 8);
axes.setIdleRelease(2000);      // release after 2 s at rest; 0 (default) never
```

There is nothing to call before a move. `update()` energises the line as soon as
any axis has steps outstanding and withholds the pulses until the drivers have
come back up - a pulse sent into a driver that is still waking is a step the
counter claims and the axis never took. The hold is a gate in each engine
(`StepEngine::allowPulses()`), shut while the line is released and while it
wakes, so it also stops an engine that steps on its own.

The line belongs to the manager rather than to an axis because it is one pin for
the whole fleet: the CNC shield wires every socket to the same one, so no axis
can decide to drop it without stopping the others mid-move, and only the manager
knows when everything has gone quiet. Axes on such a board are built with
`NO_ENABLE_PIN` and leave it alone:

```cpp
axes.addMotor(new StepDirStepper(2, 5, 3200, StepDirStepper::NO_ENABLE_PIN, 1000));
axes.addMotor(new StepDirStepper(3, 6, 3200, StepDirStepper::NO_ENABLE_PIN, 1000));
```

An axis wired to its own EN pin keeps driving it itself, through
`StepDirStepper::enable()` and `disable()`; construct the manager without a pin and
those axes are untouched. A manager without a pin does no power management at
all, which is the default.

The cost of releasing is holding torque. A released axis can be back-driven by
gravity, a belt or a hand, and nothing notices - the step counter goes on
reporting where the axis was left, and a homed axis's base position becomes a
guess again. Use it for axes that cannot back-drive, or that are re-homed before
their position is trusted; not for one holding a load up.

## Logging

Everything the library prints goes through `Log.h`. Each project picks a level
in its `platformio.ini`:

```ini
build_flags = -D MC_LOG_LEVEL=MC_LOG_DEBUG
```

| Level | Prints |
| --- | --- |
| `MC_LOG_NONE` | nothing |
| `MC_LOG_ERROR` | failures, e.g. a homing search that did not find its switch |
| `MC_LOG_INFO` (default) | start-up: servos driven to their default angle, homing progress, a skipped seek pass |
| `MC_LOG_DEBUG` | every commanded target, for motors with `toggleDebug()` on |
| `MC_LOG_TRACE` | every step or degree moved, for motors with `toggleDebug()` on |

Messages above the chosen level are removed by the preprocessor, text and all.
`toggleDebug()` switches the DEBUG and TRACE messages per motor at runtime, but
only when the level compiles them in. TRACE prints once per pulse, which slows
the step loop down - keep it for the bench.

Output goes to `Serial` unless the project points it elsewhere. The stream is
not begun by the library; the sketch still calls `begin()` on it:

```ini
build_flags = -D MC_LOG_LEVEL=MC_LOG_INFO -D MC_LOG_STREAM=Serial1
```

Set both through `build_flags` rather than a `#define` in `main.cpp`, so every
translation unit sees the same values. Inside the library, log with
`MC_ERROR(F("text "), value, ...)` and its siblings: arguments are printed in
turn with `F()` text kept in flash, and no `String` is built.

## Adding a motor type

A different step+dir driver needs no code: give `PolledEngine` its pulse width,
and `MC_DIR_SETUP_US` its DIR setup time. A different way of making pulses is a
new `StepEngine`. If our own code still decides when each pulse is due, subclass
`SoftEngine` and supply `begin()`, `service()`, `stepsOnItsOwn()`,
`applyDirection(bool)` and `pulseStep()` - see `PolledEngine.h`. Otherwise
implement `StepEngine` whole. Either way, any `Stepper` can drive it and any
`StepperManager` fleet can hold that `Stepper`.

For a servo, subclass `AngularMotor` and implement `begin()` (hardware setup + drive to
the default angle) and `writeAngle(uint16_t)` (one hardware write). `AngularMotor`'s
degree-by-degree `update()` suits anything that takes an absolute angle command;
if it does not fit, the actuator probably wants its own root rather than an
`update()` override - as `Stepper` is.

A stepper that should be commanded in degrees and join a servo fleet is the one
case that crosses over: implement `AngularMotor` and hold a `Stepper` underneath,
converting with `getStepsPerRevolution()`.

## Adding a manager

Only if a third root appears. `MotorManager<T>` needs nothing from `T` beyond
`begin()`, `update()`, `getState()`, `reachedTarget()`, `toggleDebug()`,
`getDebug()` and a `State` enum with `MOVING` and `HOLD`; subclass it and add the
commands in whatever unit that actuator speaks.

The fleet loop itself is `beginMotors()` / `updateMotors()`, which are protected:
the subclass publishes `begin()` and `update()`, wrapping them with whatever it
has to do around the fleet. `StepperManager` is the example - its `update()` is
where the enable line is energised and the pulses withheld while the drivers come
up - and because the base pair is protected, nothing can call past a subclass and
skip that.

## Tests

The logic runs on the host against a fake Arduino core (`test/fake/`), so it is
checked without a board:

```sh
pio test -e native     # unit tests only
scripts/check.sh       # unit tests, plus the examples built for each board
```

`test/fake/FakeEngine.h` is a `StepEngine` with no pins, for testing `Stepper`,
homing and the fleet on their own - both as a polled engine and as one that
steps on its own. Projects that use the library have their own
`scripts/check.sh`; run it after a library change too.
