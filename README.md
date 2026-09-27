# MotorControl

Project-private PlatformIO library for the Ruka arm and the CNC-shield test rig.
Everything motor-related lives here; each project's `src/main.cpp` only holds
its sketch (Bluetooth protocol, playback, serial jog console).

## Layout

| File | Role |
| --- | --- |
| `src/Motor.h` | Abstract base: target clamping, non-blocking move loop, state + `reachedTarget()` edge |
| `src/ServoMotor.h` | Backed by the Arduino `Servo` library (one hardware PWM pin per motor) |
| `src/PCA9685Servo.h` | Backed by a shared PCA9685 16-channel PWM driver over I2C |
| `src/Stepper.h` | Abstract stepper base: position in steps from a base position, non-blocking step loop, `StepProfile` seam |
| `src/LinearRamp.h` | A `StepProfile` that eases each move in and out |
| `src/A4988Stepper.h` | Stepper on an A4988/DRV8825 step+dir driver (a CNC Shield socket); owns only the pins |
| `src/HomedStepper.h` | A4988Stepper that finds its base position against an endstop in `begin()` |
| `src/MotorManager.h` | Owns a fixed-size fleet of actuators, addressed by index; the part that is the same for servos and steppers |
| `src/ServoManager.h` | `MotorManager<Motor>` + commands in degrees |
| `src/StepperManager.h` | `MotorManager<Stepper>` + commands in steps, and the fleet's enable line |

Header-only: there are no `.cpp` files, so nothing is compiled unless a
translation unit includes it.

## The two roots

`Motor` models an absolute, clamped, unsigned angle written straight to the
hardware - a servo. A stepper's natural unit is a signed, unbounded step count
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
#include "A4988Stepper.h"

StepperManager axes(2, /*shared enable pin*/ 8);

void setup() {
    axes.addMotor(new A4988Stepper(/*step*/ 2, /*dir*/ 5, /*stepsPerRev*/ 3200,
                                   A4988Stepper::NO_ENABLE_PIN,
                                   /*stepIntervalUs*/ 1000));                // id 0
    axes.addMotor(new A4988Stepper(3, 6, 3200,
                                   A4988Stepper::NO_ENABLE_PIN, 1000));      // id 1

    axes.begin();

    axes.moveSteps(0, 400);    // both start now and advance together:
    axes.moveToStep(1, -800);  // update() interleaves their pulses
}

void loop() {
    axes.update();
}
```

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

Profiles are stateless and `const` - the move's state lives on the `Stepper` - so
one instance serves a whole fleet and costs each axis nothing but a pointer.
`nullptr` (the default) is a flat interval, exactly as before.

Two things behave differently once a profile is attached:

- `stopHere()` stays **immediate**, because that is what an endstop hit needs.
  `stopRamped()` is the one that slows down over the profile's stopping distance,
  for a stop that is asked for rather than forced.
- Homing detaches the profile for the duration of the search. The trip point of a
  snap-action switch is only repeatable at a constant low speed, which is the
  whole reason the search has a slow second pass.

To write another profile - constant acceleration, an S-curve, a lookup table -
implement `StepProfile::intervalFor()`; see `LinearRamp.h`.

`onReached(cb)` fires once per axis as it settles, instead of polling
`reachedTarget()` per motor. A mixed fleet is fine - `HomedStepper` next to a
plain `A4988Stepper` - because the manager only talks through `Stepper`; reach a
subtype's own methods through `operator[]`:

```cpp
static_cast<HomedStepper&>(axes[0]).isHomed();
```

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
counter claims and the axis never took.

The line belongs to the manager rather than to an axis because it is one pin for
the whole fleet: the CNC shield wires every socket to the same one, so no axis
can decide to drop it without stopping the others mid-move, and only the manager
knows when everything has gone quiet. Axes on such a board are built with
`NO_ENABLE_PIN` and leave it alone:

```cpp
axes.addMotor(new A4988Stepper(2, 5, 3200, A4988Stepper::NO_ENABLE_PIN, 1000));
axes.addMotor(new A4988Stepper(3, 6, 3200, A4988Stepper::NO_ENABLE_PIN, 1000));
```

An axis wired to its own EN pin keeps driving it itself, through
`A4988Stepper::enable()` and `disable()`; construct the manager without a pin and
those axes are untouched. A manager without a pin does no power management at
all, which is the default.

The cost of releasing is holding torque. A released axis can be back-driven by
gravity, a belt or a hand, and nothing notices - the step counter goes on
reporting where the axis was left, and a `HomedStepper`'s base position becomes a
guess again. Use it for axes that cannot back-drive, or that are re-homed before
their position is trusted; not for one holding a load up.

## Adding a motor type

To support a different stepper driver chip, subclass `Stepper` and implement
`applyDirection(bool)` and `pulseStep()` - see `A4988Stepper.h`. It can then join
any `StepperManager` fleet.

For a servo, subclass `Motor` and implement `begin()` (hardware setup + drive to
the default angle) and `writeAngle(uint16_t)` (one hardware write). `Motor`'s
degree-by-degree `update()` suits anything that takes an absolute angle command;
if it does not fit, the actuator probably wants its own root rather than an
`update()` override - see `Stepper.h`.

A stepper that should be commanded in degrees and join a servo fleet is the one
case that crosses over: implement `Motor` and hold a `Stepper` underneath,
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
