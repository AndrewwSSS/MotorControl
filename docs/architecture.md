# Architecture

## Layout

| File | Role |
| --- | --- |
| `src/AngularMotor.h` | Abstract base: target clamping, non-blocking move loop, state + `reachedTarget()` edge |
| `src/ServoMotor.h` | Backed by the Arduino `Servo` library, or `ESP32Servo` on the ESP32 (one PWM pin per motor) |
| `src/PCA9685Servo.h` | Backed by a shared PCA9685 16-channel PWM driver over I2C |
| `src/Stepper.h` | One step-driven axis: what it should do - target, endstop guard, `MOVING`/`HOLD`, `reachedTarget()` - with the pulses left to its `StepEngine` |
| `src/StepEngine.h` | Interface: how an axis's pulses get made. Owns position, target and the ramp |
| `src/SoftEngine.h` | The engine code shared whenever our own code times the pulses: counting, ramp, endstop inhibit, `tick()` |
| `src/PolledEngine.h` | A `SoftEngine` on step+dir pins, timed against `micros()` from `update()`; any board |
| `src/FasEngine.h` | A `StepEngine` on FastAccelStepper: pulses and ramp in hardware. ESP32; the sketch adds the library |
| `src/StepProfile.h` | Interface for a speed profile: the interval before the next pulse |
| `src/LinearRamp.h` | A `StepProfile` that eases each move in and out |
| `src/EngineStepper.h` | A `Stepper` that owns its engine, of any kind: one `new` gives a whole axis |
| `src/StepDirStepper.h` | An `EngineStepper<PolledEngine>`, for any step+dir driver - A4988/DRV8825 (a CNC Shield socket), TB6600, DM542 |
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
