# Releasing the coils when idle

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
