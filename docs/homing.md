# Endstops and homing

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
