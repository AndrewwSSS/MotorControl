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

### How the search behaves

- **Starting on the switch:** the seek pass is skipped and the axis goes straight
  to the back-off. The base position always comes from a fresh closing edge
  during the slow re-approach.
- **Back-off:** the axis retreats in `backoffSteps` increments until the switch
  releases, not by one fixed distance, because it may start further past the
  trip point than any single increment (pushed by hand, or left there by an
  earlier failed run). The retreat is bounded by `maxSteps`, so a contact stuck
  closed fails instead of driving the axis off the far end.
- **Both switches pressed:** if the switch at the other end stops the retreat,
  the axis is pinned between two "pressed" switches. That is a wiring fault, and
  homing reports it instead of retrying.
- **A move in flight** is cancelled before the search starts, so it is not added
  to the seek.
- **The speed profile** is detached during the search and restored for the move
  to the rest step.
- **Soft limits** are switched off during the search, which has to run past them
  to find the switch. They are armed when homing succeeds, and restored as they
  were when it fails.
- **No reached callback** fires for the search's own stops, only for moves the
  sketch commands.
- **Engines that step on their own** are checked once a millisecond during the
  search instead of spinning on `update()`, leaving the CPU to other tasks.

## Endstop debounce

A snap-action switch chatters for a few milliseconds around the transition.
`Endstop` acts on the first edge it sees and then ignores the pin until the
contacts settle: waiting for a stable level would delay detection and let the
axis overshoot. A homing run reads the pin directly (`resync()`), which opens no
ignore window, so a press right after it is seen at once.

## Soft limits

Soft limits are the range of steps an axis may be sent to:

```cpp
axes.setLimits(0, /*minStep*/ 0, /*maxStep*/ 20000);
```

- A move whose target is outside the range is refused and logged; the axis keeps
  its previous target. An axis already outside the range may still be sent back
  towards it, so it never gets stuck.
- Limits only mean something once step 0 is a known place, so they are enforced
  only when **armed**: after a successful `home()`, or after `setOrigin()`
  defines step 0 by hand.
- A move already running past a new limit is cut short at the limit.
