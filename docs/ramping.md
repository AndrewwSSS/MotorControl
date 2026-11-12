# Ramping

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
- Homing (see [Endstops and homing](homing.md)) detaches the profile for the duration of the search. The trip point of a
  snap-action switch is only repeatable at a constant low speed, which is the
  whole reason the search has a slow second pass.

## How `LinearRamp` shapes a move

The interval goes linearly from `startIntervalUs` down to the axis's own
interval over the first `rampSteps` steps, and back up over the last
`rampSteps`. It is linear in *interval*, not in speed, so the pull away from
rest is brisker than the approach to cruise. That is a cheap approximation: it
needs no division per step.

The shape comes from two curves, one measured from each end of the move, and the
slower of the two wins at every step. A move too short to reach cruise speed
simply peaks where the curves cross, with no special case.

- `startIntervalUs` must be larger (slower) than the axis's own interval;
  otherwise there is nothing to ramp and the profile has no effect.
- Keep `startIntervalUs * rampSteps` within 32 bits.
- `setRampSteps()` and `setStartInterval()` work while running, which is how a
  ramp is tuned on a bench. The instance is shared, so a change affects every
  axis that uses it.
- For engines that ramp in hardware, `accelerationFor()` turns the ramp into one
  constant acceleration, (v_top² − v_start²) / (2 · rampSteps) in steps/s². It
  is approximate, because the ramp is linear in interval, but moves start,
  cruise and end in the same places: 1000 µs down to 200 µs over 1000 steps
  gives 12,000 steps/s². It uses integer maths only, so AVR builds don't pull in
  floating point.

To write another profile - constant acceleration, an S-curve, a lookup table -
implement `StepProfile::intervalFor()`; see `LinearRamp.h`.

`onReached(cb)` fires once per axis as it settles, instead of polling
`reachedTarget()` per motor. A mixed fleet is fine because the manager only talks
through `Stepper`; reach a subtype's own methods through `operator[]`:

```cpp
static_cast<StepDirStepper&>(axes[0]).getStepPin();
```
