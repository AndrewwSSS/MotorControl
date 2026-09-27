#pragma once
#include <Arduino.h>
#include "Stepper.h"


// Eases a move in and out instead of starting and stopping at full speed.
//
// A stepper can only be pulled in from rest below some step rate, set by the
// rotor and the load. Command it faster and the rotor stops following the
// field: it buzzes, slips, and the counter goes on counting steps the axis
// never took. Without a ramp the top speed of an axis is therefore whatever it
// can start at, which is well below what it can sustain - the ramp is what
// buys the difference.
//
// The interval walks linearly from startIntervalUs down to the axis's own
// interval over rampSteps steps, and back up over the last rampSteps of the
// move. Linear in interval rather than in speed, so the pull away from rest is
// brisker than the approach to cruise; that is the cheap approximation, and it
// costs no division per step. The shape is decided by the pair of curves, one
// measured from each end of the move, so a move too short to reach cruise
// simply peaks where they cross - there is no special case for it.
//
// One instance can drive a whole fleet: it holds no per-move state.
//
//   LinearRamp ramp(4000, 300);     // ease in over 300 steps, from 4000us
//   axes.setProfiles(&ramp);
//
// startIntervalUs must be slower (larger) than the axis's own interval, or
// there is nothing to ramp and the profile stands aside. Keep
// startIntervalUs * rampSteps inside 32 bits, which any sane pair is.
class LinearRamp : public StepProfile {
public:
    LinearRamp(unsigned long startIntervalUs, uint16_t rampSteps)
        : startIntervalUs(startIntervalUs),
          rampSteps(rampSteps ? rampSteps : 1) {}

    unsigned long intervalFor(long stepsDone, long stepsLeft,
                              unsigned long cruiseUs) const override {
        unsigned long accelerating = at(stepsDone, cruiseUs);
        unsigned long decelerating = at(stepsLeft, cruiseUs);

        // Whichever end of the move is more restrictive wins.
        return (accelerating > decelerating) ? accelerating : decelerating;
    }

    long stopDistance() const override {
        return rampSteps;
    }

    // Tunable on the fly, which is how a ramp length gets found on a bench.
    // Shared instance, so this moves every axis using it.
    void setRampSteps(uint16_t steps) {
        rampSteps = steps ? steps : 1;
    }

    uint16_t getRampSteps() const {
        return rampSteps;
    }

    void setStartInterval(unsigned long us) {
        startIntervalUs = us;
    }

    unsigned long getStartInterval() const {
        return startIntervalUs;
    }

private:
    // The interval k steps into the ramp: startIntervalUs at k = 0, the axis's
    // own interval once k reaches rampSteps.
    unsigned long at(long k, unsigned long cruiseUs) const {
        if (startIntervalUs <= cruiseUs) return cruiseUs;   // nothing to ramp
        if (k <= 0) return startIntervalUs;
        if (k >= (long)rampSteps) return cruiseUs;

        unsigned long span = startIntervalUs - cruiseUs;
        return startIntervalUs - (span * (unsigned long)k) / rampSteps;
    }

    unsigned long startIntervalUs;
    uint16_t      rampSteps;
};
