#pragma once
#include <Arduino.h>
#include "Stepper.h"


// Eases a move in and out: the interval goes linearly from startIntervalUs to
// the axis's own interval over rampSteps at each end. Stateless, so one
// instance can serve a fleet. Shape and limits: docs/ramping.md.
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

    // (v_top^2 - v_start^2) / (2 * rampSteps), with v = 1e6 / interval steps/s.
    // Approximate (the ramp is linear in interval). Integer-only, so AVR builds
    // avoid float maths; saturates at UINT32_MAX.
    uint32_t accelerationFor(unsigned long cruiseUs) const override {
        if (cruiseUs == 0 || startIntervalUs <= cruiseUs) return 0;   // nothing to ramp

        uint64_t top   = 1000000UL / cruiseUs;
        uint64_t start = 1000000UL / startIntervalUs;
        uint64_t a = (top * top - start * start) / (2UL * rampSteps);
        return (a > UINT32_MAX) ? UINT32_MAX : (uint32_t)a;
    }

    // Takes effect at once for every axis sharing this instance.
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
    // The interval k steps into the ramp.
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
