#pragma once
#include <Arduino.h>

// A speed profile: the interval before the next pulse, given the move's
// progress. Stateless, so one instance can serve a whole fleet. See
// docs/ramping.md.
class StepProfile {
public:
    virtual ~StepProfile() = default;

    // stepsDone  how far this move has travelled from where it started
    // stepsLeft  how far it still has to go
    // cruiseUs   the interval the axis was asked for, i.e. its top speed
    virtual unsigned long intervalFor(long stepsDone, long stepsLeft,
                                      unsigned long cruiseUs) const = 0;

    // Steps needed to stop from full speed, for stopRamped(); 0 stops dead.
    virtual long stopDistance() const { return 0; }

    // The profile as one constant acceleration in steps/s^2, for engines that
    // ramp in hardware. 0 means no ramp.
    virtual uint32_t accelerationFor(unsigned long cruiseUs) const {
        (void)cruiseUs;
        return 0;
    }
};
