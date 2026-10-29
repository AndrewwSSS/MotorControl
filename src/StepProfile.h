#pragma once
#include <Arduino.h>


// A speed profile: how long to wait before the next pulse, given where the move
// has got to. Attach one to a Stepper and it decides the pace; attach none (the
// default) and the axis runs at one flat interval, as it always has.
//
// Stateless and const on purpose - the move's state belongs to the axis, so a
// single profile instance can be shared by every axis in a fleet, and costs
// each of them nothing but the pointer. Concrete profiles live in their own
// headers (LinearRamp.h); a sketch that wants none includes none.
class StepProfile {
public:
    virtual ~StepProfile() = default;

    // stepsDone  how far this move has travelled from where it started
    // stepsLeft  how far it still has to go
    // cruiseUs   the interval the axis was asked for, i.e. its top speed
    virtual unsigned long intervalFor(long stepsDone, long stepsLeft,
                                      unsigned long cruiseUs) const = 0;

    // Steps needed to come to rest from full speed, for stopRamped(). 0 says
    // the axis can stop dead.
    virtual long stopDistance() const { return 0; }
};
