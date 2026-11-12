#pragma once
#include <Arduino.h>
#include "StepProfile.h"

// DIR setup time before the next STEP edge, in microseconds; spent only on a
// direction change. 5 us covers the slowest (opto-isolated) drivers.
#ifndef MC_DIR_SETUP_US
#define MC_DIR_SETUP_US 5
#endif

// Makes an axis's step pulses and owns its position, target and ramp; the
// Stepper decides what the axis does. Positions are signed steps from the base
// position. Borrowed by the Stepper, so it must outlive it. See docs/engines.md.
class StepEngine {
public:
    virtual ~StepEngine() = default;

    // Sets up pins or hardware. Called by Stepper::begin().
    virtual void begin() = 0;

    // --- Motion --------------------------------------------------------------

    // Absolute move. A move commanded mid-move continues the current ramp.
    virtual void moveTo(long step) = 0;

    // Stops immediately: the current position becomes the target.
    virtual void stopNow() = 0;

    // Stops over the profile's stopping distance, never past the move's end.
    // Same as stopNow() without a profile.
    virtual void stopRamped() = 0;

    // Redefines the current position and makes it the target.
    virtual void setPosition(long step) = 0;

    virtual long position() const = 0;
    virtual long target() const = 0;

    // True while there are steps left to make.
    virtual bool isRunning() const = 0;

    // --- Pace ----------------------------------------------------------------

    // The flat interval between pulses, or the cruise interval under a profile.
    virtual void setStepInterval(unsigned long us) = 0;
    virtual unsigned long stepInterval() const = 0;

    // The interval right now, which differs from stepInterval() while ramping.
    virtual unsigned long currentInterval() const = 0;

    // Borrowed; nullptr for a flat interval.
    virtual void setProfile(const StepProfile* p) = 0;
    virtual const StepProfile* profile() const = 0;

    // --- Endstops ------------------------------------------------------------

    // Direction blocked by a pressed endstop: +1 (MAX), -1 (MIN) or 0. Set by
    // the Stepper on every update() and enforced here, so an engine that steps
    // on its own refuses pulses between updates. A move that way is cancelled.
    virtual void inhibit(int8_t direction) = 0;

    // --- Work ----------------------------------------------------------------

    // Called on every Stepper::update(). Returns true if it made a pulse; a
    // polled engine makes at most one.
    virtual bool service() = 0;

    // True when pulses come from a timer or hardware rather than service().
    virtual bool stepsOnItsOwn() const = 0;

    // The fleet's pulse gate. While shut, moves wait and no pulse goes out:
    // StepperManager shuts it while the drivers are released or waking, when a
    // pulse would be counted but not taken. Open by default.
    virtual void allowPulses(bool allowed) = 0;

    // --- Driver enable -------------------------------------------------------

    // For an engine that owns its driver's EN pin; no-ops by default.
    virtual void enable() {}
    virtual void disable() {}
};
