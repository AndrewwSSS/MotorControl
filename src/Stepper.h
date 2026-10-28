#pragma once
#include <Arduino.h>
#include "Log.h"
#include "Endstop.h"


// A speed profile: how long to wait before the next pulse, given where the move
// has got to. Attach one to a Stepper and it decides the pace; attach none (the
// default) and the axis runs at one flat interval, as it always has.
//
// Stateless and const on purpose - the move's state belongs to the Stepper, so
// a single profile instance can be shared by every axis in a fleet, and costs
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


class Stepper {
public:
    enum State {
        MOVING,
        HOLD
    };

    virtual ~Stepper() = default;

    virtual void begin() {
        setOrigin(0);
        lastStepAt = 0;

        for (uint8_t i = 0; i < 2; i++) {
            if (endstops[i]) endstops[i]->begin();
        }
    }

    // Emits at most one step pulse and returns immediately. Call repeatedly
    // from loop(); movement is non-blocking.
    virtual void update() {
        updateEndstops();

        if (currentStep == targetStep) {
            setState(HOLD);
            return;
        }

        // Driving further onto a pressed endstop crashes the axis. Stop and
        // hold; moving away from the switch stays allowed.
        Endstop* hit = blockingEndstop();
        if (hit) {
            MC_INFO(F("Endstop "), hit->sideName(), F(" on pin "), hit->getPin(),
                    F(" stopped the axis at step "), currentStep);
            stopHere();
            setState(HOLD);
            return;
        }

        unsigned long now = micros();
        if (now - lastStepAt < currentInterval()) return;

        bool forward = (targetStep > currentStep);
        applyDirection(forward);
        pulseStep();

        currentStep += forward ? 1 : -1;
        lastStepAt = now;
        setState(MOVING);

        if (debugEnabled) MC_TRACE(F("Current step: "), currentStep, F(" of "), targetStep);
    }

    // Absolute move, in steps from the base position. May be negative.
    void moveToStep(long step) {
        // A move commanded while the axis is already running composes onto the
        // one in flight, so it keeps that move's starting point: restarting the
        // ramp would slow a running axis back down to its starting speed.
        if (currentStep == targetStep) moveStartStep = currentStep;

        targetStep = step;

        if (debugEnabled) MC_DEBUG(F("Target step: "), targetStep);
    }

    // Relative move. Anchored on the target rather than the current position
    // so repeated jogs compose while a move is still in flight.
    void moveSteps(long deltaSteps) {
        moveToStep(targetStep + deltaSteps);
    }

    // Cancels a move in flight by making the current position the target.
    // Immediate: the axis stops on the next pulse it was going to emit. This is
    // what an endstop hit wants, ramp or no ramp.
    void stopHere() {
        moveToStep(currentStep);
    }

    // Brings the axis to rest over the profile's stopping distance instead of
    // dropping it dead, for a stop that is asked for rather than forced. Same
    // thing as stopHere() when there is no profile, and it never extends a move
    // that was already shorter than the stopping distance.
    void stopRamped() {
        if (currentStep == targetStep) return;

        long left = targetStep - currentStep;
        long dir = (left > 0) ? 1 : -1;
        long span = profile ? profile->stopDistance() : 0;
        long remaining = (left > 0) ? left : -left;

        if (span > remaining) span = remaining;

        moveToStep(currentStep + dir * span);
    }

    // Redefines where the axis believes it is
    void setOrigin(long step = 0) {
        currentStep = step;
        targetStep = step;
        moveStartStep = step;
        setState(HOLD);
    }

    long getStep() const {
        return currentStep;
    }

    long getTargetStep() const {
        return targetStep;
    }

    State getState() const {
        return state;
    }

    // True exactly once, the first time this is polled after the axis settles
    // into HOLD.
    bool reachedTarget() {
        bool r = justReached;
        justReached = false;
        return r;
    }

    // Attach a limit switch. It goes into the slot for its own side, so an axis
    // with a switch at each end takes two calls. Borrowed, not owned. Attach
    // before begin(), which sets up the pin.
    void setEndstop(Endstop* endstop) {
        if (endstop) endstops[slot(endstop->getSide())] = endstop;
    }

    void removeEndstop(Endstop::Side side) {
        endstops[slot(side)] = nullptr;
    }

    // nullptr when there is no switch at that end.
    Endstop* getEndstop(Endstop::Side side) const {
        return endstops[slot(side)];
    }

    // Attach a speed profile, or nullptr for a flat interval (the default).
    // The profile is borrowed, not owned - it is expected to outlive the axis,
    // which a file-scope instance in the sketch does.
    void setProfile(const StepProfile* p) {
        profile = p;
    }

    const StepProfile* getProfile() const {
        return profile;
    }

    // The interval the next pulse will actually wait out: the profile's idea of
    // the pace here, or the flat interval when there is no profile. Distinct
    // from getStepInterval(), which is the top speed the axis was asked for.
    unsigned long currentInterval() const {
        if (!profile) return stepIntervalUs;

        long done = currentStep - moveStartStep;
        long left = targetStep - currentStep;

        return profile->intervalFor((done > 0) ? done : -done,
                                    (left > 0) ? left : -left,
                                    stepIntervalUs);
    }

    // Pulse spacing, i.e. speed. With a profile attached this is the top speed
    // the ramp works towards. Settable after construction because homing
    // switches between a fast seek and a slow, repeatable re-approach.
    void setStepInterval(unsigned long us) {
        stepIntervalUs = us;
    }

    unsigned long getStepInterval() const {
        return stepIntervalUs;
    }

    // The one piece of calibration a degree layer needs; unused by step-space
    // motion itself.
    uint16_t getStepsPerRevolution() const {
        return stepsPerRevolution;
    }

    // Per-axis switch for the DEBUG and TRACE messages. Has no effect unless
    // MC_LOG_LEVEL compiles them in - see Log.h.
    void toggleDebug() {
        this->debugEnabled = !this->debugEnabled;
    }

    bool getDebug() const {
        return this->debugEnabled;
    }

protected:
    Stepper(uint16_t stepsPerRevolution, unsigned long stepIntervalUs)
        : stepsPerRevolution(stepsPerRevolution),
          stepIntervalUs(stepIntervalUs),
          currentStep(0),
          targetStep(0),
          moveStartStep(0),
          profile(nullptr),
          lastStepAt(0),
          state(HOLD),
          justReached(false),
          debugEnabled(false) {
        endstops[0] = nullptr;
        endstops[1] = nullptr;
    }

    // Sets the driver's direction input ahead of a pulse. forward means the
    // step counter is about to increase.
    virtual void applyDirection(bool forward) = 0;

    // Emits exactly one step pulse.
    virtual void pulseStep() = 0;

    // Subclasses report motion through here rather than writing `state`, so
    // the MOVING -> HOLD edge behind reachedTarget() is detected in one place.
    void setState(State newState) {
        if (newState == HOLD && state != HOLD) {
            justReached = true;
        }
        state = newState;
    }

    static uint8_t slot(Endstop::Side side) {
        return (side == Endstop::MIN) ? 0 : 1;
    }

    void updateEndstops() {
        for (uint8_t i = 0; i < 2; i++) {
            if (endstops[i]) endstops[i]->update();
        }
    }

    // The pressed endstop in the direction of travel, if any.
    Endstop* blockingEndstop() const {
        Endstop* ahead = endstops[(targetStep > currentStep) ? 1 : 0];
        return (ahead && ahead->isTriggered()) ? ahead : nullptr;
    }

    uint16_t stepsPerRevolution;
    unsigned long stepIntervalUs;

    long currentStep;
    long targetStep;
    long moveStartStep;          // where the move in flight began, for the ramp
    const StepProfile* profile;  // borrowed; nullptr means a flat interval
    Endstop* endstops[2];        // borrowed; [0] MIN end, [1] MAX end
    unsigned long lastStepAt;

    State state;
    bool justReached;
    bool debugEnabled;
};
