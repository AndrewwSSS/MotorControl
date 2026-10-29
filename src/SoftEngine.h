#pragma once
#include <Arduino.h>
#include "StepEngine.h"


// The part of a StepEngine that is the same whenever our own code makes the
// pulses, whatever calls it: position, target, the move's starting point for
// the ramp, the endstop inhibit, and tick(), which decides whether the next
// pulse is due. PolledEngine calls tick() from update(); TimerEngine will call
// it from a hardware timer.
class SoftEngine : public StepEngine {
public:
    void moveTo(long step) override {
        // A move commanded while the axis is already running composes onto the
        // one in flight, so it keeps that move's starting point: restarting the
        // ramp would slow a running axis back down to its starting speed.
        if (currentStep == targetStep) moveStartStep = currentStep;

        targetStep = step;
    }

    void stopNow() override {
        moveTo(currentStep);
    }

    void stopRamped() override {
        if (currentStep == targetStep) return;

        long left = targetStep - currentStep;
        long dir = (left > 0) ? 1 : -1;
        long span = profile_ ? profile_->stopDistance() : 0;
        long remaining = (left > 0) ? left : -left;

        if (span > remaining) span = remaining;

        moveTo(currentStep + dir * span);
    }

    void setPosition(long step) override {
        currentStep = step;
        targetStep = step;
        moveStartStep = step;
    }

    long position() const override {
        return currentStep;
    }

    long target() const override {
        return targetStep;
    }

    bool isRunning() const override {
        return currentStep != targetStep;
    }

    // Settable after construction because homing switches between a fast seek
    // and a slow, repeatable re-approach.
    void setStepInterval(unsigned long us) override {
        stepIntervalUs = us;
    }

    unsigned long stepInterval() const override {
        return stepIntervalUs;
    }

    // The profile's idea of the pace here, or the flat interval when there is
    // no profile.
    unsigned long currentInterval() const override {
        if (!profile_) return stepIntervalUs;

        long done = currentStep - moveStartStep;
        long left = targetStep - currentStep;

        return profile_->intervalFor((done > 0) ? done : -done,
                                     (left > 0) ? left : -left,
                                     stepIntervalUs);
    }

    // The profile is borrowed, not owned - it is expected to outlive the axis,
    // which a file-scope instance in the sketch does.
    void setProfile(const StepProfile* p) override {
        profile_ = p;
    }

    const StepProfile* profile() const override {
        return profile_;
    }

    void inhibit(int8_t direction) override {
        inhibited = direction;
    }

    void allowPulses(bool allowed) override {
        pulsesAllowed = allowed;
    }

protected:
    explicit SoftEngine(unsigned long stepIntervalUs)
        : stepIntervalUs(stepIntervalUs),
          currentStep(0),
          targetStep(0),
          moveStartStep(0),
          profile_(nullptr),
          lastStepAt(0),
          inhibited(0),
          pulsesAllowed(true) {}

    // Emits at most one step pulse, if one is due at now, and says whether it
    // did. Never waits.
    bool tick(unsigned long now) {
        if (currentStep == targetStep) return false;
        if (!pulsesAllowed) return false;

        int8_t dir = (targetStep > currentStep) ? 1 : -1;

        // Driving further onto a pressed endstop crashes the axis. Cancel the
        // move; moving away from the switch stays allowed.
        if (dir == inhibited) {
            stopNow();
            return false;
        }

        if (now - lastStepAt < currentInterval()) return false;

        applyDirection(dir > 0);
        pulseStep();

        currentStep += dir;
        lastStepAt = now;
        return true;
    }

    // Forgets when the last pulse went out, for a fresh begin().
    void resetTiming() {
        lastStepAt = 0;
    }

    // Sets the driver's direction input ahead of a pulse. forward means the
    // step counter is about to increase.
    virtual void applyDirection(bool forward) = 0;

    // Emits exactly one step pulse.
    virtual void pulseStep() = 0;

    unsigned long stepIntervalUs;

    long currentStep;
    long targetStep;
    long moveStartStep;           // where the move in flight began, for the ramp
    const StepProfile* profile_;  // borrowed; nullptr means a flat interval
    unsigned long lastStepAt;
    int8_t inhibited;             // the direction a pressed endstop forbids, or 0
    bool pulsesAllowed;           // the fleet's gate, see StepEngine::allowPulses()
};
