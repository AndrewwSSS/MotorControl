#pragma once
#include <Arduino.h>
#include "StepEngine.h"

// Shared base for engines whose pulses are timed by our own code: position,
// target, ramp, endstop inhibit, and tick(), which emits a pulse when one is due.
class SoftEngine : public StepEngine {
public:
    void moveTo(long step) override {
        // Mid-move, keep the original start so the ramp does not restart.
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

    // Changed at runtime by homing (fast seek, slow re-approach).
    void setStepInterval(unsigned long us) override {
        stepIntervalUs = us;
    }

    unsigned long stepInterval() const override {
        return stepIntervalUs;
    }

    unsigned long currentInterval() const override {
        if (!profile_) return stepIntervalUs;

        long done = currentStep - moveStartStep;
        long left = targetStep - currentStep;

        return profile_->intervalFor((done > 0) ? done : -done,
                                     (left > 0) ? left : -left,
                                     stepIntervalUs);
    }

    // Borrowed: the profile must outlive the axis.
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

    // Emits at most one pulse if one is due at `now`; returns whether it did.
    bool tick(unsigned long now) {
        if (currentStep == targetStep) return false;
        if (!pulsesAllowed) return false;

        int8_t dir = (targetStep > currentStep) ? 1 : -1;

        // Never drive further onto a pressed endstop; moving away is allowed.
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

    // Sets DIR before a pulse; forward means the step count increases.
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
