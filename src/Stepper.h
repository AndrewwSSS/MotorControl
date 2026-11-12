#pragma once
#include <Arduino.h>
#include "Log.h"
#include "Endstop.h"
#include "StepEngine.h"
#include "StepProfile.h"


// One step-driven axis: target, endstop guard, soft limits and MOVING/HOLD
// state. Pulses come from a borrowed StepEngine; StepDirStepper bundles one.
class Stepper {
public:
    enum State {
        MOVING,
        HOLD
    };

    // stepsPerRevolution is full steps x microsteps (200 x 16 = 3200 for a
    // 1.8 deg/step motor at 1/16).
    Stepper(StepEngine& engine, uint16_t stepsPerRevolution)
        : engine(engine),
          stepsPerRevolution(stepsPerRevolution),
          minStep(0),
          maxStep(0),
          limited(false),
          armed(false),
          state(HOLD),
          justReached(false),
          debugEnabled(false) {
        endstops[0] = nullptr;
        endstops[1] = nullptr;
    }

    virtual ~Stepper() = default;

    void begin() {
        engine.begin();
        setOrigin(0);

        for (uint8_t i = 0; i < 2; i++) {
            if (endstops[i]) endstops[i]->begin();
        }
    }

    // Non-blocking; call from loop(). A polled engine makes at most one pulse per call.
    void update() {
        updateEndstops();

        if (!engine.isRunning()) {
            setState(HOLD);
            return;
        }

        // Tell the engine which way is blocked, so a self-stepping engine also refuses it.
        Endstop* hit = blockingEndstop();
        engine.inhibit(hit ? hit->getSide() : 0);
        if (hit) {
            MC_INFO(F("Endstop "), hit->sideName(), F(" on pin "), hit->getPin(),
                    F(" stopped the axis at step "), engine.position());
            engine.stopNow();
            setState(HOLD);
            return;
        }

        if (engine.service()) {
            setState(MOVING);
            if (debugEnabled) MC_TRACE(F("Current step: "), engine.position(), F(" of "), engine.target());
        } else if (engine.stepsOnItsOwn()) {
            setState(engine.isRunning() ? MOVING : HOLD);
        }
    }

    // Absolute move in steps (may be negative). Refused if outside armed soft limits.
    void moveToStep(long step) {
        if (!withinLimits(step)) {
            MC_INFO(F("Move to step "), step, F(" refused: limits are "),
                    minStep, F(".."), maxStep);
            return;
        }

        setTarget(step);
    }

    // Relative to the target, so repeated jogs add up mid-move.
    void moveSteps(long deltaSteps) {
        moveToStep(engine.target() + deltaSteps);
    }

    // Stops immediately, ramp or not.
    void stopHere() {
        setTarget(engine.position());
    }

    // Stops over the profile's stopping distance; stopHere() without a profile.
    void stopRamped() {
        if (!engine.isRunning()) return;

        engine.stopRamped();

        if (debugEnabled) MC_DEBUG(F("Target step: "), engine.target());
    }

    // Soft limits in steps, enforced once armed (after homing or setOrigin()).
    // See docs/homing.md.
    void setLimits(long minStep, long maxStep) {
        if (minStep > maxStep) {
            long t = minStep;
            minStep = maxStep;
            maxStep = t;
        }

        this->minStep = minStep;
        this->maxStep = maxStep;
        limited = true;
        cutShortAtLimits();
    }

    void clearLimits() {
        limited = false;
    }

    bool hasLimits() const {
        return limited;
    }

    long getMinStep() const {
        return minStep;
    }

    long getMaxStep() const {
        return maxStep;
    }

    // Homing disarms the limits during its search.
    void armLimits(bool on = true) {
        armed = on;
        cutShortAtLimits();
    }

    bool limitsArmed() const {
        return armed;
    }

    // Redefines the current position.
    void setOrigin(long step = 0) {
        engine.setPosition(step);
        setState(HOLD);
    }

    long getStep() const {
        return engine.position();
    }

    long getTargetStep() const {
        return engine.target();
    }

    State getState() const {
        return state;
    }

    // True once, on the first poll after the axis settles into HOLD.
    bool reachedTarget() {
        bool r = justReached;
        justReached = false;
        return r;
    }

    // Attaches a borrowed switch to the slot for its side. Call before begin().
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

    // Borrowed speed profile, or nullptr for a flat interval.
    void setProfile(const StepProfile* p) {
        engine.setProfile(p);
    }

    const StepProfile* getProfile() const {
        return engine.profile();
    }

    // The interval right now; differs from getStepInterval() while ramping.
    unsigned long currentInterval() const {
        return engine.currentInterval();
    }

    // Pulse spacing; the cruise interval when a profile is attached.
    void setStepInterval(unsigned long us) {
        engine.setStepInterval(us);
    }

    unsigned long getStepInterval() const {
        return engine.stepInterval();
    }

    // For converting to degrees; not used by step-space motion.
    uint16_t getStepsPerRevolution() const {
        return stepsPerRevolution;
    }

    StepEngine& getEngine() const {
        return engine;
    }

    // Per-axis DEBUG/TRACE output, if MC_LOG_LEVEL compiles it in.
    void toggleDebug() {
        this->debugEnabled = !this->debugEnabled;
    }

    bool getDebug() const {
        return this->debugEnabled;
    }

protected:
    void setTarget(long step) {
        engine.moveTo(step);

        if (debugEnabled) MC_DEBUG(F("Target step: "), engine.target());
    }

    // In range, or no further out of range than the axis already is.
    bool withinLimits(long step) const {
        if (!limited || !armed) return true;
        if (step >= minStep && step <= maxStep) return true;

        long at = engine.position();
        if (step < minStep) return at < minStep && step >= at;
        return at > maxStep && step <= at;
    }

    void cutShortAtLimits() {
        if (!limited || !armed || !engine.isRunning()) return;

        long target = engine.target();
        long at = engine.position();
        if (target > maxStep && at <= maxStep) setTarget(maxStep);
        if (target < minStep && at >= minStep) setTarget(minStep);
    }

    // The only writer of `state`, so the HOLD edge is detected in one place.
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
        Endstop* ahead = endstops[(engine.target() > engine.position()) ? 1 : 0];
        return (ahead && ahead->isTriggered()) ? ahead : nullptr;
    }

    StepEngine& engine;          // borrowed
    uint16_t stepsPerRevolution;
    Endstop* endstops[2];        // borrowed; [0] MIN end, [1] MAX end

    long minStep;                // soft limits, see setLimits()
    long maxStep;
    bool limited;
    bool armed;

    State state;
    bool justReached;
    bool debugEnabled;
};
