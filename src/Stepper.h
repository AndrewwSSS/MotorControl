#pragma once
#include <Arduino.h>


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
    }

    // Emits at most one step pulse and returns immediately. Call repeatedly
    // from loop(); movement is non-blocking.
    virtual void update() {
        if (currentStep == targetStep) {
            setState(HOLD);
            return;
        }

        unsigned long now = micros();
        if (now - lastStepAt < stepIntervalUs) return;

        bool forward = (targetStep > currentStep);
        applyDirection(forward);
        pulseStep();

        currentStep += forward ? 1 : -1;
        lastStepAt = now;
        setState(MOVING);

        if (DEBUG) {
            Serial.println("Current step: " + String(currentStep) +
                           " of " + String(targetStep));
        }
    }

    // Absolute move, in steps from the base position. May be negative: the
    // base position is a reference, not a floor.
    void moveToStep(long step) {
        targetStep = step;

        if (DEBUG) {
            Serial.println("Target step: " + String(targetStep));
        }
    }

    // Relative move. Anchored on the target rather than the current position
    // so repeated jogs compose while a move is still in flight.
    void moveSteps(long deltaSteps) {
        moveToStep(targetStep + deltaSteps);
    }

    // Cancels a move in flight by making the current position the target.
    void stopHere() {
        moveToStep(currentStep);
    }

    // Redefines where the axis believes it is
    void setOrigin(long step = 0) {
        currentStep = step;
        targetStep = step;
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

    // Pulse spacing, i.e. speed. Settable after construction because homing
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

    void toggleDebug() {
        this->DEBUG = !this->DEBUG;
    }

    bool getDebug() const {
        return this->DEBUG;
    }

protected:
    Stepper(uint16_t stepsPerRevolution, unsigned long stepIntervalUs)
        : stepsPerRevolution(stepsPerRevolution),
          stepIntervalUs(stepIntervalUs),
          currentStep(0),
          targetStep(0),
          lastStepAt(0),
          state(HOLD),
          justReached(false),
          DEBUG(false) {}

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

    uint16_t stepsPerRevolution;
    unsigned long stepIntervalUs;

    long currentStep;
    long targetStep;
    unsigned long lastStepAt;

    State state;
    bool justReached;
    bool DEBUG;
};
