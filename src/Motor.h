#pragma once
#include <Arduino.h>
#include "Log.h"


// Common interface for any angle-driven actuator (hobby servo, PCA9685-driven
// servo, geared stepper acting as a joint, ...).
class Motor {
public:
    enum State {
        MOVING,
        HOLD
    };

    virtual ~Motor() = default;

    // Attaches/initializes the underlying hardware and drives it to defaultAngle.
    virtual void begin() = 0;

    // Advances the motor towards target_angle. Must be called repeatedly (e.g.
    // from loop()).
    virtual void update() {
        if (current_angle == target_angle) {
            setState(HOLD);
            return;
        }

        unsigned long now = millis();
        if (now - lastMove < rotationDelay) return;

        current_angle += (current_angle < target_angle) ? 1 : -1;
        writeAngle(current_angle);
        lastMove = now;
        setState(MOVING);

        if (debugEnabled) MC_TRACE(F("Current angle: "), current_angle);
    }

    void setAngle(uint16_t angle) {
        if (angle < this->minAngle) angle = this->minAngle;
        if (angle > this->maxAngle) angle = this->maxAngle;

        this->target_angle = angle;

        if (debugEnabled) MC_DEBUG(F("Setting target angle to "), target_angle);
    }

    void setDefaultAngle() {
        setAngle(this->defaultAngle);
    }

    uint16_t getAngle() const {
        return this->current_angle;
    }

    // Per-motor switch for the DEBUG and TRACE messages. Has no effect unless
    // MC_LOG_LEVEL compiles them in - see Log.h.
    void toggleDebug() {
        this->debugEnabled = !this->debugEnabled;
    }

    bool getDebug() const {
        return this->debugEnabled;
    }

    State getState() const {
        return this->state;
    }

    // True exactly once, the first time this is polled after the motor
    // settles into HOLD. Lets a manager fire a "reached" callback without
    // keeping its own shadow copy of every motor's previous state.
    bool reachedTarget() {
        bool r = justReached;
        justReached = false;
        return r;
    }

protected:
    // rotationDelay is the time between 1-degree steps in the default update()
    // loop (lower = faster sweep speed). Motor types with their own update()
    // override can ignore it.
    explicit Motor(
        uint16_t defaultAngle,
        uint16_t rotationDelay = DEFAULT_ROTATION_DELAY,
        uint16_t minAngle = MIN_ANGLE,
        uint16_t maxAngle = MAX_ANGLE
    ) {
        this->current_angle = defaultAngle;
        this->target_angle = defaultAngle;
        this->defaultAngle = defaultAngle;
        this->lastMove = 0;
        this->rotationDelay = rotationDelay;
        this->minAngle = minAngle;
        this->maxAngle = maxAngle;
        this->state = HOLD;
        this->justReached = false;
    }

    static constexpr uint16_t DEFAULT_ANGLE = 45;
    static constexpr uint16_t MIN_ANGLE = 0;
    static constexpr uint16_t MAX_ANGLE = 180;
    static constexpr uint16_t DEFAULT_ROTATION_DELAY = 20; // ms

    // Derived classes report motion state through here instead of writing
    // `state` directly, so the MOVING -> HOLD edge (used by reachedTarget())
    // is only ever detected in one place.
    void setState(State newState) {
        if (newState == HOLD && state != HOLD) {
            justReached = true;
        }
        state = newState;
    }

    // Physically drives the actuator to the given angle. Called from
    // update() on every step and from begin() to reach the initial position.
    virtual void writeAngle(uint16_t angle) = 0;

    bool debugEnabled = false;

    uint16_t current_angle;
    uint16_t target_angle;
    uint16_t defaultAngle;
    unsigned long lastMove;
    uint16_t rotationDelay;
    uint16_t minAngle;
    uint16_t maxAngle;
private:
    State state;
    bool justReached;
};
