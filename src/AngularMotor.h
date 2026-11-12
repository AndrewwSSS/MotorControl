#pragma once
#include <Arduino.h>
#include "Log.h"


// Base for angle-driven actuators (hobby servo, PCA9685 servo, geared joint).
class AngularMotor {
public:
    enum State {
        MOVING,
        HOLD
    };

    virtual ~AngularMotor() = default;

    // Attaches/initializes the underlying hardware and drives it to defaultAngle.
    virtual void begin() = 0;

    // Moves towards targetAngle; call from loop().
    virtual void update() {
        if (currentAngle == targetAngle) {
            setState(HOLD);
            return;
        }

        unsigned long now = millis();
        if (now - lastMove < rotationDelay) return;

        currentAngle += (currentAngle < targetAngle) ? 1 : -1;
        writeAngle(currentAngle);
        lastMove = now;
        setState(MOVING);

        if (debugEnabled) MC_TRACE(F("Current angle: "), currentAngle);
    }

    void setAngle(uint16_t angle) {
        if (angle < this->minAngle) angle = this->minAngle;
        if (angle > this->maxAngle) angle = this->maxAngle;

        this->targetAngle = angle;

        if (debugEnabled) MC_DEBUG(F("Setting target angle to "), targetAngle);
    }

    void setDefaultAngle() {
        setAngle(this->defaultAngle);
    }

    uint16_t getAngle() const {
        return this->currentAngle;
    }

    // Per-motor DEBUG/TRACE output, if MC_LOG_LEVEL compiles it in.
    void toggleDebug() {
        this->debugEnabled = !this->debugEnabled;
    }

    bool getDebug() const {
        return this->debugEnabled;
    }

    State getState() const {
        return this->state;
    }

    // True once, on the first poll after the motor settles into HOLD.
    bool reachedTarget() {
        bool r = justReached;
        justReached = false;
        return r;
    }

protected:
    // rotationDelay: ms between 1-degree steps in the default update().
    explicit AngularMotor(
        uint16_t defaultAngle,
        uint16_t rotationDelay = DEFAULT_ROTATION_DELAY,
        uint16_t minAngle = MIN_ANGLE,
        uint16_t maxAngle = MAX_ANGLE
    ) {
        this->currentAngle = defaultAngle;
        this->targetAngle = defaultAngle;
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

    // The only writer of `state`, so the HOLD edge is detected in one place.
    void setState(State newState) {
        if (newState == HOLD && state != HOLD) {
            justReached = true;
        }
        state = newState;
    }

    // One hardware write to the given angle.
    virtual void writeAngle(uint16_t angle) = 0;

    bool debugEnabled = false;

    uint16_t currentAngle;
    uint16_t targetAngle;
    uint16_t defaultAngle;
    unsigned long lastMove;
    uint16_t rotationDelay;
    uint16_t minAngle;
    uint16_t maxAngle;
private:
    State state;
    bool justReached;
};
