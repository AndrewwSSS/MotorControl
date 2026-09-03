#pragma once
#include <Arduino.h>


// Common interface for any angle-driven actuator (hobby servo, PCA9685-driven
// servo, geared stepper acting as a joint, ...). Concrete motor types plug
// into ServoManager/Robot code by implementing begin() (hardware setup) and
// writeAngle() (a single hardware write); the gradual move-to-target loop,
// target clamping, default angle, state tracking and debug flag all live
// here so they aren't duplicated per motor type.
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
    // from loop()). Default implementation steps 1 degree at a time through
    // writeAngle() - fine for a hobby servo, but override this in motor types
    // that need a different movement strategy (e.g. a stepper that computes a
    // step count and drives it with its own pulse/acceleration timing instead
    // of walking degree by degree).
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

        if (DEBUG) {
            Serial.println("Current angle: " + String(current_angle));
        }
    }

    void setAngle(uint16_t angle) {
        if (angle < this->minAngle) angle = this->minAngle;
        if (angle > this->maxAngle) angle = this->maxAngle;

        this->target_angle = angle;

        if (DEBUG) {
            Serial.println("Setting target angle to " + String(target_angle));
        }
    }

    void setDefaultAngle() {
        setAngle(this->defaultAngle);
    }

    uint16_t getAngle() const {
        return this->current_angle;
    }

    void toggleDebug() {
        this->DEBUG = !this->DEBUG;
    }

    bool getDebug() const {
        return this->DEBUG;
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
    // override (e.g. StepMotor) can ignore it.
    explicit Motor(
        uint8_t defaultAngle,
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

    bool DEBUG = false;

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
