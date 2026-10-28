#pragma once
#include <Arduino.h>


// Owns and drives a fixed-size fleet of actuators, addressed by index.
//
// T is an actuator root: AngularMotor for angle-driven servos, Stepper for
// step-driven axes. Both expose the same lifecycle - begin(), a non-blocking
// update(), a State enum with MOVING and HOLD, the one-shot reachedTarget()
// edge and the debug toggle - and that lifecycle is all this class touches. Commands in the
// actuator's own unit (degrees, steps) live on the subclasses, ServoManager and
// StepperManager.
//
// Motors are registered through addMotor(), so a fleet can hold any mix of
// subtypes: steppers on different driver chips, a PCA9685Servo next to
// a ServoMotor. Swapping one out is a change at the new-expression and nowhere
// else, because the manager only ever talks through T.
template <typename T>
class MotorManager {
public:
    typedef void (*ReachedCallback)(uint8_t id);

    static const uint8_t INVALID_ID = 0xFF;

    explicit MotorManager(uint8_t capacity) {
        this->count = 0;
        this->capacity = capacity;
        this->reachedCallback = nullptr;
        this->motors = new T*[capacity];
    }

    ~MotorManager() {
        for (uint8_t i = 0; i < count; i++)
            delete motors[i];

        delete[] motors;
    }

    // Takes ownership of motor. Returns its id, or INVALID_ID if capacity is
    // full - ids are handed out in registration order, starting at 0.
    uint8_t addMotor(T* motor) {
        if (motor == nullptr || count >= capacity) return INVALID_ID;
        motors[count] = motor;
        return count++;
    }

    typename T::State getState(uint8_t id) const {
        return (id < count) ? motors[id]->getState() : T::HOLD;
    }

    bool isHold(uint8_t id) const {
        return (id < count) ? (motors[id]->getState() == T::HOLD) : true;
    }

    bool allHold() const {
        for (uint8_t i = 0; i < count; i++) {
            if (motors[i]->getState() != T::HOLD) return false;
        }
        return true;
    }

    void toggleDebug(uint8_t id) {
        if (id < count) motors[id]->toggleDebug();
    }

    bool getDebug(uint8_t id) const {
        return (id < count) ? motors[id]->getDebug() : false;
    }

    void onReached(ReachedCallback cb) { reachedCallback = cb; }

    uint8_t size() const { return count; }

    bool has(uint8_t id) const { return id < count; }

    // Unchecked, for reaching subtype-specific methods on a known-good id:
    //   static_cast<StepDirStepper&>(fleet[0]).getStepPin()
    T& operator[](uint8_t id) { return *motors[id]; }

    // Checked variant: nullptr for an id that was never registered.
    T* get(uint8_t id) const { return (id < count) ? motors[id] : nullptr; }

protected:
    // The fleet loop. Deliberately not called begin()/update(): a subclass that
    // has work of its own to do around them (StepperManager and the enable
    // line) owns those names, and naming these apart means no caller can reach
    // past it through a MotorManager reference and skip that work.
    void beginMotors() {
        for (uint8_t i = 0; i < count; i++) motors[i]->begin();
    }

    void updateMotors() {
        for (uint8_t i = 0; i < count; i++) {
            motors[i]->update();

            if (motors[i]->reachedTarget() && reachedCallback) {
                reachedCallback(i);
            }
        }
    }

    T**             motors;
    uint8_t         count;
    uint8_t         capacity;
    ReachedCallback reachedCallback;
};
