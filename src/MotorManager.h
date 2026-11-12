#pragma once
#include <Arduino.h>


// Owns a fixed-size fleet of actuators of root type T (AngularMotor or
// Stepper), addressed by index. Unit-specific commands live in the subclasses.
// See docs/architecture.md.
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

    // Takes ownership. Returns the id (0, 1, ... in order), or INVALID_ID when full.
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
    // Named apart from begin()/update() so callers cannot bypass a subclass's
    // own begin()/update() (e.g. StepperManager's enable line).
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
