#pragma once
#include "Motor.h"

typedef void (*ServoReachedCallback)(uint8_t id);

// Owns and drives a fixed-size fleet of Motor instances, addressed by index.
// Motors are registered via addMotor() so the manager works with any mix of
// Motor subtypes (ServoMotor, a future PCA9685-backed servo, a stepper
// joint, ...) - it only ever talks to them through the Motor interface.
class ServoManager {
public:
    explicit ServoManager(uint8_t capacity) {
        this->count = 0;
        this->capacity = capacity;
        this->reachedCallback = nullptr;
        motors = new Motor*[capacity];
    }

    ~ServoManager() {
        for (uint8_t i = 0; i < count; i++)
            delete motors[i];
    
        delete[] motors;
    }

    // Takes ownership of motor. Returns its id, or 0xFF if capacity is full.
    uint8_t addMotor(Motor* motor) {
        if (count >= capacity) return 0xFF;
        motors[count] = motor;
        return count++;
    }

    void begin() {
        for (uint8_t i = 0; i < count; i++) motors[i]->begin();
    }

    void update() {
        for (uint8_t i = 0; i < count; i++) {
            motors[i]->update();

            if (motors[i]->reachedTarget() && reachedCallback) {
                reachedCallback(i);
            }
        }
    }

    void setDefaultAngles() {
        for (uint8_t i = 0; i < count; i++) motors[i]->setDefaultAngle();
    }

    void setAngle(uint8_t id, int angle) {
        if (id < count) motors[id]->setAngle(angle);
    }

    int getAngle(uint8_t id) const {
        return (id < count) ? motors[id]->getAngle() : -1;
    }

    Motor::State getState(uint8_t id) const {
        return (id < count) ? motors[id]->getState() : Motor::HOLD;
    }

    bool isHold(uint8_t id) const {
        return (id < count) ? (motors[id]->getState() == Motor::HOLD) : true;
    }

    bool allHold() const {
        for (uint8_t i = 0; i < count; i++) {
            if (motors[i]->getState() != Motor::HOLD) return false;
        }
        return true;
    }

    void setAngles(const uint8_t* ids, const int* angles, uint8_t n) {
        for (uint8_t i = 0; i < n; i++) setAngle(ids[i], angles[i]);
    }

    void onReached(ServoReachedCallback cb) { reachedCallback = cb; }

    uint8_t size() const { return count; }

    Motor& operator[](uint8_t id) { return *motors[id]; }

private:
    Motor**              motors;
    uint8_t              count;
    uint8_t              capacity;
    ServoReachedCallback reachedCallback;
};
