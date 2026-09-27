#pragma once
#include "Motor.h"
#include "MotorManager.h"

typedef MotorManager<Motor>::ReachedCallback ServoReachedCallback;

// A fleet of angle-driven motors: MotorManager's lifecycle plus commands in
// degrees. Any Motor subtype can join (ServoMotor, PCA9685Servo, a degree
// wrapper around a stepper joint, ...) - the manager only talks through Motor.
class ServoManager : public MotorManager<Motor> {
public:
    explicit ServoManager(uint8_t capacity) : MotorManager<Motor>(capacity) {}

    void begin() {
        beginMotors();
    }

    void update() {
        updateMotors();
    }

    void setDefaultAngles() {
        for (uint8_t i = 0; i < count; i++) motors[i]->setDefaultAngle();
    }

    void setAngle(uint8_t id, int angle) {
        // Motor::setAngle clamps, but takes an unsigned angle: a negative one
        // would wrap to a huge value and clamp to maxAngle - the far end of
        // travel - rather than to the near one meant here.
        if (angle < 0) angle = 0;
        if (id < count) motors[id]->setAngle((uint16_t)angle);
    }

    int getAngle(uint8_t id) const {
        return (id < count) ? (int)motors[id]->getAngle() : -1;
    }

    void setAngles(const uint8_t* ids, const int* angles, uint8_t n) {
        for (uint8_t i = 0; i < n; i++) setAngle(ids[i], angles[i]);
    }
};
