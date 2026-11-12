#pragma once
#include "AngularMotor.h"
#include "MotorManager.h"

typedef MotorManager<AngularMotor>::ReachedCallback ServoReachedCallback;

// A fleet of AngularMotors with commands in degrees.
class ServoManager : public MotorManager<AngularMotor> {
public:
    explicit ServoManager(uint8_t capacity) : MotorManager<AngularMotor>(capacity) {}

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
        // Clamp negatives here: as unsigned they would wrap and clamp to maxAngle.
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
