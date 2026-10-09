#pragma once
#include <Servo.h>
#include <Arduino.h>
#include "AngularMotor.h"
#include "Log.h"


// AngularMotor backed by the Arduino Servo library (one hardware PWM pin per
// motor).
class ServoMotor : public AngularMotor {
public:
    ServoMotor(uint8_t pin, uint16_t defaultAngle = DEFAULT_ANGLE, uint16_t rotationDelay = DEFAULT_ROTATION_DELAY)
        : AngularMotor(defaultAngle, rotationDelay), servoPin(pin) {}

    ~ServoMotor() override {
        servo.detach();
    }

    void begin() override {
        MC_INFO(F("Turning servo to default angle: "), targetAngle, F(" on pin "), servoPin);

        servo.attach(servoPin);
        writeAngle(targetAngle);

        // Wait for the servo to reach the default position
        delay(1000);

        setState(HOLD);
    }

protected:
    void writeAngle(uint16_t angle) override {
        servo.write(angle);
    }

private:
    Servo servo;
    uint8_t servoPin;
};
