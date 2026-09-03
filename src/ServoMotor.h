#pragma once
#include <Servo.h>
#include <Arduino.h>
#include "Motor.h"


// Motor implementation backed by the Arduino Servo library (one hardware PWM
// pin per motor).
class ServoMotor : public Motor {
public:
    ServoMotor(uint8_t pin, uint16_t defaultAngle = DEFAULT_ANGLE, uint16_t rotationDelay = DEFAULT_ROTATION_DELAY)
        : Motor(defaultAngle, rotationDelay), servoPin(pin) {}

    ~ServoMotor() override {
        servo.detach();
    }

    void begin() override {
        Serial.println("turning servo to default angle: " + String(target_angle) + " on pin " + String(servoPin));

        servo.attach(servoPin);
        writeAngle(target_angle);

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
