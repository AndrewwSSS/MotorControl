#pragma once
#include <Arduino.h>
// The Arduino Servo library does not support the ESP32; ESP32Servo offers the
// same Servo class on the chip's LEDC PWM.
#if defined(ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif
#include "AngularMotor.h"
#include "Log.h"


// AngularMotor backed by the Arduino Servo library, or ESP32Servo on the ESP32
// (one PWM pin per motor).
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
