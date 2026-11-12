#pragma once
#include <Arduino.h>
// ESP32Servo provides the same Servo class on the ESP32.
#if defined(ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif
#include "AngularMotor.h"
#include "Log.h"


// AngularMotor on one PWM pin via the Servo library.
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

        // Give the servo time to reach the default position.
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
