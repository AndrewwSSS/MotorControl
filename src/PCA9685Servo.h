#pragma once
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "AngularMotor.h"
#include "Log.h"

// AngularMotor on one channel of a shared PCA9685 board. Call pwm.begin() and
// pwm.setPWMFreq(50) before any begin() here. See docs/usage.md.
class PCA9685Servo : public AngularMotor {
public:
    PCA9685Servo(
        Adafruit_PWMServoDriver& driver,
        uint8_t channel,
        uint16_t defaultAngle = DEFAULT_ANGLE,
        uint16_t rotationDelay = DEFAULT_ROTATION_DELAY
    ) : AngularMotor(defaultAngle, rotationDelay), driver(driver), channel(channel) {}

    void begin() override {
        MC_INFO(F("Turning PCA9685 servo to default angle: "), targetAngle, F(" on channel "), channel);

        writeAngle(targetAngle);

        // Give the servo time to reach the default position.
        delay(1000);

        setState(HOLD);
    }

protected:
    void writeAngle(uint16_t angle) override {
        uint16_t pulse = map(angle, MIN_ANGLE, MAX_ANGLE, MIN_PULSE_US, MAX_PULSE_US);
        driver.writeMicroseconds(channel, pulse);
    }

private:
    static constexpr uint16_t MIN_PULSE_US = 500;  // pulse width at MIN_ANGLE
    static constexpr uint16_t MAX_PULSE_US = 2500; // pulse width at MAX_ANGLE

    Adafruit_PWMServoDriver& driver;
    uint8_t channel;
};

