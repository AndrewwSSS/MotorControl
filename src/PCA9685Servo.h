#pragma once
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "Motor.h"

// Motor implementation for a servo driven through a PCA9685 16-channel PWM
// driver over I2C. One PCA9685 board is shared by every PCA9685Servo built
// on top of it (each just claims one of its 16 channels), so the driver is
// taken by reference rather than owned here. Initialize the shared driver
// once, before any PCA9685Servo::begin() runs (e.g. before ServoManager::begin()):
//
//   Adafruit_PWMServoDriver pwm;
//   pwm.begin();
//   pwm.setPWMFreq(50); // standard analog servo frequency
//
class PCA9685Servo : public Motor {
public:
    PCA9685Servo(
        Adafruit_PWMServoDriver& driver,
        uint8_t channel,
        uint16_t defaultAngle = DEFAULT_ANGLE,
        uint16_t rotationDelay = DEFAULT_ROTATION_DELAY
    ) : Motor(defaultAngle, rotationDelay), driver(driver), channel(channel) {}

    void begin() override {
        Serial.println(
            "turning PCA9685 servo to default angle: " + String(target_angle) + " on channel " + String(channel)
        );

        writeAngle(target_angle);

        // Wait for the servo to reach the default position
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

