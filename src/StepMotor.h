#pragma once
#include <Arduino.h>
#include "Motor.h"

// Motor implementation for a stepper driven through an A4988 step/direction
// driver. Unlike a hobby servo, a stepper has no absolute position feedback:
// begin() assumes the joint is already physically at defaultAngle (e.g. from
// a prior homing routine or a known mechanical rest position) rather than
// actually moving it there.
//
// update() is overridden instead of using Motor's default degree-by-degree
// loop: a stepper's natural unit is a step pulse, not a whole degree, so
// position is tracked internally in steps and only rounded to a public
// degree value for getAngle()/setAngle().
class StepMotor : public Motor {
public:
    static constexpr uint8_t NO_ENABLE_PIN = 0xFF;

    StepMotor(uint8_t stepPin, uint8_t dirPin, uint16_t stepsPerRevolution,
              uint16_t defaultAngle = DEFAULT_ANGLE,
              uint8_t enablePin = NO_ENABLE_PIN,
              unsigned long stepIntervalUs = 800)
        : Motor(defaultAngle),
          stepPin(stepPin), dirPin(dirPin), enablePin(enablePin),
          stepsPerRevolution(stepsPerRevolution),
          stepIntervalUs(stepIntervalUs),
          currentStep(angleToSteps(defaultAngle)),
          lastStep(0) {}

    void begin() override {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        digitalWrite(stepPin, LOW);

        if (enablePin != NO_ENABLE_PIN) {
            pinMode(enablePin, OUTPUT);
            digitalWrite(enablePin, LOW); // A4988 EN is active-low
        }

        Serial.println("StepMotor assuming default angle: " + String(target_angle) + " on step pin " + String(stepPin));

        writeAngle(target_angle);
        setState(HOLD);
    }

    void update() override {
        long targetStep = angleToSteps(target_angle);

        if (currentStep == targetStep) {
            setState(HOLD);
            return;
        }

        unsigned long now = micros();
        if (now - lastStep < stepIntervalUs) return;

        digitalWrite(dirPin, targetStep > currentStep ? HIGH : LOW);

        digitalWrite(stepPin, HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(stepPin, LOW);

        currentStep += (targetStep > currentStep) ? 1 : -1;
        current_angle = stepsToAngle(currentStep);
        lastStep = now;
        setState(MOVING);

        if (DEBUG) {
            Serial.println("Current step: " + String(currentStep) + " (~" + String(current_angle) + " deg) pin: " + String(stepPin));
        }
    }

protected:
    // Records the joint as already being at `angle` without pulsing the
    // driver - used only by begin(), since a stepper can't be commanded to
    // an absolute angle without a prior homing reference.
    void writeAngle(uint16_t angle) override {
        currentStep = angleToSteps(angle);
        current_angle = angle;
    }

private:
    long angleToSteps(uint16_t angle) const {
        return (long)angle * stepsPerRevolution / 360;
    }

    uint16_t stepsToAngle(long steps) const {
        return (uint16_t)(steps * 360 / stepsPerRevolution);
    }

    static constexpr unsigned long STEP_PULSE_US = 2; // A4988 min STEP high time

    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;
    uint16_t stepsPerRevolution;
    unsigned long stepIntervalUs;

    long currentStep;
    unsigned long lastStep;
};
