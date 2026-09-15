#pragma once
#include <Arduino.h>
#include "Stepper.h"

// Stepper driven through an A4988 / DRV8825 style step+direction driver
//
// Microstepping is set by jumpers or MS pins on the driver board, not from
// here, so stepsPerRevolution must be given as full steps x microsteps (a
// 1.8 deg/step motor at 1/16 is 200 x 16 = 3200).
class A4988Stepper : public Stepper {
public:
    static constexpr uint8_t NO_ENABLE_PIN = 0xFF;

    A4988Stepper(uint8_t stepPin, uint8_t dirPin, uint16_t stepsPerRevolution,
                 uint8_t enablePin = NO_ENABLE_PIN,
                 unsigned long stepIntervalUs = 800)
        : Stepper(stepsPerRevolution, stepIntervalUs),
          stepPin(stepPin), dirPin(dirPin), enablePin(enablePin) {}

    void begin() override {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        digitalWrite(stepPin, LOW);

        if (enablePin != NO_ENABLE_PIN) {
            pinMode(enablePin, OUTPUT);
        }

        // Energising here is what makes the shaft go stiff, which is the first
        // sign that driver, motor supply and Vref are all doing their job.
        enable();

        Stepper::begin();
    }

    // The driver's EN input is active low. Releasing it frees the shaft and
    // stops the coils heating, at the cost of losing position - the counter
    // keeps saying where the axis was, not where it drifted to.
    void enable() {
        if (enablePin != NO_ENABLE_PIN) digitalWrite(enablePin, LOW);
    }

    void disable() {
        if (enablePin != NO_ENABLE_PIN) digitalWrite(enablePin, HIGH);
    }

    uint8_t getStepPin() const {
        return stepPin;
    }

protected:
    void applyDirection(bool forward) override {
        digitalWrite(dirPin, forward ? HIGH : LOW);
    }

    void pulseStep() override {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(stepPin, LOW);
    }

    static constexpr unsigned long STEP_PULSE_US = 2; // A4988 min STEP high time

    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;
};
