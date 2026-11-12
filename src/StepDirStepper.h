#pragma once
#include <Arduino.h>
#include "EngineStepper.h"
#include "PolledEngine.h"

// An EngineStepper<PolledEngine> for any step+dir driver (A4988, DRV8825,
// TB6600, DM542). Microstepping is set on the driver, so stepsPerRevolution is
// full steps x microsteps (200 x 16 = 3200 for a 1.8 deg motor at 1/16).
class StepDirStepper : public EngineStepper<PolledEngine> {
public:
    static constexpr uint8_t NO_ENABLE_PIN = PolledEngine::NO_ENABLE_PIN;

    // Minimum STEP high time, per the driver's datasheet.
    static constexpr uint8_t A4988_PULSE_US  = PolledEngine::A4988_PULSE_US;
    static constexpr uint8_t TB6600_PULSE_US = PolledEngine::TB6600_PULSE_US;

    StepDirStepper(uint8_t stepPin, uint8_t dirPin, uint16_t stepsPerRevolution,
                   uint8_t enablePin = NO_ENABLE_PIN,
                   unsigned long stepIntervalUs = 800,
                   uint8_t stepPulseUs = A4988_PULSE_US)
        : EngineStepper<PolledEngine>(stepsPerRevolution, stepPin, dirPin,
                                      enablePin, stepIntervalUs, stepPulseUs) {}

    // EN is active LOW. Disabling frees the shaft and stops the coils heating.
    void enable() {
        getEngine().enable();
    }

    void disable() {
        getEngine().disable();
    }

    uint8_t getStepPin() const {
        return getEngine().getStepPin();
    }

    uint8_t getStepPulse() const {
        return getEngine().getStepPulse();
    }
};
