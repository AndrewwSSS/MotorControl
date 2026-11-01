#pragma once
#include <Arduino.h>
#include "EngineStepper.h"
#include "PolledEngine.h"

// A Stepper on any step+dir driver (A4988, DRV8825, TB6600, DM542 and the
// like), with its own PolledEngine built in: the one-line way to get an axis,
// and the shape every sketch written before the engine split already uses.
// It is an EngineStepper<PolledEngine> with the constructor sketches already
// call. To drive the axis from another engine, use EngineStepper with that
// engine instead.
//
// Microstepping is set by jumpers, MS pins or DIP switches on the driver, not
// from here, so stepsPerRevolution must be given as full steps x microsteps (a
// 1.8 deg/step motor at 1/16 is 200 x 16 = 3200).
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

    // The driver's EN input is active low. Releasing it frees the shaft and
    // stops the coils heating
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
