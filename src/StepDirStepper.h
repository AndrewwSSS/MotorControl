#pragma once
#include <Arduino.h>
#include "PolledEngine.h"
#include "Stepper.h"

// A Stepper on any step+dir driver (A4988, DRV8825, TB6600, DM542 and the
// like), with its own PolledEngine built in: the one-line way to get an axis,
// and the shape every sketch written before the engine split already uses.
// To drive the axis from another engine, build a Stepper with that engine
// instead.
//
// Microstepping is set by jumpers, MS pins or DIP switches on the driver, not
// from here, so stepsPerRevolution must be given as full steps x microsteps (a
// 1.8 deg/step motor at 1/16 is 200 x 16 = 3200).
class StepDirStepper : public Stepper {
public:
    static constexpr uint8_t NO_ENABLE_PIN = PolledEngine::NO_ENABLE_PIN;

    // Minimum STEP high time, per the driver's datasheet.
    static constexpr uint8_t A4988_PULSE_US  = PolledEngine::A4988_PULSE_US;
    static constexpr uint8_t TB6600_PULSE_US = PolledEngine::TB6600_PULSE_US;

    // The base is handed a reference to `polled` before that member is built.
    // That is safe because Stepper's constructor only stores the reference; it
    // first calls the engine from begin().
    StepDirStepper(uint8_t stepPin, uint8_t dirPin, uint16_t stepsPerRevolution,
                   uint8_t enablePin = NO_ENABLE_PIN,
                   unsigned long stepIntervalUs = 800,
                   uint8_t stepPulseUs = A4988_PULSE_US)
        : Stepper(polled, stepsPerRevolution),
          polled(stepPin, dirPin, enablePin, stepIntervalUs, stepPulseUs) {}

    // The base holds a reference to this object's own engine, which a copy
    // would leave pointing at the original.
    StepDirStepper(const StepDirStepper&) = delete;
    StepDirStepper& operator=(const StepDirStepper&) = delete;

    // The driver's EN input is active low. Releasing it frees the shaft and
    // stops the coils heating
    void enable() {
        polled.enable();
    }

    void disable() {
        polled.disable();
    }

    uint8_t getStepPin() const {
        return polled.getStepPin();
    }

    uint8_t getStepPulse() const {
        return polled.getStepPulse();
    }

private:
    PolledEngine polled;
};
