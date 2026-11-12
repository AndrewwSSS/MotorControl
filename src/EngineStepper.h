#pragma once
#include <Arduino.h>
#include "Stepper.h"

// Holds the engine in a base listed before Stepper: bases are constructed in
// order, so the engine exists before Stepper takes a reference to it.
template <class Engine>
struct OwnedEngine {
    // By value: the arguments are small, and AVR has no std::forward.
    template <class... Args>
    explicit OwnedEngine(Args... args) : ownEngine(args...) {}

    Engine ownEngine;
};

// A Stepper that owns its engine. Arguments after stepsPerRevolution go to the
// engine's constructor: new EngineStepper<PolledEngine>(3200, stepPin, dirPin).
template <class Engine>
class EngineStepper : private OwnedEngine<Engine>, public Stepper {
public:
    template <class... EngineArgs>
    explicit EngineStepper(uint16_t stepsPerRevolution, EngineArgs... engineArgs)
        : OwnedEngine<Engine>(engineArgs...),
          Stepper(this->ownEngine, stepsPerRevolution) {}

    // Stepper references this object's own engine; a copy would point at the original.
    EngineStepper(const EngineStepper&) = delete;
    EngineStepper& operator=(const EngineStepper&) = delete;

    // The engine as its concrete type, for engine-specific calls.
    Engine& getEngine() {
        return this->ownEngine;
    }

    const Engine& getEngine() const {
        return this->ownEngine;
    }
};
