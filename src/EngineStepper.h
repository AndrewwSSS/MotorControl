#pragma once
#include <Arduino.h>
#include "Stepper.h"

// Holds an EngineStepper's engine. It is a base class listed before Stepper,
// and bases are built in the order they are listed, so the engine is whole
// before Stepper is handed a reference to it - and outlives Stepper on the way
// down.
template <class Engine>
struct OwnedEngine {
    // By value: engine arguments are pins and intervals, and AVR has no
    // std::forward.
    template <class... Args>
    explicit OwnedEngine(Args... args) : ownEngine(args...) {}

    Engine ownEngine;
};

// A Stepper that owns its engine, of any kind: one `new` gives a whole axis.
// The arguments after stepsPerRevolution go to the engine's constructor:
//
//   Stepper* x = new EngineStepper<PolledEngine>(3200, /*step*/ 2, /*dir*/ 5);
//
// Use a plain Stepper instead when the engine has to live somewhere else, such
// as at file scope next to the endstops.
template <class Engine>
class EngineStepper : private OwnedEngine<Engine>, public Stepper {
public:
    template <class... EngineArgs>
    explicit EngineStepper(uint16_t stepsPerRevolution, EngineArgs... engineArgs)
        : OwnedEngine<Engine>(engineArgs...),
          Stepper(this->ownEngine, stepsPerRevolution) {}

    // The base holds a reference to this object's own engine, which a copy
    // would leave pointing at the original.
    EngineStepper(const EngineStepper&) = delete;
    EngineStepper& operator=(const EngineStepper&) = delete;

    // The engine as its own type, for what only that engine offers. Through a
    // Stepper& the same call gives the StepEngine interface.
    Engine& getEngine() {
        return this->ownEngine;
    }

    const Engine& getEngine() const {
        return this->ownEngine;
    }
};
