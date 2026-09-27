#pragma once
#include <Arduino.h>
#include "A4988Stepper.h"

// An A4988Stepper that *finds* its base position instead of assuming one.
class HomedStepper : public A4988Stepper {
public:
    enum LimitWiring {
        NORMALLY_OPEN,
        NORMALLY_CLOSED
    };

    enum HomingResult {
        NOT_HOMED,
        HOMED,
        HOMING_FAILED
    };

    HomedStepper(uint8_t stepPin, uint8_t dirPin, uint16_t stepsPerRevolution,
                 uint8_t limitPin,
                 uint8_t enablePin = NO_ENABLE_PIN,
                 unsigned long stepIntervalUs = 800,
                 LimitWiring wiring = NORMALLY_CLOSED)
        : A4988Stepper(stepPin, dirPin, stepsPerRevolution, enablePin, stepIntervalUs),
          limitPin(limitPin), limitWiring(wiring),
          limitState(false), limitLastChange(0),
          restStep(0), homeDirection(-1),
          homeMaxSteps(2000), homeBackoffSteps(40), homeSlowIntervalUs(4000),
          result(NOT_HOMED) {}

    // Optional tuning; call before begin().
    // Optional tuning; call before begin().
    void configureHoming(long restStep,
                         int8_t homeDirection = -1,
                         long maxSteps = 2000,
                         long backoffSteps = 40,
                         unsigned long slowIntervalUs = 4000) {
        this->restStep = restStep;
        this->homeDirection = homeDirection;
        this->homeMaxSteps = maxSteps;
        this->homeBackoffSteps = backoffSteps;
        this->homeSlowIntervalUs = slowIntervalUs;
    }

    // Sets up the pins, runs the search, then drives to restStep.
    void begin() override {
        A4988Stepper::begin();

        pinMode(limitPin, INPUT_PULLUP);
        resyncLimit();

        Serial.println("HomedStepper homing on step pin " + String(getStepPin()) +
                       " towards limit pin " + String(limitPin));

        result = home() ? HOMED : HOMING_FAILED;

        if (result != HOMED) {
            Serial.println("HomedStepper HOMING FAILED on step pin " + String(getStepPin()));
            return;
        }

        Serial.println("HomedStepper homed, moving to rest step: " + String(restStep));
        moveToStep(restStep);
        driveBlocking();
    }

    // Runs the search on demand. Returns true and makes the trip point step 0;
    bool home() {
        const StepProfile* saved = getProfile();
        setProfile(nullptr);

        bool ok = search();

        setProfile(saved);
        return ok;
    }

private:
    bool search() {
        resyncLimit();

        // Starting already on the switch leaves the seek pass nothing to find,
        // so skip to the back-off. Either way the base position is set from a
        // fresh closing edge during the slow re-approach.
        if (!limitState) {
            moveSteps((long)homeDirection * homeMaxSteps);

            if (!driveUntilLimit()) {
                stopHere();
                return false;   // ran the whole allowance, switch never closed
            }
        }
        stopHere();

        long backedOff = backOffUntilClear();
        if (backedOff == 0) return false;   // contact never released

        unsigned long fast = getStepInterval();
        setStepInterval(homeSlowIntervalUs);
        // Allowance covers the retreat plus a margin, so a long retreat still
        // leaves enough travel to reach the switch again.
        moveSteps((long)homeDirection * (backedOff + homeBackoffSteps));
        bool hit = driveUntilLimit();
        setStepInterval(fast);

        stopHere();
        if (!hit) return false;

        // The switch is the base position.
        setOrigin(0);
        return true;
    }

public:
    // Adds an endstop guard on top of the normal step loop.
    void update() override {
        updateLimit();
        

        // Driving further onto a closed endstop crashes the axis. Stop and
        // hold; moving away from the switch stays allowed.
        if (limitState && movingTowardsLimit()) {
            stopHere();
            setState(HOLD);
            return;
        }

        Stepper::update();
    }

    bool isLimitTriggered() const {
        return limitState;
    }

    HomingResult getHomingResult() const {
        return result;
    }

    bool isHomed() const {
        return result == HOMED;
    }

private:
    bool readLimitRaw() const {
        return (limitWiring == NORMALLY_OPEN) ? (digitalRead(limitPin) == LOW)
                                              : (digitalRead(limitPin) == HIGH);
    }

    void updateLimit() {
        if (millis() - limitLastChange < LIMIT_DEBOUNCE_MS) return;

        bool raw = readLimitRaw();
        if (raw == limitState) return;

        limitState = raw;
        limitLastChange = millis();
    }

    void resyncLimit() {
        limitState = readLimitRaw();
        limitLastChange = millis();
    }

    bool movingTowardsLimit() const {
        if (targetStep == currentStep) return false;
        return ((targetStep > currentStep) ? 1 : -1) == homeDirection;
    }

    // Steps towards the current target, stopping early if the endstop trips.
    bool driveUntilLimit() {
        while (currentStep != targetStep) {
            updateLimit();
            if (limitState) return true;
            Stepper::update();
        }

        updateLimit();
        return limitState;
    }

    void driveBlocking() {
        while (currentStep != targetStep) Stepper::update();
    }

    // Retreats away from the switch until it releases
    long backOffUntilClear() {
        long travelled = 0;

        while (travelled < homeMaxSteps) {
            moveSteps((long)-homeDirection * homeBackoffSteps);
            driveBlocking();
            travelled += homeBackoffSteps;

            updateLimit();
            if (!limitState) return travelled;
        }

        return 0;
    }

    static constexpr uint16_t LIMIT_DEBOUNCE_MS = 5;

    uint8_t limitPin;
    LimitWiring limitWiring;
    bool limitState;
    unsigned long limitLastChange;

    long restStep;
    int8_t homeDirection;
    long homeMaxSteps;
    long homeBackoffSteps;
    unsigned long homeSlowIntervalUs;

    HomingResult result;
};
