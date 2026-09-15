#pragma once
#include <Arduino.h>
#include "A4988Stepper.h"

// An A4988Stepper that *finds* its base position instead of assuming one.
//
// Stepper::begin() can only declare "wherever the shaft is now is step 0",
// which is an assumption that silently poisons every later position if it is
// wrong. This class earns the base position against a mechanical endstop
// (an Omron D3V-style SPDT microswitch, or any snap-action switch): begin()
// drives the axis onto the switch, makes that point step 0, and then moves to
// the configured rest position. After begin() returns, step 0 is a real,
// repeatable place on the machine.
//
// The search is two-pass. The trip point of a snap-action switch is only
// repeatable at low speed - approach it fast and the contact closes a little
// further along each time, by an amount that varies with speed. So the seek
// pass finds it quickly, the axis backs off to release it, and a slow
// re-approach takes the reading that actually sets the base position.
class HomedStepper : public A4988Stepper {
public:
    // Prefer NORMALLY_CLOSED. Wired that way the loop is closed while the axis
    // is clear, so a cut wire, a cold crimp or an unplugged connector reads
    // exactly like a trigger and stops the axis. Wired NORMALLY_OPEN those
    // same faults read "all clear" and the axis drives into the stop.
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

    // Optional tuning; call before begin(). homeDirection is the sign of
    // travel towards the switch: -1 when the endstop sits at the low end of
    // travel (the usual arrangement), +1 at the high end. maxSteps bounds the
    // seek so a dead or miswired switch cannot drive the axis into the stop
    // forever. restStep is where the axis parks once homed, in steps from the
    // switch.
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

    // Sets up the pins, runs the search, then drives to restStep. Blocking:
    // homing is a start-up operation, and until it finishes there is no
    // position worth reporting. A failed search leaves the axis where it
    // stopped and reports HOMING_FAILED rather than pretending.
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
    // returns false with the axis stopped wherever the failure was detected.
    bool home() {
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
        // INPUT_PULLUP idles the pin HIGH. A NO contact closes to GND when hit
        // (LOW means triggered); a NC contact opens when hit and lets the
        // pull-up win (HIGH means triggered).
        return (limitWiring == NORMALLY_OPEN) ? (digitalRead(limitPin) == LOW)
                                              : (digitalRead(limitPin) == HIGH);
    }

    // A snap-action switch chatters for a few milliseconds around the
    // transition. Waiting for a stable level would delay detection and let the
    // axis overshoot the trip point, so act on the first edge seen and then
    // ignore the pin until the contacts have settled.
    void updateLimit() {
        if (millis() - limitLastChange < LIMIT_DEBOUNCE_MS) return;

        bool raw = readLimitRaw();
        if (raw == limitState) return;

        limitState = raw;
        limitLastChange = millis();
    }

    // Ground truth, bypassing the debounce lock-out. A run may start within
    // the debounce window of a previous one or of a knock, and opening the
    // search in the wrong phase from a stale reading wastes a pass.
    void resyncLimit() {
        limitState = readLimitRaw();
        limitLastChange = millis();
    }

    bool movingTowardsLimit() const {
        if (targetStep == currentStep) return false;
        return ((targetStep > currentStep) ? 1 : -1) == homeDirection;
    }

    // Steps towards the current target, stopping early if the endstop trips.
    // True if it tripped, false if the move ran out without triggering. Calls
    // Stepper::update() directly: the endstop guard in our own update() would
    // fight a pass that is deliberately driving onto the switch.
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

    // Retreats away from the switch until it releases, in backoffSteps
    // increments. A fixed retreat is not enough: the axis can power up further
    // past the trip point than any single increment - shoved there by hand, or
    // left there by a previous failed run - and would then never clear. Bounded
    // by the seek allowance so a contact stuck closed fails instead of driving
    // the axis off the far end. Returns the distance travelled, 0 on failure.
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
