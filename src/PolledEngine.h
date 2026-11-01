#pragma once
#include <Arduino.h>
#include "SoftEngine.h"

// Makes pulses on a step+dir driver's pins, each one timed against micros()
// when update() comes round: A4988, DRV8825, TB6600, DM542 and the like. They
// differ only in timing, which is the pulse width below and the DIR setup
// above.
//
// Works on any board, with no timer or interrupt. The price is that the pace is
// only as steady as the caller's loop: anything that holds loop() up holds the
// next pulse up with it.
class PolledEngine : public SoftEngine {
public:
    static constexpr uint8_t NO_ENABLE_PIN = 0xFF;

    // Minimum STEP high time, per the driver's datasheet.
    static constexpr uint8_t A4988_PULSE_US  = 2;
    static constexpr uint8_t TB6600_PULSE_US = 5;

    PolledEngine(uint8_t stepPin, uint8_t dirPin,
                 uint8_t enablePin = NO_ENABLE_PIN,
                 unsigned long stepIntervalUs = 800,
                 uint8_t stepPulseUs = A4988_PULSE_US)
        : SoftEngine(stepIntervalUs),
          stepPin(stepPin), dirPin(dirPin), enablePin(enablePin),
          stepPulseUs(stepPulseUs), lastForward(DIR_UNKNOWN) {}

    void begin() override {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        digitalWrite(stepPin, LOW);
        lastForward = DIR_UNKNOWN;

        if (enablePin != NO_ENABLE_PIN) {
            pinMode(enablePin, OUTPUT);
        }

        enable();
        resetTiming();
    }

    bool service() override {
        return tick(micros());
    }

    bool stepsOnItsOwn() const override {
        return false;
    }

    // The driver's EN input is active low. Releasing it frees the shaft and
    // stops the coils heating
    void enable() override {
        if (enablePin != NO_ENABLE_PIN) digitalWrite(enablePin, LOW);
    }

    void disable() override {
        if (enablePin != NO_ENABLE_PIN) digitalWrite(enablePin, HIGH);
    }

    uint8_t getStepPin() const {
        return stepPin;
    }

    uint8_t getStepPulse() const {
        return stepPulseUs;
    }

protected:
    // tick() pulses straight after this, so a change of direction waits out
    // the setup time here. Without it a slow driver takes the first step of a
    // reversal the old way, and the counter drifts a step per reversal.
    void applyDirection(bool forward) override {
        int8_t dir = forward ? 1 : 0;
        if (dir == lastForward) return;

        digitalWrite(dirPin, forward ? HIGH : LOW);
        delayMicroseconds(MC_DIR_SETUP_US);
        lastForward = dir;
    }

    void pulseStep() override {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepPulseUs);
        digitalWrite(stepPin, LOW);
    }

    static constexpr int8_t DIR_UNKNOWN = -1;   // forces the first write

    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;
    uint8_t stepPulseUs;
    int8_t  lastForward;
};
