#pragma once
#include <Arduino.h>
#include "SoftEngine.h"

// Step+dir pulses timed against micros() from update(), on any board. The pace
// is only as steady as the caller's loop. See docs/engines.md.
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

    // EN is active LOW. Disabling frees the shaft and stops the coils heating.
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
    // Waits out the DIR setup time on a reversal; without it a slow driver takes
    // the first step the old way and the count drifts.
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
