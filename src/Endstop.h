#pragma once
#include <Arduino.h>

// A limit switch between a pin and GND at one end of an axis's travel. Borrowed
// by the Stepper, so it must outlive it. See docs/homing.md.
class Endstop {
public:
    // Prefer NORMALLY_CLOSED: a broken wire then reads as triggered, not clear.
    enum Wiring : uint8_t {
        NORMALLY_OPEN,
        NORMALLY_CLOSED
    };

    // The end of travel, as the sign of the step direction towards it.
    enum Side : int8_t {
        MIN = -1,
        MAX = 1
    };

    Endstop(uint8_t pin, Wiring wiring, Side side)
        : pin(pin), wiring(wiring), side(side),
          triggered(false), lastChange(0) {}

    // Called by Stepper::begin() for every attached endstop.
    void begin() {
        pinMode(pin, INPUT_PULLUP);
        resync();
    }

    // Acts on the first edge, then ignores the pin while the contacts chatter:
    // waiting for a stable level would let the axis overshoot.
    void update() {
        if (millis() - lastChange < DEBOUNCE_MS) return;

        bool raw = readRaw();
        if (raw == triggered) return;

        triggered = raw;
        lastChange = millis();
    }

    // Reads the pin directly, bypassing the debounce window, and opens no window
    // of its own. Dated one window back; unsigned subtraction keeps that safe.
    void resync() {
        triggered = readRaw();
        lastChange = millis() - DEBOUNCE_MS;
    }

    // Debounced state, as of the last update().
    bool isTriggered() const {
        return triggered;
    }

    // The pin now, through the wiring: NO triggers LOW, NC triggers HIGH (pull-up).
    bool readRaw() const {
        bool high = (digitalRead(pin) == HIGH);
        return (wiring == NORMALLY_OPEN) ? !high : high;
    }

    uint8_t getPin() const {
        return pin;
    }

    Wiring getWiring() const {
        return wiring;
    }

    Side getSide() const {
        return side;
    }

    const __FlashStringHelper* sideName() const {
        return (side == MIN) ? F("MIN") : F("MAX");
    }

private:
    static constexpr uint16_t DEBOUNCE_MS = 5;

    uint8_t       pin;
    Wiring        wiring;
    Side          side;
    bool          triggered;
    unsigned long lastChange;
};
