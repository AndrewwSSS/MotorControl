#pragma once
#include <Arduino.h>

// A limit switch at one end of an axis's travel: an Omron D3V-style SPDT
// microswitch, or any snap-action switch, wired between a pin and GND.
//
// Attach it to a Stepper with setEndstop() and the axis refuses to drive
// further onto it once it is pressed; moving away stays allowed. An axis has a
// slot at each end, so a second switch on its own pin is a second Endstop with
// the other Side. Homing (Homing.h) drives the axis onto one of them to find
// its base position.
//
// Borrowed, not owned, like a StepProfile: declare it at file scope in the
// sketch so it outlives the axis.
class Endstop {
public:
    // Prefer NORMALLY_CLOSED. Wired that way the loop is closed while the axis
    // is clear, so a cut wire, a cold crimp or an unplugged connector reads
    // exactly like a trigger and stops the axis. Wired NORMALLY_OPEN those
    // same faults read "all clear" and the axis drives into the stop.
    enum Wiring : uint8_t {
        NORMALLY_OPEN,
        NORMALLY_CLOSED
    };

    // Which end of travel the switch sits at, as the sign of the step count
    // moving towards it.
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

    // A snap-action switch chatters for a few milliseconds around the
    // transition. Waiting for a stable level would delay detection and let the
    // axis overshoot the trip point, so act on the first edge seen and then
    // ignore the pin until the contacts have settled.
    void update() {
        if (millis() - lastChange < DEBOUNCE_MS) return;

        bool raw = readRaw();
        if (raw == triggered) return;

        triggered = raw;
        lastChange = millis();
    }

    // Ground truth, bypassing the debounce lock-out. A homing run may start
    // within the debounce window of a previous one or of a knock, and opening
    // the search in the wrong phase from a stale reading wastes a pass.
    //
    // A reading is not an edge, so it opens no lock-out of its own: the next
    // update() acts on a press at once. Dated one window back to say so - the
    // unsigned subtraction in update() makes that safe at any millis().
    void resync() {
        triggered = readRaw();
        lastChange = millis() - DEBOUNCE_MS;
    }

    // Debounced state, as of the last update().
    bool isTriggered() const {
        return triggered;
    }

    // The pin right now, interpreted through the wiring. INPUT_PULLUP idles the
    // pin HIGH: a NO contact closes to GND when hit (LOW means triggered), a NC
    // contact opens when hit and lets the pull-up win (HIGH means triggered).
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
