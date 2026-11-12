#pragma once

// A StepEngine without pins or timing, in POLLED or OWN (self-stepping) mode,
// for testing Stepper, homing and the fleet. See docs/testing.md.

#include <Arduino.h>
#include "StepEngine.h"

class FakeEngine : public StepEngine {
public:
    enum Mode { POLLED, OWN };

    explicit FakeEngine(Mode mode = POLLED, long stepsPerService = 1)
        : mode(mode), stepsPerService(stepsPerService) {}

    void begin() override { begins++; }

    void moveTo(long step) override {
        if (pos == tgt) moveStart = pos;
        tgt = step;
    }

    void stopNow() override {
        stopNows++;
        tgt = pos;
    }

    void stopRamped() override {
        long left = tgt - pos;
        long span = profile_ ? profile_->stopDistance() : 0;
        long remaining = (left > 0) ? left : -left;
        if (span > remaining) span = remaining;
        tgt = pos + ((left > 0) ? span : -span);
    }

    void setPosition(long step) override {
        pos = tgt = moveStart = step;
    }

    long position() const override { return pos; }
    long target() const override { return tgt; }
    bool isRunning() const override { return pos != tgt; }

    void setStepInterval(unsigned long us) override { interval = us; }
    unsigned long stepInterval() const override { return interval; }

    unsigned long currentInterval() const override {
        if (!profile_) return interval;
        long done = pos - moveStart;
        long left = tgt - pos;
        return profile_->intervalFor((done > 0) ? done : -done,
                                     (left > 0) ? left : -left, interval);
    }

    void setProfile(const StepProfile* p) override { profile_ = p; }
    const StepProfile* profile() const override { return profile_; }

    void inhibit(int8_t direction) override { inhibited = direction; }
    void allowPulses(bool allowed) override { pulsesAllowed = allowed; }

    bool service() override {
        services++;
        long n = (mode == POLLED) ? 1 : stepsPerService;
        bool stepped = false;
        for (long i = 0; i < n && step(); i++) stepped = true;
        return (mode == POLLED) && stepped;
    }

    bool stepsOnItsOwn() const override { return mode == OWN; }

    void enable() override { enables++; }
    void disable() override { disables++; }

    long shaft() const { return shaftAt; }
    void placeShaft(long at) { shaftAt = at; }

    Mode mode;
    long stepsPerService;

    // What the engine was told.
    int           begins = 0;
    int           stopNows = 0;
    int           services = 0;
    int           enables = 0;
    int           disables = 0;
    int8_t        inhibited = 0;
    bool          pulsesAllowed = true;
    unsigned long interval = 800;
    long          pulses = 0;

private:
    // The same rules as SoftEngine::tick(), minus the timing.
    bool step() {
        if (pos == tgt || !pulsesAllowed) return false;

        int8_t dir = (tgt > pos) ? 1 : -1;
        if (dir == inhibited) {
            tgt = pos;
            return false;
        }

        pos += dir;
        shaftAt += dir;
        pulses++;
        fake::advanceUs(interval);
        return true;
    }

    long pos = 0;
    long tgt = 0;
    long moveStart = 0;
    long shaftAt = 0;
    const StepProfile* profile_ = nullptr;
};
