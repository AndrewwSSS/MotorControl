#pragma once

#if !__has_include(<FastAccelStepper.h>)
#error "FasEngine.h needs FastAccelStepper: add gin66/FastAccelStepper@1.4.0 to lib_deps (ESP32 only)"
#endif

#include <Arduino.h>
#include <FastAccelStepper.h>
#include "Log.h"
#include "StepEngine.h"

// The single FastAccelStepper engine shared by all axes, started on first use.
inline FastAccelStepperEngine& fasEngine() {
    static FastAccelStepperEngine engine;
    static bool started = false;
    if (!started) {
        engine.init();
        started = true;
    }
    return engine;
}

// A StepEngine on FastAccelStepper: pulses and ramps in hardware on the ESP32.
// Bridges the library's differences (reversals, immediate stops, the pulse
// gate) as described in docs/engines.md.
class FasEngine : public StepEngine {
public:
    // Steps/s^2 used when there is no profile: full speed within a step or two.
    static constexpr uint32_t NO_RAMP_ACCELERATION = 10000000UL;

    FasEngine(uint8_t stepPin, uint8_t dirPin, unsigned long stepIntervalUs = 800)
        : stepPin(stepPin), dirPin(dirPin), stepIntervalUs(stepIntervalUs) {}

    void begin() override {
        if (!fas) fas = fasEngine().stepperConnectToPin(stepPin);
        if (!fas) {
            MC_ERROR(F("FasEngine: no free FastAccelStepper for step pin "), stepPin);
            return;
        }

        fas->setDirectionPin(dirPin, true, MC_DIR_SETUP_US);
        applyPace();
    }

    // --- Motion --------------------------------------------------------------

    // Ignored if begin() found no free library stepper.
    void moveTo(long step) override {
        if (!fas) return;

        targetStep = step;
        pending = true;
        send();
    }

    void stopNow() override {
        pending = false;
        if (!fas) return;

        long here = fas->getCurrentPosition();
        fas->forceStopAndNewPosition(here);
        targetStep = here;
    }

    void stopRamped() override {
        pending = false;
        int8_t moving = runningDirection();
        if (moving == 0) return;

        // Where the library will stop: its stopping distance, capped at the move's end.
        long here = fas->getCurrentPosition();
        long end = fas->targetPos();
        long stopAt = here + moving * (long)fas->stepsToStop();
        if ((moving > 0 && stopAt > end) || (moving < 0 && stopAt < end)) stopAt = end;

        fas->stopMove();
        targetStep = stopAt;
    }

    void setPosition(long step) override {
        pending = false;
        targetStep = step;
        if (!fas) return;

        if (fas->isRunning()) fas->forceStopAndNewPosition(step);
        else                  fas->setCurrentPosition(step);
    }

    long position() const override {
        return fas ? fas->getCurrentPosition() : targetStep;
    }

    // Once stopped with nothing pending, the stop position is the target.
    long target() const override {
        if (fas && !pending && !fas->isRunning()) return fas->getCurrentPosition();
        return targetStep;
    }

    bool isRunning() const override {
        return pending || (fas && fas->isRunning());
    }

    // --- Pace ----------------------------------------------------------------

    void setStepInterval(unsigned long us) override {
        stepIntervalUs = us;
        applyPace();
    }

    unsigned long stepInterval() const override {
        return stepIntervalUs;
    }

    unsigned long currentInterval() const override {
        if (!fas || !fas->isRunning()) return stepIntervalUs;

        int32_t us = fas->getCurrentSpeedInUs();
        if (us < 0) us = -us;
        return us ? (unsigned long)us : stepIntervalUs;
    }

    void setProfile(const StepProfile* p) override {
        profile_ = p;
        applyPace();
    }

    const StepProfile* profile() const override {
        return profile_;
    }

    // --- Endstops ------------------------------------------------------------

    void inhibit(int8_t direction) override {
        inhibited = direction;
        if (direction != 0 && direction == runningDirection()) stopNow();
    }

    // --- Work ----------------------------------------------------------------

    // Sends a pending move once a reversal has ramped down or the gate opens.
    bool service() override {
        if (pending) send();
        return false;
    }

    bool stepsOnItsOwn() const override {
        return true;
    }

    void allowPulses(bool allowed) override {
        gateOpen = allowed;
        if (allowed || !fas || !fas->isRunning()) return;

        long resumeAt = targetStep;
        stopNow();
        targetStep = resumeAt;
        pending = true;
    }

    // The library's handle for features StepEngine lacks; nullptr before begin()
    // or if no stepper was free.
    FastAccelStepper* stepper() const {
        return fas;
    }

private:
    // +1 or -1 while the library is moving the axis that way, 0 at rest.
    int8_t runningDirection() const {
        if (!fas || !fas->isRunning()) return 0;
        int32_t speed = fas->getCurrentSpeedInMilliHz();
        return (speed > 0) ? 1 : (speed < 0) ? -1 : 0;
    }

    void send() {
        if (!fas || !gateOpen) return;

        long here = fas->getCurrentPosition();
        int8_t way = (targetStep > here) ? 1 : (targetStep < here) ? -1 : 0;

        if (way != 0 && way == inhibited) {
            pending = false;
            targetStep = here;
            return;
        }

        int8_t moving = runningDirection();
        if (moving != 0 && way != moving) {
            // The library ignores reversals: ramp down and retry from service().
            // A move to the current position stops dead, as a polled engine would.
            if (way == 0) stopNow();
            else          fas->stopMove();
            return;
        }

        pending = false;
        MoveResultCode result = fas->moveTo(targetStep);
        if (result != MOVE_OK) {
            MC_ERROR(F("FasEngine: move to "), targetStep, F(" refused, code "), (int)result);
        }
    }

    void applyPace() {
        if (!fas) return;

        if (fas->setSpeedInUs(stepIntervalUs) != 0) {
            MC_ERROR(F("FasEngine: step interval "), stepIntervalUs, F(" us is out of range"));
        }

        uint32_t a = profile_ ? profile_->accelerationFor(stepIntervalUs) : 0;
        if (a == 0 || a > NO_RAMP_ACCELERATION) a = NO_RAMP_ACCELERATION;
        fas->setAcceleration((int32_t)a);

        if (fas->isRunning()) fas->applySpeedAcceleration();
    }

    uint8_t            stepPin;
    uint8_t            dirPin;
    unsigned long      stepIntervalUs;
    const StepProfile* profile_ = nullptr;
    FastAccelStepper*  fas = nullptr;

    long   targetStep = 0;
    bool   pending = false;      // a move waiting to be sent
    bool   gateOpen = true;
    int8_t inhibited = 0;
};
