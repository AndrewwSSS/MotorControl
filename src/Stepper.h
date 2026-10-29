#pragma once
#include <Arduino.h>
#include "Log.h"
#include "Endstop.h"
#include "StepEngine.h"
#include "StepProfile.h"


// One step-driven axis: what it should do. Where to go, which end is blocked by
// a pressed endstop, whether it is moving or holding, when it has arrived.
// How the pulses get made is the StepEngine's business, so the same axis logic
// runs on a polled loop, a hardware timer or FastAccelStepper - pick the engine
// and hand it over:
//
//   PolledEngine xEngine(2, 5);
//   Stepper x(xEngine, 3200);
//
// The engine is borrowed, not owned, like the endstops and the profile. For the
// common case of a polled step+dir driver, StepDirStepper bundles the two.
class Stepper {
public:
    enum State {
        MOVING,
        HOLD
    };

    // stepsPerRevolution is full steps x microsteps (200 x 16 = 3200 for a
    // 1.8 deg/step motor at 1/16).
    Stepper(StepEngine& engine, uint16_t stepsPerRevolution)
        : engine(engine),
          stepsPerRevolution(stepsPerRevolution),
          state(HOLD),
          justReached(false),
          debugEnabled(false) {
        endstops[0] = nullptr;
        endstops[1] = nullptr;
    }

    virtual ~Stepper() = default;

    void begin() {
        engine.begin();
        setOrigin(0);

        for (uint8_t i = 0; i < 2; i++) {
            if (endstops[i]) endstops[i]->begin();
        }
    }

    // Returns immediately. Call repeatedly from loop(); movement is
    // non-blocking. A polled engine makes at most one pulse per call.
    void update() {
        updateEndstops();

        if (!engine.isRunning()) {
            setState(HOLD);
            return;
        }

        // Driving further onto a pressed endstop crashes the axis. The engine
        // is told which way is blocked, so one that steps on its own refuses
        // the pulse too; moving away from the switch stays allowed.
        Endstop* hit = blockingEndstop();
        engine.inhibit(hit ? hit->getSide() : 0);
        if (hit) {
            MC_INFO(F("Endstop "), hit->sideName(), F(" on pin "), hit->getPin(),
                    F(" stopped the axis at step "), engine.position());
            engine.stopNow();
            setState(HOLD);
            return;
        }

        if (engine.service()) {
            setState(MOVING);
            if (debugEnabled) MC_TRACE(F("Current step: "), engine.position(), F(" of "), engine.target());
        } else if (engine.stepsOnItsOwn()) {
            setState(engine.isRunning() ? MOVING : HOLD);
        }
    }

    // Absolute move, in steps from the base position. May be negative.
    void moveToStep(long step) {
        engine.moveTo(step);

        if (debugEnabled) MC_DEBUG(F("Target step: "), engine.target());
    }

    // Relative move. Anchored on the target rather than the current position
    // so repeated jogs compose while a move is still in flight.
    void moveSteps(long deltaSteps) {
        moveToStep(engine.target() + deltaSteps);
    }

    // Cancels a move in flight by making the current position the target.
    // Immediate: the axis stops on the next pulse it was going to emit. This is
    // what an endstop hit wants, ramp or no ramp.
    void stopHere() {
        moveToStep(engine.position());
    }

    // Brings the axis to rest over the profile's stopping distance instead of
    // dropping it dead, for a stop that is asked for rather than forced. Same
    // thing as stopHere() when there is no profile, and it never extends a move
    // that was already shorter than the stopping distance.
    void stopRamped() {
        if (!engine.isRunning()) return;

        engine.stopRamped();

        if (debugEnabled) MC_DEBUG(F("Target step: "), engine.target());
    }

    // Redefines where the axis believes it is
    void setOrigin(long step = 0) {
        engine.setPosition(step);
        setState(HOLD);
    }

    long getStep() const {
        return engine.position();
    }

    long getTargetStep() const {
        return engine.target();
    }

    State getState() const {
        return state;
    }

    // True exactly once, the first time this is polled after the axis settles
    // into HOLD.
    bool reachedTarget() {
        bool r = justReached;
        justReached = false;
        return r;
    }

    // Attach a limit switch. It goes into the slot for its own side, so an axis
    // with a switch at each end takes two calls. Borrowed, not owned. Attach
    // before begin(), which sets up the pin.
    void setEndstop(Endstop* endstop) {
        if (endstop) endstops[slot(endstop->getSide())] = endstop;
    }

    void removeEndstop(Endstop::Side side) {
        endstops[slot(side)] = nullptr;
    }

    // nullptr when there is no switch at that end.
    Endstop* getEndstop(Endstop::Side side) const {
        return endstops[slot(side)];
    }

    // Attach a speed profile, or nullptr for a flat interval (the default).
    // The profile is borrowed, not owned - it is expected to outlive the axis,
    // which a file-scope instance in the sketch does.
    void setProfile(const StepProfile* p) {
        engine.setProfile(p);
    }

    const StepProfile* getProfile() const {
        return engine.profile();
    }

    // The interval the next pulse will actually wait out: the profile's idea of
    // the pace here, or the flat interval when there is no profile. Distinct
    // from getStepInterval(), which is the top speed the axis was asked for.
    unsigned long currentInterval() const {
        return engine.currentInterval();
    }

    // Pulse spacing, i.e. speed. With a profile attached this is the top speed
    // the ramp works towards. Settable after construction because homing
    // switches between a fast seek and a slow, repeatable re-approach.
    void setStepInterval(unsigned long us) {
        engine.setStepInterval(us);
    }

    unsigned long getStepInterval() const {
        return engine.stepInterval();
    }

    // The one piece of calibration a degree layer needs; unused by step-space
    // motion itself.
    uint16_t getStepsPerRevolution() const {
        return stepsPerRevolution;
    }

    StepEngine& getEngine() const {
        return engine;
    }

    // Per-axis switch for the DEBUG and TRACE messages. Has no effect unless
    // MC_LOG_LEVEL compiles them in - see Log.h.
    void toggleDebug() {
        this->debugEnabled = !this->debugEnabled;
    }

    bool getDebug() const {
        return this->debugEnabled;
    }

protected:
    // Motion is reported through here rather than by writing `state`, so the
    // MOVING -> HOLD edge behind reachedTarget() is detected in one place.
    void setState(State newState) {
        if (newState == HOLD && state != HOLD) {
            justReached = true;
        }
        state = newState;
    }

    static uint8_t slot(Endstop::Side side) {
        return (side == Endstop::MIN) ? 0 : 1;
    }

    void updateEndstops() {
        for (uint8_t i = 0; i < 2; i++) {
            if (endstops[i]) endstops[i]->update();
        }
    }

    // The pressed endstop in the direction of travel, if any.
    Endstop* blockingEndstop() const {
        Endstop* ahead = endstops[(engine.target() > engine.position()) ? 1 : 0];
        return (ahead && ahead->isTriggered()) ? ahead : nullptr;
    }

    StepEngine& engine;          // borrowed
    uint16_t stepsPerRevolution;
    Endstop* endstops[2];        // borrowed; [0] MIN end, [1] MAX end

    State state;
    bool justReached;
    bool debugEnabled;
};
