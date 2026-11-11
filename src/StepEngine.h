#pragma once
#include <Arduino.h>
#include "StepProfile.h"

// How long DIR must have settled before the next STEP edge, in microseconds.
// Only paid when the direction actually changes, never per step. 5 us covers
// the opto-isolated drivers (TB6600, DM542), which are the slow ones
#ifndef MC_DIR_SETUP_US
#define MC_DIR_SETUP_US 5
#endif



// A StepEngine is responsible for making the actual step pulses.
// A Stepper decides what the axis should
// do - where to go, which end is blocked by a pressed switch, when it has
// arrived - and hands the doing to one of these.
//
// The split exists because there is more than one good way to make pulses:
//
//   PolledEngine  times each pulse against micros() from update(). Any board;
//                 the pace is only as steady as the caller's loop.
//   FasEngine     FastAccelStepper on the ESP32: pulses come from hardware.
//   TimerEngine   an ESP32 hardware timer running our own stepping code.
//
// It works in moves, not pulses: the engine owns position, target and the
// ramp, because an engine that steps on its own (FasEngine, TimerEngine) is
// the only thing that knows where the axis is between two update() calls. The
// Stepper never counts steps itself; it asks.
//
// Positions are in steps from the base position and may be negative.
//
// Borrowed, not owned, like an Endstop or a StepProfile: declare it at file
// scope in the sketch so it outlives the axis.
class StepEngine {
public:
    virtual ~StepEngine() = default;

    // Sets up pins or hardware. Called by Stepper::begin().
    virtual void begin() = 0;

    // --- Motion --------------------------------------------------------------

    // Absolute move. A move commanded while one is in flight composes onto
    // it: the ramp carries on from where it is rather than starting over.
    virtual void moveTo(long step) = 0;

    // Cancels the move in flight: the current position becomes the target, and
    // nothing more is sent. What an endstop hit wants, ramp or no ramp.
    virtual void stopNow() = 0;

    // Brings the axis to rest over the profile's stopping distance instead of
    // dropping it dead. Same as stopNow() without a profile, and never extends
    // a move that was already shorter than the stopping distance.
    virtual void stopRamped() = 0;

    // Redefines where the axis believes it is, and makes that the target too.
    virtual void setPosition(long step) = 0;

    virtual long position() const = 0;
    virtual long target() const = 0;

    // True while there are steps left to make.
    virtual bool isRunning() const = 0;

    // --- Pace ----------------------------------------------------------------

    // The flat interval between pulses, or the top speed when a profile is
    // attached.
    virtual void setStepInterval(unsigned long us) = 0;
    virtual unsigned long stepInterval() const = 0;

    // The interval the axis is running at right now, which under a ramp is not
    // the one it was asked for.
    virtual unsigned long currentInterval() const = 0;

    // Borrowed; nullptr for a flat interval.
    virtual void setProfile(const StepProfile* p) = 0;
    virtual const StepProfile* profile() const = 0;

    // --- Endstops ------------------------------------------------------------

    // Which way the axis must not travel: +1 (towards MAX), -1 (towards MIN) or
    // 0 for neither. The Stepper decides from its switches and says so on every
    // update(); the engine enforces it, so an engine that steps on its own can
    // refuse a pulse between two update() calls. A move heading the forbidden
    // way is cancelled, as stopNow() would. Moving the other way stays allowed.
    virtual void inhibit(int8_t direction) = 0;

    // --- Work ----------------------------------------------------------------

    // Called on every Stepper::update(). A polled engine makes at most one
    // pulse here, and returns true when it did. An engine that steps on its own
    // has little or nothing to do, and returns false.
    virtual bool service() = 0;

    // True when pulses happen without service() being called - from a timer or
    // from hardware. Homing and the fleet use it to decide whether to spin on
    // update() or to sleep while a move runs.
    virtual bool stepsOnItsOwn() const = 0;

    // The fleet's gate: while it is shut no pulse may go out, though moves can
    // still be commanded and wait. StepperManager shuts it whenever the drivers
    // cannot take a pulse - released by the idle timer, or still waking after
    // EN came back - because a pulse sent then is a step the counter claims but
    // the axis never took. Open by default, so an axis outside a fleet runs.
    virtual void allowPulses(bool allowed) = 0;

    // --- Driver enable -------------------------------------------------------

    // For an engine that owns its driver's EN pin. Most axes share one EN line
    // that the fleet owns, and leave these as the no-ops they are by default.
    virtual void enable() {}
    virtual void disable() {}
};
