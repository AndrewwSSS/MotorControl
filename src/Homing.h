#pragma once
#include <Arduino.h>
#include "Log.h"
#include "Endstop.h"
#include "Stepper.h"

// Homing: earning a base position against an endstop instead of assuming one.
//
// Stepper::begin() can only declare "wherever the shaft is now is step 0",
// which is an assumption that silently poisons every later position if it is
// wrong. homeAxis() drives the axis onto the endstop at one end, makes the trip
// point step 0, and parks at the configured rest step. Afterwards step 0 is a
// real, repeatable place on the machine.
//
// The search is two-pass. The trip point of a snap-action switch is only
// repeatable at low speed - approach it fast and the contact closes a little
// further along each time, by an amount that varies with speed. So the seek
// pass finds it quickly, the axis backs off to release it, and a slow
// re-approach takes the reading that actually sets the base position.
//
// Works with any Stepper subtype: it only needs an Endstop attached on the
// side it homes towards.

struct HomingConfig {
    Endstop::Side towards;          // the end whose switch sets step 0
    long          restStep;         // where to park afterwards, in steps from the switch
    long          maxSteps;         // seek allowance; a dead switch fails instead of driving on
    long          backoffSteps;     // retreat increment while releasing the switch
    unsigned long slowIntervalUs;   // pace of the re-approach that takes the reading
};

enum HomingResult : uint8_t {
    NOT_HOMED,                  // never run, or the origin was redefined since
    HOMED,
    HOMING_NO_ENDSTOP,          // nothing attached on the side asked for
    HOMING_NEVER_CLOSED,        // seek ran the whole allowance without a trigger
    HOMING_NEVER_RELEASED,      // contact stayed closed while backing off
    HOMING_NOT_FOUND            // the slow re-approach missed the switch
};

inline const __FlashStringHelper* homingResultName(HomingResult r) {
    switch (r) {
        case HOMED:                 return F("homed");
        case HOMING_NO_ENDSTOP:     return F("no endstop");
        case HOMING_NEVER_CLOSED:   return F("switch never closed");
        case HOMING_NEVER_RELEASED: return F("switch never released");
        case HOMING_NOT_FOUND:      return F("switch not found on re-approach");
        default:                    return F("not homed");
    }
}

namespace mc_homing {

// Runs the current move to its end. The endstop guard in Stepper::update()
// still applies, so a move towards a pressed switch ends there. The last
// update() settles the axis into HOLD, which the loop alone would leave to the
// caller's next update().
//
// A polled engine only steps when update() is called, so it gets called as
// fast as the loop allows. An engine that steps on its own needs no help with
// the pulses: checking once a millisecond still runs the endstop guard, and
// leaves the CPU to everything else (on the ESP32, the other tasks).
inline void drive(Stepper& axis) {
    bool own = axis.getEngine().stepsOnItsOwn();

    while (axis.getStep() != axis.getTargetStep()) {
        axis.update();
        if (own) delay(1);
    }
    axis.update();
}

// Runs the current move and reports whether the switch closed on the way.
inline bool driveUntilTriggered(Stepper& axis, Endstop& endstop) {
    drive(axis);
    endstop.update();
    return endstop.isTriggered();
}

// Retreats away from the switch until it releases, in backoffSteps
// increments. A fixed retreat is not enough: the axis can power up further past
// the trip point than any single increment - shoved there by hand, or left
// there by a previous failed run - and would then never clear. Bounded by the
// seek allowance so a contact stuck closed fails instead of driving the axis
// off the far end. Returns the distance travelled, 0 on failure.
inline long backOffUntilClear(Stepper& axis, Endstop& endstop, const HomingConfig& cfg) {
    long travelled = 0;

    while (travelled < cfg.maxSteps) {
        long target = axis.getStep() - (long)cfg.towards * cfg.backoffSteps;
        axis.moveToStep(target);
        drive(axis);

        // Cut short means the endstop at the other end stopped the retreat:
        // the axis is pinned between two "pressed" switches, which is a wiring
        // fault, not something more retreating will fix.
        if (axis.getStep() != target) {
            MC_ERROR(F("Homing: backing off ran into the opposite endstop"));
            return 0;
        }
        travelled += cfg.backoffSteps;

        endstop.update();
        if (!endstop.isTriggered()) return travelled;
    }

    return 0;
}

inline HomingResult search(Stepper& axis, Endstop& endstop, const HomingConfig& cfg) {
    endstop.resync();

    // Starting already on the switch leaves the seek pass nothing to find, so
    // skip to the back-off. Either way the base position is set from a fresh
    // closing edge during the slow re-approach.
    if (!endstop.isTriggered()) {
        axis.moveSteps((long)cfg.towards * cfg.maxSteps);

        if (!driveUntilTriggered(axis, endstop)) {
            axis.stopHere();
            MC_ERROR(F("Homing: switch on pin "), endstop.getPin(),
                     F(" never closed within "), cfg.maxSteps, F(" steps"));
            return HOMING_NEVER_CLOSED;
        }
    } else {
        MC_INFO(F("Homing: already on the switch on pin "), endstop.getPin(),
                F(", skipping seek pass"));
    }
    axis.stopHere();

    long backedOff = backOffUntilClear(axis, endstop, cfg);
    if (backedOff == 0) {
        MC_ERROR(F("Homing: switch on pin "), endstop.getPin(),
                 F(" never released while backing off"));
        return HOMING_NEVER_RELEASED;
    }

    unsigned long fast = axis.getStepInterval();
    axis.setStepInterval(cfg.slowIntervalUs);
    // Allowance covers the retreat plus a margin, so a long retreat still
    // leaves enough travel to reach the switch again.
    axis.moveSteps((long)cfg.towards * (backedOff + cfg.backoffSteps));
    bool hit = driveUntilTriggered(axis, endstop);
    axis.setStepInterval(fast);

    axis.stopHere();
    if (!hit) {
        MC_ERROR(F("Homing: switch on pin "), endstop.getPin(),
                 F(" not found on slow re-approach"));
        return HOMING_NOT_FOUND;
    }

    // The switch is the base position.
    axis.setOrigin(0);
    return HOMED;
}

}

// Runs the search, then drives to cfg.restStep. Blocking: homing is a start-up
// operation, and until it finishes there is no position worth reporting. A
// failed search leaves the axis where it stopped and says why rather than
// pretending. The driver must already be energised - StepperManager::home()
// sees to that.
//
// The speed profile is detached for the search: the trip point is only
// repeatable at a constant low speed, which is the whole reason for the slow
// pass. It is back in place for the move to the rest step.
inline HomingResult homeAxis(Stepper& axis, const HomingConfig& cfg) {
    Endstop* endstop = axis.getEndstop(cfg.towards);
    if (!endstop) {
        MC_ERROR(F("Homing: no endstop attached at the requested end"));
        return HOMING_NO_ENDSTOP;
    }

    MC_INFO(F("Homing towards "), endstop->sideName(),
            F(" endstop on pin "), endstop->getPin());

    // Homing can be asked for at any time, not only at start-up. A move still
    // in flight would otherwise be added to the seek, since relative moves
    // build on the target.
    axis.stopHere();

    const StepProfile* saved = axis.getProfile();
    axis.setProfile(nullptr);
    HomingResult result = mc_homing::search(axis, *endstop, cfg);
    axis.setProfile(saved);

    if (result == HOMED) {
        MC_INFO(F("Homed, moving to rest step "), cfg.restStep);
        axis.moveToStep(cfg.restStep);
        mc_homing::drive(axis);
    }

    // The search settles the axis several times; none of that is a move the
    // caller commanded, so swallow the edge rather than fire a reached callback.
    axis.reachedTarget();
    return result;
}
