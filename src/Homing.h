#pragma once
#include <Arduino.h>
#include "Log.h"
#include "Endstop.h"
#include "Stepper.h"

// Two-pass homing against an endstop: fast seek, back off, slow re-approach.
// The trip point becomes step 0. See docs/homing.md.

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

// Runs the current move to its end, endstop guard included, and settles HOLD.
// Self-stepping engines are checked once a millisecond instead of spinning.
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

// Retreats in backoffSteps increments until the switch releases, bounded by
// maxSteps. Returns the distance travelled, 0 on failure.
inline long backOffUntilClear(Stepper& axis, Endstop& endstop, const HomingConfig& cfg) {
    long travelled = 0;

    while (travelled < cfg.maxSteps) {
        long target = axis.getStep() - (long)cfg.towards * cfg.backoffSteps;
        axis.moveToStep(target);
        drive(axis);

        // Stopped by the other endstop: both read pressed, a wiring fault.
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

    // Already on the switch: skip the seek and back off first.
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
    // Allow the retreat distance plus a margin to reach the switch again.
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

// Runs the search, then moves to cfg.restStep. Blocking. A failure leaves the
// axis where it stopped. The driver must be energised (StepperManager::home()).
inline HomingResult homeAxis(Stepper& axis, const HomingConfig& cfg) {
    Endstop* endstop = axis.getEndstop(cfg.towards);
    if (!endstop) {
        MC_ERROR(F("Homing: no endstop attached at the requested end"));
        return HOMING_NO_ENDSTOP;
    }

    MC_INFO(F("Homing towards "), endstop->sideName(),
            F(" endstop on pin "), endstop->getPin());

    // Cancel any move in flight; relative moves would otherwise add to the seek.
    axis.stopHere();

    // The search runs past the soft limits; re-armed on success, restored on failure.
    bool wasArmed = axis.limitsArmed();
    axis.armLimits(false);

    const StepProfile* saved = axis.getProfile();
    axis.setProfile(nullptr);
    HomingResult result = mc_homing::search(axis, *endstop, cfg);
    axis.setProfile(saved);

    if (result == HOMED) {
        MC_INFO(F("Homed, moving to rest step "), cfg.restStep);
        axis.moveToStep(cfg.restStep);
        mc_homing::drive(axis);
        axis.armLimits(true);
    } else {
        axis.armLimits(wasArmed);
    }

    // The search's own stops are not commanded moves: swallow the reached edge.
    axis.reachedTarget();
    return result;
}
