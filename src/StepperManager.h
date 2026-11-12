#pragma once
#include <Arduino.h>
#include "Stepper.h"
#include "MotorManager.h"
#include "Homing.h"

typedef MotorManager<Stepper>::ReachedCallback StepperReachedCallback;

// A fleet of step-driven axes: MotorManager's lifecycle plus commands in steps,
// plus the driver enable line the axes share.
class StepperManager : public MotorManager<Stepper> {
public:
    static constexpr uint8_t NO_ENABLE_PIN = 0xFF;

    // Driver wake-up time after EN is energised, before the first pulse.
    static constexpr unsigned long DEFAULT_WAKE_US = 1000;

    explicit StepperManager(uint8_t capacity, uint8_t enablePin = NO_ENABLE_PIN)
        : MotorManager<Stepper>(capacity),
          enablePin(enablePin),
          idleReleaseMs(0), wakeUs(DEFAULT_WAKE_US),
          powered(false), waking(false), enabledAt(0), lastActiveAt(0) {
        homing = new HomingResult[capacity];
        for (uint8_t i = 0; i < capacity; i++) homing[i] = NOT_HOMED;
    }

    ~StepperManager() {
        delete[] homing;
    }

    // Energises the drivers and sets up every axis. Homing is separate: home().
    void begin() {
        if (enablePin != NO_ENABLE_PIN) pinMode(enablePin, OUTPUT);

        enable();
        lastActiveAt = millis();

        beginMotors();
    }

    void update() {
        if (hasWork()) {
            lastActiveAt = millis();
            if (!powered) enable();

            // Withhold pulses while the drivers wake (they would be counted but
            // lost), without blocking the caller's loop.
            if (!ready()) return;
        } else if (releasesWhenIdle() && powered &&
                   millis() - lastActiveAt >= idleReleaseMs) {
            disable();
        }

        updateMotors();
    }

    // From position, not state: state turns MOVING only after a pulse, and
    // pulses are what the gate holds back.
    bool hasWork() const {
        for (uint8_t i = 0; i < count; i++) {
            if (motors[i]->getStep() != motors[i]->getTargetStep()) return true;
        }
        return false;
    }

    // Releases EN after idleMs with the fleet at rest; the next move takes it
    // back. 0 (default) never releases. See docs/power.md.
    void setIdleRelease(unsigned long idleMs, unsigned long wakeUs = DEFAULT_WAKE_US) {
        this->idleReleaseMs = idleMs;
        this->wakeUs = wakeUs;
    }

    unsigned long getIdleRelease() const {
        return idleReleaseMs;
    }

    // EN is active LOW. Releasing stops the coils heating but loses holding torque.
    void enable() {
        if (enablePin != NO_ENABLE_PIN) {
            digitalWrite(enablePin, LOW);

            // Only a released -> energised transition opens a wake-up window.
            if (!powered) {
                enabledAt = micros();
                waking = true;
                allowPulses(false);
            }
        }

        powered = true;

        if (!waking) allowPulses(true);
    }

    void disable() {
        if (enablePin != NO_ENABLE_PIN) digitalWrite(enablePin, HIGH);

        powered = false;
        waking = false;
        allowPulses(false);
    }

    bool isPowered() const {
        return powered;
    }

    uint8_t getEnablePin() const {
        return enablePin;
    }

    // Absolute move, in steps from the base position.
    void moveToStep(uint8_t id, long step) {
        if (id < count) motors[id]->moveToStep(step);
    }

    // Relative to the target, so repeated jogs add up mid-move.
    void moveSteps(uint8_t id, long deltaSteps) {
        if (id < count) motors[id]->moveSteps(deltaSteps);
    }

    void moveToSteps(const uint8_t* ids, const long* steps, uint8_t n) {
        for (uint8_t i = 0; i < n; i++) moveToStep(ids[i], steps[i]);
    }

    void moveSteps(const uint8_t* ids, const long* deltaSteps, uint8_t n) {
        for (uint8_t i = 0; i < n; i++) moveSteps(ids[i], deltaSteps[i]);
    }

    // Cancels a move in flight, holding wherever the axis got to.
    void stopHere(uint8_t id) {
        if (id < count) motors[id]->stopHere();
    }

    void stopAll() {
        for (uint8_t i = 0; i < count; i++) motors[i]->stopHere();
    }

    // Redefines the position: the axis is no longer homed, but its soft limits arm.
    void setOrigin(uint8_t id, long step = 0) {
        if (id >= count) return;
        motors[id]->setOrigin(step);
        motors[id]->armLimits(true);
        homing[id] = NOT_HOMED;
    }

    void setOrigins(long step = 0) {
        for (uint8_t i = 0; i < count; i++) setOrigin(i, step);
    }

    // Blocking two-pass homing (Homing.h). Energises the drivers first.
    HomingResult home(uint8_t id, const HomingConfig& cfg) {
        if (id >= count) return NOT_HOMED;

        enable();
        while (!ready()) {}

        homing[id] = homeAxis(*motors[id], cfg);
        lastActiveAt = millis();
        return homing[id];
    }

    HomingResult getHomingResult(uint8_t id) const {
        return (id < count) ? homing[id] : NOT_HOMED;
    }

    bool isHomed(uint8_t id) const {
        return getHomingResult(id) == HOMED;
    }

    long getStep(uint8_t id) const {
        return (id < count) ? motors[id]->getStep() : 0;
    }

    long getTargetStep(uint8_t id) const {
        return (id < count) ? motors[id]->getTargetStep() : 0;
    }

    // Borrowed speed profile, or nullptr for a flat interval. One can serve the fleet.
    void setProfile(uint8_t id, const StepProfile* p) {
        if (id < count) motors[id]->setProfile(p);
    }

    void setProfiles(const StepProfile* p) {
        for (uint8_t i = 0; i < count; i++) motors[i]->setProfile(p);
    }

    const StepProfile* getProfile(uint8_t id) const {
        return (id < count) ? motors[id]->getProfile() : nullptr;
    }

    // The interval right now; differs from the set interval while ramping.
    unsigned long getCurrentInterval(uint8_t id) const {
        return (id < count) ? motors[id]->currentInterval() : 0;
    }

    // Stops over the profile's stopping distance; stopHere()/stopAll() are immediate.
    void stopRamped(uint8_t id) {
        if (id < count) motors[id]->stopRamped();
    }

    void stopAllRamped() {
        for (uint8_t i = 0; i < count; i++) motors[i]->stopRamped();
    }

    // Soft limits in steps, armed by home() and setOrigin(). See docs/homing.md.
    void setLimits(uint8_t id, long minStep, long maxStep) {
        if (id < count) motors[id]->setLimits(minStep, maxStep);
    }

    void clearLimits(uint8_t id) {
        if (id < count) motors[id]->clearLimits();
    }

    bool hasLimits(uint8_t id) const {
        return (id < count) && motors[id]->hasLimits();
    }

    bool limitsArmed(uint8_t id) const {
        return (id < count) && motors[id]->limitsArmed();
    }

    // Pulse spacing, per axis.
    void setStepInterval(uint8_t id, unsigned long us) {
        if (id < count) motors[id]->setStepInterval(us);
    }

    void setStepIntervals(unsigned long us) {
        for (uint8_t i = 0; i < count; i++) motors[i]->setStepInterval(us);
    }

    unsigned long getStepInterval(uint8_t id) const {
        return (id < count) ? motors[id]->getStepInterval() : 0;
    }

    uint16_t getStepsPerRevolution(uint8_t id) const {
        return (id < count) ? motors[id]->getStepsPerRevolution() : 0;
    }

private:
    bool releasesWhenIdle() const {
        return idleReleaseMs != 0 && enablePin != NO_ENABLE_PIN;
    }

    // Closes the wake window once passed, so micros() rollover cannot reopen it.
    bool ready() {
        if (!waking) return true;
        if (micros() - enabledAt < wakeUs) return false;

        waking = false;
        allowPulses(true);
        return true;
    }

    // Self-stepping engines must be told; skipping update() only holds polled ones.
    void allowPulses(bool allowed) {
        for (uint8_t i = 0; i < count; i++) motors[i]->getEngine().allowPulses(allowed);
    }

    uint8_t       enablePin;
    unsigned long idleReleaseMs;
    unsigned long wakeUs;
    bool          powered;
    bool          waking;
    unsigned long enabledAt;
    unsigned long lastActiveAt;
    HomingResult* homing;        // per axis, indexed like motors
};
