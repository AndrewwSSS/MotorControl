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

    // Time the drivers are given to bring their outputs back up after the EN
    // line is energised, before the first pulse is sent.
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

    // Energises the drivers and sets up every axis, endstop pins included.
    // Homing is a separate, explicit step: home().
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

            // Pulses sent while the drivers are still coming up would be lost,
            // and a lost pulse is a step the counter claims but the axis never
            // took. Withhold them rather than blocking, so the caller's loop
            // stays responsive.
            if (!ready()) return;
        } else if (releasesWhenIdle() && powered &&
                   millis() - lastActiveAt >= idleReleaseMs) {
            disable();
        }

        updateMotors();
    }

    // True while any axis still has steps to make. Answered from position
    // rather than state, because state only turns MOVING once a pulse has gone
    // out - and pulses are the thing being gated here.
    bool hasWork() const {
        for (uint8_t i = 0; i < count; i++) {
            if (motors[i]->getStep() != motors[i]->getTargetStep()) return true;
        }
        return false;
    }

    // Idle release: drop the EN line once the whole fleet has been at rest for
    // idleMs, and take it back automatically on the next move. 0 (the default)
    // keeps the coils energised the whole time. Needs an enable pin; without
    // one there is nothing to release.
    void setIdleRelease(unsigned long idleMs, unsigned long wakeUs = DEFAULT_WAKE_US) {
        this->idleReleaseMs = idleMs;
        this->wakeUs = wakeUs;
    }

    unsigned long getIdleRelease() const {
        return idleReleaseMs;
    }

    // The EN input is active low. Energising is what gives the axes their
    // holding torque; releasing stops the coils heating, at the cost of the
    // position the counters claim.
    void enable() {
        if (enablePin != NO_ENABLE_PIN) {
            digitalWrite(enablePin, LOW);

            // Only the transition opens a wake-up window. Re-energising a live
            // line would otherwise stall the next pulse for no reason.
            if (!powered) {
                enabledAt = micros();
                waking = true;
                allowPulses(false);
            }
        }

        powered = true;

        // With a wake-up window open, ready() lets the pulses through once it
        // closes; without one they can go now.
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

    // Relative move, anchored on the target, so repeated jogs compose while a
    // move is still in flight.
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

    // Redefines where an axis believes it is. Step 0 is then no longer the
    // switch, so the axis stops counting as homed.
    void setOrigin(uint8_t id, long step = 0) {
        if (id >= count) return;
        motors[id]->setOrigin(step);
        homing[id] = NOT_HOMED;
    }

    void setOrigins(long step = 0) {
        for (uint8_t i = 0; i < count; i++) setOrigin(i, step);
    }

    // Finds the axis's base position against its endstop - see Homing.h.
    // Blocking. Energises the drivers first and waits out their wake-up, since
    // a search that cannot move the axis just fails.
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

    // Attach a speed profile (see LinearRamp.h), or nullptr for a flat
    // interval. One instance can serve the whole fleet: profiles hold no
    // per-move state.
    void setProfile(uint8_t id, const StepProfile* p) {
        if (id < count) motors[id]->setProfile(p);
    }

    void setProfiles(const StepProfile* p) {
        for (uint8_t i = 0; i < count; i++) motors[i]->setProfile(p);
    }

    const StepProfile* getProfile(uint8_t id) const {
        return (id < count) ? motors[id]->getProfile() : nullptr;
    }

    // The pace the axis is running at right now, which under a ramp is not the
    // interval it was asked for.
    unsigned long getCurrentInterval(uint8_t id) const {
        return (id < count) ? motors[id]->currentInterval() : 0;
    }

    // Brings an axis to rest over its profile's stopping distance rather than
    // dropping it dead. stopHere()/stopAll() remain immediate.
    void stopRamped(uint8_t id) {
        if (id < count) motors[id]->stopRamped();
    }

    void stopAllRamped() {
        for (uint8_t i = 0; i < count; i++) motors[i]->stopRamped();
    }

    // Pulse spacing, i.e. speed. Per axis, because a fleet rarely shares one
    // reduction ratio.
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

    // Closes the wake window as it passes, so a line that has been live for
    // hours answers without consulting the clock - and cannot be caught out by
    // the micros() rollover an open-ended comparison would trip over.
    bool ready() {
        if (!waking) return true;
        if (micros() - enabledAt < wakeUs) return false;

        waking = false;
        allowPulses(true);
        return true;
    }

    // Opens or shuts every axis's gate. Skipping update() while the drivers are
    // not ready only holds back an engine that pulses from update(); one that
    // steps on its own has to be told.
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
