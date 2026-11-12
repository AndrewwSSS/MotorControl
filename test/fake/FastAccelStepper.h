#pragma once

// Fake FastAccelStepper 1.4.0 with the behaviour FasEngine depends on; a test
// calls run(n) to let steps go out. See docs/testing.md.

#include <stdint.h>

enum class MoveResultCode : int8_t {
    OK = 0,
    ErrorNoDirectionPin = -1
};
#define MOVE_OK MoveResultCode::OK

class FastAccelStepper {
public:
    void setDirectionPin(uint8_t pin, bool highCountsUp = true, uint16_t delayUs = 0) {
        dirPin = pin;
        dirHighCountsUp = highCountsUp;
        dirDelayUs = delayUs;
    }

    // The real library refuses intervals faster than the hardware can step.
    int8_t setSpeedInUs(uint32_t us) {
        if (us < MIN_SPEED_US) return -1;
        speedUs = us;
        return 0;
    }

    int8_t setAcceleration(int32_t a) {
        if (a <= 0) return -1;
        acceleration = a;
        return 0;
    }

    void applySpeedAcceleration() { applies++; }

    MoveResultCode moveTo(int32_t p, bool = false) {
        moves++;
        if (isRunning() && direction(p) != direction(tgt)) {
            ignoredMoves++;
            return MOVE_OK;
        }
        tgt = p;
        return MOVE_OK;
    }

    void stopMove() {
        stopMoves++;
        if (!isRunning()) return;

        int32_t left = tgt - pos;
        int32_t span = (int32_t)stopSteps;
        if (left > 0 && span < left)  tgt = pos + span;
        if (left < 0 && span < -left) tgt = pos - span;
    }

    uint32_t stepsToStop() const { return stopSteps; }

    void forceStopAndNewPosition(int32_t p) {
        forceStops++;
        pos = tgt = p;
    }

    void setCurrentPosition(int32_t p) {
        positionSets++;
        pos = tgt = p;
    }

    int32_t getCurrentPosition() const { return pos; }
    int32_t targetPos() const { return tgt; }
    bool isRunning() const { return pos != tgt; }

    int32_t getCurrentSpeedInMilliHz(bool = true) const {
        return isRunning() ? direction(tgt) * (int32_t)(1000000000UL / speedUs) : 0;
    }

    int32_t getCurrentSpeedInUs(bool = true) const {
        return isRunning() ? direction(tgt) * (int32_t)speedUs : 0;
    }

    // Lets up to n steps go out towards the target.
    void run(int32_t n) {
        for (int32_t i = 0; i < n && pos != tgt; i++) pos += direction(tgt);
    }

    void runToEnd() { pos = tgt; }

    static constexpr uint32_t MIN_SPEED_US = 5;

    uint8_t  stepPin = 0;
    uint8_t  dirPin = 0;
    bool     dirHighCountsUp = false;
    uint16_t dirDelayUs = 0;
    uint32_t speedUs = 0;
    int32_t  acceleration = 0;
    uint32_t stopSteps = 10;

    // What the stepper was told.
    int moves = 0;
    int ignoredMoves = 0;
    int stopMoves = 0;
    int forceStops = 0;
    int positionSets = 0;
    int applies = 0;

private:
    int8_t direction(int32_t to) const {
        return (to > pos) ? 1 : (to < pos) ? -1 : 0;
    }

    int32_t pos = 0;
    int32_t tgt = 0;
};

class FastAccelStepperEngine {
public:
    static constexpr int CAPACITY = 4;

    void init() { inits++; }

    FastAccelStepper* stepperConnectToPin(uint8_t pin) {
        if (used >= CAPACITY) return nullptr;
        pool[used] = FastAccelStepper();
        pool[used].stepPin = pin;
        return &pool[used++];
    }

    // Frees every stepper; the engine itself is a static in fasEngine().
    static void reset() { used = 0; }

    static inline FastAccelStepper pool[CAPACITY];
    static inline int used = 0;
    static inline int inits = 0;
};
