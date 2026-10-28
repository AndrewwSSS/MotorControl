// Characterisation of homeAxis() against a simulated switch. The switch follows
// the shaft, not the axis's step counter: homing redefines the counter, so only
// the pulses actually sent say where the shaft is.

#include <unity.h>
#include <Arduino.h>

#include "Endstop.h"
#include "Homing.h"
#include "StepDirStepper.h"

static const uint8_t STEP_PIN   = 2;
static const uint8_t DIR_PIN    = 5;
static const uint8_t SWITCH_PIN = 9;

// Where the shaft is, worked out from every STEP pulse and the DIR level it was
// sent with. The switch trips at shaft position 0 and below.
struct Shaft {
    long    position;
    size_t  seen;
    uint8_t dir;
    uint8_t step;

    void reset(long start) {
        position = start;
        seen = 0;
        dir = LOW;
        step = LOW;
    }

    long at() {
        const std::vector<fake::PinWrite>& w = fake::writes();
        for (; seen < w.size(); seen++) {
            if (w[seen].pin == DIR_PIN) dir = w[seen].level;
            if (w[seen].pin != STEP_PIN) continue;
            if (step == LOW && w[seen].level == HIGH) position += (dir == HIGH) ? 1 : -1;
            step = w[seen].level;
        }
        return position;
    }
};

static Shaft shaft;

enum SwitchBehaviour {
    WORKS,          // closed whenever the shaft is at or past the trip point
    DEAD,           // never closes
    STUCK_CLOSED,   // never opens
    CLOSES_ONCE     // closes on the seek, then never again
};

static SwitchBehaviour behaviour;
static bool pressedOnce;   // CLOSES_ONCE: the seek has found it
static bool gone;          // CLOSES_ONCE: the back-off has released it

// Normally open: LOW while pressed.
static int readSwitch(uint8_t pin) {
    if (pin != SWITCH_PIN) return HIGH;

    bool past = shaft.at() <= 0;
    switch (behaviour) {
        case WORKS:        return past ? LOW : HIGH;
        case DEAD:         return HIGH;
        case STUCK_CLOSED: return LOW;
        case CLOSES_ONCE:
            if (gone) return HIGH;
            if (past) {
                pressedOnce = true;
                return LOW;
            }
            if (pressedOnce) gone = true;
            return HIGH;
    }
    return HIGH;
}

static HomingConfig config(long maxSteps = 2000) {
    HomingConfig cfg;
    cfg.towards        = Endstop::MIN;
    cfg.restStep       = 100;
    cfg.maxSteps       = maxSteps;
    cfg.backoffSteps   = 50;
    cfg.slowIntervalUs = 2000;
    return cfg;
}

struct Rig {
    Endstop        minSwitch;
    StepDirStepper axis;

    explicit Rig(long shaftStart)
        : minSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MIN),
          axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, 800) {
        shaft.reset(shaftStart);
        axis.setEndstop(&minSwitch);
        axis.begin();
    }
};

static bool contains(const char* text) {
    return strstr(fake::serialOutput().c_str(), text) != nullptr;
}

void setUp() {
    fake::reset();
    // homeAxis() spins on update() until each move ends; time has to move on
    // its own for that to finish.
    fake::setAutoAdvance(10);
    fake::onRead(readSwitch);
    behaviour = WORKS;
    pressedOnce = false;
    gone = false;
}

void tearDown() {}

void test_homing_sets_zero_at_the_switch_and_parks_at_rest() {
    Rig rig(500);

    HomingResult result = homeAxis(rig.axis, config());

    TEST_ASSERT_EQUAL(HOMED, result);
    TEST_ASSERT_EQUAL(100, rig.axis.getStep());
    TEST_ASSERT_EQUAL(100, shaft.at());   // step 0 really is the trip point
    TEST_ASSERT_EQUAL(Stepper::HOLD, rig.axis.getState());
    TEST_ASSERT_TRUE(contains("Homed, moving to rest step 100"));
}

void test_homing_lands_on_the_same_place_from_anywhere() {
    Rig near(30);
    TEST_ASSERT_EQUAL(HOMED, homeAxis(near.axis, config()));
    long fromNear = shaft.at();

    fake::reset();
    fake::setAutoAdvance(10);
    fake::onRead(readSwitch);

    Rig far(1800);
    TEST_ASSERT_EQUAL(HOMED, homeAxis(far.axis, config()));

    TEST_ASSERT_EQUAL(fromNear, shaft.at());
}

void test_homing_from_on_the_switch_skips_the_seek() {
    Rig rig(-20);

    HomingResult result = homeAxis(rig.axis, config());

    TEST_ASSERT_EQUAL(HOMED, result);
    TEST_ASSERT_EQUAL(100, shaft.at());
    TEST_ASSERT_TRUE(contains("already on the switch on pin 9, skipping seek pass"));
}

void test_homing_leaves_the_speed_and_profile_as_they_were() {
    Rig rig(500);

    homeAxis(rig.axis, config());

    TEST_ASSERT_EQUAL_UINT32(800, rig.axis.getStepInterval());
    TEST_ASSERT_NULL(rig.axis.getProfile());
    TEST_ASSERT_FALSE(rig.axis.reachedTarget());   // homing swallows the edge
}

void test_a_dead_switch_fails_within_the_allowance() {
    behaviour = DEAD;
    Rig rig(500);

    HomingResult result = homeAxis(rig.axis, config(200));

    TEST_ASSERT_EQUAL(HOMING_NEVER_CLOSED, result);
    TEST_ASSERT_EQUAL(300, shaft.at());   // drove the 200 allowance, no further
    TEST_ASSERT_TRUE(contains("never closed within 200 steps"));
}

void test_a_stuck_switch_fails_while_backing_off() {
    behaviour = STUCK_CLOSED;
    Rig rig(500);

    HomingResult result = homeAxis(rig.axis, config(200));

    TEST_ASSERT_EQUAL(HOMING_NEVER_RELEASED, result);
    // Backed off in 50-step increments up to the allowance, then gave up.
    TEST_ASSERT_EQUAL(700, shaft.at());
    TEST_ASSERT_TRUE(contains("never released while backing off"));
}

void test_a_switch_missing_on_the_re_approach_fails() {
    behaviour = CLOSES_ONCE;
    Rig rig(500);

    HomingResult result = homeAxis(rig.axis, config());

    TEST_ASSERT_EQUAL(HOMING_NOT_FOUND, result);
    TEST_ASSERT_TRUE(contains("not found on slow re-approach"));
}

void test_homing_without_an_endstop_on_that_side_refuses() {
    Rig rig(500);
    HomingConfig cfg = config();
    cfg.towards = Endstop::MAX;

    TEST_ASSERT_EQUAL(HOMING_NO_ENDSTOP, homeAxis(rig.axis, cfg));
    TEST_ASSERT_EQUAL(500, shaft.at());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_homing_sets_zero_at_the_switch_and_parks_at_rest);
    RUN_TEST(test_homing_lands_on_the_same_place_from_anywhere);
    RUN_TEST(test_homing_from_on_the_switch_skips_the_seek);
    RUN_TEST(test_homing_leaves_the_speed_and_profile_as_they_were);
    RUN_TEST(test_a_dead_switch_fails_within_the_allowance);
    RUN_TEST(test_a_stuck_switch_fails_while_backing_off);
    RUN_TEST(test_a_switch_missing_on_the_re_approach_fails);
    RUN_TEST(test_homing_without_an_endstop_on_that_side_refuses);
    return UNITY_END();
}
