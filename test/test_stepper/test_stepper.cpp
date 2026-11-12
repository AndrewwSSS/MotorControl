// StepDirStepper at the pins: what reaches STEP and DIR for a move, and how the
// endstop guard treats a pressed switch.

#include <unity.h>
#include <Arduino.h>

#include "Endstop.h"
#include "StepDirStepper.h"

static const uint8_t STEP_PIN   = 2;
static const uint8_t DIR_PIN    = 5;
static const uint8_t SWITCH_PIN = 9;
static const unsigned long INTERVAL_US = 800;

// update() every 10 us until the axis reaches step, or a time limit runs out
// so a test that never gets there fails instead of hanging.
static void runUntilStep(Stepper& axis, long step) {
    unsigned long until = micros() + 10000000UL;
    while (axis.getStep() != step && micros() < until) {
        axis.update();
        fake::advanceUs(10);
    }
}

// Runs the move in flight to its end, then one more update() so the axis
// settles into HOLD.
static void runToEnd(Stepper& axis) {
    runUntilStep(axis, axis.getTargetStep());
    axis.update();
}

// update() for a while without any expectation of where the axis gets to.
static void runFor(Stepper& axis, unsigned long us) {
    unsigned long until = micros() + us;
    while (micros() < until) {
        axis.update();
        fake::advanceUs(10);
    }
}

static size_t writesTo(uint8_t pin) {
    size_t n = 0;
    for (const fake::PinWrite& w : fake::writes()) {
        if (w.pin == pin) n++;
    }
    return n;
}

void setUp() {
    fake::reset();
}

void tearDown() {}

void test_a_move_of_n_steps_emits_n_pulses() {
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, INTERVAL_US);
    axis.begin();

    axis.moveSteps(10);
    runToEnd(axis);

    TEST_ASSERT_EQUAL(10, fake::risingEdges(STEP_PIN));
    TEST_ASSERT_EQUAL(10, axis.getStep());
    TEST_ASSERT_EQUAL(Stepper::HOLD, axis.getState());

    // Nothing more once the target is reached.
    runFor(axis, 10000);
    TEST_ASSERT_EQUAL(10, fake::risingEdges(STEP_PIN));
}

void test_dir_is_set_before_the_first_pulse() {
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, INTERVAL_US);
    axis.begin();
    fake::clearWrites();

    axis.moveSteps(3);
    runToEnd(axis);

    const fake::PinWrite* dir = nullptr;
    for (const fake::PinWrite& w : fake::writes()) {
        if (w.pin == DIR_PIN) { dir = &w; break; }
    }
    TEST_ASSERT_NOT_NULL(dir);
    TEST_ASSERT_EQUAL_UINT8(HIGH, dir->level);   // forward is DIR high

    std::vector<unsigned long> steps = fake::risingEdgeTimes(STEP_PIN);
    TEST_ASSERT_EQUAL(3, steps.size());
    TEST_ASSERT_TRUE(dir->atUs + MC_DIR_SETUP_US <= steps[0]);

    // Written once per change of direction, not once per step.
    TEST_ASSERT_EQUAL(1, writesTo(DIR_PIN));
}

void test_a_reversal_rewrites_dir_once() {
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, INTERVAL_US);
    axis.begin();

    axis.moveSteps(3);
    runToEnd(axis);
    fake::clearWrites();

    axis.moveSteps(-2);
    runToEnd(axis);

    TEST_ASSERT_EQUAL(1, writesTo(DIR_PIN));
    TEST_ASSERT_EQUAL_UINT8(LOW, fake::levelOf(DIR_PIN));
    TEST_ASSERT_EQUAL(2, fake::risingEdges(STEP_PIN));
    TEST_ASSERT_EQUAL(1, axis.getStep());
}

void test_pulses_are_spaced_by_the_interval_and_last_the_pulse_width() {
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, INTERVAL_US);
    axis.begin();
    fake::clearWrites();

    axis.moveSteps(5);
    runToEnd(axis);

    std::vector<unsigned long> rises = fake::risingEdgeTimes(STEP_PIN);
    TEST_ASSERT_EQUAL(5, rises.size());
    for (size_t i = 1; i < rises.size(); i++) {
        // The DIR setup delay sits in front of the first pulse only, so the
        // first gap can come up short by that much.
        TEST_ASSERT_TRUE(rises[i] - rises[i - 1] + MC_DIR_SETUP_US >= INTERVAL_US);
        TEST_ASSERT_TRUE(rises[i] - rises[i - 1] <= INTERVAL_US + 20);
    }

    // Each HIGH is followed by its LOW after exactly the pulse width.
    const std::vector<fake::PinWrite>& w = fake::writes();
    for (size_t i = 0; i + 1 < w.size(); i++) {
        if (w[i].pin != STEP_PIN || w[i].level != HIGH) continue;
        TEST_ASSERT_EQUAL_UINT8(STEP_PIN, w[i + 1].pin);
        TEST_ASSERT_EQUAL_UINT8(LOW, w[i + 1].level);
        TEST_ASSERT_EQUAL_UINT32(StepDirStepper::A4988_PULSE_US, w[i + 1].atUs - w[i].atUs);
    }
}

void test_reached_target_fires_once_per_move() {
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, INTERVAL_US);
    axis.begin();

    axis.moveSteps(4);
    runUntilStep(axis, 2);
    TEST_ASSERT_EQUAL(Stepper::MOVING, axis.getState());
    TEST_ASSERT_FALSE(axis.reachedTarget());

    runToEnd(axis);
    TEST_ASSERT_TRUE(axis.reachedTarget());
    TEST_ASSERT_FALSE(axis.reachedTarget());
}

void test_relative_moves_compose_on_the_target() {
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, INTERVAL_US);
    axis.begin();

    axis.moveSteps(10);
    runUntilStep(axis, 4);
    axis.moveSteps(10);   // added to the target 10, not to the step 4
    TEST_ASSERT_EQUAL(20, axis.getTargetStep());

    runToEnd(axis);
    TEST_ASSERT_EQUAL(20, axis.getStep());
    TEST_ASSERT_EQUAL(20, fake::risingEdges(STEP_PIN));
}

void test_stop_here_ends_the_move_on_the_spot() {
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, INTERVAL_US);
    axis.begin();

    axis.moveSteps(100);
    runUntilStep(axis, 7);
    axis.stopHere();
    runFor(axis, 10000);

    TEST_ASSERT_EQUAL(7, axis.getStep());
    TEST_ASSERT_EQUAL(7, axis.getTargetStep());
    TEST_ASSERT_EQUAL(7, fake::risingEdges(STEP_PIN));
}

void test_endstop_stops_travel_onto_a_pressed_switch() {
    // Normally open: the pin reads LOW once the switch is hit.
    Endstop maxSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MAX);
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, INTERVAL_US);
    axis.setEndstop(&maxSwitch);
    axis.begin();

    axis.moveSteps(100);
    runUntilStep(axis, 20);
    fake::setInput(SWITCH_PIN, LOW);
    runFor(axis, 50000);

    TEST_ASSERT_EQUAL(20, axis.getStep());
    TEST_ASSERT_EQUAL(20, axis.getTargetStep());
    TEST_ASSERT_EQUAL(Stepper::HOLD, axis.getState());
    TEST_ASSERT_NOT_NULL(strstr(fake::serialOutput().c_str(),
                                "Endstop MAX on pin 9 stopped the axis at step 20"));
}

void test_endstop_allows_travel_away_from_a_pressed_switch() {
    Endstop maxSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MAX);
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, INTERVAL_US);
    axis.setEndstop(&maxSwitch);
    fake::setInput(SWITCH_PIN, LOW);   // already pressed at power-on
    axis.begin();

    axis.moveSteps(10);
    runFor(axis, 50000);
    TEST_ASSERT_EQUAL(0, fake::risingEdges(STEP_PIN));
    TEST_ASSERT_EQUAL(0, axis.getStep());

    axis.moveSteps(-5);
    runToEnd(axis);
    TEST_ASSERT_EQUAL(-5, axis.getStep());
    TEST_ASSERT_EQUAL(5, fake::risingEdges(STEP_PIN));
}

void test_a_min_switch_only_guards_travel_downwards() {
    Endstop minSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MIN);
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, INTERVAL_US);
    axis.setEndstop(&minSwitch);
    fake::setInput(SWITCH_PIN, LOW);
    axis.begin();

    axis.moveSteps(-10);
    runFor(axis, 50000);
    TEST_ASSERT_EQUAL(0, axis.getStep());

    axis.moveSteps(5);
    runToEnd(axis);
    TEST_ASSERT_EQUAL(5, axis.getStep());
}

// A resync must not open a debounce window: homing starts with one, and a window
// would let an axis near the switch overshoot by up to 5 ms of travel.
void test_a_press_straight_after_a_resync_is_seen() {
    Endstop maxSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MAX);
    maxSwitch.begin();
    fake::advanceMs(100);

    maxSwitch.resync();
    fake::advanceMs(1);
    fake::setInput(SWITCH_PIN, LOW);
    maxSwitch.update();

    TEST_ASSERT_TRUE(maxSwitch.isTriggered());
}

void test_the_guard_sees_a_press_in_the_first_steps_after_begin() {
    Endstop maxSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MAX);
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, 200);
    axis.setEndstop(&maxSwitch);
    axis.begin();

    axis.moveSteps(100);
    runUntilStep(axis, 2);   // 0.4 ms in, well inside the old 5 ms blind spot
    fake::setInput(SWITCH_PIN, LOW);
    runFor(axis, 10000);

    TEST_ASSERT_EQUAL(2, axis.getStep());
}

// The debounce itself still holds: once a change has been seen, the contacts
// get their 5 ms to settle.
void test_bounce_after_a_real_edge_is_still_ignored() {
    Endstop maxSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MAX);
    maxSwitch.begin();
    fake::advanceMs(100);

    fake::setInput(SWITCH_PIN, LOW);
    maxSwitch.update();
    TEST_ASSERT_TRUE(maxSwitch.isTriggered());

    fake::advanceMs(2);
    fake::setInput(SWITCH_PIN, HIGH);   // a bounce
    maxSwitch.update();
    TEST_ASSERT_TRUE(maxSwitch.isTriggered());

    fake::advanceMs(4);                 // 6 ms after the edge: settled
    maxSwitch.update();
    TEST_ASSERT_FALSE(maxSwitch.isTriggered());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_a_move_of_n_steps_emits_n_pulses);
    RUN_TEST(test_dir_is_set_before_the_first_pulse);
    RUN_TEST(test_a_reversal_rewrites_dir_once);
    RUN_TEST(test_pulses_are_spaced_by_the_interval_and_last_the_pulse_width);
    RUN_TEST(test_reached_target_fires_once_per_move);
    RUN_TEST(test_relative_moves_compose_on_the_target);
    RUN_TEST(test_stop_here_ends_the_move_on_the_spot);
    RUN_TEST(test_endstop_stops_travel_onto_a_pressed_switch);
    RUN_TEST(test_endstop_allows_travel_away_from_a_pressed_switch);
    RUN_TEST(test_a_min_switch_only_guards_travel_downwards);
    RUN_TEST(test_a_press_straight_after_a_resync_is_seen);
    RUN_TEST(test_the_guard_sees_a_press_in_the_first_steps_after_begin);
    RUN_TEST(test_bounce_after_a_real_edge_is_still_ignored);
    return UNITY_END();
}
