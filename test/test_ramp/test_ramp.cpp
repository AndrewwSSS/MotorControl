// Characterisation of LinearRamp, on its own and as a Stepper actually paces
// pulses with it. Uses the ramp the BigHardDick rig runs: 1000 us down to a
// 200 us cruise over 1000 steps.

#include <unity.h>
#include <Arduino.h>

#include "LinearRamp.h"
#include "StepDirStepper.h"

static const uint8_t STEP_PIN = 2;
static const uint8_t DIR_PIN  = 5;
static const unsigned long CRUISE_US = 200;

static void runUntilStep(Stepper& axis, long step) {
    unsigned long until = micros() + 10000000UL;
    while (axis.getStep() != step && micros() < until) {
        axis.update();
        fake::advanceUs(10);
    }
}

static void runToEnd(Stepper& axis) {
    runUntilStep(axis, axis.getTargetStep());
    axis.update();
}

void setUp() {
    fake::reset();
}

void tearDown() {}

void test_a_move_starts_at_the_start_interval() {
    LinearRamp ramp(1000, 1000);
    TEST_ASSERT_EQUAL_UINT32(1000, ramp.intervalFor(0, 3000, CRUISE_US));
}

void test_the_interval_falls_linearly_while_accelerating() {
    LinearRamp ramp(1000, 1000);
    TEST_ASSERT_EQUAL_UINT32(600, ramp.intervalFor(500, 2500, CRUISE_US));
    TEST_ASSERT_EQUAL_UINT32(200, ramp.intervalFor(1000, 2000, CRUISE_US));
}

void test_the_middle_of_a_long_move_cruises() {
    LinearRamp ramp(1000, 1000);
    TEST_ASSERT_EQUAL_UINT32(CRUISE_US, ramp.intervalFor(1500, 1500, CRUISE_US));
}

void test_the_end_of_a_move_slows_back_down() {
    LinearRamp ramp(1000, 1000);
    TEST_ASSERT_EQUAL_UINT32(600, ramp.intervalFor(2500, 500, CRUISE_US));
    // Integer maths: one step in, 800 * 1 / 1000 rounds to no change.
    TEST_ASSERT_EQUAL_UINT32(1000, ramp.intervalFor(2999, 1, CRUISE_US));
    TEST_ASSERT_EQUAL_UINT32(1000, ramp.intervalFor(3000, 0, CRUISE_US));
}

void test_a_short_move_peaks_where_the_two_ends_meet() {
    LinearRamp ramp(1000, 1000);
    // 400 steps never reach cruise: the top speed is 200 steps into the ramp.
    TEST_ASSERT_EQUAL_UINT32(840, ramp.intervalFor(200, 200, CRUISE_US));
    TEST_ASSERT_EQUAL_UINT32(920, ramp.intervalFor(100, 300, CRUISE_US));
    TEST_ASSERT_EQUAL_UINT32(920, ramp.intervalFor(300, 100, CRUISE_US));
}

void test_no_ramp_when_the_start_is_not_slower_than_cruise() {
    LinearRamp ramp(150, 1000);
    TEST_ASSERT_EQUAL_UINT32(CRUISE_US, ramp.intervalFor(0, 3000, CRUISE_US));
}

void test_stop_distance_is_the_ramp_length() {
    LinearRamp ramp(1000, 1000);
    TEST_ASSERT_EQUAL(1000, ramp.stopDistance());

    ramp.setRampSteps(0);   // treated as 1, so there is never a divide by zero
    TEST_ASSERT_EQUAL(1, ramp.getRampSteps());
}

void test_a_stepper_paces_its_pulses_by_the_ramp() {
    LinearRamp ramp(1000, 10);
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, CRUISE_US);
    axis.setProfile(&ramp);
    axis.begin();

    const long n = 30;
    axis.moveSteps(n);
    runToEnd(axis);

    std::vector<unsigned long> rises = fake::risingEdgeTimes(STEP_PIN);
    TEST_ASSERT_EQUAL(n, rises.size());

    // The wait before pulse k is the ramp's interval k steps into the move.
    // Polled every 10 us, a pulse can come up to one poll late.
    for (long k = 1; k < n; k++) {
        unsigned long expected = ramp.intervalFor(k, n - k, CRUISE_US);
        unsigned long gap = rises[k] - rises[k - 1];
        TEST_ASSERT_TRUE(gap + MC_DIR_SETUP_US >= expected);
        TEST_ASSERT_TRUE(gap <= expected + 15);
    }
}

void test_stop_ramped_comes_to_rest_over_the_ramp() {
    LinearRamp ramp(1000, 100);
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, CRUISE_US);
    axis.setProfile(&ramp);
    axis.begin();

    axis.moveSteps(1000);
    runUntilStep(axis, 500);
    axis.stopRamped();
    TEST_ASSERT_EQUAL(600, axis.getTargetStep());
}

void test_stop_ramped_never_extends_a_move() {
    LinearRamp ramp(1000, 100);
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, CRUISE_US);
    axis.setProfile(&ramp);
    axis.begin();

    axis.moveSteps(1000);
    runUntilStep(axis, 950);
    axis.stopRamped();
    TEST_ASSERT_EQUAL(1000, axis.getTargetStep());
}

void test_stop_ramped_without_a_profile_stops_dead() {
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, CRUISE_US);
    axis.begin();

    axis.moveSteps(1000);
    runUntilStep(axis, 50);
    axis.stopRamped();
    TEST_ASSERT_EQUAL(50, axis.getTargetStep());
}

void test_a_move_added_in_flight_keeps_the_ramp_going() {
    LinearRamp ramp(1000, 100);
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, CRUISE_US);
    axis.setProfile(&ramp);
    axis.begin();

    axis.moveSteps(1000);
    runUntilStep(axis, 500);
    TEST_ASSERT_EQUAL_UINT32(CRUISE_US, axis.currentInterval());

    // Composed onto the running move: still cruising, not back to the start.
    axis.moveSteps(500);
    TEST_ASSERT_EQUAL_UINT32(CRUISE_US, axis.currentInterval());
}

void test_a_move_from_rest_starts_a_fresh_ramp() {
    LinearRamp ramp(1000, 100);
    StepDirStepper axis(STEP_PIN, DIR_PIN, 3200, StepDirStepper::NO_ENABLE_PIN, CRUISE_US);
    axis.setProfile(&ramp);
    axis.begin();

    axis.moveSteps(300);
    runToEnd(axis);

    axis.moveSteps(300);
    TEST_ASSERT_EQUAL_UINT32(1000, axis.currentInterval());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_a_move_starts_at_the_start_interval);
    RUN_TEST(test_the_interval_falls_linearly_while_accelerating);
    RUN_TEST(test_the_middle_of_a_long_move_cruises);
    RUN_TEST(test_the_end_of_a_move_slows_back_down);
    RUN_TEST(test_a_short_move_peaks_where_the_two_ends_meet);
    RUN_TEST(test_no_ramp_when_the_start_is_not_slower_than_cruise);
    RUN_TEST(test_stop_distance_is_the_ramp_length);
    RUN_TEST(test_a_stepper_paces_its_pulses_by_the_ramp);
    RUN_TEST(test_stop_ramped_comes_to_rest_over_the_ramp);
    RUN_TEST(test_stop_ramped_never_extends_a_move);
    RUN_TEST(test_stop_ramped_without_a_profile_stops_dead);
    RUN_TEST(test_a_move_added_in_flight_keeps_the_ramp_going);
    RUN_TEST(test_a_move_from_rest_starts_a_fresh_ramp);
    return UNITY_END();
}
