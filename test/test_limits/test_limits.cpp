// Soft limits: refused moves, arming, a move cut short at a new limit, and
// homing, which has to run past the limits to find its switch.

#include <unity.h>
#include <Arduino.h>

#include "FakeEngine.h"
#include "Endstop.h"
#include "Homing.h"
#include "LinearRamp.h"
#include "StepDirStepper.h"
#include "Stepper.h"
#include "StepperManager.h"

static const uint8_t SWITCH_PIN = 9;

static bool contains(const char* text) {
    return strstr(fake::serialOutput().c_str(), text) != nullptr;
}

static void runToEnd(Stepper& axis) {
    for (int i = 0; i < 100000 && axis.getStep() != axis.getTargetStep(); i++) {
        axis.update();
        fake::advanceUs(10);
    }
    axis.update();
}

void setUp() {
    fake::reset();
}

void tearDown() {}

// --- Refusal ------------------------------------------------------------------

void test_without_limits_any_move_goes() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.begin();
    axis.armLimits();

    axis.moveToStep(1000000);
    TEST_ASSERT_EQUAL(1000000, axis.getTargetStep());
    TEST_ASSERT_FALSE(axis.hasLimits());
}

void test_unarmed_limits_are_not_enforced() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.setLimits(0, 3200);
    axis.begin();

    axis.moveToStep(5000);
    TEST_ASSERT_EQUAL(5000, axis.getTargetStep());
    TEST_ASSERT_TRUE(axis.hasLimits());
    TEST_ASSERT_FALSE(axis.limitsArmed());
}

void test_armed_limits_allow_moves_inside_the_range() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.setLimits(0, 3200);
    axis.begin();
    axis.armLimits();

    axis.moveToStep(3200);
    TEST_ASSERT_EQUAL(3200, axis.getTargetStep());
    axis.moveToStep(0);
    TEST_ASSERT_EQUAL(0, axis.getTargetStep());
    TEST_ASSERT_FALSE(contains("refused"));
}

void test_a_move_past_a_limit_is_refused_and_logged() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.setLimits(0, 3200);
    axis.begin();
    axis.armLimits();

    axis.moveToStep(1000);
    axis.moveToStep(3201);
    TEST_ASSERT_EQUAL(1000, axis.getTargetStep());   // kept the target it had
    TEST_ASSERT_TRUE(contains("Move to step 3201 refused: limits are 0..3200"));

    fake::clearSerial();
    axis.moveToStep(-1);
    TEST_ASSERT_EQUAL(1000, axis.getTargetStep());
    TEST_ASSERT_TRUE(contains("Move to step -1 refused"));
}

void test_a_relative_move_past_a_limit_is_refused() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.setLimits(0, 3200);
    axis.begin();
    axis.armLimits();

    axis.moveSteps(3000);
    axis.moveSteps(500);   // 3000 + 500 lands past 3200
    TEST_ASSERT_EQUAL(3000, axis.getTargetStep());

    axis.moveSteps(200);   // exactly to the limit
    TEST_ASSERT_EQUAL(3200, axis.getTargetStep());
}

void test_limits_given_backwards_are_put_in_order() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.setLimits(500, -500);

    TEST_ASSERT_EQUAL(-500, axis.getMinStep());
    TEST_ASSERT_EQUAL(500, axis.getMaxStep());
}

void test_an_axis_outside_its_range_may_only_head_back() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.begin();
    axis.setOrigin(5000);
    axis.setLimits(0, 3200);
    axis.armLimits();

    axis.moveToStep(5100);   // further out
    TEST_ASSERT_EQUAL(5000, axis.getTargetStep());

    axis.moveSteps(-100);    // back towards the range, still outside it
    TEST_ASSERT_EQUAL(4900, axis.getTargetStep());

    axis.moveToStep(2000);   // into the range
    TEST_ASSERT_EQUAL(2000, axis.getTargetStep());
}

void test_stops_are_never_refused() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.begin();
    axis.setOrigin(5000);
    axis.setLimits(0, 3200);
    axis.armLimits();

    axis.moveToStep(4000);
    for (int i = 0; i < 10; i++) axis.update();
    axis.stopHere();
    TEST_ASSERT_EQUAL(4990, axis.getTargetStep());
    TEST_ASSERT_FALSE(contains("refused"));
}

void test_cleared_limits_stop_refusing() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.setLimits(0, 3200);
    axis.begin();
    axis.armLimits();

    axis.clearLimits();
    axis.moveToStep(9000);
    TEST_ASSERT_EQUAL(9000, axis.getTargetStep());
}

// --- A move already in flight ---------------------------------------------------

void test_a_move_in_flight_is_cut_short_at_a_new_limit() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.begin();
    axis.armLimits();

    axis.moveToStep(5000);
    for (int i = 0; i < 100; i++) axis.update();
    axis.setLimits(-1000, 1000);
    TEST_ASSERT_EQUAL(1000, axis.getTargetStep());

    runToEnd(axis);
    TEST_ASSERT_EQUAL(1000, axis.getStep());
}

void test_arming_cuts_short_a_move_already_past_the_limit() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.setLimits(0, 1000);
    axis.begin();

    axis.moveToStep(5000);   // unarmed: allowed
    for (int i = 0; i < 100; i++) axis.update();
    axis.armLimits();
    TEST_ASSERT_EQUAL(1000, axis.getTargetStep());
}

// On real pulses and a ramp: the axis slows down and stops on the limit, never
// a step past it.
void test_a_ramped_move_stops_on_the_limit_mid_ramp() {
    LinearRamp ramp(1000, 100);
    StepDirStepper axis(2, 5, 3200, StepDirStepper::NO_ENABLE_PIN, 200);
    axis.setProfile(&ramp);
    axis.begin();
    axis.armLimits();

    axis.moveToStep(3000);
    for (int i = 0; i < 1000000 && axis.getStep() < 1500; i++) {
        axis.update();
        fake::advanceUs(10);
    }
    TEST_ASSERT_EQUAL_UINT32(200, axis.currentInterval());   // cruising

    axis.setLimits(0, 1550);
    runToEnd(axis);

    TEST_ASSERT_EQUAL(1550, axis.getStep());
    TEST_ASSERT_EQUAL(1550, fake::risingEdges(2));
}

// --- Arming through the fleet and homing ----------------------------------------

void test_a_hand_set_origin_arms_the_limits() {
    FakeEngine engine;
    StepperManager fleet(1);
    fleet.addMotor(new Stepper(engine, 3200));
    fleet.setLimits(0, 0, 3200);
    fleet.begin();

    TEST_ASSERT_FALSE(fleet.limitsArmed(0));
    fleet.moveToStep(0, 5000);
    TEST_ASSERT_EQUAL(5000, fleet.getTargetStep(0));

    fleet.stopAll();
    fleet.setOrigin(0);
    TEST_ASSERT_TRUE(fleet.limitsArmed(0));
    fleet.moveToStep(0, 5000);
    TEST_ASSERT_EQUAL(0, fleet.getTargetStep(0));
}

// The switch sits at shaft 0 and below; the axis starts at shaft 500.
static FakeEngine* homingEngine;

static int switchAtZero(uint8_t pin) {
    if (pin != SWITCH_PIN) return HIGH;
    return (homingEngine->shaft() <= 0) ? LOW : HIGH;   // normally open
}

static HomingConfig homingConfig() {
    HomingConfig cfg;
    cfg.towards        = Endstop::MIN;
    cfg.restStep       = 100;
    cfg.maxSteps       = 2000;
    cfg.backoffSteps   = 50;
    cfg.slowIntervalUs = 2000;
    return cfg;
}

void test_homing_runs_past_the_limits_and_arms_them() {
    FakeEngine engine;
    Endstop minSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MIN);
    StepperManager fleet(1);
    fleet.addMotor(new Stepper(engine, 3200));
    fleet[0].setEndstop(&minSwitch);
    homingEngine = &engine;
    engine.placeShaft(500);
    fake::onRead(switchAtZero);

    // Before homing, step 0 is wherever the axis woke up: shaft 500. A range
    // of 0..400 from there would forbid the seek reaching the switch.
    fleet.setLimits(0, 0, 400);
    fleet.begin();
    fleet.setOrigin(0);
    TEST_ASSERT_TRUE(fleet.limitsArmed(0));

    TEST_ASSERT_EQUAL(HOMED, fleet.home(0, homingConfig()));
    TEST_ASSERT_EQUAL(100, fleet.getStep(0));
    TEST_ASSERT_TRUE(fleet.limitsArmed(0));
    TEST_ASSERT_FALSE(contains("refused"));

    fleet.moveToStep(0, 401);
    TEST_ASSERT_EQUAL(100, fleet.getTargetStep(0));
}

void test_failed_homing_leaves_the_arming_as_it_was() {
    FakeEngine engine;
    Endstop minSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MIN);
    Stepper axis(engine, 3200);
    axis.setEndstop(&minSwitch);
    axis.setLimits(-100, 100);
    axis.begin();   // the pin reads HIGH: the switch never closes

    HomingConfig cfg = homingConfig();
    cfg.maxSteps = 200;

    TEST_ASSERT_EQUAL(HOMING_NEVER_CLOSED, homeAxis(axis, cfg));
    TEST_ASSERT_FALSE(axis.limitsArmed());

    axis.setOrigin(0);
    axis.armLimits();
    TEST_ASSERT_EQUAL(HOMING_NEVER_CLOSED, homeAxis(axis, cfg));
    TEST_ASSERT_TRUE(axis.limitsArmed());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_without_limits_any_move_goes);
    RUN_TEST(test_unarmed_limits_are_not_enforced);
    RUN_TEST(test_armed_limits_allow_moves_inside_the_range);
    RUN_TEST(test_a_move_past_a_limit_is_refused_and_logged);
    RUN_TEST(test_a_relative_move_past_a_limit_is_refused);
    RUN_TEST(test_limits_given_backwards_are_put_in_order);
    RUN_TEST(test_an_axis_outside_its_range_may_only_head_back);
    RUN_TEST(test_stops_are_never_refused);
    RUN_TEST(test_cleared_limits_stop_refusing);
    RUN_TEST(test_a_move_in_flight_is_cut_short_at_a_new_limit);
    RUN_TEST(test_arming_cuts_short_a_move_already_past_the_limit);
    RUN_TEST(test_a_ramped_move_stops_on_the_limit_mid_ramp);
    RUN_TEST(test_a_hand_set_origin_arms_the_limits);
    RUN_TEST(test_homing_runs_past_the_limits_and_arms_them);
    RUN_TEST(test_failed_homing_leaves_the_arming_as_it_was);
    return UNITY_END();
}
