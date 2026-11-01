// EngineStepper: an axis that owns its engine behaves exactly like a Stepper
// handed that engine - moving, the endstop guard, homing, soft limits and the
// fleet - for an engine that steps when update() calls it and for one that
// steps on its own.

#include <unity.h>
#include <Arduino.h>

#include "FakeEngine.h"
#include "EngineStepper.h"
#include "Endstop.h"
#include "Homing.h"
#include "StepDirStepper.h"
#include "StepperManager.h"

static const uint8_t SWITCH_PIN = 9;
static const uint8_t ENABLE_PIN = 8;

typedef EngineStepper<FakeEngine> FakeAxis;

static void updateUntilIdle(Stepper& axis, FakeEngine& engine) {
    for (int i = 0; i < 1000 && engine.isRunning(); i++) {
        axis.update();
        fake::advanceMs(1);
    }
}

// The switch follows the shaft of the engine under test, and trips at shaft
// position `tripAt` and beyond, in the direction `towards`.
static FakeEngine* switchEngine;
static long tripAt;
static int8_t towards;

// Normally open: LOW while pressed.
static int readSwitch(uint8_t pin) {
    if (pin != SWITCH_PIN || !switchEngine) return HIGH;
    long at = switchEngine->shaft();
    bool pressed = (towards > 0) ? (at >= tripAt) : (at <= tripAt);
    return pressed ? LOW : HIGH;
}

static void watchSwitch(FakeEngine& engine, long trip, int8_t side) {
    switchEngine = &engine;
    tripAt = trip;
    towards = side;
    fake::onRead(readSwitch);
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

void setUp() {
    fake::reset();
    switchEngine = nullptr;
}

void tearDown() {}

// --- Construction -------------------------------------------------------------

void test_the_engine_gets_the_arguments_after_steps_per_rev() {
    FakeAxis axis(3200, FakeEngine::OWN, 5L);

    TEST_ASSERT_EQUAL(FakeEngine::OWN, axis.getEngine().mode);
    TEST_ASSERT_EQUAL(5, axis.getEngine().stepsPerService);
}

void test_the_stepper_drives_the_engine_it_owns() {
    FakeAxis axis(3200);
    Stepper& asStepper = axis;

    TEST_ASSERT_EQUAL_PTR(&axis.getEngine(), &asStepper.getEngine());

    axis.begin();
    TEST_ASSERT_EQUAL(1, axis.getEngine().begins);
}

// --- Moving -------------------------------------------------------------------

void test_moves_like_a_stepper_on_either_engine() {
    FakeEngine borrowed(FakeEngine::OWN, 3);
    Stepper reference(borrowed, 3200);
    FakeAxis polled(3200);
    FakeAxis own(3200, FakeEngine::OWN, 3L);
    reference.begin();
    polled.begin();
    own.begin();

    reference.moveSteps(40);
    polled.moveSteps(40);
    own.moveSteps(40);
    updateUntilIdle(reference, borrowed);
    updateUntilIdle(polled, polled.getEngine());
    updateUntilIdle(own, own.getEngine());

    TEST_ASSERT_EQUAL(40, polled.getEngine().shaft());
    TEST_ASSERT_EQUAL(40, own.getEngine().shaft());
    TEST_ASSERT_EQUAL(borrowed.shaft(), own.getEngine().shaft());
    TEST_ASSERT_EQUAL(Stepper::HOLD, own.getState());
    TEST_ASSERT_TRUE(own.reachedTarget());
}

void test_the_guard_stops_travel_onto_a_pressed_switch() {
    FakeAxis axis(3200);
    Endstop maxSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MAX);
    axis.setEndstop(&maxSwitch);
    watchSwitch(axis.getEngine(), 20, +1);
    axis.begin();

    axis.moveSteps(100);
    for (int i = 0; i < 200 && axis.getEngine().isRunning(); i++) axis.update();

    TEST_ASSERT_EQUAL(20, axis.getStep());
    TEST_ASSERT_EQUAL(1, axis.getEngine().inhibited);
    TEST_ASSERT_EQUAL(1, axis.getEngine().stopNows);
}

// --- Homing -------------------------------------------------------------------

void test_homes_to_the_same_place_as_a_stepper() {
    FakeEngine borrowed(FakeEngine::POLLED);
    Stepper reference(borrowed, 3200);
    Endstop refSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MIN);
    reference.setEndstop(&refSwitch);
    borrowed.placeShaft(500);
    watchSwitch(borrowed, 0, -1);
    reference.begin();
    TEST_ASSERT_EQUAL(HOMED, homeAxis(reference, homingConfig()));

    fake::reset();
    FakeAxis axis(3200);
    Endstop minSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MIN);
    axis.setEndstop(&minSwitch);
    axis.getEngine().placeShaft(500);
    watchSwitch(axis.getEngine(), 0, -1);
    axis.begin();

    TEST_ASSERT_EQUAL(HOMED, homeAxis(axis, homingConfig()));
    TEST_ASSERT_EQUAL(100, axis.getStep());
    TEST_ASSERT_EQUAL(borrowed.shaft(), axis.getEngine().shaft());
}

// --- Soft limits --------------------------------------------------------------

void test_armed_limits_refuse_a_move_out_of_range() {
    FakeAxis axis(3200);
    axis.begin();
    axis.setLimits(-50, 50);
    axis.armLimits();

    axis.moveToStep(80);
    TEST_ASSERT_EQUAL(0, axis.getTargetStep());

    axis.moveToStep(40);
    updateUntilIdle(axis, axis.getEngine());
    TEST_ASSERT_EQUAL(40, axis.getEngine().shaft());
}

// --- Fleet --------------------------------------------------------------------

// The fleet holds Stepper pointers and deletes through them. An EngineStepper
// has two bases, so this checks the Stepper one is the axis the fleet drives
// and that deleting through it is clean.
void test_the_fleet_drives_and_deletes_engine_steppers() {
    StepperManager* fleet = new StepperManager(2, ENABLE_PIN);
    FakeAxis* a = new FakeAxis(3200);
    FakeAxis* b = new FakeAxis(3200, FakeEngine::OWN, 4L);
    fleet->addMotor(a);
    fleet->addMotor(b);
    fleet->begin();
    fake::advanceUs(1000);   // the drivers wake

    fleet->moveSteps(0, 6);
    fleet->moveSteps(1, 8);
    for (int i = 0; i < 20; i++) fleet->update();

    TEST_ASSERT_EQUAL(6, a->getEngine().shaft());
    TEST_ASSERT_EQUAL(8, b->getEngine().shaft());
    TEST_ASSERT_EQUAL_PTR(static_cast<Stepper*>(b), &(*fleet)[1]);

    delete fleet;
}

// --- StepDirStepper -----------------------------------------------------------

void test_step_dir_stepper_is_an_engine_stepper_on_a_polled_engine() {
    StepDirStepper axis(2, 5, 3200, StepDirStepper::NO_ENABLE_PIN, 800,
                        StepDirStepper::TB6600_PULSE_US);
    EngineStepper<PolledEngine>& owner = axis;

    TEST_ASSERT_EQUAL(2, owner.getEngine().getStepPin());
    TEST_ASSERT_EQUAL(StepDirStepper::TB6600_PULSE_US, axis.getStepPulse());
    TEST_ASSERT_EQUAL_UINT32(800, axis.getStepInterval());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_the_engine_gets_the_arguments_after_steps_per_rev);
    RUN_TEST(test_the_stepper_drives_the_engine_it_owns);
    RUN_TEST(test_moves_like_a_stepper_on_either_engine);
    RUN_TEST(test_the_guard_stops_travel_onto_a_pressed_switch);
    RUN_TEST(test_homes_to_the_same_place_as_a_stepper);
    RUN_TEST(test_armed_limits_refuse_a_move_out_of_range);
    RUN_TEST(test_the_fleet_drives_and_deletes_engine_steppers);
    RUN_TEST(test_step_dir_stepper_is_an_engine_stepper_on_a_polled_engine);
    return UNITY_END();
}
