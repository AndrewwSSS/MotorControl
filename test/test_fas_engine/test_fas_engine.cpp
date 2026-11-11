// FasEngine against a fake FastAccelStepper: the bridge between StepEngine and
// the library - pace and ramp, reversals, stops, the endstop inhibit and the
// fleet's gate. The library's own pulse making is not under test.

#include <unity.h>
#include <Arduino.h>

#include "FasEngine.h"
#include "EngineStepper.h"
#include "Endstop.h"
#include "LinearRamp.h"

static const uint8_t STEP_PIN   = 25;
static const uint8_t DIR_PIN    = 26;
static const uint8_t SWITCH_PIN = 34;

static bool contains(const char* text) {
    return strstr(fake::serialOutput().c_str(), text) != nullptr;
}

void setUp() {
    fake::reset();
    FastAccelStepperEngine::reset();
}

void tearDown() {}

// A begun engine at step 0, and its library stepper.
struct Rig {
    FasEngine engine;
    FastAccelStepper* fas;

    explicit Rig(unsigned long intervalUs = 200) : engine(STEP_PIN, DIR_PIN, intervalUs) {
        engine.begin();
        fas = engine.stepper();
    }
};

// --- Set-up and pace ----------------------------------------------------------

void test_begin_connects_the_pins() {
    Rig rig;

    TEST_ASSERT_NOT_NULL(rig.fas);
    TEST_ASSERT_EQUAL(STEP_PIN, rig.fas->stepPin);
    TEST_ASSERT_EQUAL(DIR_PIN, rig.fas->dirPin);
    TEST_ASSERT_TRUE(rig.fas->dirHighCountsUp);   // forward is DIR high, as on PolledEngine
    TEST_ASSERT_EQUAL(MC_DIR_SETUP_US, rig.fas->dirDelayUs);
    TEST_ASSERT_TRUE(FastAccelStepperEngine::inits >= 1);
}

void test_without_a_profile_the_axis_does_not_ramp() {
    Rig rig;

    TEST_ASSERT_EQUAL_UINT32(200, rig.fas->speedUs);
    TEST_ASSERT_EQUAL(FasEngine::NO_RAMP_ACCELERATION, rig.fas->acceleration);
}

void test_a_profile_becomes_the_acceleration() {
    Rig rig;
    LinearRamp ramp(1000, 1000);

    rig.engine.setProfile(&ramp);
    TEST_ASSERT_EQUAL(12000, rig.fas->acceleration);

    // A new top speed changes the acceleration that ramp stands for.
    rig.engine.setStepInterval(1000);
    TEST_ASSERT_EQUAL_UINT32(1000, rig.fas->speedUs);
    TEST_ASSERT_EQUAL(FasEngine::NO_RAMP_ACCELERATION, rig.fas->acceleration);

    rig.engine.setProfile(nullptr);
    rig.engine.setStepInterval(200);
    TEST_ASSERT_EQUAL(FasEngine::NO_RAMP_ACCELERATION, rig.fas->acceleration);
}

void test_a_change_of_pace_mid_move_is_applied_at_once() {
    Rig rig;
    rig.engine.moveTo(1000);
    rig.fas->run(10);

    rig.engine.setStepInterval(400);
    TEST_ASSERT_EQUAL(1, rig.fas->applies);
}

void test_an_interval_too_fast_for_the_library_is_reported() {
    Rig rig;
    rig.engine.setStepInterval(1);
    TEST_ASSERT_TRUE(contains("step interval 1 us is out of range"));
}

void test_with_no_stepper_left_the_axis_stays_put() {
    FasEngine engines[FastAccelStepperEngine::CAPACITY + 1] = {
        FasEngine(1, 2), FasEngine(3, 4), FasEngine(5, 6), FasEngine(7, 8), FasEngine(9, 10)
    };
    for (FasEngine& e : engines) e.begin();
    FasEngine& last = engines[FastAccelStepperEngine::CAPACITY];

    TEST_ASSERT_NULL(last.stepper());
    TEST_ASSERT_TRUE(contains("no free FastAccelStepper for step pin 9"));

    last.moveTo(100);
    TEST_ASSERT_FALSE(last.isRunning());
    TEST_ASSERT_EQUAL(0, last.target());
}

// --- Moves --------------------------------------------------------------------

void test_a_move_goes_to_the_library() {
    Rig rig;
    rig.engine.moveTo(100);

    TEST_ASSERT_EQUAL(100, rig.fas->targetPos());
    TEST_ASSERT_TRUE(rig.engine.isRunning());

    rig.fas->run(40);
    TEST_ASSERT_EQUAL(40, rig.engine.position());
    TEST_ASSERT_EQUAL(100, rig.engine.target());
}

void test_a_move_the_same_way_extends_the_one_running() {
    Rig rig;
    rig.engine.moveTo(100);
    rig.fas->run(20);

    rig.engine.moveTo(200);
    TEST_ASSERT_EQUAL(200, rig.fas->targetPos());
    TEST_ASSERT_EQUAL(0, rig.fas->stopMoves);
}

void test_a_reversal_ramps_down_first_then_goes_back() {
    Rig rig;
    rig.engine.moveTo(100);
    rig.fas->run(30);

    rig.engine.moveTo(0);
    TEST_ASSERT_EQUAL(1, rig.fas->stopMoves);
    TEST_ASSERT_EQUAL(0, rig.fas->ignoredMoves);   // never sent while it would be dropped
    TEST_ASSERT_EQUAL(0, rig.engine.target());
    TEST_ASSERT_TRUE(rig.engine.isRunning());

    // Still ramping down: the move keeps waiting.
    rig.fas->run(5);
    rig.engine.service();
    TEST_ASSERT_EQUAL(40, rig.fas->targetPos());

    rig.fas->runToEnd();
    rig.engine.service();
    TEST_ASSERT_EQUAL(0, rig.fas->targetPos());

    rig.fas->runToEnd();
    TEST_ASSERT_EQUAL(0, rig.engine.position());
    TEST_ASSERT_FALSE(rig.engine.isRunning());
}

void test_a_move_to_where_it_is_stops_dead() {
    Rig rig;
    rig.engine.moveTo(100);
    rig.fas->run(30);

    rig.engine.moveTo(30);
    TEST_ASSERT_EQUAL(1, rig.fas->forceStops);
    TEST_ASSERT_FALSE(rig.engine.isRunning());
    TEST_ASSERT_EQUAL(30, rig.engine.target());
}

// --- Stops --------------------------------------------------------------------

void test_stop_now_drops_the_queue_where_it_is() {
    Rig rig;
    rig.engine.moveTo(100);
    rig.fas->run(30);

    rig.engine.stopNow();
    TEST_ASSERT_EQUAL(1, rig.fas->forceStops);
    TEST_ASSERT_EQUAL(30, rig.engine.position());
    TEST_ASSERT_EQUAL(30, rig.engine.target());
    TEST_ASSERT_FALSE(rig.engine.isRunning());
}

void test_stop_ramped_targets_the_library_stopping_point() {
    Rig rig;
    rig.engine.moveTo(1000);
    rig.fas->run(100);

    rig.engine.stopRamped();
    TEST_ASSERT_EQUAL(1, rig.fas->stopMoves);
    TEST_ASSERT_EQUAL(110, rig.engine.target());

    rig.fas->runToEnd();
    TEST_ASSERT_EQUAL(110, rig.engine.position());
    TEST_ASSERT_FALSE(rig.engine.isRunning());
}

void test_stop_ramped_never_extends_a_move() {
    Rig rig;
    rig.engine.moveTo(105);
    rig.fas->run(100);

    rig.engine.stopRamped();
    TEST_ASSERT_EQUAL(105, rig.engine.target());
}

void test_stop_ramped_at_rest_does_nothing() {
    Rig rig;
    rig.engine.stopRamped();
    TEST_ASSERT_EQUAL(0, rig.fas->stopMoves);
}

// After a ramped stop the library rests a little off the projected point, or
// refuses a move: either way, where it stopped is the target.
void test_at_rest_the_target_is_where_the_library_stopped() {
    Rig rig;
    rig.engine.moveTo(1000);
    rig.fas->run(100);
    rig.engine.stopRamped();
    rig.fas->forceStopAndNewPosition(107);   // stopped short of 110

    TEST_ASSERT_EQUAL(107, rig.engine.target());
}

void test_set_position_at_rest_and_mid_move() {
    Rig rig;
    rig.engine.setPosition(500);
    TEST_ASSERT_EQUAL(1, rig.fas->positionSets);
    TEST_ASSERT_EQUAL(500, rig.engine.position());
    TEST_ASSERT_EQUAL(500, rig.engine.target());

    rig.engine.moveTo(600);
    rig.fas->run(10);
    rig.engine.setPosition(0);
    TEST_ASSERT_EQUAL(1, rig.fas->forceStops);
    TEST_ASSERT_EQUAL(0, rig.engine.position());
    TEST_ASSERT_FALSE(rig.engine.isRunning());
}

// --- Endstop inhibit ----------------------------------------------------------

void test_inhibit_stops_travel_the_forbidden_way_dead() {
    Rig rig;
    rig.engine.moveTo(100);
    rig.fas->run(30);

    rig.engine.inhibit(+1);
    TEST_ASSERT_EQUAL(1, rig.fas->forceStops);
    TEST_ASSERT_EQUAL(30, rig.engine.target());
}

void test_inhibit_leaves_travel_the_other_way_alone() {
    Rig rig;
    rig.engine.moveTo(-100);
    rig.fas->run(30);

    rig.engine.inhibit(+1);
    TEST_ASSERT_EQUAL(0, rig.fas->forceStops);
    TEST_ASSERT_TRUE(rig.engine.isRunning());
}

void test_a_move_the_forbidden_way_is_cancelled() {
    Rig rig;
    rig.engine.inhibit(-1);

    rig.engine.moveTo(-50);
    TEST_ASSERT_EQUAL(0, rig.fas->moves);
    TEST_ASSERT_EQUAL(0, rig.engine.target());
    TEST_ASSERT_FALSE(rig.engine.isRunning());

    rig.engine.moveTo(50);
    TEST_ASSERT_EQUAL(50, rig.fas->targetPos());
}

// --- The fleet's gate ---------------------------------------------------------

void test_a_move_waits_while_the_gate_is_shut() {
    Rig rig;
    rig.engine.allowPulses(false);

    rig.engine.moveTo(50);
    rig.engine.service();
    TEST_ASSERT_EQUAL(0, rig.fas->moves);
    TEST_ASSERT_TRUE(rig.engine.isRunning());
    TEST_ASSERT_EQUAL(50, rig.engine.target());

    rig.engine.allowPulses(true);
    rig.engine.service();
    TEST_ASSERT_EQUAL(50, rig.fas->targetPos());
}

void test_shutting_the_gate_mid_move_stops_and_resumes() {
    Rig rig;
    rig.engine.moveTo(100);
    rig.fas->run(30);

    rig.engine.allowPulses(false);
    TEST_ASSERT_EQUAL(1, rig.fas->forceStops);
    TEST_ASSERT_EQUAL(30, rig.engine.position());
    TEST_ASSERT_EQUAL(100, rig.engine.target());
    TEST_ASSERT_TRUE(rig.engine.isRunning());

    rig.engine.allowPulses(true);
    rig.engine.service();
    TEST_ASSERT_EQUAL(100, rig.fas->targetPos());
}

// --- As an axis -----------------------------------------------------------------

void test_an_axis_on_fas_engine_steps_on_its_own() {
    EngineStepper<FasEngine> axis(3200, STEP_PIN, DIR_PIN, 200UL);
    axis.begin();

    TEST_ASSERT_TRUE(axis.getEngine().stepsOnItsOwn());
    TEST_ASSERT_FALSE(axis.getEngine().service());

    axis.moveSteps(50);
    axis.getEngine().stepper()->runToEnd();
    axis.update();
    TEST_ASSERT_EQUAL(50, axis.getStep());
    TEST_ASSERT_EQUAL(Stepper::HOLD, axis.getState());
}

void test_an_axis_refuses_to_drive_onto_a_pressed_switch() {
    EngineStepper<FasEngine> axis(3200, STEP_PIN, DIR_PIN, 200UL);
    Endstop maxSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MAX);
    axis.setEndstop(&maxSwitch);
    fake::setInput(SWITCH_PIN, LOW);
    axis.begin();

    axis.moveSteps(50);
    axis.update();

    TEST_ASSERT_EQUAL(0, axis.getStep());
    TEST_ASSERT_FALSE(axis.getEngine().isRunning());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_begin_connects_the_pins);
    RUN_TEST(test_without_a_profile_the_axis_does_not_ramp);
    RUN_TEST(test_a_profile_becomes_the_acceleration);
    RUN_TEST(test_a_change_of_pace_mid_move_is_applied_at_once);
    RUN_TEST(test_an_interval_too_fast_for_the_library_is_reported);
    RUN_TEST(test_with_no_stepper_left_the_axis_stays_put);
    RUN_TEST(test_a_move_goes_to_the_library);
    RUN_TEST(test_a_move_the_same_way_extends_the_one_running);
    RUN_TEST(test_a_reversal_ramps_down_first_then_goes_back);
    RUN_TEST(test_a_move_to_where_it_is_stops_dead);
    RUN_TEST(test_stop_now_drops_the_queue_where_it_is);
    RUN_TEST(test_stop_ramped_targets_the_library_stopping_point);
    RUN_TEST(test_stop_ramped_never_extends_a_move);
    RUN_TEST(test_stop_ramped_at_rest_does_nothing);
    RUN_TEST(test_at_rest_the_target_is_where_the_library_stopped);
    RUN_TEST(test_set_position_at_rest_and_mid_move);
    RUN_TEST(test_inhibit_stops_travel_the_forbidden_way_dead);
    RUN_TEST(test_inhibit_leaves_travel_the_other_way_alone);
    RUN_TEST(test_a_move_the_forbidden_way_is_cancelled);
    RUN_TEST(test_a_move_waits_while_the_gate_is_shut);
    RUN_TEST(test_shutting_the_gate_mid_move_stops_and_resumes);
    RUN_TEST(test_an_axis_on_fas_engine_steps_on_its_own);
    RUN_TEST(test_an_axis_refuses_to_drive_onto_a_pressed_switch);
    return UNITY_END();
}
