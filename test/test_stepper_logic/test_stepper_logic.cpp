// Stepper, homing and the fleet against FakeEngine: the logic above the engine,
// checked without pins or timing, and checked for both kinds of engine - one
// that steps when update() calls it, and one that steps on its own.

#include <unity.h>
#include <Arduino.h>

#include "FakeEngine.h"
#include "Endstop.h"
#include "Homing.h"
#include "LinearRamp.h"
#include "Stepper.h"
#include "StepperManager.h"

static const uint8_t SWITCH_PIN = 9;
static const uint8_t ENABLE_PIN = 8;

static bool contains(const char* text) {
    return strstr(fake::serialOutput().c_str(), text) != nullptr;
}

static void updateTimes(Stepper& axis, int n) {
    for (int i = 0; i < n; i++) {
        axis.update();
        fake::advanceUs(10);
    }
}

// The switch follows the shaft of whichever engine is under test, and trips
// at shaft position `tripAt` and beyond, in the direction `towards`.
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

void setUp() {
    fake::reset();
    switchEngine = nullptr;
}

void tearDown() {}

// --- State and commands -------------------------------------------------------

void test_begin_starts_the_engine_at_step_zero() {
    FakeEngine engine;
    engine.setPosition(42);
    Stepper axis(engine, 3200);

    axis.begin();

    TEST_ASSERT_EQUAL(1, engine.begins);
    TEST_ASSERT_EQUAL(0, axis.getStep());
    TEST_ASSERT_EQUAL(Stepper::HOLD, axis.getState());
}

void test_commands_pass_through_to_the_engine() {
    FakeEngine engine;
    LinearRamp ramp(1000, 100);
    Stepper axis(engine, 3200);
    axis.begin();

    axis.setStepInterval(250);
    axis.setProfile(&ramp);
    TEST_ASSERT_EQUAL_UINT32(250, engine.interval);
    TEST_ASSERT_EQUAL_PTR(&ramp, axis.getProfile());
    TEST_ASSERT_EQUAL_PTR(&engine, &axis.getEngine());

    axis.moveSteps(10);
    axis.moveSteps(5);   // anchored on the target
    TEST_ASSERT_EQUAL(15, engine.target());

    axis.setOrigin(-7);
    TEST_ASSERT_EQUAL(-7, engine.position());
    TEST_ASSERT_EQUAL(-7, engine.target());
}

void test_a_polled_axis_is_moving_only_once_a_pulse_went_out() {
    FakeEngine engine(FakeEngine::POLLED);
    Stepper axis(engine, 3200);
    axis.begin();

    engine.allowPulses(false);
    axis.moveSteps(3);
    updateTimes(axis, 5);
    TEST_ASSERT_EQUAL(Stepper::HOLD, axis.getState());   // has work, sent nothing

    engine.allowPulses(true);
    axis.update();
    TEST_ASSERT_EQUAL(Stepper::MOVING, axis.getState());

    updateTimes(axis, 5);
    TEST_ASSERT_EQUAL(3, axis.getStep());
    TEST_ASSERT_EQUAL(Stepper::HOLD, axis.getState());
    TEST_ASSERT_TRUE(axis.reachedTarget());
    TEST_ASSERT_FALSE(axis.reachedTarget());
}

void test_an_axis_on_its_own_engine_is_moving_while_the_engine_runs() {
    FakeEngine engine(FakeEngine::OWN, 4);
    Stepper axis(engine, 3200);
    axis.begin();

    axis.moveSteps(10);
    axis.update();
    TEST_ASSERT_EQUAL(4, axis.getStep());
    TEST_ASSERT_EQUAL(Stepper::MOVING, axis.getState());
    TEST_ASSERT_FALSE(axis.reachedTarget());

    axis.update();
    axis.update();   // the last 2 steps: the engine has stopped
    TEST_ASSERT_EQUAL(10, axis.getStep());
    TEST_ASSERT_EQUAL(Stepper::HOLD, axis.getState());
    TEST_ASSERT_TRUE(axis.reachedTarget());
}

void test_stop_ramped_and_stop_here_go_to_the_engine() {
    FakeEngine engine;
    LinearRamp ramp(1000, 100);
    Stepper axis(engine, 3200);
    axis.setProfile(&ramp);
    axis.begin();

    axis.moveSteps(1000);
    updateTimes(axis, 300);
    axis.stopRamped();
    TEST_ASSERT_EQUAL(400, axis.getTargetStep());

    axis.stopHere();
    TEST_ASSERT_EQUAL(300, axis.getTargetStep());
}

void test_debug_logs_the_target() {
    FakeEngine engine;
    Stepper axis(engine, 3200);
    axis.begin();
    axis.toggleDebug();

    axis.moveSteps(25);
    TEST_ASSERT_TRUE(contains("Target step: 25"));

    axis.update();
    TEST_ASSERT_TRUE(contains("Current step: 1 of 25"));
}

// --- Endstop guard ------------------------------------------------------------

void test_the_guard_inhibits_and_stops_travel_onto_a_pressed_switch() {
    FakeEngine engine;
    Endstop maxSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MAX);
    Stepper axis(engine, 3200);
    axis.setEndstop(&maxSwitch);
    watchSwitch(engine, 20, +1);
    axis.begin();

    axis.moveSteps(100);
    for (int i = 0; i < 200 && engine.isRunning(); i++) axis.update();

    TEST_ASSERT_EQUAL(20, axis.getStep());
    TEST_ASSERT_EQUAL(20, axis.getTargetStep());
    TEST_ASSERT_EQUAL(1, engine.inhibited);
    TEST_ASSERT_EQUAL(1, engine.stopNows);
    TEST_ASSERT_TRUE(contains("Endstop MAX on pin 9 stopped the axis at step 20"));
}

void test_the_guard_lifts_the_inhibit_for_travel_away() {
    FakeEngine engine;
    Endstop maxSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MAX);
    Stepper axis(engine, 3200);
    axis.setEndstop(&maxSwitch);
    fake::setInput(SWITCH_PIN, LOW);
    axis.begin();

    axis.moveSteps(10);
    axis.update();
    TEST_ASSERT_EQUAL(1, engine.inhibited);
    TEST_ASSERT_EQUAL(0, axis.getStep());

    axis.moveSteps(-5);
    updateTimes(axis, 10);
    TEST_ASSERT_EQUAL(0, engine.inhibited);
    TEST_ASSERT_EQUAL(-5, axis.getStep());
}

void test_a_min_switch_inhibits_downwards() {
    FakeEngine engine;
    Endstop minSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MIN);
    Stepper axis(engine, 3200);
    axis.setEndstop(&minSwitch);
    fake::setInput(SWITCH_PIN, LOW);
    axis.begin();

    axis.moveSteps(-10);
    axis.update();
    TEST_ASSERT_EQUAL(-1, engine.inhibited);
    TEST_ASSERT_EQUAL(0, axis.getStep());
}

// An engine that steps on its own can make several steps between two endstop checks.
void test_an_engine_on_its_own_overshoots_by_at_most_one_burst() {
    FakeEngine engine(FakeEngine::OWN, 5);
    Endstop maxSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MAX);
    Stepper axis(engine, 3200);
    axis.setEndstop(&maxSwitch);
    watchSwitch(engine, 22, +1);
    axis.begin();
    fake::advanceMs(10);   // clear of the debounce window

    axis.moveSteps(100);
    for (int i = 0; i < 100 && engine.isRunning(); i++) {
        axis.update();
        fake::advanceMs(1);
    }

    TEST_ASSERT_TRUE(axis.getStep() >= 22);
    TEST_ASSERT_TRUE(axis.getStep() < 22 + 5);
    TEST_ASSERT_FALSE(engine.isRunning());
}

// --- Homing -------------------------------------------------------------------

static HomingConfig homingConfig() {
    HomingConfig cfg;
    cfg.towards        = Endstop::MIN;
    cfg.restStep       = 100;
    cfg.maxSteps       = 2000;
    cfg.backoffSteps   = 50;
    cfg.slowIntervalUs = 2000;
    return cfg;
}

void test_homing_on_a_polled_engine_lands_exactly() {
    FakeEngine engine(FakeEngine::POLLED);
    Endstop minSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MIN);
    Stepper axis(engine, 3200);
    axis.setEndstop(&minSwitch);
    engine.placeShaft(500);
    watchSwitch(engine, 0, -1);
    axis.begin();

    TEST_ASSERT_EQUAL(HOMED, homeAxis(axis, homingConfig()));
    TEST_ASSERT_EQUAL(100, axis.getStep());
    TEST_ASSERT_EQUAL(100, engine.shaft());
    TEST_ASSERT_EQUAL_UINT32(800, engine.interval);   // restored after the slow pass
}

void test_homing_on_an_engine_on_its_own_sleeps_between_checks() {
    FakeEngine engine(FakeEngine::OWN, 3);
    Endstop minSwitch(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MIN);
    Stepper axis(engine, 3200);
    axis.setEndstop(&minSwitch);
    engine.placeShaft(500);
    watchSwitch(engine, 0, -1);
    axis.begin();

    unsigned long startMs = millis();
    HomingResult result = homeAxis(axis, homingConfig());

    TEST_ASSERT_EQUAL(HOMED, result);
    TEST_ASSERT_EQUAL(100, axis.getStep());
    // The trip point is seen up to one burst late on the slow pass, so step 0
    // sits up to 2 steps past it.
    TEST_ASSERT_TRUE(engine.shaft() <= 100);
    TEST_ASSERT_TRUE(engine.shaft() > 100 - 3);

    // Every update() was followed by a 1 ms sleep: about 1 ms per burst.
    unsigned long ms = millis() - startMs;
    TEST_ASSERT_TRUE(ms >= (unsigned long)(engine.services - 10));
}

void test_homing_failure_is_the_same_on_either_engine() {
    FakeEngine polled(FakeEngine::POLLED);
    FakeEngine own(FakeEngine::OWN, 4);
    Stepper a(polled, 3200), b(own, 3200);
    Endstop sa(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MIN);
    Endstop sb(SWITCH_PIN, Endstop::NORMALLY_OPEN, Endstop::MIN);
    a.setEndstop(&sa);
    b.setEndstop(&sb);
    a.begin();
    b.begin();

    HomingConfig cfg = homingConfig();
    cfg.maxSteps = 200;   // nothing ever presses the switch

    TEST_ASSERT_EQUAL(HOMING_NEVER_CLOSED, homeAxis(a, cfg));
    TEST_ASSERT_EQUAL(HOMING_NEVER_CLOSED, homeAxis(b, cfg));
    TEST_ASSERT_EQUAL(-200, polled.shaft());
    TEST_ASSERT_EQUAL(-200, own.shaft());
}

// --- Fleet --------------------------------------------------------------------

void test_the_fleet_gates_every_engine() {
    FakeEngine e0(FakeEngine::OWN, 4), e1(FakeEngine::OWN, 4);
    StepperManager fleet(2, ENABLE_PIN);
    fleet.addMotor(new Stepper(e0, 3200));
    fleet.addMotor(new Stepper(e1, 3200));
    fleet.setIdleRelease(200, 1000);
    fleet.begin();

    // Waking after begin().
    TEST_ASSERT_FALSE(e0.pulsesAllowed);
    TEST_ASSERT_FALSE(e1.pulsesAllowed);

    // The gate opens on the first update() with work once the drivers are
    // awake, not by the clock alone.
    fake::advanceUs(1000);
    fleet.update();
    TEST_ASSERT_FALSE(e0.pulsesAllowed);

    fleet.moveSteps(0, 4);
    fleet.update();
    TEST_ASSERT_TRUE(e0.pulsesAllowed);
    TEST_ASSERT_TRUE(e1.pulsesAllowed);
    TEST_ASSERT_EQUAL(4, e0.position());

    // Released after the idle time.
    fake::advanceMs(250);
    fleet.update();
    TEST_ASSERT_FALSE(fleet.isPowered());
    TEST_ASSERT_FALSE(e0.pulsesAllowed);

    // A move re-energises; the gate opens once the drivers are awake.
    fleet.moveSteps(1, 8);
    fleet.update();
    TEST_ASSERT_FALSE(e1.pulsesAllowed);
    TEST_ASSERT_EQUAL(0, e1.position());

    fake::advanceUs(1000);
    fleet.update();
    fleet.update();
    TEST_ASSERT_TRUE(e1.pulsesAllowed);
    TEST_ASSERT_EQUAL(8, e1.position());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_begin_starts_the_engine_at_step_zero);
    RUN_TEST(test_commands_pass_through_to_the_engine);
    RUN_TEST(test_a_polled_axis_is_moving_only_once_a_pulse_went_out);
    RUN_TEST(test_an_axis_on_its_own_engine_is_moving_while_the_engine_runs);
    RUN_TEST(test_stop_ramped_and_stop_here_go_to_the_engine);
    RUN_TEST(test_debug_logs_the_target);
    RUN_TEST(test_the_guard_inhibits_and_stops_travel_onto_a_pressed_switch);
    RUN_TEST(test_the_guard_lifts_the_inhibit_for_travel_away);
    RUN_TEST(test_a_min_switch_inhibits_downwards);
    RUN_TEST(test_an_engine_on_its_own_overshoots_by_at_most_one_burst);
    RUN_TEST(test_homing_on_a_polled_engine_lands_exactly);
    RUN_TEST(test_homing_on_an_engine_on_its_own_sleeps_between_checks);
    RUN_TEST(test_homing_failure_is_the_same_on_either_engine);
    RUN_TEST(test_the_fleet_gates_every_engine);
    return UNITY_END();
}
