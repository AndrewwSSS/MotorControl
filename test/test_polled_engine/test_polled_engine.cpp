// PolledEngine on its own: STEP pulses, DIR handling, ramp pacing and the
// endstop inhibit.

#include <unity.h>
#include <Arduino.h>

#include "LinearRamp.h"
#include "PolledEngine.h"

static const uint8_t STEP_PIN   = 2;
static const uint8_t DIR_PIN    = 5;
static const uint8_t ENABLE_PIN = 8;
static const unsigned long INTERVAL_US = 800;

// service() every 10 us until the engine reaches step, or a time limit runs
// out so a test that never gets there fails instead of hanging.
static void runUntil(StepEngine& engine, long step) {
    unsigned long until = micros() + 10000000UL;
    while (engine.position() != step && micros() < until) {
        engine.service();
        fake::advanceUs(10);
    }
}

static void runToEnd(StepEngine& engine) {
    runUntil(engine, engine.target());
}

static void runFor(StepEngine& engine, unsigned long us) {
    unsigned long until = micros() + us;
    while (micros() < until) {
        engine.service();
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

void test_begin_sets_up_the_pins_and_energises() {
    PolledEngine engine(STEP_PIN, DIR_PIN, ENABLE_PIN);
    engine.begin();

    TEST_ASSERT_EQUAL_UINT8(OUTPUT, fake::modeOf(STEP_PIN));
    TEST_ASSERT_EQUAL_UINT8(OUTPUT, fake::modeOf(DIR_PIN));
    TEST_ASSERT_EQUAL_UINT8(OUTPUT, fake::modeOf(ENABLE_PIN));
    TEST_ASSERT_EQUAL_UINT8(LOW, fake::levelOf(ENABLE_PIN));   // active low

    engine.disable();
    TEST_ASSERT_EQUAL_UINT8(HIGH, fake::levelOf(ENABLE_PIN));
}

void test_a_move_of_n_steps_emits_n_pulses() {
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, INTERVAL_US);
    engine.begin();

    engine.moveTo(10);
    TEST_ASSERT_TRUE(engine.isRunning());
    runToEnd(engine);
    runFor(engine, 10000);

    TEST_ASSERT_EQUAL(10, fake::risingEdges(STEP_PIN));
    TEST_ASSERT_EQUAL(10, engine.position());
    TEST_ASSERT_FALSE(engine.isRunning());
}

void test_service_says_when_a_pulse_went_out() {
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, INTERVAL_US);
    engine.begin();
    engine.moveTo(2);

    TEST_ASSERT_FALSE(engine.service());   // t = 0: not due yet
    fake::advanceUs(INTERVAL_US);
    TEST_ASSERT_TRUE(engine.service());
    TEST_ASSERT_FALSE(engine.service());   // the next one is a full interval away
    TEST_ASSERT_FALSE(engine.stepsOnItsOwn());
}

void test_dir_is_set_before_the_first_pulse() {
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, INTERVAL_US);
    engine.begin();
    fake::clearWrites();

    engine.moveTo(3);
    runToEnd(engine);

    const fake::PinWrite* dir = nullptr;
    for (const fake::PinWrite& w : fake::writes()) {
        if (w.pin == DIR_PIN) { dir = &w; break; }
    }
    TEST_ASSERT_NOT_NULL(dir);
    TEST_ASSERT_EQUAL_UINT8(HIGH, dir->level);

    std::vector<unsigned long> steps = fake::risingEdgeTimes(STEP_PIN);
    TEST_ASSERT_EQUAL(3, steps.size());
    TEST_ASSERT_TRUE(dir->atUs + MC_DIR_SETUP_US <= steps[0]);
    TEST_ASSERT_EQUAL(1, writesTo(DIR_PIN));
}

void test_a_reversal_rewrites_dir_once() {
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, INTERVAL_US);
    engine.begin();

    engine.moveTo(3);
    runToEnd(engine);
    fake::clearWrites();

    engine.moveTo(1);
    runToEnd(engine);

    TEST_ASSERT_EQUAL(1, writesTo(DIR_PIN));
    TEST_ASSERT_EQUAL_UINT8(LOW, fake::levelOf(DIR_PIN));
    TEST_ASSERT_EQUAL(2, fake::risingEdges(STEP_PIN));
}

void test_pulses_are_spaced_by_the_interval_and_last_the_pulse_width() {
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, INTERVAL_US,
                        PolledEngine::TB6600_PULSE_US);
    engine.begin();
    fake::clearWrites();

    engine.moveTo(5);
    runToEnd(engine);

    std::vector<unsigned long> rises = fake::risingEdgeTimes(STEP_PIN);
    TEST_ASSERT_EQUAL(5, rises.size());
    for (size_t i = 1; i < rises.size(); i++) {
        TEST_ASSERT_TRUE(rises[i] - rises[i - 1] + MC_DIR_SETUP_US >= INTERVAL_US);
        TEST_ASSERT_TRUE(rises[i] - rises[i - 1] <= INTERVAL_US + 20);
    }

    const std::vector<fake::PinWrite>& w = fake::writes();
    for (size_t i = 0; i + 1 < w.size(); i++) {
        if (w[i].pin != STEP_PIN || w[i].level != HIGH) continue;
        TEST_ASSERT_EQUAL_UINT8(LOW, w[i + 1].level);
        TEST_ASSERT_EQUAL_UINT32(PolledEngine::TB6600_PULSE_US, w[i + 1].atUs - w[i].atUs);
    }
}

void test_inhibit_cancels_a_move_the_forbidden_way() {
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, INTERVAL_US);
    engine.begin();

    engine.moveTo(100);
    runUntil(engine, 20);
    engine.inhibit(1);
    runFor(engine, 50000);

    TEST_ASSERT_EQUAL(20, engine.position());
    TEST_ASSERT_EQUAL(20, engine.target());
    TEST_ASSERT_FALSE(engine.isRunning());
}

void test_inhibit_allows_travel_the_other_way() {
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, INTERVAL_US);
    engine.begin();
    engine.inhibit(1);

    engine.moveTo(10);
    runFor(engine, 50000);
    TEST_ASSERT_EQUAL(0, fake::risingEdges(STEP_PIN));

    engine.moveTo(-5);
    runToEnd(engine);
    TEST_ASSERT_EQUAL(-5, engine.position());

    // Lifted, the forbidden way is open again.
    engine.inhibit(0);
    engine.moveTo(5);
    runToEnd(engine);
    TEST_ASSERT_EQUAL(5, engine.position());
}

void test_pulses_are_paced_by_the_ramp() {
    LinearRamp ramp(1000, 10);
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, 200);
    engine.setProfile(&ramp);
    engine.begin();

    const long n = 30;
    engine.moveTo(n);
    runToEnd(engine);

    std::vector<unsigned long> rises = fake::risingEdgeTimes(STEP_PIN);
    TEST_ASSERT_EQUAL(n, rises.size());
    for (long k = 1; k < n; k++) {
        unsigned long expected = ramp.intervalFor(k, n - k, 200);
        unsigned long gap = rises[k] - rises[k - 1];
        TEST_ASSERT_TRUE(gap + MC_DIR_SETUP_US >= expected);
        TEST_ASSERT_TRUE(gap <= expected + 15);
    }
}

void test_stop_ramped_comes_to_rest_over_the_ramp() {
    LinearRamp ramp(1000, 100);
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, 200);
    engine.setProfile(&ramp);
    engine.begin();

    engine.moveTo(1000);
    runUntil(engine, 500);
    engine.stopRamped();
    TEST_ASSERT_EQUAL(600, engine.target());
}

void test_stop_ramped_never_extends_a_move() {
    LinearRamp ramp(1000, 100);
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, 200);
    engine.setProfile(&ramp);
    engine.begin();

    engine.moveTo(1000);
    runUntil(engine, 950);   // already inside the stopping distance
    engine.stopRamped();
    TEST_ASSERT_EQUAL(1000, engine.target());
}

void test_a_move_added_in_flight_keeps_the_ramp_going() {
    LinearRamp ramp(1000, 100);
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, 200);
    engine.setProfile(&ramp);
    engine.begin();

    engine.moveTo(1000);
    runUntil(engine, 500);
    engine.moveTo(1500);
    TEST_ASSERT_EQUAL_UINT32(200, engine.currentInterval());

    runToEnd(engine);
    engine.moveTo(1800);   // from rest: a fresh ramp
    TEST_ASSERT_EQUAL_UINT32(1000, engine.currentInterval());
}

void test_set_position_redefines_where_it_is() {
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, INTERVAL_US);
    engine.begin();

    engine.moveTo(50);
    runUntil(engine, 10);
    engine.setPosition(-300);

    TEST_ASSERT_EQUAL(-300, engine.position());
    TEST_ASSERT_EQUAL(-300, engine.target());
    TEST_ASSERT_FALSE(engine.isRunning());

    runFor(engine, 10000);
    TEST_ASSERT_EQUAL(10, fake::risingEdges(STEP_PIN));
}

void test_a_shut_gate_holds_pulses_back_until_opened() {
    PolledEngine engine(STEP_PIN, DIR_PIN, PolledEngine::NO_ENABLE_PIN, INTERVAL_US);
    engine.begin();

    engine.allowPulses(false);
    engine.moveTo(5);
    runFor(engine, 50000);
    TEST_ASSERT_EQUAL(0, fake::risingEdges(STEP_PIN));
    TEST_ASSERT_TRUE(engine.isRunning());   // the move waits, it is not dropped

    engine.allowPulses(true);
    runToEnd(engine);
    TEST_ASSERT_EQUAL(5, fake::risingEdges(STEP_PIN));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_begin_sets_up_the_pins_and_energises);
    RUN_TEST(test_a_move_of_n_steps_emits_n_pulses);
    RUN_TEST(test_service_says_when_a_pulse_went_out);
    RUN_TEST(test_dir_is_set_before_the_first_pulse);
    RUN_TEST(test_a_reversal_rewrites_dir_once);
    RUN_TEST(test_pulses_are_spaced_by_the_interval_and_last_the_pulse_width);
    RUN_TEST(test_inhibit_cancels_a_move_the_forbidden_way);
    RUN_TEST(test_inhibit_allows_travel_the_other_way);
    RUN_TEST(test_pulses_are_paced_by_the_ramp);
    RUN_TEST(test_stop_ramped_comes_to_rest_over_the_ramp);
    RUN_TEST(test_stop_ramped_never_extends_a_move);
    RUN_TEST(test_a_move_added_in_flight_keeps_the_ramp_going);
    RUN_TEST(test_set_position_redefines_where_it_is);
    RUN_TEST(test_a_shut_gate_holds_pulses_back_until_opened);
    return UNITY_END();
}
