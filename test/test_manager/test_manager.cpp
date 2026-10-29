// Characterisation of StepperManager's shared EN line: pulses are withheld
// while the drivers wake up, and EN is released once the fleet has idled for
// the configured time.

#include <unity.h>
#include <Arduino.h>

#include "StepDirStepper.h"
#include "StepperManager.h"

static const uint8_t STEP_PIN   = 2;
static const uint8_t DIR_PIN    = 5;
static const uint8_t ENABLE_PIN = 8;
static const unsigned long WAKE_US = 1000;
static const unsigned long IDLE_MS = 200;

static int reachedCount;
static uint8_t reachedId;

static void onReached(uint8_t id) {
    reachedCount++;
    reachedId = id;
}

// update() every 10 us for a while.
static void runFor(StepperManager& fleet, unsigned long us) {
    unsigned long until = micros() + us;
    while (micros() < until) {
        fleet.update();
        fake::advanceUs(10);
    }
}

static void runUntilIdle(StepperManager& fleet) {
    unsigned long until = micros() + 10000000UL;
    while (fleet.hasWork() && micros() < until) {
        fleet.update();
        fake::advanceUs(10);
    }
    fleet.update();
}

// The time of the last write of level to pin, or 0 if there was none.
static unsigned long lastWriteAt(uint8_t pin, uint8_t level) {
    unsigned long at = 0;
    for (const fake::PinWrite& w : fake::writes()) {
        if (w.pin == pin && w.level == level) at = w.atUs;
    }
    return at;
}

struct Fleet {
    StepperManager manager;

    Fleet() : manager(2, ENABLE_PIN) {
        manager.addMotor(new StepDirStepper(STEP_PIN, DIR_PIN, 3200,
                                            StepDirStepper::NO_ENABLE_PIN, 800));
        manager.setIdleRelease(IDLE_MS, WAKE_US);
        manager.onReached(onReached);
        manager.begin();
    }
};

void setUp() {
    fake::reset();
    reachedCount = 0;
    reachedId = 0xFF;
}

void tearDown() {}

void test_begin_energises_the_drivers() {
    Fleet fleet;

    TEST_ASSERT_EQUAL_UINT8(OUTPUT, fake::modeOf(ENABLE_PIN));
    TEST_ASSERT_EQUAL_UINT8(LOW, fake::levelOf(ENABLE_PIN));   // active low
    TEST_ASSERT_TRUE(fleet.manager.isPowered());
}

void test_pulses_wait_out_the_wake_up_after_begin() {
    Fleet fleet;

    fleet.manager.moveSteps(0, 5);
    runFor(fleet.manager, WAKE_US - 20);
    TEST_ASSERT_EQUAL(0, fake::risingEdges(STEP_PIN));

    runUntilIdle(fleet.manager);
    std::vector<unsigned long> rises = fake::risingEdgeTimes(STEP_PIN);
    TEST_ASSERT_EQUAL(5, rises.size());
    TEST_ASSERT_TRUE(rises[0] >= WAKE_US);
}

void test_en_is_released_after_the_idle_time() {
    Fleet fleet;

    fleet.manager.moveSteps(0, 5);
    runUntilIdle(fleet.manager);
    unsigned long lastPulse = fake::risingEdgeTimes(STEP_PIN).back();

    runFor(fleet.manager, (IDLE_MS - 5) * 1000UL);
    TEST_ASSERT_EQUAL_UINT8(LOW, fake::levelOf(ENABLE_PIN));

    runFor(fleet.manager, 10 * 1000UL);
    TEST_ASSERT_EQUAL_UINT8(HIGH, fake::levelOf(ENABLE_PIN));
    TEST_ASSERT_FALSE(fleet.manager.isPowered());

    // Idle time is counted in whole milliseconds from millis(), so the release
    // can land up to 1 ms short of IDLE_MS after the last pulse.
    unsigned long releasedAt = lastWriteAt(ENABLE_PIN, HIGH);
    TEST_ASSERT_TRUE(releasedAt - lastPulse >= (IDLE_MS - 1) * 1000UL);
    TEST_ASSERT_TRUE(releasedAt - lastPulse <= (IDLE_MS + 2) * 1000UL);
}

void test_the_next_move_re_energises_and_waits_again() {
    Fleet fleet;
    runFor(fleet.manager, (IDLE_MS + 10) * 1000UL);
    TEST_ASSERT_FALSE(fleet.manager.isPowered());
    fake::clearWrites();

    unsigned long movedAt = micros();
    fleet.manager.moveSteps(0, 5);
    fleet.manager.update();
    TEST_ASSERT_EQUAL_UINT8(LOW, fake::levelOf(ENABLE_PIN));
    TEST_ASSERT_TRUE(fleet.manager.isPowered());

    runFor(fleet.manager, WAKE_US - 20);
    TEST_ASSERT_EQUAL(0, fake::risingEdges(STEP_PIN));

    runUntilIdle(fleet.manager);
    std::vector<unsigned long> rises = fake::risingEdgeTimes(STEP_PIN);
    TEST_ASSERT_EQUAL(5, rises.size());
    TEST_ASSERT_TRUE(rises[0] - movedAt >= WAKE_US);
}

void test_without_idle_release_en_stays_on() {
    Fleet fleet;
    fleet.manager.setIdleRelease(0);

    runFor(fleet.manager, 1000 * 1000UL);

    TEST_ASSERT_EQUAL_UINT8(LOW, fake::levelOf(ENABLE_PIN));
    TEST_ASSERT_TRUE(fleet.manager.isPowered());
}

void test_reaching_the_target_calls_back_once() {
    Fleet fleet;

    fleet.manager.moveSteps(0, 5);
    runUntilIdle(fleet.manager);
    runFor(fleet.manager, 10000);

    TEST_ASSERT_EQUAL(1, reachedCount);
    TEST_ASSERT_EQUAL_UINT8(0, reachedId);
    TEST_ASSERT_EQUAL(5, fleet.manager.getStep(0));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_begin_energises_the_drivers);
    RUN_TEST(test_pulses_wait_out_the_wake_up_after_begin);
    RUN_TEST(test_en_is_released_after_the_idle_time);
    RUN_TEST(test_the_next_move_re_energises_and_waits_again);
    RUN_TEST(test_without_idle_release_en_stays_on);
    RUN_TEST(test_reaching_the_target_calls_back_once);
    return UNITY_END();
}
