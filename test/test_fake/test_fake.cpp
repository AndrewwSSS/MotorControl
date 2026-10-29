// Checks the stand-in Arduino core itself, so a later failing test points at
// the library and not at the fake. Including the stepper headers here also
// proves they compile on the host.

#include <unity.h>
#include <Arduino.h>

#include "Endstop.h"
#include "Homing.h"
#include "LinearRamp.h"
#include "StepDirStepper.h"
#include "StepEngine.h"
#include "StepperManager.h"

void setUp() {
    fake::reset();
}

void tearDown() {}

void test_time_stands_still_until_moved() {
    TEST_ASSERT_EQUAL_UINT32(0, micros());
    TEST_ASSERT_EQUAL_UINT32(0, micros());

    fake::advanceUs(250);
    TEST_ASSERT_EQUAL_UINT32(250, micros());

    fake::advanceMs(3);
    TEST_ASSERT_EQUAL_UINT32(3250, micros());
    TEST_ASSERT_EQUAL_UINT32(3, millis());
}

void test_delay_moves_time() {
    delayMicroseconds(5);
    TEST_ASSERT_EQUAL_UINT32(5, micros());

    delay(2);
    TEST_ASSERT_EQUAL_UINT32(2005, micros());
}

void test_auto_advance_moves_time_on_each_read() {
    fake::setAutoAdvance(10);
    TEST_ASSERT_EQUAL_UINT32(0, micros());
    TEST_ASSERT_EQUAL_UINT32(10, micros());
    TEST_ASSERT_EQUAL_UINT32(20, micros());

    // millis() does not advance time, or a debounce check would drift it
    TEST_ASSERT_EQUAL_UINT32(0, millis());
}

void test_writes_are_recorded_with_their_time() {
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
    fake::advanceUs(2);
    digitalWrite(2, LOW);

    TEST_ASSERT_EQUAL_UINT8(OUTPUT, fake::modeOf(2));
    TEST_ASSERT_EQUAL_UINT8(LOW, fake::levelOf(2));
    TEST_ASSERT_EQUAL(2, fake::writes().size());
    TEST_ASSERT_EQUAL_UINT8(HIGH, fake::writes()[0].level);
    TEST_ASSERT_EQUAL_UINT32(0, fake::writes()[0].atUs);
    TEST_ASSERT_EQUAL_UINT32(2, fake::writes()[1].atUs);
}

void test_rising_edges_count_pulses_per_pin() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(2, HIGH);
        digitalWrite(2, LOW);
    }
    digitalWrite(5, HIGH);
    digitalWrite(5, HIGH);   // already high: not a new edge

    TEST_ASSERT_EQUAL(3, fake::risingEdges(2));
    TEST_ASSERT_EQUAL(1, fake::risingEdges(5));
    TEST_ASSERT_EQUAL(0, fake::risingEdges(9));
}

static int switchClosed(uint8_t pin) {
    return (pin == 9) ? LOW : HIGH;
}

void test_reads_are_scripted() {
    TEST_ASSERT_EQUAL(HIGH, digitalRead(9));   // pulled up, nothing pressed

    fake::setInput(9, LOW);
    TEST_ASSERT_EQUAL(LOW, digitalRead(9));

    fake::setInput(9, HIGH);
    fake::onRead(switchClosed);
    TEST_ASSERT_EQUAL(LOW, digitalRead(9));
    TEST_ASSERT_EQUAL(HIGH, digitalRead(10));
}

void test_log_output_is_captured() {
    MC_INFO(F("Endstop "), F("MIN"), F(" on pin "), (uint8_t)7, F(" at step "), -42L);
    TEST_ASSERT_EQUAL_STRING("Endstop MIN on pin 7 at step -42\r\n",
                             fake::serialOutput().c_str());

    fake::clearSerial();
    Serial.print(800UL);
    Serial.print('x');
    TEST_ASSERT_EQUAL_STRING("800x", fake::serialOutput().c_str());
}

void test_reset_returns_to_power_on() {
    fake::advanceUs(100);
    fake::setAutoAdvance(1);
    fake::setInput(9, LOW);
    digitalWrite(2, HIGH);
    Serial.print(F("hello"));

    fake::reset();

    TEST_ASSERT_EQUAL_UINT32(0, micros());
    TEST_ASSERT_EQUAL_UINT32(0, micros());
    TEST_ASSERT_EQUAL(HIGH, digitalRead(9));
    TEST_ASSERT_EQUAL(0, fake::writes().size());
    TEST_ASSERT_TRUE(fake::serialOutput().empty());
}

// Not a test of stepping (that is the characterisation suite), just proof that
// a real library class runs on the fake.
void test_a_stepper_sets_up_its_pins() {
    StepDirStepper axis(2, 5, 3200, 8);
    axis.begin();

    TEST_ASSERT_EQUAL_UINT8(OUTPUT, fake::modeOf(2));
    TEST_ASSERT_EQUAL_UINT8(OUTPUT, fake::modeOf(5));
    TEST_ASSERT_EQUAL_UINT8(OUTPUT, fake::modeOf(8));
    TEST_ASSERT_EQUAL_UINT8(LOW, fake::levelOf(8));   // EN is active low
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_time_stands_still_until_moved);
    RUN_TEST(test_delay_moves_time);
    RUN_TEST(test_auto_advance_moves_time_on_each_read);
    RUN_TEST(test_writes_are_recorded_with_their_time);
    RUN_TEST(test_rising_edges_count_pulses_per_pin);
    RUN_TEST(test_reads_are_scripted);
    RUN_TEST(test_log_output_is_captured);
    RUN_TEST(test_reset_returns_to_power_on);
    RUN_TEST(test_a_stepper_sets_up_its_pins);
    return UNITY_END();
}
