// Two step+dir axes sharing one EN line: X homes against a limit switch at
// start-up, then both swing back and forth under a speed ramp. X is a
// StepDirStepper; Y is a Stepper on a separately declared engine.
// Uno (CNC Shield pins) or ESP32.

#include <Arduino.h>
#include <Endstop.h>
#include <Homing.h>
#include <LinearRamp.h>
#include <PolledEngine.h>
#include <StepDirStepper.h>
#include <StepperManager.h>

#if defined(ESP32)
static constexpr uint8_t X_STEP = 25, X_DIR = 26, X_LIMIT = 34;
static constexpr uint8_t Y_STEP = 27, Y_DIR = 33;
static constexpr uint8_t EN_PIN = 4;
#else
static constexpr uint8_t X_STEP = 2, X_DIR = 5, X_LIMIT = 9;
static constexpr uint8_t Y_STEP = 3, Y_DIR = 6;
static constexpr uint8_t EN_PIN = 8;
#endif

// 1.8 deg motors at 1/16 microstepping.
static constexpr uint16_t STEPS_PER_REV = 200 * 16;

// Borrowed by the axes, so they live at file scope.
Endstop xMin(X_LIMIT, Endstop::NORMALLY_CLOSED, Endstop::MIN);
LinearRamp ramp(1000, 1000);   // ease in from 1000 us over 1000 steps
PolledEngine yEngine(Y_STEP, Y_DIR, PolledEngine::NO_ENABLE_PIN, 200,
                     PolledEngine::TB6600_PULSE_US);

StepperManager axes(2, EN_PIN);

//                     towards        rest  maxSteps backoff slowUs
static const HomingConfig X_HOMING = { Endstop::MIN, 200, 20000,   40,     4000 };

static void onReached(uint8_t id) {
    // Swing to the other end of a one-revolution stroke.
    long next = (axes.getStep(id) > 0) ? 0 : STEPS_PER_REV;
    axes.moveToStep(id, next);
}

void setup() {
    Serial.begin(115200);

    // The fleet owns EN, so the axes are built without one.
    Stepper* x = new StepDirStepper(X_STEP, X_DIR, STEPS_PER_REV,
                                    StepDirStepper::NO_ENABLE_PIN, 200);
    x->setEndstop(&xMin);
    axes.addMotor(x);

    axes.addMotor(new Stepper(yEngine, STEPS_PER_REV));

    axes.setProfiles(&ramp);
    axes.setIdleRelease(200);
    axes.onReached(onReached);
    axes.begin();

    HomingResult r = axes.home(0, X_HOMING);
    Serial.print(F("X: "));
    Serial.println(homingResultName(r));

    axes.moveToStep(0, STEPS_PER_REV);
    axes.moveToStep(1, STEPS_PER_REV);
}

void loop() {
    axes.update();
}
