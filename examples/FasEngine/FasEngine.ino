// Two axes on the ESP32 whose pulses come from FastAccelStepper, in hardware:
// the swing stays smooth however long loop() is held up. Each axis owns its
// engine through EngineStepper; the ramp is the same LinearRamp a polled axis
// uses, which FasEngine hands the library as one acceleration.
//
// ESP32 only, and needs gin66/FastAccelStepper@1.4.0 in lib_deps.
// scripts/check.sh compiles it against the real library.

#include <Arduino.h>
#include <EngineStepper.h>
#include <FasEngine.h>
#include <LinearRamp.h>
#include <StepperManager.h>

static constexpr uint8_t X_STEP = 25, X_DIR = 26;
static constexpr uint8_t Y_STEP = 27, Y_DIR = 33;
static constexpr uint8_t EN_PIN = 4;

// 1.8 deg motors at 1/16 microstepping.
static constexpr uint16_t STEPS_PER_REV = 200 * 16;

LinearRamp ramp(1000, 1000);   // 12,000 steps/s^2 at a 200 us cruise

StepperManager axes(2, EN_PIN);

static void onReached(uint8_t id) {
    long next = (axes.getStep(id) > 0) ? 0 : STEPS_PER_REV;
    axes.moveToStep(id, next);
}

void setup() {
    Serial.begin(115200);

    // The fleet owns EN, so the engines are built without one.
    axes.addMotor(new EngineStepper<FasEngine>(STEPS_PER_REV, X_STEP, X_DIR, 200UL));
    axes.addMotor(new EngineStepper<FasEngine>(STEPS_PER_REV, Y_STEP, Y_DIR, 200UL));

    axes.setProfiles(&ramp);
    axes.onReached(onReached);
    axes.begin();

    axes.moveToStep(0, STEPS_PER_REV);
    axes.moveToStep(1, STEPS_PER_REV);
}

void loop() {
    axes.update();
}
