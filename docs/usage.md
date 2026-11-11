# Usage

## Servos

```cpp
#include "ServoManager.h"
#include "PCA9685Servo.h"

Adafruit_PWMServoDriver pwm;   // SDA -> A4, SCL -> A5 on an Uno
ServoManager arm(6);

void setup() {
    pwm.begin();
    pwm.setPWMFreq(50);        // must run before any PCA9685Servo::begin()

    for (uint8_t ch = 0; ch < 6; ch++)
        arm.addMotor(new PCA9685Servo(pwm, ch, /*defaultAngle*/ 90, /*rotationDelay*/ 10));

    arm.begin();
}

void loop() {
    arm.update();              // call every iteration; movement is non-blocking
}
```

## Steppers

```cpp
#include "StepperManager.h"
#include "StepDirStepper.h"

StepperManager axes(2, /*shared enable pin*/ 8);

void setup() {
    axes.addMotor(new StepDirStepper(/*step*/ 2, /*dir*/ 5, /*stepsPerRev*/ 3200,
                                     StepDirStepper::NO_ENABLE_PIN,
                                     /*stepIntervalUs*/ 1000));              // id 0
    axes.addMotor(new StepDirStepper(3, 6, 3200,
                                     StepDirStepper::NO_ENABLE_PIN, 1000,
                                     StepDirStepper::TB6600_PULSE_US));      // id 1, on a TB6600

    axes.begin();

    axes.moveSteps(0, 400);    // both start now and advance together:
    axes.moveToStep(1, -800);  // update() interleaves their pulses
}

void loop() {
    axes.update();
}
```

