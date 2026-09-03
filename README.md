# MotorControl

Project-private PlatformIO library for the Ruka arm. Everything motor-related
lives here; `src/main.cpp` only holds the sketch (Bluetooth protocol, playback).

## Layout

| File | Role |
| --- | --- |
| `src/Motor.h` | Abstract base: target clamping, non-blocking move loop, state + `reachedTarget()` edge |
| `src/ServoMotor.h` | Backed by the Arduino `Servo` library (one hardware PWM pin per motor) |
| `src/PCA9685Servo.h` | Backed by a shared PCA9685 16-channel PWM driver over I2C |
| `src/StepMotor.h` | Stepper driven through an A4988 step/dir driver |
| `src/ServoManager.h` | Owns a fixed-size fleet of `Motor*`, addressed by index |

Header-only: there are no `.cpp` files, so nothing is compiled unless a
translation unit includes it.

## Usage

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

## Adding a motor type

Subclass `Motor`, implement `begin()` (hardware setup + drive to the default
angle) and `writeAngle(uint16_t)` (one hardware write). Override `update()`
only if degree-by-degree stepping is the wrong movement strategy - see
`StepMotor.h`.
