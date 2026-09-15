# MotorControl

Project-private PlatformIO library for the Ruka arm. Everything motor-related
lives here; `src/main.cpp` only holds the sketch (Bluetooth protocol, playback).

## Layout

| File | Role |
| --- | --- |
| `src/Motor.h` | Abstract base: target clamping, non-blocking move loop, state + `reachedTarget()` edge |
| `src/ServoMotor.h` | Backed by the Arduino `Servo` library (one hardware PWM pin per motor) |
| `src/PCA9685Servo.h` | Backed by a shared PCA9685 16-channel PWM driver over I2C |
| `src/Stepper.h` | Abstract stepper base: position in steps from a base position, non-blocking step loop |
| `src/A4988Stepper.h` | Stepper on an A4988/DRV8825 step+dir driver (a CNC Shield socket); owns only the pins |
| `src/HomedStepper.h` | A4988Stepper that finds its base position against an endstop in `begin()` |
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

Steppers do **not** use `Motor`. `Motor` models an absolute, clamped, unsigned
angle written straight to the hardware - a servo. A stepper's natural unit is a
signed, unbounded step count relative to a base position it has to earn, so it
has its own root, `Stepper`. Degrees can be layered on later without touching
it: `stepsPerRevolution` is carried on `Stepper` for that, and a degree class
that also needs to join a `ServoManager` fleet can implement `Motor` itself and
hold a `Stepper` underneath.

To support a different driver chip, subclass `Stepper` and implement
`applyDirection(bool)` and `pulseStep()` - see `A4988Stepper.h`.

## Adding a servo type

Subclass `Motor`, implement `begin()` (hardware setup + drive to the default
angle) and `writeAngle(uint16_t)` (one hardware write). `Motor`'s degree-by-degree
`update()` suits anything that takes an absolute angle command; if it does not
fit, the actuator probably wants its own root rather than an `update()`
override - see `Stepper.h`.
