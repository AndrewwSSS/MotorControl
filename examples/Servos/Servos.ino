// A hobby servo on its own pin and one on a PCA9685 channel, in one fleet,
// sweeping between two angles.
//
// Builds for the Uno and the ESP32; scripts/check.sh compiles it for both.
// On the ESP32 the PCA9685 sits on the default I2C pins, SDA 21 and SCL 22.

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <PCA9685Servo.h>
#include <ServoManager.h>
#include <ServoMotor.h>

// GPIO 9 is wired to the flash on an ESP32 module.
#if defined(ESP32)
static constexpr uint8_t SERVO_PIN = 13;
#else
static constexpr uint8_t SERVO_PIN = 9;
#endif
static constexpr uint8_t PCA_CHANNEL = 0;

static constexpr int SWEEP_LOW = 30;
static constexpr int SWEEP_HIGH = 150;

// Shared by every PCA9685Servo on the board.
Adafruit_PWMServoDriver pwm;

ServoManager servos(2);

static void onReached(uint8_t id) {
    int next = (servos.getAngle(id) >= SWEEP_HIGH) ? SWEEP_LOW : SWEEP_HIGH;
    servos.setAngle(id, next);
}

void setup() {
    Serial.begin(115200);

    pwm.begin();
    pwm.setPWMFreq(50);   // standard analog servo frequency

    servos.addMotor(new ServoMotor(SERVO_PIN));
    servos.addMotor(new PCA9685Servo(pwm, PCA_CHANNEL));
    servos.onReached(onReached);
    servos.begin();

    servos.setAngle(0, SWEEP_HIGH);
    servos.setAngle(1, SWEEP_HIGH);
}

void loop() {
    servos.update();
}
