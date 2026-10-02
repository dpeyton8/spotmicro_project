/*
  SpotMicro - ESP32 control for one leg
  PCA9685 servo driver over I2C

  Install the Adafruit PWM Servo Driver Library before compiling.
  This sketch defaults to the left-front leg (LF).

  Channel 0 = shoulder/q1, channel 1 = hip/q2, channel 2 = knee/q3.
  GPIO 21 and GPIO 22 remain the ESP32 I2C SDA and SCL pins.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685(0x40);

constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint16_t PWM_FREQUENCY_HZ = 50;

// SpotMicro calibrated servo limit and PCA9685 values.
constexpr float MAX_JOINT_ANGLE_DEG = 82.5f;
constexpr uint16_t PWM_MIN = 80;
constexpr uint16_t PWM_MAX = 520;

struct Joint {
  const char* name;
  uint8_t channel;
  int16_t centerPwm;
  int16_t rangePwm;
  int8_t direction;
  float centerAngleDeg;
};

// Standard leg order: channel 0 = shoulder/q1, 1 = hip/q2, 2 = knee/q3.
// Calibration values below are for the left-front leg (LF).
Joint leg[] = {
  {"shoulder/q1", 0, 306, 389, 1, -7.6f},
  {"hip/q2",      1, 306, 397, 1, 38.6f},
  {"knee/q3",     2, 306, 387, 1, -82.8f}
};

constexpr size_t JOINT_COUNT = sizeof(leg) / sizeof(leg[0]);

uint16_t angleToPwm(const Joint& joint, float angleDeg) {
  angleDeg = constrain(angleDeg,
                       joint.centerAngleDeg - MAX_JOINT_ANGLE_DEG,
                       joint.centerAngleDeg + MAX_JOINT_ANGLE_DEG);

  const float pwm = joint.centerPwm +
                    joint.direction * (angleDeg - joint.centerAngleDeg) *
                    joint.rangePwm / (2.0f * MAX_JOINT_ANGLE_DEG);
  return constrain(static_cast<int>(lroundf(pwm)), PWM_MIN, PWM_MAX);
}

void writeJoint(const Joint& joint, float angleDeg) {
  const uint16_t pwm = angleToPwm(joint, angleDeg);
  pca9685.setPWM(joint.channel, 0, pwm);

  Serial.print(joint.name);
  Serial.print(" angle=");
  Serial.print(angleDeg, 1);
  Serial.print(" pwm=");
  Serial.println(pwm);
}

void moveJoint(const Joint& joint, float targetAngleDeg, uint16_t stepDelayMs) {
  const float startAngleDeg = joint.centerAngleDeg;
  const int step = targetAngleDeg >= startAngleDeg ? 1 : -1;

  for (float angleDeg = startAngleDeg;
       (step > 0 && angleDeg <= targetAngleDeg) ||
       (step < 0 && angleDeg >= targetAngleDeg);
       angleDeg += step) {
    writeJoint(joint, angleDeg);
    delay(stepDelayMs);
  }
  writeJoint(joint, targetAngleDeg);
}

void moveLegToNeutral() {
  for (size_t index = 0; index < JOINT_COUNT; ++index) {
    writeJoint(leg[index], leg[index].centerAngleDeg);
  }
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("SpotMicro one-leg servo test");
  Serial.println("WARNING: keep the leg unloaded and verify channel order first.");

  pca9685.begin();
  pca9685.setOscillatorFrequency(27000000);
  pca9685.setPWMFreq(PWM_FREQUENCY_HZ);
  delay(10);

  moveLegToNeutral();
}

void loop() {
  // Test one joint at a time so the leg is easier to observe and stop.
  for (size_t index = 0; index < JOINT_COUNT; ++index) {
    moveJoint(leg[index], leg[index].centerAngleDeg + 25.0f, 20);
    delay(300);
    moveJoint(leg[index], leg[index].centerAngleDeg - 25.0f, 20);
    delay(300);
    moveJoint(leg[index], leg[index].centerAngleDeg, 20);
    delay(500);
  }

  delay(1000);
}
