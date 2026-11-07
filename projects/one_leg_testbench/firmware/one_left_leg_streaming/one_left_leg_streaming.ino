// Streaming version of one_left_leg_angle_control for the ROS serial bridge.
// Accepts "shoulder,hip,knee\n" (degrees) at any rate, slews smoothly toward the
// latest target without blocking, and clamps to the calibrated joint range.
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685(0x40);

constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint16_t PWM_FREQUENCY_HZ = 50;
constexpr float MAX_JOINT_ANGLE_DEG = 82.5f;
constexpr uint16_t PWM_MIN = 80;
constexpr uint16_t PWM_MAX = 520;
constexpr float MAX_STEP_DEG = 1.0f;         // per update
constexpr uint32_t UPDATE_INTERVAL_MS = 10;  // 100 deg/s max slew

struct Joint {
  uint8_t channel;
  int16_t centerPwm;
  int16_t rangePwm;
  int8_t direction;
  float centerAngleDeg;
  float currentDeg;
  float targetDeg;
};

// Channel 0 = shoulder/q1, 1 = hip/q2, 2 = knee/q3 (left-front leg calibration).
Joint leg[] = {
  {0, 306, 389, 1,  -7.6f,  -7.6f,  -7.6f},
  {1, 306, 397, 1,  38.6f,  38.6f,  38.6f},
  {2, 306, 387, 1, -82.8f, -82.8f, -82.8f}
};
constexpr size_t JOINT_COUNT = sizeof(leg) / sizeof(leg[0]);

uint16_t angleToPwm(const Joint& joint, float angleDeg) {
  const float pwm = joint.centerPwm +
                    joint.direction * (angleDeg - joint.centerAngleDeg) *
                    joint.rangePwm / (2.0f * MAX_JOINT_ANGLE_DEG);
  return constrain(static_cast<int>(lroundf(pwm)), PWM_MIN, PWM_MAX);
}

void writeJoint(const Joint& joint) {
  pca9685.setPWM(joint.channel, 0, angleToPwm(joint, joint.currentDeg));
}

bool parseAngles(const String& input, float out[3]) {
  const int first = input.indexOf(',');
  const int second = input.indexOf(',', first + 1);
  if (first <= 0 || second <= first + 1 || second >= input.length() - 1) return false;
  out[0] = input.substring(0, first).toFloat();
  out[1] = input.substring(first + 1, second).toFloat();
  out[2] = input.substring(second + 1).toFloat();
  return true;
}

String line;
uint32_t lastUpdateMs = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  pca9685.begin();
  pca9685.setOscillatorFrequency(27000000);
  pca9685.setPWMFreq(PWM_FREQUENCY_HZ);
  delay(10);
  for (size_t i = 0; i < JOINT_COUNT; i++) writeJoint(leg[i]);
  Serial.println("READY shoulder,hip,knee");
}

void loop() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c != '\n') {
      if (line.length() < 64) line += c;
      continue;
    }
    line.trim();
    float angles[3];
    if (parseAngles(line, angles)) {
      for (size_t i = 0; i < JOINT_COUNT; i++) {
        leg[i].targetDeg = constrain(angles[i],
                                     leg[i].centerAngleDeg - MAX_JOINT_ANGLE_DEG,
                                     leg[i].centerAngleDeg + MAX_JOINT_ANGLE_DEG);
      }
    } else {
      Serial.println("ERR use shoulder,hip,knee");
    }
    line = "";
  }

  const uint32_t now = millis();
  if (now - lastUpdateMs < UPDATE_INTERVAL_MS) return;
  lastUpdateMs = now;
  for (size_t i = 0; i < JOINT_COUNT; i++) {
    const float error = leg[i].targetDeg - leg[i].currentDeg;
    if (fabsf(error) < 0.01f) continue;
    leg[i].currentDeg += constrain(error, -MAX_STEP_DEG, MAX_STEP_DEG);
    writeJoint(leg[i]);
  }
}
