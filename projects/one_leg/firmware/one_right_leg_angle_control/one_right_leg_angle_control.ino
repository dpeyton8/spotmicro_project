/*
  SpotMicro - ESP32 right-front one-leg angle control
  PCA9685 servo driver over I2C

  Install the Adafruit PWM Servo Driver Library before compiling.
  This sketch is for the right-front leg (RF).

  Channel 0 = shoulder/q1, channel 1 = hip/q2, channel 2 = knee/q3.
  The center/range/direction values are RF references from the ROS 2 config,
  not verified MG996R hardware calibration. Keep horns detached for first tests.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685(0x40);

constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint16_t PWM_FREQUENCY_HZ = 50;
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

// RF calibration references: RF_1 shoulder, RF_2 hip, RF_3 knee.
Joint leg[] = {
  {"shoulder/q1", 0, 306, 396, -1, -5.4f},
  {"hip/q2",      1, 306, 389,  1, -27.6f},
  {"knee/q3",     2, 306, 372,  1,  88.2f}
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
  Serial.print(" angle: ");
  Serial.print(angleDeg, 1);
  Serial.print(" degrees, PWM: ");
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

bool parseAngles(const String& input, float& shoulderAngle, float& hipAngle, float& kneeAngle) {
  const int firstComma = input.indexOf(',');
  const int secondComma = input.indexOf(',', firstComma + 1);
  if (firstComma <= 0 || secondComma <= firstComma + 1 || secondComma >= input.length() - 1) {
    return false;
  }
  shoulderAngle = input.substring(0, firstComma).toFloat();
  hipAngle = input.substring(firstComma + 1, secondComma).toFloat();
  kneeAngle = input.substring(secondComma + 1).toFloat();
  return true;
}

void printPrompt() {
  Serial.println("Enter SHOULDER/Q1,HIP/Q2,KNEE/Q3 angles in degrees:");
  Serial.println("RF reference centers: -5.4,-27.6,88.2");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("SpotMicro right-front three-motor leg angle control");
  Serial.println("PCA9685 channels: shoulder=0, hip=1, knee=2");
  pca9685.begin();
  pca9685.setOscillatorFrequency(27000000);
  pca9685.setPWMFreq(PWM_FREQUENCY_HZ);
  delay(10);
  moveLegToNeutral();
  printPrompt();
}

void loop() {
  if (!Serial.available()) return;

  String inputResult = Serial.readStringUntil('\n');
  inputResult.trim();
  float shoulderAngle;
  float hipAngle;
  float kneeAngle;

  if (!parseAngles(inputResult, shoulderAngle, hipAngle, kneeAngle)) {
    Serial.println("Invalid input. Use: shoulder,hip,knee");
    printPrompt();
    return;
  }

  moveJoint(leg[0], shoulderAngle, 20);
  moveJoint(leg[1], hipAngle, 20);
  moveJoint(leg[2], kneeAngle, 20);
  printPrompt();
}
