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

// Three motors connected to PCA9685 channels 0, 1, and 2.
Joint leg[] = {
  {"knee",     0, 306, 387, 1, -82.8f},
  {"shoulder", 1, 306, 397, 1,  38.6f},
  {"hip",      2, 306, 389, 1,  -7.6f}
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
  for (size_t index = 0; index < JOINT_COUNT; index++) {
    writeJoint(leg[index], leg[index].centerAngleDeg);
  }
  delay(1000);
}

bool parseAngles(const String& input, float& kneeAngle, float& shoulderAngle, float& hipAngle) {
  const int firstComma = input.indexOf(',');
  const int secondComma = input.indexOf(',', firstComma + 1);
  if (firstComma <= 0 || secondComma <= firstComma + 1 || secondComma >= input.length() - 1) {
    return false;
  }
  kneeAngle = input.substring(0, firstComma).toFloat();
  shoulderAngle = input.substring(firstComma + 1, secondComma).toFloat();
  hipAngle = input.substring(secondComma + 1).toFloat();
  return true;
}

void printPrompt() {
  Serial.println("Enter KNEE,SHOULDER,HIP angles in degrees:");
  Serial.println("Example: -82.8,38.6,-7.6");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("SpotMicro three-motor leg angle control");
  Serial.println("PCA9685 channels: 0, 1, 2");
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
  float kneeAngle;
  float shoulderAngle;
  float hipAngle;

  if (!parseAngles(inputResult, kneeAngle, shoulderAngle, hipAngle)) {
    Serial.println("Invalid input. Use: knee,shoulder,hip");
    printPrompt();
    return;
  }

  moveJoint(leg[0], kneeAngle, 20);
  moveJoint(leg[1], shoulderAngle, 20);
  moveJoint(leg[2], hipAngle, 20);
  printPrompt();
}
