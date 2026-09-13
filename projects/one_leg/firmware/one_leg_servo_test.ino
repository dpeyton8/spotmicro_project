/*
  SpotMicro one-right-leg manual servo commissioning tool.

  Hardware:
    ESP32
    PCA9685 at I2C address 0x40
    q1/q2/q3 servos on PCA9685 channels 0/1/2

  There is intentionally no automatic sweep. Mount the leg in a fixture and
  calibrate one disconnected horn/servo at a time.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint8_t SERVO_CHANNELS[] = {0, 1, 2};
constexpr uint16_t SAFE_MIN_US = 1100;
constexpr uint16_t SAFE_CENTER_US = 1500;
constexpr uint16_t SAFE_MAX_US = 1900;

bool isLegChannel(int channel) {
  for (uint8_t legChannel : SERVO_CHANNELS) {
    if (channel == legChannel) return true;
  }
  return false;
}

void disableChannel(uint8_t channel) {
  pwm.setPWM(channel, 0, 0);
}

void disableAll() {
  for (uint8_t channel : SERVO_CHANNELS) disableChannel(channel);
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  center <channel>       center channel 0, 1, or 2");
  Serial.println("  set <channel> <usec>   command 1100..1900 microseconds");
  Serial.println("  off <channel>          stop pulses on one channel");
  Serial.println("  offall                 stop pulses on all leg channels");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  disableAll();
  Serial.println("SpotMicro one-leg servo test ready; outputs are OFF.");
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  if (command == "help") {
    printHelp();
    return;
  }
  if (command == "offall") {
    disableAll();
    Serial.println("All leg channels OFF");
    return;
  }

  int firstSpace = command.indexOf(' ');
  if (firstSpace < 0) {
    Serial.println("Invalid command; enter help");
    return;
  }
  String verb = command.substring(0, firstSpace);
  String arguments = command.substring(firstSpace + 1);
  int secondSpace = arguments.indexOf(' ');
  int channel = (secondSpace < 0 ? arguments : arguments.substring(0, secondSpace)).toInt();
  if (!isLegChannel(channel)) {
    Serial.println("Channel must be 0, 1, or 2");
    return;
  }

  if (verb == "off") {
    disableChannel(static_cast<uint8_t>(channel));
    Serial.printf("Channel %d OFF\n", channel);
    return;
  }
  if (verb == "center") {
    pwm.writeMicroseconds(static_cast<uint8_t>(channel), SAFE_CENTER_US);
    Serial.printf("Channel %d = %u us\n", channel, SAFE_CENTER_US);
    return;
  }
  if (verb == "set" && secondSpace >= 0) {
    int pulseUs = arguments.substring(secondSpace + 1).toInt();
    if (pulseUs < SAFE_MIN_US || pulseUs > SAFE_MAX_US) {
      Serial.printf("Rejected: pulse must be %u..%u us\n", SAFE_MIN_US, SAFE_MAX_US);
      return;
    }
    pwm.writeMicroseconds(static_cast<uint8_t>(channel), static_cast<uint16_t>(pulseUs));
    Serial.printf("Channel %d = %d us\n", channel, pulseUs);
    return;
  }

  Serial.println("Invalid command; enter help");
}

