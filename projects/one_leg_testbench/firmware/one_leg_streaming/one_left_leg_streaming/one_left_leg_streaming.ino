/*
  SpotMicro LEFT-leg ROS serial-bridge firmware.

  PCA9685 address 0x40:
    channel 0 = left shoulder/q1
    channel 1 = left upper leg/hip/q2
    channel 2 = left lower leg/knee/q3

  Protocol: set <channel> <usec>, off <channel>, offall.
  Boot behavior: all outputs OFF; joint targets come from the ROS bridge.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint8_t SERVO_CHANNELS[] = {0, 1, 2};
constexpr uint16_t COMMISSIONING_MIN_US = 500;
constexpr uint16_t CENTER_US = 1500;
constexpr uint16_t COMMISSIONING_MAX_US = 2500;

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
  Serial.println("LEFT LEG: ch0 shoulder/q1, ch1 upper/hip/q2, ch2 lower/knee/q3");
  Serial.println("Commands:");
  Serial.println("  center <channel>       command 1500 us on channel 0, 1, or 2");
  Serial.println("  set <channel> <usec>   command 500..2500 us (commissioning)");
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
  Serial.println("SpotMicro LEFT-leg streaming bridge ready; outputs are OFF.");
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
    pwm.writeMicroseconds(static_cast<uint8_t>(channel), CENTER_US);
    Serial.printf("Channel %d = %u us\n", channel, CENTER_US);
    return;
  }
  if (verb == "set" && secondSpace >= 0) {
    int pulseUs = arguments.substring(secondSpace + 1).toInt();
    if (pulseUs < COMMISSIONING_MIN_US || pulseUs > COMMISSIONING_MAX_US) {
      Serial.printf("Rejected: pulse must be %u..%u us\n", COMMISSIONING_MIN_US, COMMISSIONING_MAX_US);
      return;
    }
    pwm.writeMicroseconds(static_cast<uint8_t>(channel), static_cast<uint16_t>(pulseUs));
    Serial.printf("Channel %d = %d us\n", channel, pulseUs);
    return;
  }

  Serial.println("Invalid command; enter help");
}
