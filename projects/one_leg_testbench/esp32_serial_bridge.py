#!/usr/bin/env python3
"""Forward the one-leg testbench joint angles to the ESP32/PCA9685 over USB serial.

Subscribes to /joint_states (published by the testbench with --ros), so the physical
leg follows whatever the sim does: stand, sit, repo/zhang walk and angle commands.
Protocols:
  usec    (default) "set <channel> <usec>" for the commissioning sketch
          one_left_leg_servo_test.ino; no reflash needed. Pulses are clamped to 1100..1900 us.
  degrees "shoulder,hip,knee\\n" for the (untested) one_left_leg_streaming.ino.

SAFE BY DEFAULT: nothing is written to the serial port unless --enable is given;
without it the bridge only prints what it would send. Servo calibration below is derived
from the old PWM-count sketch and is UNVERIFIED on hardware.

  python3 esp32_serial_bridge.py                          # print only
  python3 esp32_serial_bridge.py --port /dev/ttyS0 --enable
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# joint name, PCA9685 channel, ROS sign, center angle (kinematic deg), direction
JOINT_TABLE = (
    ("front_left_shoulder", 0, -1.0, -7.6, 1),
    ("front_left_leg", 1, 1.0, 38.6, 1),
    ("front_left_foot", 2, 1.0, -82.8, 1),
)
JOINTS = tuple(row[0] for row in JOINT_TABLE)
ROS_SIGN = tuple(row[2] for row in JOINT_TABLE)  # ROS angle = ROS_SIGN * kinematic angle
CENTER_US = 1500
SAFE_MIN_US, SAFE_MAX_US = 1100, 1900
# Old sketch: 389 PWM counts (4.883 us each at 50 Hz) span 165 degrees.
US_PER_DEG = 389 * (20000.0 / 4096.0) / 165.0


def angle_to_us(angle_deg, center_deg, direction):
    pulse = CENTER_US + direction * (angle_deg - center_deg) * US_PER_DEG
    return int(round(min(max(pulse, SAFE_MIN_US), SAFE_MAX_US)))


class Esp32Bridge(Node):
    def __init__(self, port, baud, rate_hz, enable, protocol):
        super().__init__("esp32_serial_bridge")
        self.enable = enable
        self.protocol = protocol
        self.period = 1.0 / rate_hz
        self.last_sent = 0.0
        self.last_lines = None
        self.serial = None
        if enable:
            import serial

            self.serial = serial.Serial(port, baud, timeout=0.01)
            time.sleep(2.0)  # ESP32 may reset when the port opens
            self.serial.write(b"offall\n")
            self.get_logger().info(f"Connected to {port} @ {baud}, protocol={protocol}")
        else:
            self.get_logger().warn("Dry run: add --enable to write to the serial port")
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)

    def build_lines(self, positions):
        degrees = [math.degrees(positions[row[0]]) * row[2] for row in JOINT_TABLE]
        if self.protocol == "degrees":
            return [",".join(f"{value:.1f}" for value in degrees)]
        return [
            f"set {row[1]} {angle_to_us(deg, row[3], row[4])}"
            for row, deg in zip(JOINT_TABLE, degrees)
        ]

    def on_joint_state(self, msg):
        now = time.monotonic()
        if now - self.last_sent < self.period:
            return
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in JOINTS):
            return
        lines = self.build_lines(positions)
        if lines == self.last_lines:
            return
        self.last_sent = now
        self.last_lines = lines
        if not self.enable:
            self.get_logger().info(" | ".join(lines))
            return
        self.serial.write(("\n".join(lines) + "\n").encode())
        time.sleep(0.02)
        reply = self.serial.read(self.serial.in_waiting or 1).decode(errors="replace").strip()
        if reply:
            self.get_logger().info("ESP32: " + reply.replace("\r\n", " | "))

    def shutdown_outputs(self):
        if self.serial is not None and self.protocol == "usec":
            self.serial.write(b"offall\n")
            self.serial.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyS0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=10.0, help="max send rate in Hz")
    parser.add_argument("--protocol", choices=("usec", "degrees"), default="usec")
    parser.add_argument("--enable", action="store_true", help="actually write to the serial port")
    args = parser.parse_args()
    rclpy.init()
    node = Esp32Bridge(args.port, args.baud, args.rate, args.enable, args.protocol)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
