#!/usr/bin/env python3
"""Forward the one-leg testbench joint angles to the ESP32/PCA9685 over USB serial.

Subscribes to /joint_states (published by the testbench with --ros), so the physical
leg follows whatever the sim does: stand, sit, repo/zhang walk and angle commands.
Protocols:
  usec    (default) "set <channel> <usec>" for the required bridge sketch
          firmware/one_leg_streaming/one_left_leg_streaming/one_left_leg_streaming.ino. Valid
          testbench angles are mapped across the broad 500..2500 us commissioning
          envelope; determine narrower per-joint hardware limits before unattended use.
  degrees "shoulder,hip,knee\\n" for the alternative (untested)
          one_left_leg_angle_streaming.ino, which does not support motor release.

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
from std_msgs.msg import Bool
from std_msgs.msg import String

# joint name, PCA9685 channel, ROS sign, center angle (kinematic deg), direction
JOINT_TABLE = (
    ("front_left_shoulder", 0, -1.0, -7.6, 1),
    ("front_left_leg", 1, 1.0, 38.6, 1),
    ("front_left_foot", 2, 1.0, -82.8, 1),
)
JOINTS = tuple(row[0] for row in JOINT_TABLE)
ROS_SIGN = tuple(row[2] for row in JOINT_TABLE)  # ROS angle = ROS_SIGN * kinematic angle
CENTER_US = 1500
# Broad commissioning envelope. The simulator/firmware joint-angle limits remain the
# primary command limits. These bounds only reject malformed/out-of-envelope serial
# values and must be replaced by measured per-joint hardware limits after calibration.
COMMISSIONING_MIN_US, COMMISSIONING_MAX_US = 500, 2500
# Old sketch: 389 PWM counts (4.883 us each at 50 Hz) span 165 degrees.
US_PER_DEG = 389 * (20000.0 / 4096.0) / 165.0
SHAKE_TARGET_DEG = (-7.6, -43.9, -60.0)
SHAKE_TOLERANCE_DEG = 1.0
SHAKE_SETTLE_SECONDS = 1.0
SHAKE_RELEASE_TIMEOUT_SECONDS = 4.0
MOTOR_CHANNELS = {"q1": 0, "q2": 1, "q3": 2}


def angle_to_us(angle_deg, center_deg, direction):
    pulse = CENTER_US + direction * (angle_deg - center_deg) * US_PER_DEG
    return int(round(min(max(pulse, COMMISSIONING_MIN_US), COMMISSIONING_MAX_US)))


class Esp32Bridge(Node):
    def __init__(self, port, baud, rate_hz, enable, protocol):
        super().__init__("esp32_serial_bridge")
        self.enable = enable
        self.protocol = protocol
        self.period = 1.0 / rate_hz
        self.last_sent = 0.0
        self.last_lines = None
        self.enabled_channels = set(MOTOR_CHANNELS.values())
        self.release_after_shake = False
        self.release_deadline = None
        self.release_timeout = None
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
        self.create_subscription(String, "/one_leg_testbench/command", self.on_command, 10)
        self.create_subscription(String, "/front_left_leg/angle_command", self.on_angle_command, 10)
        for topic in ("/stand_cmd", "/idle_cmd", "/walk_cmd", "/cometyang_walk_cmd"):
            self.create_subscription(Bool, topic, self.on_motion_bool, 10)
        self.create_subscription(
            String, "/one_leg_testbench/motor_command", self.on_motor_command, 10
        )

    def write_serial_command(self, command):
        if self.enable and self.serial is not None:
            self.serial.write((command + "\n").encode())
            self.serial.flush()
        else:
            self.get_logger().info("Dry run: " + command)

    def enable_all_outputs(self, reason):
        self.enabled_channels = set(MOTOR_CHANNELS.values())
        self.release_after_shake = False
        self.release_deadline = None
        self.release_timeout = None
        self.last_lines = None
        self.get_logger().info(f"All motor outputs enabled for {reason}")

    def on_motion_bool(self, msg):
        if msg.data:
            self.enable_all_outputs("motion command")

    def on_angle_command(self, _msg):
        self.enable_all_outputs("direct angle command")

    def on_motor_command(self, msg):
        command = msg.data.strip().lower()
        if self.protocol != "usec":
            self.get_logger().warning("Motor enable/disable commands require usec protocol")
            return
        if command == "all_off":
            self.write_serial_command("offall")
            self.enabled_channels.clear()
        elif command == "all_on":
            self.enabled_channels = set(MOTOR_CHANNELS.values())
            self.last_lines = None
        elif command.endswith("_off") and command[:-4] in MOTOR_CHANNELS:
            channel = MOTOR_CHANNELS[command[:-4]]
            self.write_serial_command(f"off {channel}")
            self.enabled_channels.discard(channel)
        elif command.endswith("_on") and command[:-3] in MOTOR_CHANNELS:
            channel = MOTOR_CHANNELS[command[:-3]]
            self.enabled_channels.add(channel)
            self.last_lines = None
        elif command == "status":
            enabled = [name for name, channel in MOTOR_CHANNELS.items() if channel in self.enabled_channels]
            disabled = [name for name, channel in MOTOR_CHANNELS.items() if channel not in self.enabled_channels]
            self.get_logger().info(
                f"Motor outputs enabled={enabled or ['none']} disabled={disabled or ['none']}"
            )
            return
        else:
            self.get_logger().warning(
                "Motor command must be q1_on/off, q2_on/off, q3_on/off, all_on/off, or status"
            )
            return
        self.release_after_shake = False
        self.release_deadline = None
        self.release_timeout = None
        self.get_logger().info(
            f"Motor command {command}; enabled channels={sorted(self.enabled_channels)}"
        )

    def on_command(self, msg):
        command = msg.data.strip().lower()
        if command == "shake":
            if self.protocol != "usec":
                self.get_logger().warning("shake release requires the usec protocol")
                return
            self.enabled_channels = set(MOTOR_CHANNELS.values())
            self.release_after_shake = True
            self.release_deadline = None
            self.release_timeout = time.monotonic() + SHAKE_RELEASE_TIMEOUT_SECONDS
            self.last_lines = None
            self.get_logger().info(
                "Shake armed; outputs will release after settling or after 4.0 s"
            )
        elif command == "shake_off":
            self.enable_all_outputs("shake_off")
        elif command in ("walk", "stop"):
            self.enable_all_outputs(command)

    def joint_degrees(self, positions):
        return [math.degrees(positions[row[0]]) * row[2] for row in JOINT_TABLE]

    def build_lines(self, positions, degrees=None):
        degrees = self.joint_degrees(positions) if degrees is None else degrees
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
        degrees = self.joint_degrees(positions)

        if self.release_after_shake:
            at_target = max(
                abs(actual - target)
                for actual, target in zip(degrees, SHAKE_TARGET_DEG)
            ) <= SHAKE_TOLERANCE_DEG
            if at_target:
                if self.release_deadline is None:
                    self.release_deadline = now + SHAKE_SETTLE_SECONDS
                    self.get_logger().info("Shake pose reached; releasing outputs in 1.0 s")
            timed_out = self.release_timeout is not None and now >= self.release_timeout
            settled = self.release_deadline is not None and now >= self.release_deadline
            if settled or timed_out:
                # Keep q1/shoulder (channel 0) and q2/hip (channel 1) powered
                # at the shake angle while releasing q3/knee for a compliant paw.
                self.write_serial_command("off 2")
                self.enabled_channels = {
                    MOTOR_CHANNELS["q1"],
                    MOTOR_CHANNELS["q2"],
                }
                self.release_after_shake = False
                self.release_deadline = None
                self.release_timeout = None
                self.last_lines = None
                reason = "settled" if settled else "timeout"
                self.get_logger().info(
                    f"Shake damping active ({reason}): q1/q2 powered; q3 pulse is OFF"
                )
                return
            if not at_target:
                self.release_deadline = None

        if not self.enabled_channels:
            return
        lines = self.build_lines(positions, degrees)
        enabled_lines = [
            line for row, line in zip(JOINT_TABLE, lines) if row[1] in self.enabled_channels
        ]
        if lines == self.last_lines:
            return
        self.last_sent = now
        self.last_lines = lines
        if not self.enable:
            self.get_logger().info(" | ".join(enabled_lines))
            return
        self.serial.write(("\n".join(enabled_lines) + "\n").encode())
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
