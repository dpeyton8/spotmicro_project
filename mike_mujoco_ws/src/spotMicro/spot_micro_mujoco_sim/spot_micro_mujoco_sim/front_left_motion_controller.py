#!/usr/bin/env python3
"""Publish walk and pose targets for only the full robot's front-left leg."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


JOINT_NAMES = (
    "front_left_shoulder",
    "front_left_leg",
    "front_left_foot",
)
STAND = (0.0, 0.0, 0.0)
SIT = (0.0, -0.65, -1.35)
WALK_CYCLE = (
    (0.00, (0.0, 0.12, -0.25)),
    (0.25, (0.0, 0.55, -1.10)),
    (0.50, (0.0, 0.18, -0.45)),
    (0.75, (0.0, -0.25, -0.80)),
)


class FrontLeftMotionController(Node):
    def __init__(self):
        super().__init__("front_left_motion_controller")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("transition_seconds", 1.0)
        self.declare_parameter("walk_cycle_seconds", 1.6)

        self.publisher = self.create_publisher(
            JointState, "/front_left_leg/joint_targets", 10
        )
        self.subscription = self.create_subscription(
            String, "/front_left_leg/command", self._command_callback, 10
        )

        self.mode = "hold"
        self.current_targets = STAND
        self.transition_start = STAND
        self.transition_target = STAND
        self.mode_start_time = self._now_seconds()
        self.timer = self.create_timer(
            1.0 / self.get_parameter("publish_rate_hz").value,
            self._publish_targets,
        )
        self.get_logger().info(
            "Commands: walk (front-left steps in place), sit, stand, stop"
        )

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _command_callback(self, msg):
        command = msg.data.strip().lower()
        if command == "walk":
            self.mode = "walk"
            self.mode_start_time = self._now_seconds()
        elif command in ("sit", "stand"):
            self.mode = "transition"
            self.transition_start = self.current_targets
            self.transition_target = SIT if command == "sit" else STAND
            self.mode_start_time = self._now_seconds()
        elif command == "stop":
            self.mode = "hold"
        else:
            self.get_logger().warning("Command must be walk, sit, stand, or stop")
            return
        self.get_logger().info(f"Front-left leg command: {command}")

    @staticmethod
    def _smoothstep(value):
        value = min(max(value, 0.0), 1.0)
        return value * value * (3.0 - 2.0 * value)

    def _walk_targets(self, now):
        cycle = self.get_parameter("walk_cycle_seconds").value
        phase = ((now - self.mode_start_time) / cycle) % 1.0
        for index, (phase_start, pose) in enumerate(WALK_CYCLE):
            next_phase, next_pose = WALK_CYCLE[(index + 1) % len(WALK_CYCLE)]
            if next_phase <= phase_start:
                next_phase += 1.0
            adjusted_phase = phase + (1.0 if phase < phase_start else 0.0)
            if phase_start <= adjusted_phase < next_phase:
                blend = (adjusted_phase - phase_start) / (next_phase - phase_start)
                blend = self._smoothstep(blend)
                return tuple(
                    start + blend * (end - start)
                    for start, end in zip(pose, next_pose)
                )
        return WALK_CYCLE[0][1]

    def _publish_targets(self):
        now = self._now_seconds()
        if self.mode == "walk":
            self.current_targets = self._walk_targets(now)
        elif self.mode == "transition":
            duration = self.get_parameter("transition_seconds").value
            blend = 1.0 if duration <= 0 else self._smoothstep(
                (now - self.mode_start_time) / duration
            )
            self.current_targets = tuple(
                start + blend * (end - start)
                for start, end in zip(self.transition_start, self.transition_target)
            )
            if blend >= 1.0:
                self.mode = "hold"

        target = JointState()
        target.header.stamp = self.get_clock().now().to_msg()
        target.name = list(JOINT_NAMES)
        target.position = list(self.current_targets)
        self.publisher.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = FrontLeftMotionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()