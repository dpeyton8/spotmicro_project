"""ROS 2 node: publishes the paper gait as front-left joint targets for the existing MuJoCo + RViz pipeline.

Run from projects/one_leg_testbench with ROS sourced:  python3 -m one_leg_testbench.ros_node
Commands on /one_leg_testbench/command: walk, stop. Targets go to /front_left_leg/joint_targets (radians).
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from . import leg
from .plan import PathError, check_path, joint_targets
from .trajectory import Gait


class TestbenchGait(Node):
    def __init__(self):
        super().__init__("one_leg_testbench_gait")
        self.declare_parameter("hip_height", Gait.hip_height)
        self.declare_parameter("scale", Gait.scale)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.gait = Gait(
            hip_height=self.get_parameter("hip_height").value,
            scale=self.get_parameter("scale").value,
        )
        check_path(self.gait)  # refuse to start on a path that leaves the joint limits

        self.publisher = self.create_publisher(JointState, "/front_left_leg/joint_targets", 10)
        self.create_subscription(String, "/one_leg_testbench/command", self._on_command, 10)
        self.create_timer(1.0 / self.get_parameter("publish_rate_hz").value, self._publish)
        self.walking = False
        self.start = self._now()
        self.hold = joint_targets(self.gait, 0.0)[0]

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == "walk":
            self.walking, self.start = True, self._now()
        elif command == "stop":
            self.walking = False
        else:
            self.get_logger().warning("Command must be walk or stop")

    def _publish(self) -> None:
        q = joint_targets(self.gait, self._now() - self.start)[0] if self.walking else self.hold
        if self.walking:
            self.hold = q
        target = JointState()
        target.header.stamp = self.get_clock().now().to_msg()
        target.name = list(leg.JOINT_NAMES)
        target.position = [float(v) for v in leg.to_ros(q)]
        self.publisher.publish(target)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = TestbenchGait()
    except PathError as error:
        print(f"Refusing to start: {error}")
        rclpy.shutdown()
        return
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
