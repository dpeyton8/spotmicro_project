"""Runs the paper's foot path on the left-leg MuJoCo rig: hold above ground, lower to the floor, then walk in place."""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from . import leg
from .plan import PathError, joint_targets
from .trajectory import Gait

MODEL = Path(__file__).resolve().parent / "model" / "left_leg_rig.xml"
COMETYANG_JOINT_NAMES = (
    "front_right_shoulder", "front_right_leg", "front_right_foot",
    "rear_right_shoulder", "rear_right_leg", "rear_right_foot",
    *leg.JOINT_NAMES,
    "rear_left_shoulder", "rear_left_leg", "rear_left_foot",
)
COMETYANG_TRANSIT_DT = 0.02
COMETYANG_TRANSIT_TAU = 0.3
COMETYANG_TRANSIT_RATE = 0.06
COMETYANG_ALPHA = COMETYANG_TRANSIT_DT / (COMETYANG_TRANSIT_TAU + COMETYANG_TRANSIT_DT)
COMETYANG_LF_HIP_X = 0.093
COMETYANG_LF_HIP_Y = 0.039
COMETYANG_STAND_MOUNT = np.array([0.0, 0.0, 0.155 + leg.TOE_RADIUS])
COMETYANG_SIT_MOUNT = np.array([-0.050, 0.0, 0.083 + leg.TOE_RADIUS])


def rate_limited_transition_step(current, target):
    filtered = (1.0 - COMETYANG_ALPHA) * current + COMETYANG_ALPHA * target
    max_delta = COMETYANG_TRANSIT_RATE * COMETYANG_TRANSIT_DT
    return current + np.clip(filtered - current, -max_delta, max_delta)


class RosJointStatePublisher:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool
        from tf2_ros import TransformBroadcaster

        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("one_leg_testbench_mujoco")
        self.publisher = self.node.create_publisher(JointState, "/joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.transform_stamped_type = TransformStamped
        self.node.create_subscription(Bool, "/stand_cmd", self._stand, 10)
        self.node.create_subscription(Bool, "/idle_cmd", self._sit, 10)
        self.node.create_subscription(Bool, "/walk_cmd", self._walk, 10)
        self.joint_state_type = JointState
        self.joint_indices = {name: index for index, name in enumerate(COMETYANG_JOINT_NAMES)}
        self.mode = "stand"

    def _set_mode(self, mode):
        self.mode = mode

    def _stand(self, message):
        if message.data:
            self._set_mode("stand")

    def _sit(self, message):
        if message.data:
            self._set_mode("sit")

    def _walk(self, message):
        if message.data:
            self._set_mode("walk")

    def spin_once(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def publish(self, front_left_positions, mount_position):
        message = self.joint_state_type()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.name = list(COMETYANG_JOINT_NAMES)
        message.position = [0.0] * len(COMETYANG_JOINT_NAMES)
        for name, position in zip(leg.JOINT_NAMES, front_left_positions):
            message.position[self.joint_indices[name]] = float(position)
        self.publisher.publish(message)

        transform = self.transform_stamped_type()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = float(mount_position[0] - COMETYANG_LF_HIP_X)
        transform.transform.translation.y = float(mount_position[1] - COMETYANG_LF_HIP_Y)
        transform.transform.translation.z = float(mount_position[2])
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def contact_force(model, data, toe_id: int, floor_id: int) -> tuple[bool, float]:
    wrench = np.zeros(6)
    total, touching = 0.0, False
    for i in range(data.ncon):
        pair = {int(data.contact[i].geom1), int(data.contact[i].geom2)}
        if pair == {toe_id, floor_id}:
            touching = True
            mujoco.mj_contactForce(model, data, i, wrench)
            total += max(0.0, float(wrench[0]))
    return touching, total


def smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def run(
    gait: Gait,
    drop: float,
    lower_seconds: float,
    hold_seconds: float,
    cycles: float,
    viewer: bool,
    ros_publisher: RosJointStatePublisher | None = None,
):
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    toe = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_toe")
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "toe_site")
    mount_touch = gait.hip_height + leg.TOE_RADIUS
    mount_start = mount_touch + drop
    total = hold_seconds + lower_seconds + cycles * gait.cycle_seconds

    q0, _, _ = joint_targets(gait, 0.0)
    mount_position = np.array([0.0, 0.0, mount_start])
    if ros_publisher is not None:
        from .poses import POSES

        q0 = np.radians(POSES["stand"])
        mount_position = COMETYANG_STAND_MOUNT.copy()
    data.qpos[:3] = leg.to_ros(q0)
    data.ctrl[:3] = leg.to_ros(q0)
    data.mocap_pos[0] = mount_position
    mujoco.mj_forward(model, data)
    planted_toe_world = None
    if ros_publisher is not None:
        planted_toe_world = np.array(data.site_xpos[site], dtype=float).copy()
        ros_publisher.publish(data.qpos[:3], mount_position)

    handle = None
    if viewer:
        handle = mujoco.viewer.launch_passive(model, data)
    rows = []
    wall0 = time.perf_counter()
    viewer_sync_period = 1.0 / 30.0
    next_viewer_sync = 0.0
    next_ros_publish = 0.0
    next_ros_control = 0.0
    walk_started = False
    walk_start_time = 0.0
    ros_q = q0.copy()
    ros_point = leg.forward(q0)
    ros_phase = "stand"
    try:
        while ros_publisher is not None or data.time < total:
            if handle is not None and not handle.is_running():
                break
            if ros_publisher is None:
                gait_t = max(0.0, data.time - hold_seconds - lower_seconds)
                mount_z = mount_start - drop * smoothstep((data.time - hold_seconds) / lower_seconds)
                q, point, phase = joint_targets(gait, gait_t)
            else:
                ros_publisher.spin_once()
                if data.time >= next_ros_control:
                    if ros_publisher.mode == "walk":
                        mount_target = COMETYANG_STAND_MOUNT
                    elif ros_publisher.mode == "sit":
                        mount_target = COMETYANG_SIT_MOUNT
                        walk_started = False
                    else:
                        mount_target = COMETYANG_STAND_MOUNT
                        walk_started = False

                    mount_position = rate_limited_transition_step(mount_position, mount_target)
                    data.mocap_pos[0] = mount_position
                    mount_z = float(mount_position[2])

                    if ros_publisher.mode == "walk" and np.allclose(
                        mount_position, COMETYANG_STAND_MOUNT, atol=0.001
                    ):
                        if not walk_started:
                            walk_started = True
                            walk_start_time = data.time
                        q, point, phase = joint_targets(gait, data.time - walk_start_time)
                        ros_q, ros_point, ros_phase = q, point, phase
                    else:
                        local_foot = leg.kinematic_from_mujoco(
                            planted_toe_world - mount_position
                        )
                        q = leg.inverse(local_foot)
                        if q is None or leg.violated_joints(q):
                            raise RuntimeError("Cometyang sit/stand target left the LF IK workspace")
                        ros_q, ros_point = q, local_foot
                        ros_phase = ros_publisher.mode
                    next_ros_control = data.time + COMETYANG_TRANSIT_DT
                else:
                    mount_z = float(mount_position[2])
                q, point, phase = ros_q, ros_point, ros_phase

            data.mocap_pos[0] = mount_position if ros_publisher is not None else [0, 0, mount_z]
            data.ctrl[:3] = leg.to_ros(q)
            mujoco.mj_step(model, data)
            if ros_publisher is not None and data.time >= next_ros_publish:
                ros_publisher.publish(data.qpos[:3], mount_position)
                next_ros_publish = data.time + 0.02

            toe_rel = data.site_xpos[site] - data.mocap_pos[0]
            touching, force = contact_force(model, data, toe, floor)
            rows.append(
                dict(
                    time_s=data.time, phase=phase if data.time > hold_seconds + lower_seconds else "lowering",
                    mount_z_m=mount_z, target_x_m=point[0], target_y_m=point[1], target_z_m=point[2],
                    toe_x_m=toe_rel[0], toe_y_m=toe_rel[1], toe_z_m=toe_rel[2],
                    tracking_error_m=float(np.linalg.norm(toe_rel - point)),
                    toe_height_m=float(data.site_xpos[site][2]) - leg.TOE_RADIUS,
                    in_contact=int(touching), normal_force_n=force,
                    **{f"q{i + 1}_cmd_rad": float(q[i]) for i in range(3)},
                    **{f"q{i + 1}_rad": float(leg.from_ros(data.qpos[:3])[i]) for i in range(3)},
                )
            )
            if handle is not None and data.time >= next_viewer_sync:
                handle.sync()
                next_viewer_sync = data.time + viewer_sync_period
            if handle is not None or ros_publisher is not None:
                time.sleep(max(0.0, wall0 + data.time - time.perf_counter()))
        # Closing programmatically races the render thread (GLXBadDrawable), so wait for the user to close the window.
        while handle is not None and handle.is_running():
            time.sleep(0.05)
    finally:
        if handle is not None and handle.is_running():
            handle.close()
    return rows, gait.cycle_seconds, hold_seconds + lower_seconds


def summarize(rows, settle_seconds: float) -> dict:
    gait_rows = [r for r in rows if r["time_s"] >= settle_seconds]
    err = np.array([r["tracking_error_m"] for r in gait_rows])
    stance = [r for r in gait_rows if r["phase"] == "stance"]
    swing = [r for r in gait_rows if r["phase"] == "swing"]
    return dict(
        rms_tracking_error_mm=float(np.sqrt(np.mean(err**2)) * 1000),
        max_tracking_error_mm=float(err.max() * 1000),
        stance_contact_fraction=float(np.mean([r["in_contact"] for r in stance])) if stance else float("nan"),
        swing_contact_fraction=float(np.mean([r["in_contact"] for r in swing])) if swing else float("nan"),
        peak_normal_force_n=float(max(r["normal_force_n"] for r in gait_rows)),
        min_toe_clearance_swing_mm=float(min(r["toe_height_m"] for r in swing) * 1000) if swing else float("nan"),
        max_toe_clearance_swing_mm=float(max(r["toe_height_m"] for r in swing) * 1000) if swing else float("nan"),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hip-height", type=float, default=Gait.hip_height, help="hip to toe center at stance, m")
    parser.add_argument("--scale", type=float, default=Gait.scale, help="fraction of the paper's leg size")
    parser.add_argument("--press", type=float, default=0.004, help="extra downward toe target at mid-stance, m")
    parser.add_argument("--drop", type=float, default=0.06, help="start height of the toe above the floor, m")
    parser.add_argument("--hold", type=float, default=0.5, help="seconds held at the start height")
    parser.add_argument("--lower", type=float, default=2.0, help="seconds to lower to the floor")
    parser.add_argument("--cycles", type=float, default=3.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--ros", action="store_true", help="publish Cometyang joint_states for RViz")
    parser.add_argument("--output", type=Path, help="write the per-step log as CSV")
    args = parser.parse_args()

    gait = Gait(hip_height=args.hip_height, scale=args.scale, stance_press=args.press)
    ros_publisher = RosJointStatePublisher() if args.ros else None
    try:
        rows, _, settle = run(
            gait, args.drop, args.lower, args.hold, args.cycles, args.viewer, ros_publisher
        )
    except PathError as error:
        print(f"Infeasible path: {error}", file=sys.stderr)
        return 2
    finally:
        if ros_publisher is not None:
            ros_publisher.close()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    for key, value in summarize(rows, settle).items():
        print(f"{key:28s} {value:10.3f}")
    if args.viewer:
        # GLFW teardown can hang or segfault after the window closes; the results are already printed.
        sys.stdout.flush()
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
