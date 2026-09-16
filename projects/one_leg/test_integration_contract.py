#!/usr/bin/env python3
"""Static compatibility checks against the Cometyang/Mike simulation assets."""

from __future__ import annotations

import ast
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
ONE_LEG_MODEL = ROOT / "projects/one_leg/model/right_leg_test_rig.xml"
COMETYANG_MODEL = (
    ROOT
    / "mike_mujoco_ws/src/spotMicro/spot_micro_mujoco_sim/models/spot_micro_sim.xml"
)
COMETYANG_NODE = (
    ROOT
    / "mike_mujoco_ws/src/spotMicro/spot_micro_mujoco_sim/spot_micro_mujoco_sim/mujoco_sim_node.py"
)
GENERATED_URDF = (
    ROOT / "mike_mujoco_ws/src/spotMicro/spot_micro_rviz/urdf/spot_micro.urdf"
)
WINDOWS_TESTBENCH = ROOT / "projects/one_leg/windows_testbench.html"

EXPECTED_JOINTS = {
    "front_right_shoulder",
    "front_right_leg",
    "front_right_foot",
}
EXPECTED_ACTUATORS = {f"{name}_actuator" for name in EXPECTED_JOINTS}


def xml_names(path: Path, element: str) -> set[str]:
    tree = ET.parse(path)
    return {
        node.attrib["name"]
        for node in tree.iter(element)
        if "name" in node.attrib
    }


def class_dict_constant(path: Path, class_name: str, constant_name: str):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in tree.body:
        if not isinstance(node, ast.ClassDef) or node.name != class_name:
            continue
        for statement in node.body:
            if isinstance(statement, ast.AnnAssign):
                if getattr(statement.target, "id", None) == constant_name:
                    return ast.literal_eval(statement.value)
    raise AssertionError(f"Could not find {class_name}.{constant_name}")


class IntegrationContract(unittest.TestCase):
    def test_one_leg_joint_names_match_cometyang_mjcf(self):
        one_leg = xml_names(ONE_LEG_MODEL, "joint")
        cometyang = xml_names(COMETYANG_MODEL, "joint")
        self.assertTrue(EXPECTED_JOINTS <= one_leg)
        self.assertTrue(EXPECTED_JOINTS <= cometyang)

    def test_one_leg_actuator_names_match_cometyang_mjcf(self):
        one_leg = xml_names(ONE_LEG_MODEL, "position")
        cometyang = xml_names(COMETYANG_MODEL, "position")
        self.assertTrue(EXPECTED_ACTUATORS <= one_leg)
        self.assertTrue(EXPECTED_ACTUATORS <= cometyang)

    def test_ros_bridge_maps_front_right_servos_to_these_joints(self):
        mapping = class_dict_constant(
            COMETYANG_NODE, "SpotMicroMujocoSim", "SERVO_TO_JOINT"
        )
        self.assertEqual(
            {mapping["RF_1"], mapping["RF_2"], mapping["RF_3"]},
            EXPECTED_JOINTS,
        )

    def test_existing_rviz_urdf_contains_front_right_joint_chain(self):
        rviz_joint_names = xml_names(GENERATED_URDF, "joint")
        self.assertTrue(EXPECTED_JOINTS <= rviz_joint_names)

    def test_windows_testbench_uses_cometyang_rviz_conventions(self):
        html = WINDOWS_TESTBENCH.read_text(encoding="utf-8")
        for joint_name in EXPECTED_JOINTS:
            self.assertIn(joint_name, html)

        signs_match = re.search(r"const RVIZ_SIGNS = (\[[^;]+\]);", html)
        limits_match = re.search(r"const RVIZ_LIMITS = (\[[^;]+\]);", html)
        self.assertIsNotNone(signs_match)
        self.assertIsNotNone(limits_match)
        self.assertEqual(ast.literal_eval(signs_match.group(1)), [1, -1, -1])

        urdf = ET.parse(GENERATED_URDF)
        joints = {node.attrib["name"]: node for node in urdf.iter("joint")}
        expected_limits = []
        for joint_name in (
            "front_right_shoulder",
            "front_right_leg",
            "front_right_foot",
        ):
            limit = joints[joint_name].find("limit")
            expected_limits.append(
                [float(limit.attrib["lower"]), float(limit.attrib["upper"])]
            )
        self.assertEqual(ast.literal_eval(limits_match.group(1)), expected_limits)


if __name__ == "__main__":
    unittest.main()
