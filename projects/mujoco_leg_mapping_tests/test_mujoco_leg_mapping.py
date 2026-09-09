#!/usr/bin/env python3
"""Regression checks for controller-to-MuJoCo SpotMicro leg mapping."""

import ast
from pathlib import Path
import unittest

import yaml


REPO = Path(__file__).resolve().parents[2]
SIM_NODE = REPO / "mike_mujoco_ws/src/spotMicro/spot_micro_mujoco_sim/spot_micro_mujoco_sim/mujoco_sim_node.py"
CONTROLLER_YAML = REPO / "mike_mujoco_ws/src/spotMicro/spot_micro_motion_cmd/config/spot_micro_motion_cmd.yaml"


def class_constant(name):
    tree = ast.parse(SIM_NODE.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "SpotMicroMujocoSim":
            for statement in node.body:
                if isinstance(statement, ast.AnnAssign) and statement.target.id == name:
                    return ast.literal_eval(statement.value)
    raise AssertionError(f"Could not find {name}")


class TestMujocoLegMapping(unittest.TestCase):
    def test_all_twelve_servo_numbers_match_controller(self):
        sim_config = class_constant("DEFAULT_SERVO_CONFIG")
        controller = yaml.safe_load(CONTROLLER_YAML.read_text(encoding="utf-8"))
        params = controller["spot_micro_motion_cmd"]["ros__parameters"]

        expected_names = {
            f"{side}{end}_{joint}"
            for side in ("R", "L")
            for end in ("F", "B")
            for joint in (1, 2, 3)
        }
        self.assertEqual(set(sim_config), expected_names)
        for servo_name in sorted(expected_names):
            self.assertEqual(
                sim_config[servo_name]["num"],
                params[servo_name]["num"],
                f"{servo_name} is connected to the wrong simulated joint",
            )

    def test_servo_numbers_are_unique(self):
        sim_config = class_constant("DEFAULT_SERVO_CONFIG")
        self.assertEqual(
            sorted(config["num"] for config in sim_config.values()),
            list(range(1, 13)),
        )


if __name__ == "__main__":
    unittest.main()
