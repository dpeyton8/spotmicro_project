import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("spot_micro_mujoco_sim")
    rviz_share = get_package_share_directory("spot_micro_rviz")
    model_path = os.path.join(sim_share, "models", "spot_micro_sim.xml")
    xacro_path = os.path.join(rviz_share, "urdf", "spot_micro.urdf.xacro")
    rviz_path = os.path.join(rviz_share, "rviz", "spot_micro.rviz")
    robot_description = subprocess.check_output(["xacro", xacro_path], text=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 for the front-left leg visualization",
        ),
        DeclareLaunchArgument(
            "use_mujoco_viewer",
            default_value="false",
            description="Open MuJoCo's native full-robot viewer alongside RViz2",
        ),
        Node(
            package="spot_micro_mujoco_sim",
            executable="mujoco_sim_node",
            name="mujoco_sim",
            output="screen",
            prefix="python3",
            parameters=[{
                "model_path": model_path,
                "initial_body_z": 0.25,
                "front_left_only_mode": True,
                "use_mujoco_viewer": LaunchConfiguration("use_mujoco_viewer"),
            }],
        ),
        Node(
            package="spot_micro_mujoco_sim",
            executable="front_left_motion_controller",
            name="front_left_motion_controller",
            output="screen",
            prefix="python3",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_path],
        ),
    ])