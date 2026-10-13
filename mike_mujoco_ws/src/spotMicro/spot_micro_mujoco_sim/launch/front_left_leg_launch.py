import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("spot_micro_mujoco_sim")
    rviz_share = get_package_share_directory("spot_micro_rviz")
    testbench_rviz_path = os.path.join(
        sim_share, "rviz", "front_left_testbench.rviz"
    )
    model_path = os.path.join(sim_share, "models", "spot_micro_sim.xml")
    xacro_path = os.path.join(rviz_share, "urdf", "spot_micro.urdf.xacro")
    rviz_path = os.path.join(rviz_share, "rviz", "spot_micro.rviz")
    testbench_dir = Path(sim_share).resolve().parents[4] / "projects" / "one_leg_testbench"
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
        DeclareLaunchArgument(
            "use_mujoco_sim",
            default_value="true",
            description="Run the full Cometyang MuJoCo simulator",
        ),
        DeclareLaunchArgument(
            "start_builtin_front_left_controller",
            default_value="true",
            description="Start the preset front-left sit/stand/walk command node",
        ),
        DeclareLaunchArgument(
            "run_testbench",
            default_value="false",
            description="Run the suspended one-leg testbench and publish its joint states to RViz",
        ),
        Node(
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("run_testbench"), "' != 'true'",
                " and '", LaunchConfiguration("use_mujoco_sim"), "' == 'true'",
            ])),
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
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("run_testbench"), "' != 'true'",
                " and '", LaunchConfiguration("start_builtin_front_left_controller"), "' == 'true'",
            ])),
            package="spot_micro_mujoco_sim",
            executable="front_left_motion_controller",
            name="front_left_motion_controller",
            output="screen",
            prefix="python3",
        ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("run_testbench")),
            cmd=["python3", "-m", "one_leg_testbench.sim", "--viewer", "--ros"],
            cwd=str(testbench_dir),
            output="screen",
            additional_env={
                "PYTHONPATH": [
                    str(testbench_dir),
                    os.pathsep,
                    EnvironmentVariable("PYTHONPATH", default_value=""),
                ],
            },
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("use_rviz"), "' == 'true'",
                " and '", LaunchConfiguration("run_testbench"), "' != 'true'",
            ])),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_path],
        ),
        Node(
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("use_rviz"), "' == 'true'",
                " and '", LaunchConfiguration("run_testbench"), "' == 'true'",
            ])),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", testbench_rviz_path],
        ),
    ])