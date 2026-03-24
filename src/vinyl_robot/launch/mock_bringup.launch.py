"""
mock_bringup.launch.py — Full stack in simulation (no hardware required).

Starts every node with mock / stub substitutes:
  - mock_canopen_master  : simulates all 5 CAN nodes at 50 Hz
  - lidar_safety         : always publishes scale=1.0 (no real LiDAR)
  - motion_coordinator   : real implementation, talks to mock CAN
  - turntable_monitor    : publishes static progress=0.0 (no camera)
  - state_machine        : stub, waits for home_all
  - led_controller       : full implementation (mock_mode=true, no SDO writes)
  - diagnostics_aggregator
  - web_interface        : rosbridge + HTTP server (optional)

Usage (inside Docker or ROS environment):
  ros2 launch vinyl_robot mock_bringup.launch.py

  # With web UI:
  ros2 launch vinyl_robot mock_bringup.launch.py web_ui:=true

  # Trigger homing manually:
  ros2 action send_goal /motion/home_all vinyl_robot_msgs/action/HomeAll {}
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("vinyl_robot"), "config")
    config_file = os.path.join(config_dir, "robot_params.yaml")

    web_ui_arg = DeclareLaunchArgument(
        "web_ui", default_value="true", description="Launch rosbridge + web UI (true/false)"
    )

    # ── Mock CANopen master (simulates all 5 CAN nodes) ───────────────────────
    mock_canopen = Node(
        package="mock_nodes",
        executable="mock_canopen_master",
        name="mock_canopen_master",
        parameters=[{"config_path": config_file}],
        output="screen",
    )

    # ── LiDAR safety — mock mode (always safe) ────────────────────────────────
    lidar_safety = Node(
        package="lidar_safety",
        executable="lidar_safety",
        name="lidar_safety",
        parameters=[
            {
                "mock_mode": True,
                "warning_radius_mm": 400.0,
                "stop_radius_mm": 200.0,
            }
        ],
        output="screen",
    )

    # ── Motion coordinator — real implementation ──────────────────────────────
    motion_coordinator = Node(
        package="motion_coordinator",
        executable="motion_coordinator",
        name="motion_coordinator",
        parameters=[{"config_path": config_file}],
        output="screen",
    )

    # ── Turntable monitor — mock mode ─────────────────────────────────────────
    turntable_monitor = Node(
        package="turntable_monitor",
        executable="turntable_monitor",
        name="turntable_monitor",
        parameters=[{"mock_mode": True}],
        output="screen",
    )

    # ── State machine — full BehaviorTree implementation ──────────────────────
    state_machine = Node(
        package="state_machine",
        executable="state_machine",
        name="state_machine",
        parameters=[
            {
                "config_path": config_file,
                "tick_rate_hz": 10.0,
                "progress_threshold": 0.92,
            }
        ],
        output="screen",
    )

    # ── LED controller ────────────────────────────────────────────────────────
    led_controller = Node(
        package="led_controller",
        executable="led_controller",
        name="led_controller",
        parameters=[{"config_path": config_file}],
        output="screen",
    )

    # ── Diagnostics aggregator ────────────────────────────────────────────────
    diagnostics = Node(
        package="diagnostics_aggregator",
        executable="diagnostics_aggregator",
        name="diagnostics_aggregator",
        output="screen",
    )

    # ── Web UI (rosbridge + HTTP server, optional) ────────────────────────────
    web_ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("web_interface"), "launch", "web_ui.launch.py"
            )
        ),
        condition=IfCondition(LaunchConfiguration("web_ui")),
    )

    return LaunchDescription(
        [
            web_ui_arg,
            mock_canopen,
            lidar_safety,
            motion_coordinator,
            turntable_monitor,
            state_machine,
            led_controller,
            diagnostics,
            web_ui,
        ]
    )
