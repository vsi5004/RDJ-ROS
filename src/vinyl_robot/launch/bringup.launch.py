"""
bringup.launch.py — Full stack on real hardware (Toradex + Mallow carrier).

Hardware prerequisites:
  1. CAN bus configured: ip link set can0 type can bitrate 1000000 && ip link set can0 up
     (systemd service can0.service handles this at boot — see docs/DEPLOYMENT.md)
  2. SICK TIM571 reachable on Ethernet (default IP: 192.168.0.1)
  3. Camera at /dev/video0
  4. All 5 CAN nodes powered and operational

Launch order (dependency chain):
  canopen_master → sick_scan_xd, usb_cam → lidar_safety → motion_coordinator
  → turntable_monitor → led_controller → state_machine

NOTE on ros2_canopen integration:
  ros2_canopen in Humble uses a lifecycle-managed device container pattern.
  The canopen_master_driver and proxy drivers run inside a ComponentManager.
  See https://ros-industrial.github.io/ros2_canopen/ for the full setup.
  The Node declarations below are simplified — adapt them to your ros2_canopen
  version if the exact executable names differ.
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
    bus_config = os.path.join(config_dir, "bus.yml")

    lidar_ip_arg = DeclareLaunchArgument(
        "lidar_ip", default_value="192.168.0.1", description="IP address of the SICK TIM571 LiDAR"
    )
    web_ui_arg = DeclareLaunchArgument(
        "web_ui", default_value="true", description="Launch rosbridge + web UI (true/false)"
    )

    # ── CANopen master + device drivers ───────────────────────────────────────
    # NOTE: ros2_canopen typically uses a lifecycle component manager.
    # This simplified version may need adaptation. See ros2_canopen docs.
    canopen_master = Node(
        package="canopen_master_driver",
        executable="canopen_master_driver_node",
        name="canopen_master",
        parameters=[
            {
                "bus_config": bus_config,
                "master_config": os.path.join(config_dir, "master.dcf"),
                "can_interface_name": "can0",
            }
        ],
        output="screen",
    )

    # ── SICK TIM571 LiDAR ─────────────────────────────────────────────────────
    sick_scan = Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        name="sick_scan_xd",
        parameters=[
            {
                "scanner_type": "sick_tim_5xx",
                "hostname": LaunchConfiguration("lidar_ip"),
                "port": "2112",
                "frame_id": "laser",
            }
        ],
        output="screen",
    )

    # ── USB camera ────────────────────────────────────────────────────────────
    usb_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        parameters=[
            {
                "video_device": "/dev/video0",
                "image_width": 640,
                "image_height": 480,
                "framerate": 15.0,
                "pixel_format": "yuyv",
            }
        ],
        output="screen",
        remappings=[("image_raw", "/camera/image_raw")],
    )

    # ── LiDAR safety (real mode) ──────────────────────────────────────────────
    lidar_safety = Node(
        package="lidar_safety",
        executable="lidar_safety",
        name="lidar_safety",
        parameters=[
            {
                "mock_mode": False,
                "warning_radius_mm": 400.0,
                "stop_radius_mm": 200.0,
            }
        ],
        output="screen",
    )

    # ── Motion coordinator ────────────────────────────────────────────────────
    motion_coordinator = Node(
        package="motion_coordinator",
        executable="motion_coordinator",
        name="motion_coordinator",
        parameters=[{"config_path": config_file}],
        output="screen",
    )

    # ── Turntable monitor (real camera) ───────────────────────────────────────
    turntable_monitor = Node(
        package="turntable_monitor",
        executable="turntable_monitor",
        name="turntable_monitor",
        parameters=[{"mock_mode": False}],
        output="screen",
    )

    # ── LED controller ────────────────────────────────────────────────────────
    led_controller = Node(
        package="led_controller",
        executable="led_controller",
        name="led_controller",
        output="screen",
    )

    # ── State machine ─────────────────────────────────────────────────────────
    state_machine = Node(
        package="state_machine",
        executable="state_machine",
        name="state_machine",
        output="screen",
    )

    # ── Diagnostics ───────────────────────────────────────────────────────────
    diagnostics = Node(
        package="diagnostics_aggregator",
        executable="diagnostics_aggregator",
        name="diagnostics_aggregator",
        output="screen",
    )

    # ── Web UI ────────────────────────────────────────────────────────────────
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
            lidar_ip_arg,
            web_ui_arg,
            canopen_master,
            sick_scan,
            usb_cam,
            lidar_safety,
            motion_coordinator,
            turntable_monitor,
            led_controller,
            state_machine,
            diagnostics,
            web_ui,
        ]
    )
