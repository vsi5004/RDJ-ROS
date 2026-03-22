import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory('web_interface'), 'web')

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
            'default_call_service_timeout': 5.0,
            'call_services_in_new_thread': True,
            'send_action_goals_in_new_thread': True,
        }],
        output='screen',
    )

    # Lightweight Python HTTP server to serve the web UI files
    http_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8080', '--directory', web_dir],
        output='screen',
    )

    return LaunchDescription([rosbridge, http_server])
